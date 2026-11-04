"""控制循环的两个节点：SimNode（MuJoCo 侧）与 ControllerNode（控制侧）。

===============================================================================
1. 为什么是两个类
===============================================================================

  单进程（loopback / tests / --local 观察器）:
      SimNode.tick() 和 ControllerNode 在同一进程里交替跑（由上层驱动）。
  双进程（udp / ros2 / dds，仿真与真机都是这个形态）:
      进程 A: sim_env/run_sim.py   -> SimNode      （物理 + 发 LowState）
      进程 B: ros2_controller/...  -> ControllerNode（策略 + 发 LowCmd）
      两者通过 Transport 通信，彼此不知道对方是 MuJoCo 还是真机。

  真机时没有 SimNode：机器人固件就是"数据源 + 物理"，ControllerNode 完全一样。

===============================================================================
2. 时序（500 Hz 通信 / 50 Hz 控制）
===============================================================================

  每个 2 ms 拍（ControllerNode.run）:
      1. recv_state()                 取最新 LowState（没有则 fresh=False）
      2. 逻辑时钟推进                 新鲜状态用 tick/sim_time；断流按 dt 推进
      3. HealthMonitor.check()        判 L0..L3（断流/NaN/姿态/电机健康）
      4. 每 10 拍（k % decim == 0）   跑策略 -> 12 维关节位置目标
      5. SafetySupervisor.process()   按等级选 track/crouch/damping/estop + 限幅
      6. publish_cmd()                500 Hz 零阶保持下发（真机后端在这里写 CRC）

  SimNode 每个 2 ms 拍：
      recv_cmd() -> SafetyLimiter（看门狗/斜率） -> MuJoCo step -> publish_state()

===============================================================================
3. 三层保护（都可单独关掉，方便定位问题）
===============================================================================

  transport 看门狗（仅仿真侧 SafetyLimiter）
      只管"有没有消息"：仿真里 60 ms 收不到 LowCmd 就把力矩清零。
      真机方向没有这个看门狗（固件行为见 real_env/README.md）。

  health.HealthMonitor
      只管"信息够不够、可不可信"，输出 L0..L3：
      L0 正常 / L1 降级 / L2 保护 / L3 急停锁存。

  safety.SafetySupervisor
      只管"这个等级该下发什么电机指令"：
      track -> crouch -> damping -> estop；关节/力矩/斜率限幅。

===============================================================================
4. 时钟与断流（真机安全的关键实现细节）
===============================================================================

  控制器的时基不能直接用 LowState.sim_time：
    * 仿真里 sim_time = tick*0.002，是确定性的；
    * 真机里 sim_time = tick*0.001（固件 tick 按 1 ms 递增）；
    * 一旦 LowState 断流，sim_time 会冻结 —— 如果 health 也用这个冻结时间，
      state_age 永远是 0，L2/L3 永远不触发，程序会一直按 500 Hz 发最后一条指令。

  所以 ControllerNode.run 维护一个单调逻辑时钟：
      新鲜状态: clock = max(clock, state.sim_time)
      断流:     clock = max(clock + dt, wall_clock)
  并且断流时给 health.check 传 None（让 health 用自己记录的"最后收到时间"算 age）。
  同时 duration 按拍数计时，不依赖是否收到状态 —— 没接机器人时也能正常退出。

===============================================================================
5. 真机与仿真的差异（只看这几点）
===============================================================================

  仿真 SimNode                         真机 ControllerNode
  --------------------------------     ------------------------------------
  物理用 MuJoCo 500 Hz step            物理在机器人 MCU，PC 只发指令
  publish_state 发 MuJoCo 真值          subscribe 固件 LowState
  watchdog 60 ms -> 清零力矩            没有软件看门狗；靠 health L1/L2/L3
  退出即停                             退出前发 100 ms 阻尼（emergency_stop）

相关文件：
  ros2_controller/controller.py   策略/站姿/零力矩控制器
  ros2_controller/health.py       健康分级
  ros2_controller/safety.py       安全监督器与限幅
  signal_io/transport/*           通信后端
  real_env/README.md              真机部署 checklist
"""

from __future__ import annotations

import csv
import os
import time
from typing import Callable, Dict, Optional

import numpy as np

from ros2_controller.controller import BaseController
from .msg import NUM_JOINT, LowCmd, LowState
from sim_env.robot import Go2Sim, RobotSpec, make_stand_cmd
from ros2_controller.safety import SafetyLimiter
from signal_io.transport import Transport


class Pacer:
    """把循环钉在墙钟上：每拍目标 dt 秒，超时记一次 overrun（用于发现调度抖动）。

    为什么需要它：
      * 仿真和真机都要求 500 Hz（2 ms）的稳定节拍，不能"能跑多快跑多快"；
      * 但 Python/WSL2/GC 会造成偶发超时，所以除了 sleep 到下一拍，还要统计：
          overruns    超时的拍数（判断 500 Hz 是否真的达标）
          lag_p95_ms  延迟的 95 分位
          rtf         实际推进的仿真时间 / 墙钟时间（1.0 = 实时）

    注意：enabled=False 时只计数不 sleep（测试/追求吞吐时用）。
    """

    def __init__(self, dt: float, enabled: bool = True):
        self.dt = float(dt)
        self.enabled = bool(enabled)
        self.t0 = time.perf_counter()
        self.next_t = self.t0
        self.n = 0
        self.overruns = 0
        self.max_lag = 0.0
        self._lags = []

    def tick(self) -> None:
        """一拍结束时调用：sleep 到下一拍起点，并记录本拍是否超时。"""
        self.n += 1
        self.next_t += self.dt
        if not self.enabled:
            return
        now = time.perf_counter()
        lag = now - self.next_t
        if lag > 0:
            self.overruns += 1
            self.max_lag = max(self.max_lag, lag)
        if len(self._lags) < 30000:
            self._lags.append(lag)
        if self.next_t > now:
            time.sleep(self.next_t - now)

    def stats(self) -> Dict[str, float]:
        wall = time.perf_counter() - self.t0
        sim = self.n * self.dt
        p95 = float(np.percentile(self._lags, 95)) if self._lags else 0.0
        return {
            "steps": self.n,
            "wall_s": round(wall, 3),
            "sim_s": round(sim, 3),
            "rtf": round(sim / wall, 3) if wall > 0 else 0.0,
            "overruns": self.overruns,
            "lag_p95_ms": round(p95 * 1e3, 3),
            "lag_max_ms": round(self.max_lag * 1e3, 3),
        }


class SimNode:
    """MuJoCo 侧节点（仅在仿真里存在）：物理 + LowState 发布 + LowCmd 接收。

    一个 tick（2 ms）做四件事：
        1) recv_cmd() 收控制器指令（可能没有；没有就沿用上一条）
        2) SafetyLimiter：看门狗（收不到指令 → 力矩清零）+ 力矩斜率限幅
        3) 把指令写进 MuJoCo（PD 由 Go2Sim.write_cmd 算），step 一次物理
        4) read_state() -> publish_state()，把 LowState 发给控制器/观察器

    真机没有这个类：机器人固件承担"物理 + 发状态"，PC 只跑 ControllerNode。
    """

    def __init__(self, xml_path: str, transport: Transport, sim_dt: float = 0.002,
                 watchdog_ms: float = 60.0, ramp_rate: float = 200.0, safety: bool = True,
                 hold_until_first_cmd: bool = True, hold_kp: float = 60.0, hold_kd: float = 5.0,
                 spec: Optional[RobotSpec] = None, foot_soft_contact: bool = False,
                 solver_iterations: Optional[int] = None,
                 solver_ls_iterations: Optional[int] = None, degrader=None):
        """建仿真节点。

        Args:
            xml_path: 带场景的 MJCF（scene.xml）。
            transport: 传输后端（loopback/udp/ros2/dds）。
            sim_dt: 物理步长，0.002 = 500 Hz（与真机低层一致）。
            watchdog_ms: 仿真侧 SafetyLimiter 的指令超时（掉 LowCmd 清零力矩）。
            ramp_rate: 力矩斜率限幅 N*m/s（保护齿轮箱）。
            safety: 关掉后退回"纯直通"（调试用）。
            hold_until_first_cmd: 上电后控制器还没接管时，自己用 PD 顶住站姿
                （否则 demo 一启动就趴下；真机对应"先别关运控服务"）。
            hold_kp/hold_kd: 上面那个保持站姿的 PD 增益。
            spec: 机型参数（力矩上限/标称站姿）；None 则从模型 keyframe 取。
            foot_soft_contact/solver_iterations/solver_ls_iterations: 物理复刻训练侧用。
            degrader: 可选 SignalDegrader，在"发出去的状态"上注入延迟/丢包/噪声/偏置，
                用来压测健康层降级路径（不改物理）。
        """
        self.sim = Go2Sim(xml_path, spec=spec, sim_dt=sim_dt, foot_soft_contact=foot_soft_contact,
                          solver_iterations=solver_iterations,
                          solver_ls_iterations=solver_ls_iterations)
        self.tr = transport
        self.safety = SafetyLimiter(ramp_rate=ramp_rate, watchdog_ms=watchdog_ms, enabled=safety)
        self.dt = self.sim.dt
        # 上电但还没人来接管时：自己顶住站姿，避免 demo 一启动就趴下。
        # 真机上的对应动作是"先别关运控服务"，等控制器准备好了再交接。
        self.hold_until_first_cmd = bool(hold_until_first_cmd)
        self._hold_cmd = make_stand_cmd(self.sim.stand_pose, kp=hold_kp, kd=hold_kd)
        # 可选：在**仿真侧**注入真机才有的信号劣化（延迟/丢帧/噪声/偏置），
        # 用来复现健康层的降级路径。注意它作用在"发出去的状态"上，不改物理。
        self.degrader = degrader
        self._ever_cmd = False
        self.n_warmup_ticks = 0
        self.n_cmd = 0
        self.n_cmd_timeout = 0
        self._cmd: Optional[LowCmd] = None
        self._last_cmd_age = 0.0
        self._last_state: Optional[LowState] = None
        self._t_pub = 0.0

    def reset(self) -> None:
        self.sim.reset()
        self.safety.reset()
        if self.degrader is not None:
            self.degrader.reset()
        self._cmd = None

    def _publish(self, st: LowState) -> None:
        if self.degrader is not None:
            st = self.degrader.degrade_state(st, self.sim.time)
        if st is not None:
            self.tr.publish_state(st)

    def tick(self) -> LowState:
        """一个 500 Hz 拍：收指令 -> 安全层 -> 施加 -> 物理 -> 发状态。

        三种情况：
          1) 还没收到过任何 LowCmd 且 hold_until_first_cmd=True：
             用标称站姿 PD 顶住，继续等控制器接管（warmup）；
          2) 正常：SafetyLimiter 处理（可能触发超时清零/斜率限幅）后写进 MuJoCo；
          3) 其余：直接沿用上一条指令（零阶保持由 SafetyLimiter 内部处理）。

        Returns:
            本拍发布出去的 LowState（观察器/日志也会用）。
        """
        raw = self.tr.recv_cmd()
        if raw is not None:
            self._cmd = raw
            self._ever_cmd = True
            self.n_cmd += 1
            self._last_cmd_age = 0.0
        if not self._ever_cmd and self.hold_until_first_cmd:
            self.n_warmup_ticks += 1
            self.sim.write_cmd(self._hold_cmd)
            self.sim.step(1)
            st = self.sim.read_state()
            self._publish(st)
            self._last_state = st
            return st
        cmd, timed_out = self.safety.process(self._cmd, dt=self.dt)
        if timed_out and self._ever_cmd:
            self.n_cmd_timeout += 1
        self.sim.write_cmd(cmd)
        self.sim.step(1)
        st = self.sim.read_state()
        self._publish(st)
        self._last_state = st
        return st

    def run(self, duration: float, realtime: bool = True, viewer: bool = False,
            viewer_every: int = 10, on_step: Optional[Callable[[Go2Sim], None]] = None,
            log_path: Optional[str] = None) -> Dict[str, object]:
        """跑 duration 秒（<=0 表示直到 viewer 关闭/Ctrl-C）。

        Args:
            duration: 墙钟时长（按拍数折算）；0 = 无限。
            realtime: True 时用 Pacer 钉 500 Hz；False 全速跑（测试用）。
            viewer: 是否开 MuJoCo 被动窗口（会占 1~2 s 启动时间，不影响物理）。
            viewer_every: 每 N 拍同步一次画面（默认 10 拍 = 50 Hz 渲染）。
            on_step: 每拍回调（观察器/记录用）。
            log_path: 落 CSV 的路径（t/x/y/z/rpy/q/tau）。
        Returns:
            summary dict：pacer/物理/安全/最终姿态/传输统计。
        """
        n_steps = None if duration <= 0 else int(round(duration / self.dt))
        vw = None
        if viewer:
            import mujoco.viewer

            vw = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        # Pacer 必须在 viewer 起来之后再建：WSLg 下 launch_passive 要 1~2 s，
        # 放在前面会把这段启动开销记成"第一步延迟 1.8 s"，看着像抖动很大其实不是。
        pacer = Pacer(self.dt, enabled=realtime)
        log = None
        if log_path:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            log = open(log_path, "w", newline="", encoding="utf-8")
            log.write("t,x,y,z,roll,pitch,yaw,"
                      + ",".join("q%d" % i for i in range(NUM_JOINT)) + ","
                      + ",".join("tau%d" % i for i in range(NUM_JOINT)) + "\n")
        try:
            k = 0
            while n_steps is None or k < n_steps:
                st = self.tick()
                if on_step is not None:
                    on_step(self.sim)
                if log is not None:
                    rpy = self.sim.orientation_rpy()
                    q = self.sim.data.qpos[self.sim.qpos_adr]
                    row = [self.sim.time, self.sim.data.qpos[0], self.sim.data.qpos[1],
                           self.sim.data.qpos[2], rpy[0], rpy[1], rpy[2]]
                    row += list(q) + list(self.sim.tau_applied)
                    log.write(",".join("%.6f" % v for v in row) + "\n")
                if vw is not None and k % viewer_every == 0:
                    if not vw.is_running():
                        break
                    vw.sync()
                pacer.tick()
                k += 1
        finally:
            if log is not None:
                log.close()
            if vw is not None:
                vw.close()
        rpy = self.sim.orientation_rpy()
        out = {"pacer": pacer.stats(), "sim": self.sim.info(), "safety": self.safety.stats(),
               "n_cmd": self.n_cmd, "n_cmd_timeout": self.n_cmd_timeout,
               "n_warmup_ticks": self.n_warmup_ticks,
               "robot": self.sim.spec.name,
               "foot_soft_contact": self.sim.foot_soft_contact,
               "final": {"z": round(self.sim.base_height, 4),
                         "rpy_deg": [round(float(np.degrees(x)), 2) for x in rpy],
                         "tilt_deg": round(float(np.degrees(max(abs(rpy[0]), abs(rpy[1])))), 2),
                         "x": round(float(self.sim.data.qpos[0]), 4),
                         "upright": bool(self.sim.is_upright(30.0))},
               "transport": self.tr.stats()}
        if self.degrader is not None:
            out["degrader"] = self.degrader.stats()
        return out


class ControllerNode:
    """控制侧节点（仿真与真机同一份）：50 Hz 算指令 + 500 Hz 下发 + 健康/安全分级。

    与 SimNode 的区别：这里没有物理，只有"状态 -> 策略 -> 指令"。
    真机时机器人固件就是数据源，这个类一字不改（数据来自哪里由 transport 决定）。

    一个 2 ms 拍的顺序（run() 里）：
        recv_state -> 逻辑时钟 -> health.check -> [每 10 拍] 策略推理
        -> supervisor.process -> publish_cmd
    """

    def __init__(self, transport: Transport, controller: BaseController, sim_dt: float = 0.002,
                 control_dt: float = 0.02, verbose: bool = True,
                 health=None, supervisor=None, on_control=None) -> None:
        """建控制节点。

        Args:
            transport: 传输后端（真机/仿真同一接口）。
            controller: 控制器实现（PolicyController/StandController/ZeroController）。
            sim_dt: 通信/下发周期，0.002 = 500 Hz（与真机低层一致）。
            control_dt: 策略周期，0.02 = 50 Hz；decim = control_dt/sim_dt = 10。
            verbose: 是否打印非致命异常（例如 on_control 失败）。
            health: HealthMonitor（None = 关闭健康层）。
            supervisor: SafetySupervisor（None = 关闭安全监督器）。
            on_control: 每个 50 Hz 控制拍后的回调（发布 obs/action/info 给观察器）。
        """
        self.tr = transport
        self.ctrl = controller
        self.dt = float(sim_dt)
        self.decim = max(1, int(round(control_dt / sim_dt)))
        self.verbose = verbose
        # 健康层（判等级）与安全层（按等级出指令）都是可选的：关掉就退回 v1 的纯直通行为。
        self.health = health
        self.supervisor = supervisor
        # 每个 50 Hz 控制拍之后回调：给 ROS 2 debug 发布器 / 观察器节点用
        self.on_control = on_control
        self.n_state = 0
        self.n_cmd = 0
        self._state: Optional[LowState] = None
        self._cmd = LowCmd().zero()
        self._k = 0
        self.latencies = []
        self.level_hist: Dict[str, int] = {}
        self.mode_hist: Dict[str, int] = {}
        self.first_level = None

    def _damping_cmd(self) -> LowCmd:
        """全关节阻尼指令（kp=0, kd=8），真机退出/急停收尾用。

        对应官方 unitree_rl_gym 的 create_damping_cmd：不发零力矩直接断流，
        固件行为未公开（保持最后指令 or 卸力）；先主动发阻尼更安全。
        """
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, 0.0, 0.0, 0.0, 0.0, 8.0
        return cmd

    def emergency_stop(self, ticks: int = 50) -> bool:
        """退出/急停收尾：连续发 ticks 拍阻尼，再停止发布。

        为什么不是直接停发：
          官方没有公开 lowcmd 断流后固件的行为（保持最后指令 or 超时卸力）。
          主动发"kp=0, kd=8"的阻尼指令，两种固件行为下都更安全。
        为什么返回 bool：
          从未收到过 LowState 时说明控制器根本没接管过，不应该对未初始化的机器人发指令。

        Args:
            ticks: 发多少拍；默认 50 拍 = 100 ms @ 500 Hz。
        Returns:
            True = 已发送；False = 还没收到过状态，未发送。
        """
        if self._state is None:
            return False
        cmd = self._damping_cmd()
        n = max(1, int(ticks))
        for _ in range(n):
            self.tr.publish_cmd(cmd)
            time.sleep(self.dt)
        self.n_cmd += n
        return True

    def run(self, duration: float, realtime: bool = True) -> Dict[str, object]:
        """跑控制循环。

        终止条件：duration 秒（<=0 表示直到 Ctrl-C）。
        计时按"拍数"而不是"是否收到状态"——没接机器人时 duration 也必须生效
        （旧实现里没状态会 continue 跳过 k+=1，导致进程永不退出）。

        Args:
            duration: 墙钟时长（按 2 ms 拍数折算）；0 = 无限。
            realtime: True 用 Pacer 钉 500 Hz；False 全速跑（测试用）。
        Returns:
            summary：pacer/n_state/n_cmd/controller/transport/health/supervisor 统计。
        """
        n_steps = None if duration <= 0 else int(round(duration / self.dt))
        pacer = Pacer(self.dt, enabled=realtime)
        t_ctrl0 = time.monotonic()
        health_state = None
        k = 0
        clock = 0.0          # 逻辑时钟：新鲜状态用 sim_time/tick，断流后按 dt 推进
        n_no_state = 0
        warned_no_state = False
        while n_steps is None or k < n_steps:
            # (1) 取最新状态。recv_state 是非阻塞的：没新消息返回 None（fresh=False）。
            st = self.tr.recv_state()
            fresh = st is not None
            if fresh:
                self._state = st
                self.n_state += 1
                n_no_state = 0
            else:
                n_no_state += 1
            # (2) 逻辑时钟（见模块 docstring §4）：
            wall = time.monotonic() - t_ctrl0
            # 时钟必须单调推进：否则 LowState 一断流，health 的 state_age 会冻结，
            # L2/L3 永远不触发（真机上就是网线掉了还在一直发最后一条指令）。
            if fresh and st is not None and float(st.sim_time) > 0.0:
                clock = max(clock, float(st.sim_time))
            else:
                clock = max(clock + self.dt, wall)
            # (3) 还没收到过任何 LowState 之前，一个字节都不发，但计时照走
            #     （duration 必须生效；否则首次上电 topic 不通会静默挂死）。
            if self._state is None:
                if not warned_no_state and n_no_state * self.dt >= 1.0:
                    print("[ctrl] ⚠ 等待 /lowstate 超时（>1 s）：检查网线 / 网段 / domain / 网卡 / 话题名")
                    warned_no_state = True
                pacer.tick()
                k += 1
                continue
            now = clock
            # (4) 健康分级：断流时传 None（见模块 docstring §4），由 health 自己记 age。
            if self.health is not None:
                # 断流后传 None：health 用自己记录的"最后一次收到状态"的时间判 age；
                # 传旧 state 会每拍刷新 _state_stamp，导致 state_age 恒 0、永不升级。
                health_state = self.health.check(self._state if fresh else None, now)
                self.level_hist[health_state.level_name] = \
                    self.level_hist.get(health_state.level_name, 0) + 1
                if self.first_level is None:
                    self.first_level = health_state.level_name
            # (5) 50 Hz 策略推理（decim=10）。50 Hz 之间沿用上一条 LowCmd（零阶保持）。
            if k % self.decim == 0:
                self._cmd = self.ctrl.update(self._state, now)
                self._k += 1
                if self.on_control is not None:
                    try:
                        self.on_control(self._state, self._cmd, self.ctrl,
                                        health_state, self.supervisor, now)
                    except Exception as exc:   # 调试发布不能让控制回路挂掉
                        if self.verbose:
                            print("[ctrl] on_control 异常:", repr(exc)[:160])
            # (6) 安全监督器：按健康等级决定最终下发什么（track/crouch/damping/estop）。
            cmd = self._cmd
            if self.supervisor is not None and health_state is not None:
                cmd = self.supervisor.process(cmd, health_state, now, self.dt)
                self.mode_hist[self.supervisor.mode] = self.mode_hist.get(self.supervisor.mode, 0) + 1
            # (7) 500 Hz 下发：真机后端在这里写固定头 + CRC（transport/ros2.py）。
            self.tr.publish_cmd(cmd)
            self.n_cmd += 1
            pacer.tick()
            k += 1
        # ---- summary：排障时先看 n_state/n_cmd、health.level_hist、transport.bad_crc
        out = {"pacer": pacer.stats(), "n_state": self.n_state, "n_cmd": self.n_cmd,
               "n_no_state": n_no_state, "ever_state": self._state is not None,
               "control_ticks": self._k, "controller": self.ctrl.stats(),
               "transport": self.tr.stats()}
        if self.health is not None:
            out["health"] = self.health.stats()
            out["level_hist"] = dict(sorted(self.level_hist.items()))
            out["first_level"] = self.first_level
        if self.supervisor is not None:
            sup = self.supervisor.stats()
            out["safety_supervisor"] = sup
            out["mode_hist"] = dict(sorted(self.mode_hist.items()))
            out["final_mode"] = self.supervisor.mode
            out["final_level"] = self.supervisor.level.name
        return out
