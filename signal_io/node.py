"""两个进程：SimNode（MuJoCo 侧）和 ControllerNode（控制侧）。

时序（和真机一致）:
    500 Hz 物理/通信:  sim 每 2 ms 走一步物理，并把 LowState 发出去；
    50 Hz 控制:        controller 每 20 ms 用最新 LowState 算一次 LowCmd；
    500 Hz 保持:       controller 每个 2 ms 都把"最近一次的 LowCmd"发出去（零阶保持）；
    健康层 + 安全层:   controller 每个 2 ms 都先判一次健康等级，再过一遍安全监督器
                       （L0 直通 / L1-L2 下蹲保护 / L3 卸力锁存）—— 这是最后一道闸；
    看门狗:            sim 侧 60 ms 收不到 LowCmd 就把力矩清零。

三层是分开的、都能单独关掉，方便定位问题：
    transport 看门狗  —— 只管"有没有消息"
    health.HealthMonitor —— 只管"信息够不够、可不可信"，输出 L0..L3
    safety.SafetySupervisor —— 只管"这个等级该下发什么电机指令"
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
    """把仿真钉在墙钟上：每步目标 2 ms，超了就记一次 overrun。"""

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
    """MuJoCo 侧节点。物理 + LowState 发布 + LowCmd 接收。"""

    def __init__(self, xml_path: str, transport: Transport, sim_dt: float = 0.002,
                 watchdog_ms: float = 60.0, ramp_rate: float = 200.0, safety: bool = True,
                 hold_until_first_cmd: bool = True, hold_kp: float = 60.0, hold_kd: float = 5.0,
                 spec: Optional[RobotSpec] = None, foot_soft_contact: bool = False,
                 solver_iterations: Optional[int] = None,
                 solver_ls_iterations: Optional[int] = None, degrader=None):
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
        """一个 500 Hz 拍：收指令 -> 安全层 -> 施加 -> 物理 -> 发状态。"""
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
    """控制侧节点。50 Hz 算指令，500 Hz 发指令（零阶保持）+ 健康/安全分级。"""

    def __init__(self, transport: Transport, controller: BaseController, sim_dt: float = 0.002,
                 control_dt: float = 0.02, verbose: bool = True,
                 health=None, supervisor=None, on_control=None) -> None:
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
        """发 ticks 拍阻尼（默认 50 拍 = 100 ms @ 500 Hz）再停止发布。"""
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
        n_steps = None if duration <= 0 else int(round(duration / self.dt))
        pacer = Pacer(self.dt, enabled=realtime)
        t_ctrl0 = time.monotonic()
        health_state = None
        k = 0
        clock = 0.0          # 逻辑时钟：新鲜状态用 sim_time/tick，断流后按 dt 推进
        n_no_state = 0
        warned_no_state = False
        while n_steps is None or k < n_steps:
            st = self.tr.recv_state()
            fresh = st is not None
            if fresh:
                self._state = st
                self.n_state += 1
                n_no_state = 0
            else:
                n_no_state += 1
            wall = time.monotonic() - t_ctrl0
            # 时钟必须单调推进：否则 LowState 一断流，health 的 state_age 会冻结，
            # L2/L3 永远不触发（真机上就是网线掉了还在一直发最后一条指令）。
            if fresh and st is not None and float(st.sim_time) > 0.0:
                clock = max(clock, float(st.sim_time))
            else:
                clock = max(clock + self.dt, wall)
            # 还没收到过任何 LowState 之前，**一个字节都不发**，但计时照走（duration 必须生效）。
            if self._state is None:
                if not warned_no_state and n_no_state * self.dt >= 1.0:
                    print("[ctrl] ⚠ 等待 /lowstate 超时（>1 s）：检查网线 / 网段 / domain / 网卡 / 话题名")
                    warned_no_state = True
                pacer.tick()
                k += 1
                continue
            now = clock
            if self.health is not None:
                # 断流后传 None：health 用自己记录的"最后一次收到状态"的时间判 age；
                # 传旧 state 会每拍刷新 _state_stamp，导致 state_age 恒 0、永不升级。
                health_state = self.health.check(self._state if fresh else None, now)
                self.level_hist[health_state.level_name] = \
                    self.level_hist.get(health_state.level_name, 0) + 1
                if self.first_level is None:
                    self.first_level = health_state.level_name
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
            cmd = self._cmd
            if self.supervisor is not None and health_state is not None:
                cmd = self.supervisor.process(cmd, health_state, now, self.dt)
                self.mode_hist[self.supervisor.mode] = self.mode_hist.get(self.supervisor.mode, 0) + 1
            self.tr.publish_cmd(cmd)
            self.n_cmd += 1
            pacer.tick()
            k += 1
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
