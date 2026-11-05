"""安全层：真机上必须有的三件事，仿真里也照做。

v2 在不变的 SafetyLimiter 之上新增 SafetySupervisor（把 HealthState.level 映射成
实际下发的电机指令：track / crouch / damping / estop）。

1. watchdog  —— 超时没收到指令就把力矩清零（真机上这是防飞车的第一道闸）
2. 斜率限幅  —— 力矩按 N*m/s 限速，避免第一条指令就是满力矩
3. 硬限幅    —— 按关节类别夹到 Go2 的力矩上限

===============================================================================
真机专属：上面 2/3 只在仿真里真的作用于物理（**必须在 audit 里说清楚**）
===============================================================================

  真机的 lowcmd 是 <position kp=20 kd=1> 模式：我们只发 q/kp/kd，力矩由**固件**
  按 tau = kp*(q_des - q) + kd*(0 - dq) 算出来，再夹到电机自身极限。
  SafetyLimiter 夹的是 cmd.tau 字段，而真机路径上 tau 恒 0（ros2_controller/
  controller.py 的 PolicyController 只写 q/kp/kd），所以：

      [真机] 23.7/23.7/45.43 N*m 硬限幅 + 200 N*m/s 斜率限幅  == 不生效
      [仿真] SimNode 自己按 PD 算力矩才夹（sim_env/robot.py write_cmd）

  真机上真正能限制"输出力矩过大"的只有两条：
      (a) q_des 不许跳  -> SafetySupervisor(q_slew_rate=...) 即本文件新增的限速
      (b) 固件自己的电机力矩极限（不可控，未知）

  所以 run_controller.py 在 real_* 目标上默认 q_slew_rate=20 rad/s
  （= 0.04 rad / 2 ms 拍）；仿真默认 None（关闭，保持训练/部署一致性）。

  最坏例子（膝关节，kp=20）：q_des 一拍跳 2 rad -> 固件侧 kp*2 = 40 N*m 首拍力矩，
  已经接近 calf 的 45.43 N*m 上限；跳 3 rad 就顶满。q_slew_rate=20 rad/s 把单拍
  跳变压到 0.04 rad -> 首拍最多 0.8 N*m。
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from ros2_controller.health import HealthState, Level
from signal_io.msg import NUM_JOINT, NUM_MOTOR, LowCmd
from sim_env.robot import TORQUE_LIMITS


class SafetyLimiter:
    def __init__(self, torque_limits=None, ramp_rate: float = 200.0,
                 watchdog_ms: float = 60.0, enabled: bool = True):
        self.limits = np.asarray(torque_limits if torque_limits is not None else TORQUE_LIMITS, dtype=np.float64)
        self.ramp_rate = float(ramp_rate)          # N*m / s
        self.watchdog_s = float(watchdog_ms) / 1000.0
        self.enabled = bool(enabled)
        self.last_ok = np.zeros(NUM_JOINT)
        self.last_cmd_time = time.monotonic()
        self.n_watchdog = 0
        self.n_clipped = 0

    def reset(self) -> None:
        self.last_ok[:] = 0.0
        self.last_cmd_time = time.monotonic()
        self.n_watchdog = 0
        self.n_clipped = 0

    def process(self, cmd: Optional[LowCmd], now: Optional[float] = None,
                dt: float = 0.002) -> Tuple[LowCmd, bool]:
        """返回 (安全后的指令, 是否处于 watchdog 触发态)。"""
        now = time.monotonic() if now is None else now
        if cmd is None:
            if now - self.last_cmd_time > self.watchdog_s:
                self.n_watchdog += 1
                safe = self._damping_cmd()
                self.last_ok[:] = 0.0
                return safe, True
            return self._hold_cmd(), False

        self.last_cmd_time = now
        safe = LowCmd(mode_pr=cmd.mode_pr, mode_machine=cmd.mode_machine)
        for i in range(NUM_JOINT):
            src = cmd.motor_cmd[i]
            dst = safe.motor_cmd[i]
            dst.mode, dst.q, dst.dq, dst.kp, dst.kd = src.mode, src.q, src.dq, src.kp, src.kd
            tau = float(src.tau)
            if not self.enabled:
                dst.tau = tau
                continue
            step = self.ramp_rate * dt
            tau = float(np.clip(tau, self.last_ok[i] - step, self.last_ok[i] + step))
            if abs(tau) > abs(src.tau) + 1e-9:
                self.n_clipped += 1
            tau = float(np.clip(tau, -self.limits[i], self.limits[i]))
            self.last_ok[i] = tau
            dst.tau = tau
        return safe, False

    def _damping_cmd(self) -> LowCmd:
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, 0.0, 0.0, 0.0, 0.0, 0.0
        return cmd

    def _hold_cmd(self) -> LowCmd:
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, 0.0, 0.0, float(self.last_ok[i]), 0.0, 0.0
        return cmd

    def stats(self) -> dict:
        return {"watchdog_trips": self.n_watchdog, "ramp_clips": self.n_clipped}

# ---------------------------------------------------------------- v2 分级监督器
# 本行之前（含 SafetyLimiter 全部代码）与 v1 逐位一致；以下为新增。
SUPERVISOR_MODES = ("track", "crouch", "damping", "estop")
# L3 急停的两档实现（见 SafetySupervisor 的 estop_policy）：
#   zero：20 个槽位全 0（含 mode=0）= 卸力/瘫软（旧行为，仿真默认）
#   damp：mode=0x01 / kp=0 / kd=estop_kd / q=0 = 纯关节阻尼（真机默认，与官方
#         create_damping_cmd 同义：走路中失联会"阻尼着倒下"而不是直接瘫倒）
ESTOP_POLICIES = ("zero", "damp")
DEFAULT_ESTOP_KD = 8.0   # estop_policy="damp" 且 config 没给 damp_kd 时用的 kd


class SafetySupervisor:
    """把 HealthState.level 映射成实际下发的电机指令（接口契约 §5）。

    模式语义:
      track  (L0)  直通 + q_des 限速(可选) + 斜率限幅 + 力矩硬限幅 + 看门狗
      crouch (L1)  在 crouch_s 内平滑插值到"收腿下蹲姿态"; kp/kd = mode_kp/mode_kd, tau=0
      damping(L2)  q_des = 当前 q, kp=0, kd=damp_kd（默认=mode_kd；只做阻尼）
      estop  (L3)  立即生效、不过斜率（卸力/阻尼不能等）。两种档位：
                   "zero" 全部字段 0（卸力 = 瘫软，旧行为，仿真默认）
                   "damp" mode=1/kp=0/kd=estop_kd/q=0（纯关节阻尼，real_* 默认）

    L2 默认策略 l2_policy="crouch_then_damp"，对应契约 §4 的"阻尼 + 缓慢下蹲"：
    先用 crouch 在 crouch_s 内蹲到位，再切到纯 damping（此时 q_des 已经接近当前 q，切换连续）。
    可改成 "damp"（进 L2 立即纯阻尼）或 "crouch"（L2 全程软 PD 顶住下蹲姿态）。

    任何模式切换的 tau 都过 SafetyLimiter 的斜率限幅（estop 除外）。

    q_slew_rate（rad/s，默认 None=关闭）：
        只作用于 track（L0）—— 把"策略这一拍希望到的 q_des"限制在
        上一拍 q_des ± q_slew_rate*dt 之内。为什么真机必须有它：
        真机的力矩是固件按 kp*(q_des-q) 算的，SafetyLimiter 夹的 tau 恒 0 等于没夹；
        q_des 一跳 2 rad（kp=20）就是 40 N*m。仿真里默认关闭以保持训练/部署一致性
        （sim 的力矩本来就被 SimNode 夹住了），run_controller 在 real_* 上默认开 20 rad/s。
        crouch/damping/estop 各自已经有限速/卸力逻辑，**不叠加**本限速。

    estop_policy（L3 下发什么，默认 "zero" = 与旧版逐位一致）：
        "zero"  20 槽全 0（含 mode=0）-> 固件侧 = 电机 stop，机器人立刻瘫软。
                适合"已经要摔/失控"；不适合"站着时 /lowstate 抖了 250 ms"。
        "damp"  mode=0x01, kp=0, kd=estop_kd, q=0, dq=0, tau=0 -> 纯关节阻尼。
                L3 最常见的触发源就是 LowState 断流（那时拿不到 q，也来不及算
                别的东西），此时发"阻尼"比"瘫软"对 30 kg 的狗和地板都更友好；
                与官方退出流程 create_damping_cmd 同义。
        run_controller 在 real_* 上默认选 "damp"，仿真默认 "zero"；
        config 的 safety.estop_policy 显式给了就以它为准。
    """

    MODE_BY_LEVEL = {
        Level.L0_NORMAL: "track",
        Level.L1_DEGRADED: "crouch",
        Level.L2_PROTECT: "damping",
        Level.L3_ESTOP: "estop",
    }

    def __init__(self, stand_pose, limits=None, ramp_rate: float = 200.0,
                 mode_kp: float = 20.0, mode_kd: float = 1.0,
                 crouch_depth: float = 0.35, crouch_s: float = 2.0,
                 enabled: bool = True, q_slew_rate: Optional[float] = None,
                 damp_kd: Optional[float] = None, estop_policy: str = "zero",
                 damping_kd: Optional[float] = None):
        """建监督器。

        Args:
            q_slew_rate: track 模式下 q_des 的限速 (rad/s)；None 或 <=0 表示关闭。
                真实取值见模块 docstring：真机用 20 rad/s（run_controller 在
                real_* 目标上默认打开），仿真默认关闭。
            damp_kd: L2 damping 模式用的 kd；None（默认）= 沿用 mode_kd，行为与
                旧版逐位一致。真机建议在 config 的 safety.damp_kd 里给 8（与退出
                阻尼同强度）：mode_kd=1 的"阻尼"其实很软，进 L2 后可能撑不住。
            estop_policy: L3 下发 "zero"（卸力，默认/仿真）或 "damp"（纯阻尼，
                真机默认由 run_controller 决定）。damp 用的 kd = damp_kd（显式给了
                就用它），否则 DEFAULT_ESTOP_KD=8.0。
        """

        self.stand_pose = np.asarray(stand_pose, dtype=np.float64).reshape(-1).copy()
        if self.stand_pose.size != NUM_JOINT:
            raise ValueError("stand_pose 必须是 %d 维，得到 %d" % (NUM_JOINT, self.stand_pose.size))
        self.limits = np.asarray(TORQUE_LIMITS if limits is None else limits, dtype=np.float64)
        self.ramp_rate = float(ramp_rate)
        self.mode_kp = float(mode_kp)
        self.mode_kd = float(mode_kd)
        # L2 damping 的 kd：默认跟 mode_kd 走（保持旧行为）；真机建议给 8。
        # 合并 origin/main 后同时接受 damping_kd（远端旧名），两个名字指向同一个值。
        self.damp_kd = float(mode_kd if damping_kd is None else damping_kd)
        if damp_kd is not None:
            self.damp_kd = float(damp_kd)
        self.damping_kd = self.damp_kd      # 兼容远端代码/测试里的旧属性名
        self.crouch_depth = float(crouch_depth)
        self.crouch_s = float(crouch_s)
        self.enabled = bool(enabled)
        self.l2_policy = "crouch_then_damp"
        # L3 急停档位：默认 "zero"（逐位等于旧行为）；真机由 run_controller 传 "damp"。
        policy = "zero" if estop_policy is None else str(estop_policy).strip().lower()
        if policy not in ESTOP_POLICIES:
            raise ValueError("estop_policy 只能是 %s，得到 %r"
                             % ("/".join(ESTOP_POLICIES), estop_policy))
        self.estop_policy = policy
        # estop=damp 用的 kd：显式 damp_kd 优先，否则 8.0（与退出阻尼同强度）。
        self.estop_kd = float(DEFAULT_ESTOP_KD if damp_kd is None else damp_kd)
        # track 模式的 q_des 限速 (rad/s)：None / <=0 = 关闭（默认，保持仿真行为）。
        # 真机路径必须开：固件按 kp*(q_des-q) 出力矩，q_des 跳变 = 力矩跳变。
        self.q_slew_rate: Optional[float] = (
            None if not q_slew_rate or float(q_slew_rate) <= 0.0 else float(q_slew_rate))
        self.limiter = SafetyLimiter(self.limits, ramp_rate=self.ramp_rate, enabled=self.enabled)
        # 关节顺序 FR/FL/RR/RL，每腿 3 个 -> calf（膝盖）是每腿第 3 个
        self.knee_idx = np.array([2, 5, 8, 11], dtype=int)
        self.crouch_pose = self.stand_pose.copy()
        self.crouch_pose[self.knee_idx] -= self.crouch_depth
        # q_des 的保守速率上限 (rad/s)：由 crouch 规格导出，保证相邻拍变化很小
        self.q_ramp_rate = 2.0 * self.crouch_depth / max(self.crouch_s, 1e-6)
        self._t = 0.0
        self.reset()

    # ---------------------------------------------------------------- 生命周期
    def reset(self) -> None:
        self.limiter.reset()
        self._mode = "track"
        self._level = Level.L0_NORMAL
        self._qdes = self.stand_pose.copy()
        self._last_qdes = self.stand_pose.copy()
        self._crouch_from = self.stand_pose.copy()
        self._crouch_t0 = None
        self._crouch_reached = False
        self._n_switches = 0
        self._n_process = 0
        self._n_damping_no_state = 0
        # q_des 限速统计（n_q_slew 按「关节·拍」累加；_q_slew_max_step 是单拍单关节被压掉的最大量）
        self._n_q_slew = 0
        self._q_slew_max_step = 0.0
        # 每次"进入 track"都要把限速历史对齐到**实测 q**（原因见 _slew_q 的 docstring）：
        # 否则第一条指令相对机器人真实姿态仍然是一个大台阶（prepose 的典型场景）。
        self._q_slew_seed_pending = True
        self._n_q_slew_seeds = 0
        self._t = 0.0
        self._dwell = {m: 0.0 for m in SUPERVISOR_MODES}
        self._level_dwell = {lvl: 0.0 for lvl in Level}

    # ---------------------------------------------------------------- 主入口
    def process(self, cmd, health, t: float, dt: float) -> LowCmd:
        t = float(t)
        dt = max(0.0, float(dt))
        self._t = t
        self._n_process += 1
        self._last_qdes = self._qdes.copy()

        if not self.enabled:
            safe, _timed_out = self.limiter.process(cmd, dt=dt)
            self._set_mode("track", t)
            self._remember(safe)
            self._accumulate(dt, Level.L0_NORMAL)
            return safe

        level = health.level if health is not None else Level.L0_NORMAL
        level = Level(int(level))
        mode = self._select_mode(level, t)
        self._set_mode(mode, t)

        if mode == "track":
            # 真机上唯一能压住"首拍力矩"的地方：q_des 不许跳（详见模块 docstring）。
            if self.q_slew_rate is not None and self._q_slew_seed_pending:
                self._seed_slew_from_state(health)
            safe, _timed_out = self.limiter.process(self._slew_q(cmd, dt), dt=dt)
        elif mode == "crouch":
            safe = self._crouch_cmd(t, dt)
        elif mode == "damping":
            safe = self._damping_cmd(health, dt)
        else:
            safe = self._estop_cmd()

        self._remember(safe)
        self._accumulate(dt, level)
        return safe

    # ---------------------------------------------------------------- 模式选择
    def _select_mode(self, level: Level, t: float) -> str:
        if level >= Level.L3_ESTOP:
            return "estop"
        if level >= Level.L2_PROTECT:
            if self.l2_policy == "crouch":
                return "crouch"
            if self.l2_policy == "damp":
                return "damping"
            # crouch_then_damp：蹲到位之前保持 crouch，之后纯阻尼
            if self._crouch_t0 is not None and self._crouch_done(t):
                return "damping"
            return "crouch"
        if level >= Level.L1_DEGRADED:
            return "crouch"
        return "track"

    def _set_mode(self, new: str, t: float) -> None:
        if new == self._mode:
            return
        self._n_switches += 1
        if new == "crouch":
            # 从 damping 回到 crouch 是同一次 L2 下蹲的延续（保留 t0/reached）；
            # 只有从 track/estop 进来才是新一轮下蹲。
            if self._mode != "damping":
                self._crouch_from = self._qdes.copy()
                self._crouch_t0 = t
                self._crouch_reached = False
        elif new in ("track", "estop"):
            self._crouch_t0 = None
            self._crouch_reached = False
            if new == "track":
                # 从 crouch/damping/estop 回到 track：q_des 历史要重新对齐实测 q
                self._q_slew_seed_pending = True
        self._mode = new
        if new == "estop":
            # 卸力：斜率限幅器的历史也清零，避免恢复时从旧力矩续着涨
            self.limiter.last_ok[:] = 0.0

    def _crouch_done(self, t: float) -> bool:
        return self._crouch_t0 is not None and (t - self._crouch_t0) >= self.crouch_s

    # ---------------------------------------------------------------- 各模式指令
    def _crouch_cmd(self, t: float, dt: float) -> LowCmd:
        if self._crouch_t0 is None:
            self._crouch_t0 = t
            self._crouch_from = self._qdes.copy()
        if self.crouch_s <= 0.0:
            alpha = 1.0
        else:
            u = min(1.0, max(0.0, (t - self._crouch_t0) / self.crouch_s))
            alpha = u * u * (3.0 - 2.0 * u)          # smoothstep：起止速度都为 0
        if alpha >= 1.0:
            self._crouch_reached = True
        target = (1.0 - alpha) * self._crouch_from + alpha * self.crouch_pose
        step = max(0.0, self.q_ramp_rate * dt)
        q = np.clip(target, self._last_qdes - step, self._last_qdes + step)
        cmd = self._pd_cmd(q, self.mode_kp, self.mode_kd)
        safe, _timed_out = self.limiter.process(cmd, dt=dt)
        return safe

    def _damping_cmd(self, health, dt: float) -> LowCmd:
        q = self._current_q(health)
        if q is None:
            q = self._qdes.copy()
            self._n_damping_no_state += 1
        cmd = self._pd_cmd(q, 0.0, self.damp_kd)
        safe, _timed_out = self.limiter.process(cmd, dt=dt)
        return safe

    def _current_q(self, health):
        if health is None:
            return None
        for name in ("state", "joint_q"):
            ch = health.channels.get(name)
            if ch is None or not ch.valid or ch.value is None:
                continue
            arr = np.asarray(ch.value, dtype=np.float64).reshape(-1)
            if arr.size == NUM_JOINT and bool(np.all(np.isfinite(arr))):
                return arr
        return None

    def _seed_slew_from_state(self, health) -> None:
        """把 q_des 限速的历史对齐到**实测 q**（拿不到就等下一拍）。

        为什么必须这么做（真机首测最容易踩）：
            限速只保证"相邻两拍的 q_des 不跳"，它管不了"第一条 q_des 相对机器人
            真实姿态的台阶"。policy 的 prepose 会把 q 直接写成训练 home
            （controller.py:377-390），而机器人实际姿态通常还是固件 home ——
            calf 上差 0.3 rad，kp=20 就是首拍 6 N*m。把历史对齐到实测 q 之后，
            这条 0.3 rad 会按 20 rad/s 在 15 ms 内走完，首拍只有 0.8 N*m。
        """
        q = self._current_q(health)
        if q is None:
            return                    # 没有可信实测 q（health 没给 state）：保持 pending
        self._qdes = q.copy()
        self._last_qdes = q.copy()
        self._q_slew_seed_pending = False
        self._n_q_slew_seeds += 1

    def _slew_q(self, cmd, dt: float):
        """track 模式的 q_des 限速：把每个关节的 q 夹到上一拍 q_des ± rate*dt。

        只改 q，其余字段（mode/dq/tau/kp/kd）原样保留；下游 SafetyLimiter 会再按
        力矩斜率/硬限幅处理，所以这里不碰 tau。
        q_slew_rate 为 None（默认）时**原对象直通**，保证仿真/旧路径逐位不变。
        """
        if self.q_slew_rate is None or cmd is None:
            return cmd
        step = self.q_slew_rate * dt
        out = LowCmd(mode_pr=cmd.mode_pr, mode_machine=cmd.mode_machine)
        for i in range(NUM_JOINT):
            src = cmd.motor_cmd[i]
            dst = out.motor_cmd[i]
            dst.mode, dst.dq, dst.tau, dst.kp, dst.kd = \
                src.mode, src.dq, src.tau, src.kp, src.kd
            lo = self._last_qdes[i] - step
            hi = self._last_qdes[i] + step
            q = float(src.q)
            q_c = q if lo <= q <= hi else (lo if q < lo else hi)
            if q_c != q:
                self._n_q_slew += 1
                self._q_slew_max_step = max(self._q_slew_max_step, abs(q_c - q))
            dst.q = q_c
        return out

    @staticmethod
    def _pd_cmd(q, kp: float, kd: float) -> LowCmd:
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, float(q[i]), 0.0, 0.0, float(kp), float(kd)
        return cmd

    def _estop_cmd(self) -> LowCmd:
        """L3 下发：按 estop_policy 选"卸力"或"纯阻尼"。

        zero: 20 槽全 0（含 mode=0）—— 与旧版逐位一致。
        damp: 20 槽 mode=0x01 / kp=0 / kd=estop_kd / q=dq=tau=0 —— 固件算
              tau = 0*(q_des-q) + kd*(0-dq) = -kd*dq，只泄能、不顶住。
              后 8 个保留槽位也一起置位（与 zero 分支一致，避免"12 个使能、8 个 stop"
              这种半接管状态）。
        """
        if self.estop_policy == "zero":
            return self._zero_cmd()
        cmd = LowCmd()
        for i in range(NUM_MOTOR):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, 0.0, 0.0, 0.0, 0.0, self.estop_kd
        return cmd

    @staticmethod
    def _zero_cmd() -> LowCmd:
        """L3 卸力：20 个电机槽位全部字段清零。"""
        cmd = LowCmd()
        cmd.mode_pr = 0
        cmd.mode_machine = 0
        for i in range(NUM_MOTOR):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0, 0.0, 0.0, 0.0, 0.0, 0.0
        return cmd

    # ---------------------------------------------------------------- 记账
    def _remember(self, safe: LowCmd) -> None:
        self._qdes = np.array([safe.motor_cmd[i].q for i in range(NUM_JOINT)], dtype=np.float64)

    def _accumulate(self, dt: float, level: Level) -> None:
        self._dwell[self._mode] = self._dwell.get(self._mode, 0.0) + dt
        self._level_dwell[level] = self._level_dwell.get(level, 0.0) + dt
        self._level = Level(int(level))

    # ---------------------------------------------------------------- 观测
    @property
    def level(self) -> Level:
        return self._level

    @property
    def mode(self) -> str:
        return self._mode

    def stats(self) -> dict:
        elapsed = None if self._crouch_t0 is None else max(0.0, self._t - self._crouch_t0)
        return {
            "mode": self._mode,
            "level": int(self._level),
            "level_name": "L%d" % int(self._level),
            "enabled": self.enabled,
            "l2_policy": self.l2_policy,
            "n_switches": self._n_switches,
            "n_process": self._n_process,
            "mode_dwell_s": {m: round(v, 6) for m, v in self._dwell.items()},
            "level_dwell_s": {"L%d" % int(l): round(v, 6) for l, v in self._level_dwell.items()},
            "dwell_total_s": round(sum(self._dwell.values()), 6),
            "crouch_reached": bool(self._crouch_reached),
            "crouch_elapsed_s": elapsed,
            "crouch_pose": [round(float(x), 4) for x in self.crouch_pose],
            "q_ramp_rate": self.q_ramp_rate,
            "q_slew_rate": self.q_slew_rate,
            "damp_kd": self.damp_kd,
            "estop_policy": self.estop_policy,
            "estop_kd": self.estop_kd,
            "q_slew_clips": self._n_q_slew,
            "q_slew_seeds": self._n_q_slew_seeds,
            "q_slew_max_step": round(self._q_slew_max_step, 6),
            "n_damping_no_state": self._n_damping_no_state,
            "watchdog_trips": self.limiter.n_watchdog,
            "ramp_clips": self.limiter.n_clipped,
        }
