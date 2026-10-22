"""安全层：真机上必须有的三件事，仿真里也照做。

v2 在不变的 SafetyLimiter 之上新增 SafetySupervisor（把 HealthState.level 映射成
实际下发的电机指令：track / crouch / damping / estop）。

1. watchdog  —— 超时没收到指令就把力矩清零（真机上这是防飞车的第一道闸）
2. 斜率限幅  —— 力矩按 N*m/s 限速，避免第一条指令就是满力矩
3. 硬限幅    —— 按关节类别夹到 Go2 的力矩上限
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


class SafetySupervisor:
    """把 HealthState.level 映射成实际下发的电机指令（接口契约 §5）。

    模式语义:
      track  (L0)  直通 + 斜率限幅 + 力矩硬限幅 + 看门狗（完全复用 SafetyLimiter）
      crouch (L1)  在 crouch_s 内平滑插值到"收腿下蹲姿态"; kp/kd = mode_kp/mode_kd, tau=0
      damping(L2)  q_des = 当前 q, kp=0, kd=mode_kd（只做阻尼）
      estop  (L3)  全部字段 0（卸力; 立即生效, 不过斜率 —— 卸力不能等）

    L2 默认策略 l2_policy="crouch_then_damp"，对应契约 §4 的"阻尼 + 缓慢下蹲"：
    先用 crouch 在 crouch_s 内蹲到位，再切到纯 damping（此时 q_des 已经接近当前 q，切换连续）。
    可改成 "damp"（进 L2 立即纯阻尼）或 "crouch"（L2 全程软 PD 顶住下蹲姿态）。

    任何模式切换的 tau 都过 SafetyLimiter 的斜率限幅（estop 除外）。
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
                 enabled: bool = True):
        self.stand_pose = np.asarray(stand_pose, dtype=np.float64).reshape(-1).copy()
        if self.stand_pose.size != NUM_JOINT:
            raise ValueError("stand_pose 必须是 %d 维，得到 %d" % (NUM_JOINT, self.stand_pose.size))
        self.limits = np.asarray(TORQUE_LIMITS if limits is None else limits, dtype=np.float64)
        self.ramp_rate = float(ramp_rate)
        self.mode_kp = float(mode_kp)
        self.mode_kd = float(mode_kd)
        self.crouch_depth = float(crouch_depth)
        self.crouch_s = float(crouch_s)
        self.enabled = bool(enabled)
        self.l2_policy = "crouch_then_damp"
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
            safe, _timed_out = self.limiter.process(cmd, dt=dt)
        elif mode == "crouch":
            safe = self._crouch_cmd(t, dt)
        elif mode == "damping":
            safe = self._damping_cmd(health, dt)
        else:
            safe = self._zero_cmd()

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
        cmd = self._pd_cmd(q, 0.0, self.mode_kd)
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

    @staticmethod
    def _pd_cmd(q, kp: float, kd: float) -> LowCmd:
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, float(q[i]), 0.0, 0.0, float(kp), float(kd)
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
            "n_damping_no_state": self._n_damping_no_state,
            "watchdog_trips": self.limiter.n_watchdog,
            "ramp_clips": self.limiter.n_clipped,
        }
