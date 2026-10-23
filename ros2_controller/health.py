"""分级健康监控（接口契约 §4）。

统一信息输入层：每路信息带 {value, stamp, valid, sigma}，聚合成四级健康状态：

    L0 正常  -> 全部在容差内
    L1 降级  -> 非致命缺失（可选通道丢失 / 单帧抖动），任务降级
    L2 保护  -> 关键传感器不可信（看门狗级超时 / IMU 无效 / sigma 超限）
    L3 急停  -> 致命（LowState 完全没来 / 姿态超限 / 关节角 NaN），**锁存**

规则（全部可由 cfg 覆盖，默认值见 DEFAULT_HEALTH_CFG）：
  * L3: LowState 年龄 > estop_ms；或 |roll|/|pitch| > max_tilt_deg；或关节角 NaN
  * L2: LowState 年龄 > protect_ms；或 IMU 无效 / NaN；或任一通道 sigma > sigma_max
  * L1: 可选通道无效；或 LowState 年龄 > stale_ms（单帧抖动）；或 sigma > sigma_warn；
        或 confidence_channel 的值 < confidence_min
  * L0: 以上都不触发

迟滞：
  * 升级立即生效；
  * 降级必须连续 recover_s 满足**低一级**的条件才回落一级（逐级回落）。
L3 锁存：一旦进入 L3，除非调用 reset()，check() 永远返回 L3。

cfg 允许两种形态：
  HealthMonitor(cfg=loaded_config)     # 自动取 cfg["health"] 子段
  HealthMonitor(cfg={"estop_ms": ...}) # 直接就是 health 段
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import numpy as np

from signal_io.msg import NUM_JOINT, LowState

__all__ = ["Level", "Channel", "HealthState", "HealthMonitor", "DEFAULT_HEALTH_CFG"]


class Level(IntEnum):
    """健康等级。数值越大越严重（可直接用 max() 聚合）。"""

    L0_NORMAL = 0    # 全部信息可用
    L1_DEGRADED = 1  # 非致命缺失：任务降级（例如速度指令丢失 -> 原地站立）
    L2_PROTECT = 2   # 关键传感器不可信：进入保护（阻尼 + 缓慢下蹲）
    L3_ESTOP = 3     # 致命：卸力并锁存，需显式 reset 才恢复


def _level_name(level: Level) -> str:
    return "L%d" % int(level)


# 出厂默认阈值。config/go2_sim.json 的 "health" 段可逐项覆盖。
DEFAULT_HEALTH_CFG: Dict[str, object] = {
    "estop_ms": 200.0,            # LowState 完全没来多久 -> L3
    "protect_ms": 60.0,           # 看门狗级超时 -> L2
    "stale_ms": 20.0,             # 单帧抖动 -> L1
    "recover_s": 1.0,             # 降级迟滞：连续满足低一级条件多久才回落
    "max_tilt_deg": 60.0,         # |roll| / |pitch| 超过 -> L3
    "sigma_max": 0.5,             # 任一通道 sigma 超过 -> L2
    "sigma_warn": 0.2,            # 任一通道 sigma 超过 -> L1
    "optional_channels": ("cmd_vel", "height_scan"),  # 缺失只降 L1 的通道
    "confidence_channel": "linvel_conf",              # 估计速度置信度通道名
    "confidence_min": 0.3,                            # 置信度低于此值 -> L1
    # 真机 LowState 自带的电机健康信号（仿真里恒 0，不影响仿真）
    "check_motor_health": True,   # motor_state[i].lost / temperature
    "temp_warn_c": 70.0,          # 电机温度 >= 此值 -> L1
    "temp_max_c": 85.0,           # 电机温度 >= 此值 -> L2
    "check_bit_flag": True,       # LowState.bit_flag: bit0 指令超时 / bit7 电机超时 -> L2
}


@dataclass
class Channel:
    """一路信息。stamp 与 HealthMonitor.check() 的 now 同一时基。"""

    name: str
    value: Optional[np.ndarray]
    stamp: float          # 该通道最后一次有效更新的时间
    valid: bool
    sigma: float = 0.0    # 1sigma 不确定度；无估计填 0.0
    age_s: float = 0.0    # 由 check() 填
    reason: str = ""


@dataclass
class HealthState:
    level: Level
    reasons: Tuple[str, ...]
    channels: Dict[str, Channel]
    entered_at: float
    n_transitions: int

    @property
    def level_name(self) -> str:
        return _level_name(self.level)

    def channel(self, name: str) -> Optional[Channel]:
        return self.channels.get(name)


def _as_float(value) -> Optional[np.ndarray]:
    """把任意标量/序列转成 float64 数组；转不了返回 None。"""
    try:
        arr = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    return arr


def _finite(arr: Optional[np.ndarray]) -> bool:
    return arr is not None and bool(np.all(np.isfinite(arr)))


class HealthMonitor:
    """把 LowState + 附加通道聚合成 HealthState。无副作用、可复现（时基由调用方给）。"""

    def __init__(self, cfg: Optional[dict] = None):
        self.cfg: Dict[str, object] = dict(DEFAULT_HEALTH_CFG)
        if cfg:
            sub = cfg.get("health") if isinstance(cfg, dict) and "health" in cfg else cfg
            if isinstance(sub, dict):
                self.cfg.update(sub)
        # 秒化
        self.estop_s = float(self.cfg["estop_ms"]) / 1000.0
        self.protect_s = float(self.cfg["protect_ms"]) / 1000.0
        self.stale_s = float(self.cfg["stale_ms"]) / 1000.0
        self.recover_s = float(self.cfg["recover_s"])
        self.max_tilt_deg = float(self.cfg["max_tilt_deg"])
        self.sigma_max = float(self.cfg["sigma_max"])
        self.sigma_warn = float(self.cfg["sigma_warn"])
        self.optional_channels = frozenset(self.cfg.get("optional_channels") or ())
        self.confidence_channel = str(self.cfg.get("confidence_channel") or "")
        self.confidence_min = float(self.cfg.get("confidence_min", 0.0))
        self.check_motor_health = bool(self.cfg.get("check_motor_health", True))
        self.temp_warn_c = float(self.cfg.get("temp_warn_c", 70.0))
        self.temp_max_c = float(self.cfg.get("temp_max_c", 85.0))
        self.check_bit_flag = bool(self.cfg.get("check_bit_flag", True))

        self._clear(0.0)
        self._initialized = False   # 首次 check() 自动把时基对齐到 now

    # ---------------------------------------------------------------- 生命周期
    def _clear(self, now: float) -> None:
        now = float(now)
        self._level: Level = Level.L0_NORMAL
        self._entered_at = now
        self._n_transitions = 0
        self._latched = False
        self._pending: Optional[Level] = None
        self._pending_since = now
        self._state_stamp = now
        self._ever_state = False
        self._last_check = now
        self._now = now
        self._n_checks = 0
        self._dwell: Dict[Level, float] = {lvl: 0.0 for lvl in Level}
        self._reasons: Tuple[str, ...] = ()
        self._channels: Dict[str, Channel] = {}

    def reset(self, now: float = 0.0) -> None:
        """清掉锁存与全部统计，把时基设到 now。"""
        self._clear(now)
        self._initialized = True

    # ---------------------------------------------------------------- 单通道判定
    @staticmethod
    def _fill_age(ch: Channel, now: float) -> None:
        # stamp <= 0 视为"时基未知"，不做 age 判定（否则默认 stamp=0 会立刻超时）
        if ch.stamp is not None and float(ch.stamp) > 0.0:
            ch.age_s = max(0.0, float(now) - float(ch.stamp))
        else:
            ch.age_s = 0.0

    def _channel_hit(self, ch: Channel) -> Tuple[Level, str]:
        name = ch.name
        optional = name in self.optional_channels
        if (not ch.valid) or ch.value is None:
            return (Level.L1_DEGRADED, "%s:invalid" % name) if optional else \
                   (Level.L2_PROTECT, "%s:invalid" % name)
        arr = _as_float(ch.value)
        if not _finite(arr):
            return Level.L2_PROTECT, "%s:nan" % name
        sigma = float(ch.sigma or 0.0)
        if sigma > self.sigma_max:
            return Level.L2_PROTECT, "%s:sigma=%.3f>%.3f" % (name, sigma, self.sigma_max)
        if sigma > self.sigma_warn:
            return Level.L1_DEGRADED, "%s:sigma_warn=%.3f" % (name, sigma)
        if ch.age_s > self.estop_s:
            if optional:
                return Level.L1_DEGRADED, "%s:timeout=%.1fms" % (name, ch.age_s * 1e3)
            return Level.L3_ESTOP, "%s:timeout=%.1fms" % (name, ch.age_s * 1e3)
        if ch.age_s > self.protect_s:
            if optional:
                return Level.L1_DEGRADED, "%s:stale=%.1fms" % (name, ch.age_s * 1e3)
            return Level.L2_PROTECT, "%s:stale=%.1fms" % (name, ch.age_s * 1e3)
        return Level.L0_NORMAL, ""

    # ---------------------------------------------------------------- 主入口
    def check(self, state: Optional[LowState], now: float, *,
              extra: Optional[Dict[str, Channel]] = None) -> HealthState:
        now = float(now)
        if not self._initialized:
            self._clear(now)
            self._initialized = True

        # 上一段区间的等级停留时间
        gap = now - self._last_check
        if gap > 0.0:
            self._dwell[self._level] += gap
        self._now = now
        self._n_checks += 1
        self._last_check = now

        hits: List[Tuple[Level, str]] = []
        channels: Dict[str, Channel] = {}

        # --- 附加通道（先填 age，再做判定；"state" 由本函数自己维护，不允许被覆盖）
        extra = extra or {}
        for name in sorted(extra):
            ch = extra[name]
            if name == "state":
                continue
            self._fill_age(ch, now)
            channels[name] = ch
            lvl, why = self._channel_hit(ch)
            if lvl != Level.L0_NORMAL:
                ch.reason = why
                hits.append((lvl, why))
            else:
                ch.reason = ""

        # --- 速度置信度（L1）
        conf = channels.get(self.confidence_channel)
        if conf is not None and conf.valid and conf.value is not None:
            carr = _as_float(conf.value)
            if _finite(carr) and carr.size > 0 and float(carr.reshape(-1)[0]) < self.confidence_min:
                hits.append((Level.L1_DEGRADED,
                             "%s:low_conf=%.3f<%.3f" % (conf.name, float(carr.reshape(-1)[0]),
                                                        self.confidence_min)))

        # --- LowState 存活时间
        if state is not None:
            self._state_stamp = now
            self._ever_state = True
        state_age = max(0.0, now - self._state_stamp)
        if state_age > self.estop_s:
            hits.append((Level.L3_ESTOP, "state_timeout=%.1fms" % (state_age * 1e3)))
        elif state_age > self.protect_s:
            hits.append((Level.L2_PROTECT, "state_watchdog=%.1fms" % (state_age * 1e3)))
        elif state_age > self.stale_s:
            hits.append((Level.L1_DEGRADED, "state_jitter=%.1fms" % (state_age * 1e3)))

        # --- LowState 内容
        q12: Optional[np.ndarray] = None
        state_ok = state is not None
        if state is not None:
            q12 = _as_float(state.q12())
            if q12 is None or q12.size != NUM_JOINT or not _finite(q12):
                hits.append((Level.L3_ESTOP, "joint_nan"))
                state_ok = False
            rpy = getattr(getattr(state, "imu_state", None), "rpy", None)
            rpy_arr = _as_float(rpy)
            rpy_arr = np.zeros(0) if rpy_arr is None else rpy_arr.reshape(-1)
            if rpy_arr.size < 3:
                hits.append((Level.L2_PROTECT, "imu_invalid"))
            elif not _finite(rpy_arr[:2]):
                hits.append((Level.L2_PROTECT, "imu_nan"))
            else:
                tilt_deg = float(np.degrees(np.max(np.abs(rpy_arr[:2]))))
                if tilt_deg > self.max_tilt_deg:
                    hits.append((Level.L3_ESTOP, "tilt=%.1fdeg>%.1f" % (tilt_deg, self.max_tilt_deg)))

            # --- 真机才有的电机健康信号（仿真里 lost=0 / temperature=0 / bit_flag=0）
            if self.check_motor_health:
                lost = [i for i in range(NUM_JOINT)
                        if int(getattr(state.motor_state[i], "lost", 0))]
                if lost:
                    hits.append((Level.L2_PROTECT, "motor_lost=%s" % lost[:6]))
                temps = [int(getattr(state.motor_state[i], "temperature", 0))
                         for i in range(NUM_JOINT)]
                tmax = max(temps) if temps else 0
                if tmax >= self.temp_max_c:
                    hits.append((Level.L2_PROTECT,
                                 "motor_temp=%dC>=%.0f" % (tmax, self.temp_max_c)))
                elif tmax >= self.temp_warn_c:
                    hits.append((Level.L1_DEGRADED,
                                 "motor_temp=%dC>=%.0f" % (tmax, self.temp_warn_c)))
            if self.check_bit_flag:
                bf = int(getattr(state, "bit_flag", 0)) & 0xFF
                if bf & 0x01:
                    # 机器人自己报"操作指令超时"：说明我们的 lowcmd 没按预期到达
                    hits.append((Level.L2_PROTECT, "robot_cmd_timeout(bit_flag=0x%02X)" % bf))
                if bf & 0x80:
                    hits.append((Level.L2_PROTECT, "motor_timeout(bit_flag=0x%02X)" % bf))
        channels["state"] = Channel(name="state", value=q12, stamp=self._state_stamp,
                                    valid=state_ok, sigma=0.0, age_s=state_age)

        # --- 聚合：取最严重一级；reasons 按严重度降序
        raw = Level.L0_NORMAL
        for lvl, _why in hits:
            if lvl > raw:
                raw = lvl
        hits.sort(key=lambda item: -int(item[0]))
        reasons = [why for _lvl, why in hits]

        # --- L3 锁存
        if self._latched or raw == Level.L3_ESTOP:
            if raw == Level.L3_ESTOP:
                self._latched = True
            if self._level != Level.L3_ESTOP:
                self._level = Level.L3_ESTOP
                self._entered_at = now
                self._n_transitions += 1
            self._pending = None
            if not reasons or raw != Level.L3_ESTOP:
                reasons = list(reasons) + ["estop_latched"]
            self._reasons = tuple(reasons)
            self._channels = channels
            return HealthState(Level.L3_ESTOP, self._reasons, channels, self._entered_at,
                               self._n_transitions)

        # --- 迟滞：升级立即；降级逐级、且需连续 recover_s
        if raw > self._level:
            self._level = raw
            self._entered_at = now
            self._n_transitions += 1
            self._pending = None
        elif raw < self._level:
            if self._pending != raw:
                self._pending = raw
                self._pending_since = now
            elif now - self._pending_since >= self.recover_s:
                self._level = Level(int(self._level) - 1)
                self._entered_at = now
                self._n_transitions += 1
                if self._level > raw:
                    self._pending = raw
                    self._pending_since = now
                else:
                    self._pending = None
        else:
            self._pending = None

        if self._pending is not None and raw < self._level:
            reasons = reasons + ["hysteresis:hold_%s_wait_%.2fs" % (_level_name(self._pending),
                                                                   self.recover_s)]
        self._reasons = tuple(reasons)
        self._channels = channels
        return HealthState(self._level, self._reasons, channels, self._entered_at,
                           self._n_transitions)

    # ---------------------------------------------------------------- 观测
    @property
    def level(self) -> Level:
        return self._level

    @property
    def latched(self) -> bool:
        return self._latched

    def stats(self) -> dict:
        state_ch = self._channels.get("state")
        return {
            "level": int(self._level),
            "level_name": _level_name(self._level),
            "reasons": list(self._reasons),
            "n_transitions": self._n_transitions,
            "entered_at": self._entered_at,
            "now": self._now,
            "time_in_level_s": max(0.0, self._now - self._entered_at),
            "dwell_s": {"%s" % _level_name(lvl): round(v, 6) for lvl, v in self._dwell.items()},
            "dwell_total_s": round(sum(self._dwell.values()), 6),
            "l3_latched": self._latched,
            "n_checks": self._n_checks,
            "ever_got_state": self._ever_state,
            "state_valid": bool(state_ch.valid) if state_ch is not None else False,
            "state_age_s": float(state_ch.age_s) if state_ch is not None else None,
            "channels": sorted(self._channels),
            "optional_channels": sorted(self.optional_channels),
            "cfg": {k: (list(v) if isinstance(v, (tuple, list)) else v) for k, v in self.cfg.items()},
        }
