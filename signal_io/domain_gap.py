"""domain_gap.py —— Go2 训练模型/策略 与 Go2 真机之间的域差清单 + 信号级降级注入。

契约：docs/interfaces.md §7。

两部分：
  * GapItem / load_gap_table / format_table —— 可核查的域差清单（数据在 config/domain_gap.json，
    每条带 sim/real 两侧事实、severity、status、缓解措施与证据出处）。
  * SignalDegrader —— **信号层**注入真机才有的劣化：延迟（N 拍队列）/ 丢帧（含突发）/
    噪声 / 偏置 / 量化。只动 LowState 信号，不动物理模型（物理差异见清单里的 open/mitigated 项）。

只依赖 numpy + 标准库 + 本包 msg（禁止 jax / rclpy）。
"""

from __future__ import annotations

import copy
import json
import os
import textwrap
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .msg import IMUState, MotorState, NUM_JOINT, LowState

# 项目根目录 = 本文件(signal_io/domain_gap.py) 往上两层
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_GAP_PATH = os.path.join(ROOT, "config", "domain_gap.json")

VALID_SEVERITY = ("high", "medium", "low")
VALID_STATUS = ("closed", "mitigated", "open")
_REQUIRED = ("id", "category", "sim", "real", "severity", "status", "mitigation")

__all__ = ["GapItem", "load_gap_table", "format_table", "SignalDegrader",
           "load_degrader_config", "load_degrader_profiles", "DEFAULT_GAP_PATH"]


# ================================================================ 域差清单
@dataclass
class GapItem:
    """一条域差。sim = 训练/仿真侧事实，real = 真机侧事实。"""

    id: str
    category: str
    sim: str
    real: str
    severity: str          # "high" | "medium" | "low"
    status: str            # "closed" | "mitigated" | "open"
    mitigation: str
    evidence: str = ""

    def __post_init__(self) -> None:
        if self.severity not in VALID_SEVERITY:
            raise ValueError("GapItem(%s): severity 必须是 %s, 收到 %r"
                             % (self.id, VALID_SEVERITY, self.severity))
        if self.status not in VALID_STATUS:
            raise ValueError("GapItem(%s): status 必须是 %s, 收到 %r"
                             % (self.id, VALID_STATUS, self.status))
        for name in ("id", "category", "sim", "real", "mitigation"):
            if not str(getattr(self, name)).strip():
                raise ValueError("GapItem(%s): 字段 %s 不能为空" % (self.id, name))

    def as_dict(self) -> dict:
        return {"id": self.id, "category": self.category, "sim": self.sim, "real": self.real,
                "severity": self.severity, "status": self.status,
                "mitigation": self.mitigation, "evidence": self.evidence}


def load_gap_table(path: Optional[str] = None) -> List[GapItem]:
    """读 config/domain_gap.json → list[GapItem]（默认路径 = 项目 config/domain_gap.json）。

    文件可以是 {"items": [...]} 或裸列表。缺字段/未知 severity/status/重复 id 都报 ValueError。
    """
    p = os.path.abspath(path) if path else DEFAULT_GAP_PATH
    with open(p, "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    rows = raw.get("items") if isinstance(raw, dict) else raw
    if not isinstance(rows, list) or not rows:
        raise ValueError("%s: 找不到非空的 items 列表" % p)

    items: List[GapItem] = []
    seen = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("%s: items 元素必须是对象, 收到 %r" % (p, type(row).__name__))
        missing = [k for k in _REQUIRED if k not in row]
        if missing:
            raise ValueError("%s: 条目 %r 缺字段 %s" % (p, row.get("id"), missing))
        item = GapItem(
            id=str(row["id"]), category=str(row["category"]),
            sim=str(row["sim"]), real=str(row["real"]),
            severity=str(row["severity"]), status=str(row["status"]),
            mitigation=str(row["mitigation"]), evidence=str(row.get("evidence", "")),
        )
        if item.id in seen:
            raise ValueError("%s: 重复 id %r" % (p, item.id))
        seen.add(item.id)
        items.append(item)
    return items


def _wrap(text: str, width: int) -> List[str]:
    """按宽度折行（保留原换行），给 format_table 用。"""
    if width <= 1:
        return [""]
    out: List[str] = []
    for para in str(text).splitlines() or [""]:
        out.extend(textwrap.wrap(para, width=width) or [""])
    return out


def format_table(items: Sequence[GapItem], width: int = 160) -> str:
    """终端友好的域差表（多行单元格自动换行）+ 汇总行。"""
    items = list(items)
    if not items:
        return "域差清单: 0 项"
    width = max(96, int(width))
    w_id, w_cat, w_sev, w_st = 7, 17, 6, 9
    sep = " | "
    remain = width - (w_id + w_cat + w_sev + w_st) - len(sep) * 5 - 1
    w_change = max(28, int(remain * 0.50))
    w_mit = max(24, remain - w_change)

    head_cells = [("ID", w_id), ("CATEGORY", w_cat), ("SEV", w_sev), ("STATUS", w_st),
                  ("SIM -> REAL", w_change), ("MITIGATION", w_mit)]
    lines: List[str] = []
    header = sep.join(h.ljust(w)[:w] for h, w in head_cells)
    lines.append(header)
    lines.append("-" * len(header))
    for it in items:
        mit = _wrap(it.mitigation, w_mit)
        if it.evidence:
            mit = mit + [""] + _wrap("evidence: " + it.evidence, w_mit)
        cols = [
            _wrap(it.id, w_id),
            _wrap(it.category, w_cat),
            _wrap(it.severity, w_sev),
            _wrap(it.status, w_st),
            _wrap("sim: %s  ||  real: %s" % (it.sim, it.real), w_change),
            mit,
        ]
        nrow = max(len(c) for c in cols)
        for r in range(nrow):
            lines.append(sep.join(((c[r] if r < len(c) else "").ljust(w)[:w])
                                  for c, (_, w) in zip(cols, head_cells)))
        lines.append("-" * len(header))
    counts = {s: sum(1 for i in items if i.status == s) for s in VALID_STATUS}
    sev = {s: sum(1 for i in items if i.severity == s) for s in VALID_SEVERITY}
    lines.append("总计 %d 项 | status: closed=%d mitigated=%d open=%d | severity: high=%d medium=%d low=%d"
                 % (len(items), counts["closed"], counts["mitigated"], counts["open"],
                    sev["high"], sev["medium"], sev["low"]))
    return "\n".join(lines)


# ================================================================ 信号级降级
_DEFAULT_DEGRADER: Dict[str, Any] = {
    "enabled": False,
    "seed": 0,
    "latency_steps": 0,
    "dropout": 0.0,
    "burst": {"prob": 0.0, "len": 0},
    "noise": {"gyro": 0.0, "accel": 0.0, "rpy": 0.0, "joint_q": 0.0, "joint_dq": 0.0, "foot_force": 0.0},
    "bias": {"gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
             "joint_q": 0.0, "joint_dq": 0.0},
    "quantization": {"gyro": 0.0, "accel": 0.0, "rpy": 0.0, "joint_q": 0.0, "joint_dq": 0.0,
                     "foot_force": 1.0},
}

_ALIAS = {
    "latency_frames": "latency_steps", "delay_steps": "latency_steps", "latency": "latency_steps",
    "dropout_rate": "dropout", "drop_rate": "dropout",
    "gyroscope": "gyro", "accelerometer": "accel", "imu_gyro": "gyro", "imu_accel": "accel",
    "q": "joint_q", "dq": "joint_dq", "motor_q": "joint_q", "motor_dq": "joint_dq",
}


def _deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _canon_keys(d: Optional[dict]) -> dict:
    """把别名键折成规范键（浅层；noise/bias/quantization 子字典也折一层）。"""
    if not d:
        return {}
    out: Dict[str, Any] = {}
    for k, v in d.items():
        key = _ALIAS.get(k, k)
        if isinstance(v, dict):
            out[key] = {_ALIAS.get(kk, kk): vv for kk, vv in v.items()}
        else:
            out[key] = v
    return out


def load_degrader_profiles(path: Optional[str] = None) -> Dict[str, dict]:
    """读 config/domain_gap.json 的 profiles 段（不存在则返回空 dict）。"""
    p = os.path.abspath(path) if path else DEFAULT_GAP_PATH
    if not os.path.isfile(p):
        return {}
    with open(p, "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    prof = raw.get("profiles", {}) if isinstance(raw, dict) else {}
    return {k: v for k, v in prof.items() if isinstance(v, dict)}


def load_degrader_config(path: Optional[str] = None) -> dict:
    """读 config/domain_gap.json 的 degrader 段（合并到内置默认值之上）。"""
    p = os.path.abspath(path) if path else DEFAULT_GAP_PATH
    if not os.path.isfile(p):
        return copy.deepcopy(_DEFAULT_DEGRADER)
    with open(p, "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    deg = raw.get("degrader", {}) if isinstance(raw, dict) else {}
    return _deep_merge(_DEFAULT_DEGRADER, _canon_keys(deg))


def _clone_state(st: LowState) -> LowState:
    """LowState 的快速深拷贝（字段与 msg.LowState 一一对应，比 copy.deepcopy 快 ~4x）。

    只在 SignalDegrader 内部用；tests 里有一条 check 与 copy.deepcopy 逐字段比对。
    """
    out = LowState()
    imu = st.imu_state
    out.imu_state = IMUState(quaternion=list(imu.quaternion),
                             gyroscope=list(imu.gyroscope),
                             accelerometer=list(imu.accelerometer),
                             rpy=list(imu.rpy))
    out.motor_state = [MotorState(m.mode, m.q, m.dq, m.ddq, m.tau_est, m.temperature, m.lost)
                       for m in st.motor_state]
    out.foot_force = list(st.foot_force)
    out.tick = st.tick
    out.mode_machine = st.mode_machine
    out.seq = st.seq
    out.sim_time = st.sim_time
    out.crc = st.crc
    return out


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> List[float]:
    """ZYX 约定（与 robot.py::_quat_to_rpy 互逆），返回 wxyz。"""
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return [float(cr * cp * cy + sr * sp * sy),
            float(sr * cp * cy - cr * sp * sy),
            float(cr * sp * cy + sr * cp * sy),
            float(cr * cp * sy - sr * sp * cy)]


class SignalDegrader:
    """在仿真里注入真机才有的信号劣化：延迟 / 丢帧 / 噪声 / 偏置 / 量化。

    调用约定：每拍把**仿真真值** LowState 送进 degrade_state(state, t)：
      * 先深拷贝并在拷贝上注入噪声/偏置/量化（不改输入 state）；
      * 把这一拍压进长度 latency_steps 的延迟队列，输出对应旧值；
      * 若本拍判定丢帧（或还在冷启动、队列未填满），返回 None。

    配置 cfg（dict）可以是：
      * None                      → 读 config/domain_gap.json 的 degrader 段（缺文件则内置默认）；
      * 带 "degrader"/"items" 的整份文件 dict → 取其中 degrader 段；
      * 带 "profile": "real_mild" → 从文件 profiles 段取该档再覆盖其余键；
      * 直接给退化参数 dict       → 与内置默认深度合并。

    退化字段：
      latency_steps    延迟 N 拍（输出第 k-N 拍的状态；前 N 拍返回 None 冷启动）
      dropout          每拍独立丢帧概率
      burst.prob/len   突发丢帧：以 prob 触发、连续丢 len 拍
      noise.*          高斯噪声 std：gyro[rad/s] accel[m/s^2] rpy[rad]
                       joint_q[rad] joint_dq[rad/s] foot_force[N]
      bias.*           常值偏置（gyro/accel/rpy 为 3 向量，joint_q/joint_dq 为标量）
      quantization.*   量化步长（编码器/ADC 分辨率；0 = 不量化）
    """

    def __init__(self, cfg: Optional[dict] = None) -> None:
        self.cfg = self._normalize(cfg)
        self.enabled = bool(self.cfg.get("enabled", False))
        self.seed = int(self.cfg.get("seed", 0))
        self.latency_steps = max(0, int(self.cfg.get("latency_steps", 0)))
        self.dropout = float(np.clip(float(self.cfg.get("dropout", 0.0)), 0.0, 1.0))
        burst = self.cfg.get("burst") or {}
        self.burst_prob = float(np.clip(float(burst.get("prob", 0.0)), 0.0, 1.0))
        self.burst_len = max(0, int(burst.get("len", 0)))
        self.noise = {k: float(v) for k, v in (self.cfg.get("noise") or {}).items()}
        self.quant = {k: float(v) for k, v in (self.cfg.get("quantization") or {}).items()}
        bias = self.cfg.get("bias") or {}

        def _vec3(name: str) -> np.ndarray:
            v = bias.get(name, [0.0, 0.0, 0.0])
            arr = np.asarray(v, dtype=np.float64).reshape(-1)
            if arr.size == 1:
                arr = np.repeat(arr, 3)
            if arr.size != 3:
                raise ValueError("bias.%s 必须是 3 元素" % name)
            return arr

        self.bias = {"gyro": _vec3("gyro"), "accel": _vec3("accel"), "rpy": _vec3("rpy"),
                     "joint_q": float(bias.get("joint_q", 0.0)),
                     "joint_dq": float(bias.get("joint_dq", 0.0))}
        self.reset()

    # ------------------------------------------------------------ 配置
    @staticmethod
    def _normalize(cfg: Optional[dict]) -> dict:
        if cfg is None:
            return load_degrader_config()
        if not isinstance(cfg, dict):
            raise TypeError("SignalDegrader cfg 必须是 dict | None")
        cfg = _canon_keys(cfg)
        profile = cfg.get("profile")
        if profile is not None:
            profs = load_degrader_profiles()
            if profile not in profs:
                raise KeyError("未知退化 profile %r（可选: %s）" % (profile, sorted(profs)))
            base = _deep_merge(_DEFAULT_DEGRADER, _canon_keys(profs[profile]))
            rest = {k: v for k, v in cfg.items() if k != "profile"}
            return _deep_merge(base, rest)
        if "degrader" in cfg:
            return _deep_merge(_DEFAULT_DEGRADER, _canon_keys(cfg["degrader"]))
        return _deep_merge(_DEFAULT_DEGRADER, cfg)

    # ------------------------------------------------------------ 状态
    def reset(self) -> None:
        """清空延迟队列、重播随机种子、计数归零（可复现）。"""
        self._delay: List[LowState] = []
        self._rng = np.random.default_rng(self.seed)
        self._burst_left = 0
        self.n_in = 0
        self.n_out = 0
        self.n_dropped = 0
        self.n_burst_dropped = 0
        self.n_frames_corrupted = 0
        self.t_last = 0.0

    def _should_drop(self) -> bool:
        if self._burst_left > 0:
            self._burst_left -= 1
            self.n_burst_dropped += 1
            return True
        if self.burst_prob > 0.0 and self.burst_len > 0 and self._rng.random() < self.burst_prob:
            self._burst_left = self.burst_len - 1
            self.n_burst_dropped += 1
            return True
        return bool(self._rng.random() < self.dropout)

    @staticmethod
    def _q(v: float, step: float) -> float:
        if step <= 0.0:
            return float(v)
        return float(np.round(float(v) / step) * step)

    # ------------------------------------------------------------ 注入
    def _corrupt(self, st: LowState, rng: np.random.Generator) -> LowState:
        nz = self.noise
        qz = self.quant
        touched = False

        imu = st.imu_state
        gyro = np.asarray(imu.gyroscope, dtype=np.float64) + self.bias["gyro"]
        if nz.get("gyro", 0.0) > 0.0:
            gyro = gyro + rng.normal(0.0, nz["gyro"], size=3)
        gyro = np.array([self._q(v, qz.get("gyro", 0.0)) for v in gyro])
        accel = np.asarray(imu.accelerometer, dtype=np.float64) + self.bias["accel"]
        if nz.get("accel", 0.0) > 0.0:
            accel = accel + rng.normal(0.0, nz["accel"], size=3)
        accel = np.array([self._q(v, qz.get("accel", 0.0)) for v in accel])
        rpy = np.asarray(imu.rpy, dtype=np.float64) + self.bias["rpy"]
        if nz.get("rpy", 0.0) > 0.0:
            rpy = rpy + rng.normal(0.0, nz["rpy"], size=3)
        rpy = np.array([self._q(v, qz.get("rpy", 0.0)) for v in rpy])
        imu.gyroscope = [float(v) for v in gyro]
        imu.accelerometer = [float(v) for v in accel]
        imu.rpy = [float(v) for v in rpy]
        if (nz.get("rpy", 0.0) > 0.0 or qz.get("rpy", 0.0) > 0.0
                or bool(np.any(self.bias["rpy"] != 0.0))):
            imu.quaternion = _quat_from_rpy(rpy[0], rpy[1], rpy[2])
            touched = True

        bq, bd = self.bias["joint_q"], self.bias["joint_dq"]
        nq, nd = nz.get("joint_q", 0.0), nz.get("joint_dq", 0.0)
        sq, sd = qz.get("joint_q", 0.0), qz.get("joint_dq", 0.0)
        if bq or bd or nq or nd or sq or sd:
            for i in range(NUM_JOINT):
                m = st.motor_state[i]
                vq = float(m.q) + bq + (rng.normal(0.0, nq) if nq > 0.0 else 0.0)
                vd = float(m.dq) + bd + (rng.normal(0.0, nd) if nd > 0.0 else 0.0)
                m.q = self._q(vq, sq)
                m.dq = self._q(vd, sd)
            touched = True

        nf = nz.get("foot_force", 0.0)
        sf = qz.get("foot_force", 0.0)
        if nf > 0.0 or sf > 0.0:
            ff = np.asarray(st.foot_force, dtype=np.float64)
            if nf > 0.0:
                ff = ff + rng.normal(0.0, nf, size=ff.shape)
            st.foot_force = [int(round(self._q(float(v), sf))) for v in ff]
            touched = True

        if touched:
            self.n_frames_corrupted += 1
        return st

    def degrade_state(self, state: Optional[LowState], t: float) -> Optional[LowState]:
        """劣化后的 LowState；丢帧（或延迟队列未填满 / 输入为 None）返回 None。

        输入 state 不会被修改（enabled=False 时原样透传，零开销）。
        """
        if not self.enabled:
            return state
        self.t_last = float(t)
        if state is None:
            return None
        self.n_in += 1

        out = self._corrupt(_clone_state(state), self._rng)
        self._delay.append(out)
        delayed: Optional[LowState] = None
        if len(self._delay) > self.latency_steps:
            delayed = self._delay.pop(0)

        if self._should_drop():
            self.n_dropped += 1
            return None
        if delayed is None:
            return None
        self.n_out += 1
        return delayed

    # ------------------------------------------------------------ 统计
    def stats(self) -> dict:
        return {
            "enabled": self.enabled,
            "seed": self.seed,
            "latency_steps": self.latency_steps,
            "dropout_cfg": self.dropout,
            "n_in": self.n_in,
            "n_out": self.n_out,
            "n_dropped": self.n_dropped,
            "n_burst_dropped": self.n_burst_dropped,
            "dropout_measured": (self.n_dropped / self.n_in) if self.n_in else 0.0,
            "n_frames_corrupted": self.n_frames_corrupted,
            "delay_buffer": len(self._delay),
            "t_last": self.t_last,
        }
