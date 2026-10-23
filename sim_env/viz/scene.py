"""sim_env/viz/scene.py —— MuJoCo 3D 叠加层（SceneOverlay）。

契约: docs/viz.md。所有数据只来自 VizFrame（viz/frame.py），
本模块**不**回头去掏 sim/controller 的字段，避免接口漂移。

这一层是整个观察脚本最值钱的部分: 把策略"眼里"的 35 个高度采样点画进 3D 场景,
用户能直接看出策略看到的地形长什么样。

实现要点
--------
* **每帧重画、不累积**: draw() 首次在某一个 scn 上绘制时把当时的 scn.ngeom 记为该
  scene 的基址 base, 之后每次都先 scn.ngeom = base 再从 base 开始写; 永远不 append 到
  上一帧后面。基址是**按 scene 分别记的**（scripts/view_policy.py 会轮流往
  renderer.scene 和 viewer.user_scn 里画, 两者基址不同, 必须分开记）。
  若外部把 scene 清空（ngeom 比上一帧少）会自动重新对齐基址。
  **本模块假定每个 scene 的 base 以上区间由自己独占**; 其它叠加层请用各自的 MjvScene。
* **ngeom 自己维护**: 写完把 scn.ngeom 设成实际写入个数; 每次写入前都查
  scn.maxgeom, 容量不足时按「高度采样点 > 足端 > 箭头 > 拖尾」优先级优雅退化,
  丢了多少记在 stats() 里（*_dropped / drawn）。
* **性能**: 采样点颜色走 65 级预计算 LUT 并整批向量化; 每帧小数组全部预分配;
  拖尾默认保留 trail_n=400 个位置, 但按 geom_budget 用均匀 stride 抽成不超过
  geom_budget - 已用 个球（既保住拖尾长度观感, 又把单帧 geom 数钉死在预算内）。
  实测单帧耗时见 stats() 的 last_ms / mean_ms。

颜色语义（与 obs.height_scan_feature 同定义: feature = clip(z_base - h_ground - 0.28, ±0.35) * 5）
    height_scan < 0  ->  地面比标称更高（上坡） ->  暖色: 白 -> 橙 -> 红, 越负越红
    height_scan > 0  ->  地面比标称更低（下陷） ->  冷色: 白 -> 青 -> 蓝, 越正越蓝
    height_scan ≈ 0  ->  灰白小点（平地 / 标称间隙）, 半径 ∝ |height_scan|
"""

from __future__ import annotations

import time
from collections import deque
from typing import TYPE_CHECKING, Dict, Optional

import numpy as np

import mujoco

if TYPE_CHECKING:  # pragma: no cover - 仅类型标注
    from .frame import VizFrame

__all__ = ["SceneOverlay", "SCAN_N", "NUM_FEET", "SCAN_COLOR_LEGEND"]

SCAN_N = 35
NUM_FEET = 4
SCAN_COLOR_LEGEND = (
    "height_scan<0 地面更高/上坡→暖色(白→橙→红), >0 下陷→冷色(白→青→蓝), "
    "≈0 灰白点; 球半径 ∝ |height_scan|; 绿球=足端接触, 蓝箭头=v_est, "
    "橙箭头=v_truth, 品红箭头=cmd(+x)"
)

_GEOM_SPHERE = int(mujoco.mjtGeom.mjGEOM_SPHERE)
_GEOM_ARROW1 = int(mujoco.mjtGeom.mjGEOM_ARROW1)
_CAM_FREE = int(mujoco.mjtCamera.mjCAMERA_FREE)
_CAM_TRACKING = int(mujoco.mjtCamera.mjCAMERA_TRACKING)
_OBJ_UNKNOWN = int(mujoco.mjtObj.mjOBJ_UNKNOWN)

_DEFAULTS: Dict[str, object] = {
    # 35 个高度采样点
    "scan_on": True,
    "scan_radius_min": 0.010,
    "scan_radius_max": 0.055,
    "scan_color_range": 1.75,   # = clip_m(0.35) * scale(5.0)，特征满量程
    "scan_lut_n": 65,
    # 足端接触
    "feet_on": True,
    "foot_force_threshold": 1.0,   # N
    "foot_contact_radius": 0.035,
    "foot_free_radius": 0.012,
    # 箭头（速度 / 指令）
    "arrows_on": True,
    "vel_scale": 0.5,              # m/(m/s): 1 m/s -> 0.5 m 长箭头
    "arrow_width": 0.012,
    "arrow_min_len": 0.02,         # 太短就不画（degenerate 会让 mjv_connector 出 NaN）
    "cmd_speed": 0.5,              # 契约: 世界系 +x, 0.5 m/s
    "truth_offset": (0.0, 0.35, 0.0),   # 真值箭头挪到旁边对比
    "cmd_offset": (0.0, -0.35, 0.0),
    # 轨迹拖尾
    "trail_on": True,
    "trail_n": 400,                # 保留多少个基座历史位置
    "trail_radius": 0.012,
    "trail_alpha_min": 0.10,
    "trail_alpha_max": 0.70,
    # 总预算 / 相机
    "geom_budget": 320,            # 本叠加层每帧最多占多少 geom（含基址之上所有）
    "camera_mode": "follow",       # follow | top | free | none
    "camera_distance": 2.2,
    "camera_azimuth": 120.0,
    "camera_elevation": -20.0,
}


def _build_scan_lut(n: int) -> np.ndarray:
    """(n,4) float32: 索引 0 = 最负(最红), 中间 = 灰白, 末尾 = 最正(最蓝)。"""
    t = np.linspace(-1.0, 1.0, max(int(n), 3))
    neutral = np.array([0.92, 0.92, 0.92], dtype=np.float64)
    warm = np.array([1.00, 0.15, 0.05], dtype=np.float64)
    cool = np.array([0.10, 0.40, 1.00], dtype=np.float64)
    lut = np.empty((t.size, 4), dtype=np.float32)
    for k, v in enumerate(t):
        c = neutral + ((warm - neutral) * (-v) if v < 0.0 else (cool - neutral) * v)
        lut[k] = (c[0], c[1], c[2], 0.35 + 0.60 * abs(v))
    return lut


def _find_base_body(model) -> int:
    """躯干 body id: 优先找 free joint 的 body, 退化用名字。找不到返回 -1。"""
    if model is None:
        return -1
    try:
        free_type = int(mujoco.mjtJoint.mjJNT_FREE)
        for j in range(int(model.njnt)):
            if int(model.jnt_type[j]) == free_type:
                return int(model.jnt_bodyid[j])
        for name in ("base", "base_link", "trunk", "torso", "floating_base"):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                return int(bid)
    except Exception:  # noqa: BLE001 - 相机是可选功能，不能因此炸掉 draw
        return -1
    return -1


class SceneOverlay:
    """把 VizFrame 里的诊断量画成 MjvScene 里的 geom。"""

    def __init__(self, model, data, cfg: Optional[dict] = None):
        self.model = model
        self.data = data
        self.cfg: Dict[str, object] = dict(_DEFAULTS)
        if cfg:
            self.cfg.update(cfg)
        self._base_body = _find_base_body(model)
        self._lut = _build_scan_lut(int(self.cfg["scan_lut_n"]))
        # 每帧复用的 scratch（避免 draw() 里 new 数组）
        self._pos = np.zeros(3, dtype=np.float64)
        self._sz = np.zeros(3, dtype=np.float64)
        self._mat = np.eye(3, dtype=np.float64).reshape(-1)
        self._rgba = np.zeros(4, dtype=np.float32)
        # 拖尾颜色 LUT（按新旧渐变）
        self._trail_lut = np.zeros((33, 4), dtype=np.float32)
        for k in range(33):
            f = k / 32.0
            a = float(self.cfg["trail_alpha_min"]) + (
                float(self.cfg["trail_alpha_max"]) - float(self.cfg["trail_alpha_min"])) * f
            self._trail_lut[k] = (0.15, 0.75, 1.00, a)
        self._cmd_v = np.zeros(3, dtype=np.float64)
        self._truth_off = np.asarray(self.cfg["truth_offset"], dtype=np.float64).reshape(3)
        self._cmd_off = np.asarray(self.cfg["cmd_offset"], dtype=np.float64).reshape(3)
        self._times = deque(maxlen=240)
        self._counts: Dict[str, int] = {}
        self._base_n: Optional[int] = None
        self._maxgeom_last = 0
        # 每个 scene 一份基址状态: id(scn) -> [scn, base, last_ngeom]
        # （存 scn 强引用, 保证 id 不会在持有期间被回收复用）
        self._scene_state: Dict[int, list] = {}
        self.reset()

    # ------------------------------------------------------------------ 生命周期
    def reset(self) -> None:
        """清空拖尾/统计（换场景、重开 episode 时调用）；按 scene 记的基址保留。"""
        self._trail: list = []
        self._last_t: Optional[float] = None
        self._frames = 0
        self._last_ms = 0.0
        self._times.clear()
        self._base_n = None
        self._maxgeom_last = 0
        # 注意: 不清 _scene_state —— scene 对象跨 episode 复用, 基址必须记住;
        # 若换了 scene 对象, 下面的 entry[0] is not scn 会自然重新对齐。
        self._counts = self._empty_counts()

    @staticmethod
    def _empty_counts() -> Dict[str, int]:
        return {
            "drawn": 0, "scan_drawn": 0, "scan_dropped": 0,
            "feet_drawn": 0, "feet_dropped": 0,
            "arrows_drawn": 0, "arrows_dropped": 0,
            "trail_drawn": 0, "trail_dropped": 0,
        }

    # ------------------------------------------------------------------ 主绘制
    def draw(self, scn, frame) -> None:  # noqa: C901 - 线性流程，拆开反而难读
        """往 scn 里重画本帧的叠加 geom。

        scn.ngeom 由本方法全权维护: 先回到自己的基址 base，再顺序写入，
        最后把 ngeom 设成 base + 实际写入数。任何情况下都 <= scn.maxgeom。
        """
        t0 = time.perf_counter()
        counts = self._empty_counts()
        self._frames += 1

        if scn is None:
            self._finish(t0, counts, 0)
            return

        maxgeom = int(scn.maxgeom)
        self._maxgeom_last = maxgeom
        n0 = int(scn.ngeom)
        # 每个 scene 一份状态 [scn, base, last_ngeom, last_frame]；多于 8 个 scene 时
        # 清掉 30 帧没露面的（view_policy 只会用 2 个，纯粹防反复重建 scene 造成引用泄漏）
        if len(self._scene_state) > 8:
            for k in [k for k, e in self._scene_state.items()
                      if self._frames - e[3] > 30]:
                del self._scene_state[k]
        entry = self._scene_state.get(id(scn))
        if entry is None or entry[0] is not scn:
            # 这个 scene 第一次画 -> 以当前 ngeom（别人的 geom 数）为基址
            entry = [scn, n0, n0, self._frames]
            self._scene_state[id(scn)] = entry
        elif n0 < entry[2] and n0 != entry[1]:
            # scene 被外部清空/重建 -> 重新对齐基址
            entry[1] = n0
        entry[3] = self._frames
        base = int(entry[1])
        self._base_n = base

        if frame is None or maxgeom <= 0:
            # 没有帧或没有容量: 清掉自己上一帧的 geom，但不碰 base 以下的东西
            scn.ngeom = base
            counts["scan_dropped"] = SCAN_N
            counts["feet_dropped"] = NUM_FEET
            counts["arrows_dropped"] = 0
            counts["trail_dropped"] = len(self._trail)
            entry[2] = base
            self._finish(t0, counts, base)
            return

        limit = min(maxgeom, base + int(self.cfg["geom_budget"]))

        self._push_trail(frame)
        i = base
        i = self._draw_scan(scn, frame, i, limit, counts)
        i = self._draw_feet(scn, frame, i, limit, counts)
        i = self._draw_arrows(scn, frame, i, limit, counts)
        i = self._draw_trail(scn, frame, i, limit, counts)
        scn.ngeom = i
        counts["drawn"] = i - base
        if counts["trail_dropped"] or counts["scan_dropped"] or counts["feet_dropped"]:
            counts["degraded"] = 1
        entry[2] = i
        self._finish(t0, counts, i)

    # ------------------------------------------------------------------ 35 个高度采样点
    def _draw_scan(self, scn, frame, i: int, limit: int, counts: Dict[str, int]) -> int:
        if not self.cfg["scan_on"]:
            counts["scan_disabled"] = 1
            return i
        xy_w = getattr(frame, "scan_xy_world", None)
        h = getattr(frame, "height_scan", None)
        if xy_w is None or h is None:
            counts["scan_dropped"] = 0
            return i
        xy_w = np.asarray(xy_w, dtype=np.float64)
        if xy_w.ndim != 2 or xy_w.shape[1] < 3:
            counts["scan_dropped"] = 0
            return i
        h = np.asarray(h, dtype=np.float64).reshape(-1)
        # v6: 不再钉死 35 点 —— 大扫描档 17x11=187 也全画 (geom_budget 由调用方放大)
        n = min(int(xy_w.shape[0]), int(h.size))
        avail = max(0, limit - i)
        drawn = min(n, avail)
        counts["scan_dropped"] = n - drawn
        if drawn <= 0:
            return i

        rng = max(float(self.cfg["scan_color_range"]), 1e-6)
        lut_n = self._lut.shape[0]
        vals = np.clip(h[:drawn] / rng, -1.0, 1.0)
        idx = np.rint((vals + 1.0) * 0.5 * (lut_n - 1)).astype(np.intp)
        rgba = self._lut[idx]
        r0 = float(self.cfg["scan_radius_min"])
        r1 = float(self.cfg["scan_radius_max"])
        radii = r0 + (r1 - r0) * np.abs(vals)
        pts = xy_w[:drawn]
        finite = np.isfinite(pts).all(axis=1)

        pos = self._pos
        sz = self._sz
        for k in range(drawn):
            if not finite[k]:
                counts["scan_dropped"] += 1
                continue
            x = pts[k]
            pos[0] = x[0]
            pos[1] = x[1]
            pos[2] = x[2]
            sz[0] = radii[k]
            sz[1] = 0.0
            sz[2] = 0.0
            g = scn.geoms[i]
            mujoco.mjv_initGeom(g, _GEOM_SPHERE, sz, pos, self._mat, rgba[k])
            g.objtype = _OBJ_UNKNOWN
            i += 1
            counts["scan_drawn"] += 1
        return i

    # ------------------------------------------------------------------ 4 个足端
    def _draw_feet(self, scn, frame, i: int, limit: int, counts: Dict[str, int]) -> int:
        if not self.cfg["feet_on"]:
            counts["feet_disabled"] = 1
            return i
        fp = getattr(frame, "foot_pos_world", None)
        if fp is None:
            counts["feet_dropped"] = NUM_FEET
            return i
        fp = np.asarray(fp, dtype=np.float64)
        if fp.ndim != 2 or fp.shape[1] < 3:
            counts["feet_dropped"] = NUM_FEET
            return i
        ff = getattr(frame, "foot_force", None)
        ff = np.zeros(NUM_FEET) if ff is None else np.asarray(ff, dtype=np.float64).reshape(-1)
        n = min(int(fp.shape[0]), int(ff.size), NUM_FEET)
        avail = max(0, limit - i)
        drawn = min(n, avail)
        counts["feet_dropped"] = n - drawn
        if drawn <= 0:
            return i
        thr = float(self.cfg["foot_force_threshold"])
        r_on = float(self.cfg["foot_contact_radius"])
        r_off = float(self.cfg["foot_free_radius"])
        pos = self._pos
        sz = self._sz
        rgba = self._rgba
        for k in range(drawn):
            x = fp[k]
            if not np.isfinite(x).all():
                counts["feet_dropped"] += 1
                continue
            contact = bool(ff[k] > thr)
            pos[0] = x[0]
            pos[1] = x[1]
            pos[2] = x[2]
            if contact:
                sz[0] = r_on
                rgba[:] = (0.10, 1.00, 0.20, 0.95)
            else:
                sz[0] = r_off
                rgba[:] = (0.55, 0.55, 0.55, 0.35)
            sz[1] = 0.0
            sz[2] = 0.0
            g = scn.geoms[i]
            mujoco.mjv_initGeom(g, _GEOM_SPHERE, sz, pos, self._mat, rgba)
            g.objtype = _OBJ_UNKNOWN
            i += 1
            counts["feet_drawn"] += 1
        return i

    # ------------------------------------------------------------------ 速度 / 指令箭头
    def _draw_arrows(self, scn, frame, i: int, limit: int, counts: Dict[str, int]) -> int:
        if not self.cfg["arrows_on"]:
            counts["arrows_disabled"] = 1
            return i
        base_pos = getattr(frame, "base_pos", None)
        if base_pos is None:
            counts["arrows_dropped"] = 3
            return i
        base_pos = np.asarray(base_pos, dtype=np.float64).reshape(-1)
        if base_pos.size < 3:
            counts["arrows_dropped"] = 3
            return i

        scale = float(self.cfg["vel_scale"])
        min_len = float(self.cfg["arrow_min_len"])
        w = float(self.cfg["arrow_width"])
        origin = np.array([base_pos[0], base_pos[1], base_pos[2]], dtype=np.float64)

        specs = []  # (起点, 速度向量, 颜色, 标签)
        v_est = getattr(frame, "base_lin_vel_world", None)
        if v_est is not None:
            v = np.asarray(v_est, dtype=np.float64).reshape(-1)
            if v.size >= 3 and np.isfinite(v[:3]).all() and float(np.linalg.norm(v[:3])) * scale >= min_len:
                specs.append((origin, v[:3].copy(), (0.15, 0.45, 1.00, 1.0), "v_est"))
        v_truth = getattr(frame, "base_lin_vel_truth", None)
        if v_truth is not None:
            v = np.asarray(v_truth, dtype=np.float64).reshape(-1)
            if v.size >= 3 and np.isfinite(v[:3]).all() and float(np.linalg.norm(v[:3])) * scale >= min_len:
                specs.append((origin + self._truth_off, v[:3].copy(),
                              (1.00, 0.55, 0.05, 1.0), "v_truth"))
        extra = getattr(frame, "extra", None) or {}
        cvx = float(extra.get("cmd_vx", self.cfg["cmd_speed"]))
        cvy = float(extra.get("cmd_vy", 0.0))
        self._cmd_v[0] = cvx
        self._cmd_v[1] = cvy
        self._cmd_v[2] = 0.0
        if np.isfinite(self._cmd_v).all() and float(np.linalg.norm(self._cmd_v)) * scale >= min_len:
            specs.append((origin + self._cmd_off, self._cmd_v.copy(),
                          (1.00, 0.15, 0.85, 1.0), "cmd"))

        avail = max(0, limit - i)
        drawn = min(len(specs), avail)
        counts["arrows_dropped"] = len(specs) - drawn
        if drawn <= 0:
            return i

        pos = self._pos
        sz = self._sz
        rgba = self._rgba
        p1 = np.empty(3, dtype=np.float64)
        for k in range(drawn):
            origin_k, v, col, label = specs[k]
            p1[0] = origin_k[0] + v[0] * scale
            p1[1] = origin_k[1] + v[1] * scale
            p1[2] = origin_k[2] + v[2] * scale
            pos[0] = origin_k[0]
            pos[1] = origin_k[1]
            pos[2] = origin_k[2]
            sz[0] = w
            sz[1] = w
            sz[2] = 0.0
            rgba[:] = col
            g = scn.geoms[i]
            mujoco.mjv_initGeom(g, _GEOM_ARROW1, sz, pos, self._mat, rgba)
            mujoco.mjv_connector(g, _GEOM_ARROW1, w, origin_k, p1)
            g.objtype = _OBJ_UNKNOWN
            try:
                g.label = label
            except Exception:  # noqa: BLE001 - 老版本没有 label 就不写
                pass
            i += 1
            counts["arrows_drawn"] += 1
        return i

    # ------------------------------------------------------------------ 轨迹拖尾
    def _push_trail(self, frame) -> None:
        t_now = float(getattr(frame, "t", 0.0))
        if self._last_t is not None and t_now == self._last_t:
            return  # 同一帧重复 draw 不重复记账，保证「不累积」
        bp = getattr(frame, "base_pos", None)
        if bp is not None:
            bp = np.asarray(bp, dtype=np.float64).reshape(-1)
            if bp.size >= 3 and np.isfinite(bp[:3]).all():
                self._trail.append((float(bp[0]), float(bp[1]), float(bp[2])))
                trail_n = max(int(self.cfg["trail_n"]), 0)
                if len(self._trail) > trail_n:
                    del self._trail[:len(self._trail) - trail_n]
        self._last_t = t_now

    def _draw_trail(self, scn, frame, i: int, limit: int, counts: Dict[str, int]) -> int:
        m = len(self._trail)
        if not self.cfg["trail_on"]:
            counts["trail_disabled"] = 1
            return i
        if m == 0:
            return i
        avail = max(0, limit - i)
        drawn = min(m, avail)
        counts["trail_dropped"] = m - drawn
        if drawn <= 0:
            return i
        stride = m / float(drawn)
        r = float(self.cfg["trail_radius"])
        pos = self._pos
        sz = self._sz
        sz[1] = 0.0
        sz[2] = 0.0
        lut = self._trail_lut
        denom = max(drawn - 1, 1)
        for j in range(drawn):
            k = int(j * stride)
            if k >= m:
                k = m - 1
            x, y, z = self._trail[k]
            pos[0] = x
            pos[1] = y
            pos[2] = z
            sz[0] = r
            g = scn.geoms[i]
            mujoco.mjv_initGeom(g, _GEOM_SPHERE, sz, pos, self._mat, lut[int(32.0 * j / denom)])
            g.objtype = _OBJ_UNKNOWN
            i += 1
            counts["trail_drawn"] += 1
        return i

    # ------------------------------------------------------------------ 计时 / 统计
    def _finish(self, t0: float, counts: Dict[str, int], ngeom: int) -> None:
        dt = time.perf_counter() - t0
        self._last_ms = dt * 1e3
        self._times.append(dt)
        counts.setdefault("base_ngeom", int(self._base_n or 0))
        counts.setdefault("ngeom", ngeom)
        counts.setdefault("maxgeom", self._maxgeom_last)
        self._counts = counts

    def stats(self) -> dict:
        times = np.asarray(self._times, dtype=np.float64)
        return {
            "frames": self._frames,
            "last_ms": self._last_ms,
            "mean_ms": float(times.mean() * 1e3) if times.size else 0.0,
            "max_ms": float(times.max() * 1e3) if times.size else 0.0,
            "base_ngeom": int(self._base_n or 0),
            "maxgeom": int(self._maxgeom_last),
            "trail_len": len(self._trail),
            "trail_n": int(self.cfg["trail_n"]),
            "geom_budget": int(self.cfg["geom_budget"]),
            "base_body": int(self._base_body),
            "counts": dict(self._counts),
        }

    def legend(self) -> str:
        """给 TextHud / 文档复用的颜色语义说明。"""
        return SCAN_COLOR_LEGEND

    # ------------------------------------------------------------------ 相机
    def set_camera(self, cam, frame) -> None:
        """可选：跟随躯干 / 俯视。cam 是 handle.cam（MjvCamera）。"""
        if cam is None:
            return
        mode = str(self.cfg["camera_mode"])
        if mode == "none":
            return
        bp = getattr(frame, "base_pos", None)
        if bp is None:
            bp = np.zeros(3, dtype=np.float64)
        else:
            bp = np.asarray(bp, dtype=np.float64).reshape(-1)
            if bp.size < 3:
                bp = np.zeros(3, dtype=np.float64)
        lookat = np.array([bp[0], bp[1], bp[2]], dtype=np.float64)
        cam.lookat[:] = lookat
        if mode == "follow" and self._base_body >= 0:
            cam.type = _CAM_TRACKING
            cam.trackbodyid = int(self._base_body)
            cam.fixedcamid = -1
        else:
            cam.type = _CAM_FREE
            cam.trackbodyid = -1
            cam.fixedcamid = -1
        cam.distance = float(self.cfg["camera_distance"])
        if mode == "top":
            cam.azimuth = -90.0
            cam.elevation = -89.0
        else:
            cam.azimuth = float(self.cfg["camera_azimuth"])
            cam.elevation = float(self.cfg["camera_elevation"])
