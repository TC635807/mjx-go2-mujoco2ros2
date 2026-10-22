"""viz/panels.py —— 原生 MjvFigure 实时曲线 + matplotlib(agg) 仪表盘图像 + 文字 HUD。

契约见 docs/viz-interfaces.md 第 4 节。三个组件互不依赖，都只消费 VizFrame：

    FigurePanel   -> handle.set_figures(viewports_figures)   原生 strip chart
    ImagePanel    -> handle.set_images(viewports_images)     matplotlib(agg) -> RGB uint8
    TextHud       -> handle.set_texts(texts)                 文字 HUD

本文件按**实测**的 MuJoCo 3.12 API 写，不要凭记忆改（mjvisualize.h / viewer.py 已逐条核对）：

  * MjvFigure 没有 figuretitle / ylabel 字段：标题字段是 title，x 轴标签是 xlabel。
  * linedata[i] 是 (x, y) **交错**存放：linedata[i][2k]=x_k、linedata[i][2k+1]=y_k；
    每行最多 mjMAXLINEPNT=1001 个点，linepnt[i] 决定用几个点（0 = 不画该线）。
  * linename 是 numpy |S100（bytes），赋值前必须 encode。
  * range[0] 是 x 轴 (min,max)，range[1] 是 y 轴；(min>=max) 表示自动。
  * MjrRect 是 (left, bottom, width, height)，原点在**左下**。
  * handle.set_texts 的 gridpos 是 mjtGridPos（**不是** MjrRect），字体是 mjtFontScale；
    text1/text2 支持多行（反斜杠 n），分别是同一横条的左右两列。
  * handle.set_images 会**硬校验** image.shape[:2] == (rect.height, rect.width) 并自行上下翻转，
    所以 viewport_image() 的 rect 必须用**实际渲染像素**，不能想当然。
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import mujoco

from signal_io.msg import NUM_JOINT
from sim_env.robot import LEG_ORDER

__all__ = ["FigurePanel", "ImagePanel", "TextHud", "JOINT_NAMES", "OBS_SEGMENTS"]

# 关节顺序 FR/FL/RR/RL × hip/thigh/calf（与 robot.py 的 MOTOR_JOINTS 一致，已核对）
JOINT_NAMES: Tuple[str, ...] = tuple(
    "%s_%s" % (leg, j) for leg in LEG_ORDER for j in ("hip", "thigh", "calf"))
assert len(JOINT_NAMES) == NUM_JOINT == 12

# 91 维 obs 的 10 个分段（与 obs.py 文件头的布局逐段对齐）
OBS_SEGMENTS: Tuple[Tuple[str, int, int], ...] = (
    ("linvel", 0, 3), ("angvel", 3, 6), ("grav", 6, 9), ("cmd", 9, 12),
    ("qpos", 12, 24), ("qvel", 24, 36), ("act", 36, 48),
    ("hscan", 48, 83), ("cos phi", 83, 87), ("sin phi", 87, 91),
)
_SEG_HEX = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
            "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf")
_SEG_COLOR_91 = [c for i, (nm, a, b) in enumerate(OBS_SEGMENTS) for c in [_SEG_HEX[i]] * (b - a)]
assert len(_SEG_COLOR_91) == 91


def _segments_for(obs_dim: int, scan_n: int) -> Tuple[Tuple[str, int, int], ...]:
    """按 obs 维度生成条形图分段表 (91 旧档 / 243 大扫描档)。

    布局与 envs/go2_walk.py 的 _get_obs 逐段对齐:
        48 base + scan_n hscan + (obs_dim-48-scan_n) phase
    """
    d = int(obs_dim)
    if d == 91:
        return OBS_SEGMENTS
    segs = [("linvel", 0, 3), ("angvel", 3, 6), ("grav", 6, 9), ("cmd", 9, 12),
            ("qpos", 12, 24), ("qvel", 24, 36), ("act", 36, 48)]
    n = int(scan_n)
    if d >= 48 + n and n > 0:
        segs.append(("hscan", 48, 48 + n))
        if d > 48 + n:
            segs.append(("phi", 48 + n, d))
    return tuple(segs)

# 每条线一个颜色：按腿分色（FR 红 / FL 蓝 / RR 绿 / RL 紫），组内 hip 深 -> calf 浅
_TAU_HEX = ("#d62728", "#f07c72", "#f7b6b2",   # FR
            "#1f77b4", "#6ba3e8", "#b6d2f2",   # FL
            "#2ca02c", "#7fc97f", "#c2e6c2",   # RR
            "#9467bd", "#b58fd6", "#d6c2e8")   # RL
_HEIGHT_HEX = ("#1f77b4", "#d62728")
_VEL_HEX = ("#1f4fd8", "#3b7ddd", "#6ba3e8", "#0b2f8a",   # est：蓝系
            "#d2691e", "#e8912f", "#f0b357", "#8b4513")   # true：橙系
_ACTION_HEX = ("#1f77b4", "#d62728", "#2ca02c")
_ERR_HEX = ("#d62728", "#2ca02c", "#1f77b4", "#111111")

_MAXPTS = int(getattr(mujoco, "mjMAXLINEPNT", 1001))   # 每行硬上限，linedata 装不下更多
_PANEL_MARGIN = 4
_PANEL_GAP = 6
# obs 条形图的 y 轴档位：固定 ±10 会让 O(1) 的 obs 变成一条线，所以按峰值自动换档（有迟滞）
_OBS_ZOOMS = (1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 110.0)


def _hex_rgb(h: str) -> Tuple[float, float, float]:
    h = h.lstrip("#")
    return (int(h[0:2], 16) / 255.0, int(h[2:4], 16) / 255.0, int(h[4:6], 16) / 255.0)


def _get(frame: Any, name: str, n: Optional[int] = None, default: float = 0.0) -> np.ndarray:
    """从 frame 取一维 float 数组；缺失 / None / 长度不足 / 非有限值都安全退化。"""
    if frame is None:
        return np.full(n, default, float) if n is not None else np.zeros(0, float)
    v = getattr(frame, name, None)
    if v is None:
        return np.full(n, default, float) if n is not None else np.zeros(0, float)
    try:
        a = np.asarray(v, dtype=np.float64).reshape(-1)
    except Exception:
        return np.full(n, default, float) if n is not None else np.zeros(0, float)
    if n is not None:
        out = np.full(n, default, float)
        m = min(n, a.size)
        out[:m] = a[:m]
        a = out
    return np.where(np.isfinite(a), a, default)


def _scalar(frame: Any, name: str, default: float = 0.0) -> float:
    v = getattr(frame, name, default) if frame is not None else default
    if v is None:
        return float(default)
    try:
        f = float(v)
    except Exception:
        return float(default)
    return f if math.isfinite(f) else float(default)


# ====================================================================== FigurePanel
class _Chart:
    """一张 MjvFigure + 它自己的滚动缓冲。"""

    def __init__(self, key: str, title: str, names: Sequence[str],
                 colors: Sequence[str], window: float):
        self.key = key
        self.title = title
        self.names = list(names)
        self.window = max(float(window), 1e-3)
        self._t: deque = deque()
        self._y: List[deque] = [deque() for _ in self.names]

        f = mujoco.MjvFigure()
        f.title = title
        f.xlabel = "t [s]"
        f.flg_legend = 1
        f.flg_extend = 0
        f.flg_barplot = 0
        f.flg_symmetric = 0
        f.flg_selection = 0
        f.flg_ticklabel = np.array([1, 1], np.int32)
        f.gridsize = np.array([4, 4], np.int32)
        f.gridwidth = 1.0
        f.linewidth = 2.0
        f.xformat = "%.1f"
        f.yformat = "%.2f"
        f.range[0] = np.array([0.0, self.window], np.float32)
        f.range[1] = np.array([-1.0, 1.0], np.float32)
        for i, nm in enumerate(self.names):
            f.linename[i] = str(nm).encode("ascii", "replace")[:99]
            f.linergb[i] = np.asarray(_hex_rgb(colors[i]), np.float32)
            f.linepnt[i] = 0
            f.linedata[i, :] = 0.0
        self.fig = f

    def __len__(self) -> int:
        return len(self._t)

    def push(self, t: float, values: np.ndarray) -> None:
        self._t.append(float(t))
        for d, v in zip(self._y, values):
            d.append(float(v))
        self._trim()

    def _trim(self) -> None:
        t = self._t
        if not t:
            return
        tmin = t[-1] - self.window
        while t and t[0] < tmin:          # 滚动窗口
            t.popleft()
            for d in self._y:
                d.popleft()
        while len(t) > _MAXPTS:           # t 不前进（同一时刻狂推）也要有硬上限
            t.popleft()
            for d in self._y:
                d.popleft()

    def refresh(self) -> None:
        """把缓冲写进 MjvFigure（linedata 交错 + linepnt + range 自动缩放）。"""
        f = self.fig
        n = min(len(self._t), _MAXPTS)
        xs = (np.fromiter(self._t, dtype=np.float32, count=len(self._t))[:n]
              if n else np.zeros(0, np.float32))
        ymin: Optional[float] = None
        ymax: Optional[float] = None
        for i, d in enumerate(self._y):
            ys = (np.fromiter(d, dtype=np.float32, count=len(self._t))[:n]
                  if n else np.zeros(0, np.float32))
            m = 0
            if ys.size and not np.all(np.isnan(ys)):
                bad = np.flatnonzero(np.isnan(ys))
                m = int(bad[0]) if bad.size else int(n)   # 只画到第一个 nan 之前
            if m > 0:
                row = np.empty(2 * m, np.float32)
                row[0::2] = xs[:m]
                row[1::2] = ys[:m]
                f.linedata[i, :2 * m] = row
                f.linepnt[i] = m
                lo = float(np.min(ys[:m]))
                hi = float(np.max(ys[:m]))
                ymin = lo if ymin is None else min(ymin, lo)
                ymax = hi if ymax is None else max(ymax, hi)
            else:
                f.linepnt[i] = 0
        if n:
            x1 = float(self._t[-1])
            x0 = x1 - self.window
            if x1 - x0 < 1e-6:
                x0 = x1 - 1e-3
        else:
            x0, x1 = -self.window, 0.0
        f.range[0] = np.array([x0, x1], np.float32)
        if ymin is None:
            f.range[1] = np.array([-1.0, 1.0], np.float32)
        else:
            span = ymax - ymin
            pad = 0.10 * span if span > 1e-6 else max(0.5 * abs(ymax), 0.1)
            f.range[1] = np.array([ymin - pad, ymax + pad], np.float32)


class FigurePanel:
    """原生 MjvFigure 实时曲线（契约第 4 节）。

    which 里可选 "tau" / "height" / "vel" / "action" / "err"；window 是滚动窗口秒数。
    """

    KEYS: Tuple[str, ...] = ("tau", "height", "vel", "action", "err")

    def __init__(self, which: Sequence[str] = ("tau",), window: float = 5.0):
        self.window = max(float(window), 1e-3)
        keys: List[str] = []
        for k in which:
            k = str(k)
            if k not in self.KEYS:
                raise ValueError("FigurePanel: 未知图 %r，可选 %s" % (k, list(self.KEYS)))
            if k not in keys:          # 去重，避免同一个图叠两次
                keys.append(k)
        self.which: Tuple[str, ...] = tuple(keys)
        self._charts: List[_Chart] = [self._make_chart(k) for k in self.which]

    def _make_chart(self, key: str) -> _Chart:
        if key == "tau":
            return _Chart("tau", "joint torque [Nm]", JOINT_NAMES, _TAU_HEX, self.window)
        if key == "height":
            return _Chart("height", "base z [m] / tilt [rad]", ("z", "tilt"),
                          _HEIGHT_HEX, self.window)
        if key == "vel":
            return _Chart("vel", "base lin vel [m/s] est vs true",
                          ("vx_est", "vy_est", "vz_est", "|v|_est",
                           "vx_true", "vy_true", "vz_true", "|v|_true"),
                          _VEL_HEX, self.window)
        if key == "action":
            return _Chart("action", "action |mean| |max| clip#",
                          ("|a|_mean", "|a|_max", "clip_n"), _ACTION_HEX, self.window)
        return _Chart("err", "|v_est - v_true| [m/s]", ("ex", "ey", "ez", "|e|"),
                      _ERR_HEX, self.window)

    def _values(self, key: str, frame: Any) -> np.ndarray:
        if key == "tau":
            return _get(frame, "joint_tau", NUM_JOINT)
        if key == "height":
            z = float(_get(frame, "base_pos", 3)[2])
            tilt = float(np.linalg.norm(_get(frame, "rpy", 3)[:2]))
            return np.array([z, tilt], float)
        if key == "vel":
            v = _get(frame, "base_lin_vel_world", 3)
            out = np.array([v[0], v[1], v[2], float(np.linalg.norm(v)),
                            np.nan, np.nan, np.nan, np.nan], float)
            if getattr(frame, "base_lin_vel_truth", None) is not None:
                tv = _get(frame, "base_lin_vel_truth", 3)
                out[4:] = (tv[0], tv[1], tv[2], float(np.linalg.norm(tv)))
            return out
        if key == "action":
            a = _get(frame, "action", NUM_JOINT)
            ap = _get(frame, "action_applied", NUM_JOINT)
            return np.array([float(np.mean(np.abs(a))), float(np.max(np.abs(a))),
                             float(np.count_nonzero(np.abs(a - ap) > 1e-6))], float)
        if key == "err":
            if getattr(frame, "base_lin_vel_truth", None) is None:
                return np.full(4, np.nan)
            e = _get(frame, "base_lin_vel_world", 3) - _get(frame, "base_lin_vel_truth", 3)
            return np.array([e[0], e[1], e[2], float(np.linalg.norm(e))], float)
        return np.zeros(0, float)

    def update(self, frame: Any) -> None:
        """推一个样本。frame 字段缺失会安全退化为 0/nan，不抛异常。"""
        t = _scalar(frame, "t", 0.0)
        for ch in self._charts:
            ch.push(t, self._values(ch.key, frame))

    def viewports(self, width: int, height: int,
                  origin: Tuple[int, int] = (0, 0)) -> List[Tuple[Any, Any]]:
        """返回 [(MjrRect, MjvFigure)]，每个 rect 都落在 (width, height) 这块区域内。

        origin=(left, bottom) 是这块区域在窗口里的左下角（viewer 坐标，原点左下，默认 (0,0)，
        即契约的 viewports(width, height) 行为不变）。要给 ImagePanel 让位时就用它，例如
        viewports(W, H*0.6, origin=(0, H*0.4)) 把曲线放上半屏、仪表盘放右下角。
        """
        ox = max(0, int(origin[0]))
        oy = max(0, int(origin[1]))
        W = max(1, int(width))
        H = max(1, int(height))
        n = len(self._charts)
        if n == 0:
            return []
        cols = 1 if n <= 2 else 2
        rows = int(math.ceil(n / float(cols)))
        cw = (W - 2 * _PANEL_MARGIN - (cols - 1) * _PANEL_GAP) / float(cols)
        chh = (H - 2 * _PANEL_MARGIN - (rows - 1) * _PANEL_GAP) / float(rows)
        out: List[Tuple[Any, Any]] = []
        for i, chart in enumerate(self._charts):
            chart.refresh()
            r, c = divmod(i, cols)
            left = int(ox + _PANEL_MARGIN + c * (cw + _PANEL_GAP))
            bottom = int(oy + H - _PANEL_MARGIN - (r + 1) * chh - r * _PANEL_GAP)
            w = max(1, int(cw))
            h = max(1, int(chh))
            w = min(w, ox + W - left) if ox + W - left >= 1 else 1
            left = max(ox, min(left, ox + W - w))
            h = min(h, oy + H - bottom) if oy + H - bottom >= 1 else 1
            bottom = max(oy, min(bottom, oy + H - h))
            out.append((mujoco.MjrRect(left, bottom, w, h), chart.fig))
        return out

    def n_samples(self) -> Dict[str, int]:
        """每个图的当前缓冲长度（给测试/自检用）。"""
        return {ch.key: len(ch) for ch in self._charts}

    def figures(self) -> List[Any]:
        return [ch.fig for ch in self._charts]


# ====================================================================== ImagePanel
try:  # matplotlib 是可选依赖：没有就降级成 None，不能抛异常
    import matplotlib
    matplotlib.use("Agg", force=True)
    from matplotlib.figure import Figure as _MplFigure
    from matplotlib.backends.backend_agg import FigureCanvasAgg as _MplCanvas
    from matplotlib.gridspec import GridSpec as _MplGridSpec
    from matplotlib.collections import LineCollection as _MplLineCollection
    _HAVE_MPL = True
except Exception:  # pragma: no cover - 取决于环境
    _MplFigure = _MplCanvas = _MplGridSpec = _MplLineCollection = None  # type: ignore
    _HAVE_MPL = False


class ImagePanel:
    """matplotlib(agg) 仪表盘 -> RGB uint8，交给 handle.set_images。

    面板内容（契约第 4 节）：
      * 左上：height scan 7x5 热图（机体系，x 向右为前）+ colorbar + 符号说明
      * 右上：4 条腿的步态相位极坐标
      * 下：  obs 91 维的 10 段条形（段名 + 当前值范围）
      * 下：  高度特征均值 / 动作 mean,max / 健康等级 / 来源

    性能：缓存背景 + blit 只重画变化的 artist，实测单帧约 6~8 ms（目标 < 15 ms）。
    """

    def __init__(self, dpi: int = 80, size=(6.4, 3.6), anchor: str = "right",
                 margin: int = 8, contact_thresh: float = 1.0,
                 obs_ylim: Optional[Tuple[float, float]] = None, ranges: bool = True,
                 hlim: float = 1.75):
        self.dpi = max(1, int(dpi))
        self.size = (float(size[0]), float(size[1]))
        self._want_px = (max(1, int(round(self.size[0] * self.dpi))),
                         max(1, int(round(self.size[1] * self.dpi))))
        self.anchor = str(anchor)
        self.margin = int(margin)
        self.contact_thresh = float(contact_thresh)
        # obs_ylim=None -> 按峰值自动换档（默认）；给定 (lo,hi) 则锁死（颜色/坐标不再变，永远不用重画背景）
        self._fixed_ylim = None if obs_ylim is None else (float(obs_ylim[0]), float(obs_ylim[1]))
        self._zoom = 3 if self._fixed_ylim is None else -1   # 默认从 ±8 开始
        self._down = 0
        self.ranges = bool(ranges)   # False = 不画 10 段当前值范围那一行（省 ~6 ms/帧）
        # 热图色标：height_scan 的定义域是 clip(.., +-scan_clip_m)*scan_scale = +-1.75（默认配置）。
        # 用固定色标才能横向比较不同帧；换了 scan 配置就传对应的 hlim。
        self._hlim = float(hlim)
        # v6: obs 维度 / 扫描网格形状可配 (默认 91 维 + 7x5; 大扫描档 243 + 17x11)
        self._obs_dim = 91
        self._scan_nx = 7
        self._scan_ny = 5
        self._segments = _segments_for(self._obs_dim, self._scan_nx * self._scan_ny)
        self._seg_colors = [c for i, (nm, a, b) in enumerate(self._segments)
                            for c in [_SEG_HEX[i % len(_SEG_HEX)]] * (b - a)]

        self._ok = bool(_HAVE_MPL)
        self.error: Optional[str] = None if self._ok else "matplotlib 不可用"
        self._img: Optional[np.ndarray] = None
        self._req_px: Tuple[int, int] = (-1, -1)   # 上次**请求**的渲染尺寸
        self._render_px: Tuple[int, int] = (0, 0)  # 实际画布像素
        self._frame: Any = None
        self._bg: Any = None
        self._ext = (-0.3, 0.9, -0.4, 0.4)
        self.n_render = 0
        self.last_ms = 0.0
        if self._ok:
            try:
                self._build()
                self._req_px = self._want_px
            except Exception as e:   # pragma: no cover - 环境问题
                self._ok = False
                self.error = "%s: %s" % (type(e).__name__, e)

    def _build(self) -> None:
        pw, ph = self._want_px
        self._fig = _MplFigure(figsize=(pw / self.dpi, ph / self.dpi), dpi=self.dpi)
        self._canvas = _MplCanvas(self._fig)
        gs = _MplGridSpec(3, 2, figure=self._fig, height_ratios=[1.25, 1.0, 0.62],
                          hspace=0.95, wspace=0.45,
                          left=0.095, right=0.905, top=0.90, bottom=0.045)
        self._ax_h = self._fig.add_subplot(gs[0, 0])
        self._ax_p = self._fig.add_subplot(gs[0, 1], projection="polar")
        self._ax_o = self._fig.add_subplot(gs[1, :])
        self._ax_t = self._fig.add_subplot(gs[2, :])
        self._ax_t.axis("off")

        # 左上：7x5 热图。height_scan 负 = 地面更高(上坡)，正 = 下陷 -> coolwarm 冷=负
        self._im = self._ax_h.imshow(
            np.zeros((5, 7)), origin="lower", aspect="auto", interpolation="nearest",
            cmap="coolwarm", vmin=-self._hlim, vmax=self._hlim,
            extent=[self._ext[0], self._ext[1], self._ext[2], self._ext[3]])
        self._ax_h.set_title("height scan  -:up  +:dip", fontsize=5.5, pad=2)
        self._ax_h.set_xlabel("x fwd [m]", fontsize=4.5, labelpad=0)
        self._ax_h.set_ylabel("y [m]", fontsize=4.5, labelpad=0)
        self._ax_h.tick_params(labelsize=4, length=1.5, pad=1)
        self._cb = self._fig.colorbar(self._im, ax=self._ax_h, fraction=0.05, pad=0.04)
        self._cb.ax.tick_params(labelsize=4, length=1.5, pad=1)

        # 右上：4 腿相位极坐标（相位角 -> 圆周位置，颜色表示是否接触）
        self._ax_p.set_ylim(0.0, 1.35)
        self._ax_p.set_yticklabels([])
        self._ax_p.set_xticklabels([])
        self._ax_p.grid(True, linewidth=0.4, alpha=0.6)
        self._ax_p.set_title("gait phase", fontsize=5.5, pad=6)
        self._ax_p.tick_params(labelsize=4)
        self._phase_scat = self._ax_p.scatter([0.0] * 4, [1.0] * 4, s=18, c=["0.6"] * 4,
                                              edgecolors="k", linewidths=0.4, zorder=3)
        # 腿名放**固定**方位角（静态，进背景缓存）；每帧变的是 4 个标记点本身
        for i, ang in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)):
            self._ax_p.text(ang, 1.32, "%s" % LEG_ORDER[i], fontsize=4.5,
                            ha="center", va="center")

        # 下：obs 条形图。用**一条** LineCollection 画全部竖条（比逐个 Rectangle 快得多）
        self._build_obs_artists()

        # 下：文字（健康等级 / 高度特征均值 / 动作统计 / 来源）
        self._txt = self._ax_t.text(0.0, 1.0, "", fontsize=6, family="monospace",
                                    va="top", ha="left", transform=self._ax_t.transAxes)
        self._render_full()

    def _build_obs_artists(self) -> None:
        """按当前 obs 维度重建条形图 artist (configure() 也复用它)。"""
        d = self._obs_dim
        n_scan = self._scan_nx * self._scan_ny
        self._ax_o.set_xlim(-0.5, d - 0.5)
        self._ax_o.set_ylim(self.obs_lim)
        segs = np.zeros((d, 2, 2))
        segs[:, 0, 0] = np.arange(d)
        segs[:, 1, 0] = np.arange(d)
        self._obs_lc = _MplLineCollection(segs, linewidths=1.7,
                                          colors=list(self._seg_colors))
        self._ax_o.add_collection(self._obs_lc)
        self._ax_o.set_title("obs %d = 48 base + %d hscan + %d phase"
                             % (d, n_scan, max(d - 48 - n_scan, 0)),
                             fontsize=5.5, pad=2)
        self._ax_o.tick_params(labelsize=4, length=1.5, pad=1)
        centers = [0.5 * (a + b) for (nm, a, b) in self._segments]
        self._ax_o.set_xticks(centers)
        # 段名静态（进背景缓存）：位置顺序与动态文字里的值范围一一对应
        self._ax_o.set_xticklabels([nm for (nm, a, b) in self._segments], fontsize=3.6)

    def configure(self, obs_dim=None, scan_nx=None, scan_ny=None) -> None:
        """v6: 按策略 obs 维度 / 扫描网格重建仪表盘 (91/7x5 -> 243/17x11)。

        必须在推第一帧之前调用 (view_policy.build_state 里做)。
        """
        if not self._ok:
            return
        new_dim = self._obs_dim if obs_dim is None else int(obs_dim)
        nx = self._scan_nx if scan_nx is None else int(scan_nx)
        ny = self._scan_ny if scan_ny is None else int(scan_ny)
        if (new_dim, nx, ny) == (self._obs_dim, self._scan_nx, self._scan_ny):
            return
        self._obs_dim, self._scan_nx, self._scan_ny = new_dim, nx, ny
        self._segments = _segments_for(new_dim, nx * ny)
        self._seg_colors = [c for i, (nm, a, b) in enumerate(self._segments)
                            for c in [_SEG_HEX[i % len(_SEG_HEX)]] * (b - a)]
        self._ax_o.clear()
        self._build_obs_artists()
        self._zoom = -1              # 重新按真实峰值定档
        self._render_full()

    def _render_full(self) -> None:
        self._canvas.draw()
        self._bg = self._canvas.copy_from_bbox(self._fig.bbox)
        self._img = np.ascontiguousarray(np.asarray(self._canvas.buffer_rgba())[..., :3])
        self._render_px = self._canvas.get_width_height()

    def _blit(self) -> None:
        # 只重画 4 个每帧会变的 artist。Agg 每画一个字符都要重新光栅化字形
        # （实测 ~0.1 ms/字，无跨帧缓存），所以动态文字必须集中到**一个** Text 且尽量短；
        # 标题/刻度/段名/腿名等全部已进背景缓存，不参与每帧重画。
        self._canvas.restore_region(self._bg)
        for a in (self._im, self._phase_scat, self._obs_lc, self._txt):
            a.axes.draw_artist(a)
        self._canvas.blit(self._fig.bbox)

    def _sync_extent(self, frame: Any) -> bool:
        """从 frame.scan_xy_body 取真实网格范围；变了才返回 True（需要重画背景）。"""
        xy = getattr(frame, "scan_xy_body", None)
        if xy is None:
            return False
        a = np.asarray(xy, dtype=np.float64)
        if a.ndim != 2 or a.shape[0] < 4 or a.shape[1] < 2 or not np.all(np.isfinite(a)):
            return False
        x0, x1 = float(a[:, 0].min()), float(a[:, 0].max())
        y0, y1 = float(a[:, 1].min()), float(a[:, 1].max())
        if x1 <= x0 or y1 <= y0:
            return False
        if (abs(x0 - self._ext[0]) < 1e-9 and abs(x1 - self._ext[1]) < 1e-9
                and abs(y0 - self._ext[2]) < 1e-9 and abs(y1 - self._ext[3]) < 1e-9):
            return False
        self._ext = (x0, x1, y0, y1)
        self._im.set_extent([x0, x1, y0, y1])
        self._ax_h.set_xlim(x0, x1)
        self._ax_h.set_ylim(y0, y1)
        return True

    @property
    def obs_lim(self) -> Tuple[float, float]:
        """当前 obs 条形图的 y 轴范围（自动换档或锁定）。"""
        if self._fixed_ylim is not None:
            return self._fixed_ylim
        if self._zoom < 0:                      # 还没根据真实数据定档
            z = _OBS_ZOOMS[3]
        else:
            z = _OBS_ZOOMS[max(0, min(self._zoom, len(_OBS_ZOOMS) - 1))]
        return (-z, z)

    def _auto_zoom(self, obs: np.ndarray) -> bool:
        """按峰值换档；只在真的换档时返回 True（需要重画背景，因为 y 轴刻度变了）。

        升档立即生效；降档要连续 25 帧都很小才动，避免在档位边界抖动。
        """
        if self._fixed_ylim is not None:
            return False
        peak = float(np.max(np.abs(obs))) if obs.size else 0.0
        want = 0
        while want < len(_OBS_ZOOMS) - 1 and _OBS_ZOOMS[want] < peak * 1.05:
            want += 1
        if self._zoom < 0:                      # 首帧：直接按真实峰值定档，只多画一次背景
            self._zoom, self._down = want, 0
            self._ax_o.set_ylim(self.obs_lim)
            return True
        lim_hi = self.obs_lim[1]
        if peak > lim_hi * 1.15:            # 明显超出当前档位才升档，否则峰值卡在边界会反复重画背景
            self._zoom, self._down = want, 0
            self._ax_o.set_ylim(self.obs_lim)
            return True
        if want < self._zoom:
            self._down += 1
            if self._down >= 25:
                self._zoom = max(want, self._zoom - 1)
                self._down = 0
                self._ax_o.set_ylim(self.obs_lim)
                return True
        else:
            self._down = 0
        return False

    def _refresh_artists(self, frame: Any) -> bool:
        """更新所有会变的 artist；返回 True 表示坐标轴变了、需要重画背景。"""
        # 热图：scan 是 x 外层 y 内层 -> reshape(ny, nx) 再转置成 [y, x]
        n_scan = self._scan_nx * self._scan_ny
        hs = _get(frame, "height_scan", n_scan)
        # scan 是 x 外层 y 内层 -> reshape(nx, ny) 再转置成 [y, x]
        self._im.set_data(hs[:n_scan].reshape(self._scan_nx, self._scan_ny).T)

        ph = _get(frame, "phase", 4)
        ff = _get(frame, "foot_force", 4)
        cols = ["#2ca02c" if ff[i] > self.contact_thresh else "0.55" for i in range(4)]
        self._phase_scat.set_offsets(np.c_[ph, np.ones(4)])
        self._phase_scat.set_color(cols)

        d = self._obs_dim
        obs = _get(frame, "obs", d)
        need_full = self._auto_zoom(obs)          # 先换档，再按新档位取条形高度
        lim = self.obs_lim
        disp = np.clip(obs, lim[0], lim[1])
        segs = np.zeros((d, 2, 2))
        segs[:, 0, 0] = np.arange(d)
        segs[:, 1, 0] = np.arange(d)
        segs[:, 1, 1] = disp
        self._obs_lc.set_segments(segs)
        self._obs_lc.set_color(list(self._seg_colors))

        act = _get(frame, "action", NUM_JOINT)
        lvl = str(getattr(frame, "level_name", None) or ("L%d" % int(_scalar(frame, "level"))))
        mode = str(getattr(frame, "mode", "?"))
        okk = "ok" if bool(getattr(frame, "obs_valid", True)) else "LOST"
        # 只有一个动态 Text：字符数直接决定每帧耗时（Agg 每字都要重新光栅化字形）。
        # 这里只放契约要求的三项 + 10 段当前值范围，并尽量压缩字符数。
        head = "%s %s obs:%s hscan %s a %s/%s" % (
            lvl, mode, okk, _num(np.mean(hs)),
            _num(np.mean(np.abs(act))), _num(np.max(np.abs(act))))
        if self.ranges:
            head += "\nrng " + " ".join(_rng_txt(obs[a:b])
                                          for (nm, a, b) in self._segments)
        self._txt.set_text(head)
        return need_full

    def update(self, frame: Any) -> None:
        """推一帧并重画。matplotlib 不可用时静默跳过（不抛异常）。"""
        self._frame = frame
        if not self._ok:
            return
        t0 = time.perf_counter()
        try:
            need_full = self._sync_extent(frame) | self._refresh_artists(frame)
            if need_full or self._bg is None:
                self._render_full()
            else:
                self._blit()
            self._img = np.ascontiguousarray(
                np.asarray(self._canvas.buffer_rgba())[..., :3])
            self.n_render += 1
        except Exception as e:   # 渲染失败降级为 None，不要把观察脚本一起弄死
            self._ok = False
            self.error = "%s: %s" % (type(e).__name__, e)
            self._img = None
        self.last_ms = (time.perf_counter() - t0) * 1000.0

    def viewport_image(self, width: int, height: int) -> Optional[Tuple[Any, np.ndarray]]:
        """返回 (MjrRect, (H,W,3) uint8)；不可用时 None。rect 与图像像素严格一致。"""
        if not self._ok or self._img is None:
            return None
        W = max(1, int(width))
        H = max(1, int(height))
        pw = min(self._want_px[0], max(1, W - 2 * self.margin))
        ph = min(self._want_px[1], max(1, H - 2 * self.margin))
        if (pw, ph) != self._req_px:
            try:
                self._fig.set_size_inches(pw / self.dpi, ph / self.dpi, forward=True)
                if self._frame is not None:
                    self._refresh_artists(self._frame)
                self._render_full()   # 尺寸变了，背景缓存作废
                self._req_px = (pw, ph)
            except Exception as e:
                self._ok = False
                self.error = "%s: %s" % (type(e).__name__, e)
                self._img = None
                return None
        img = self._img
        ih, iw = int(img.shape[0]), int(img.shape[1])
        if self.anchor in ("right", "bottomright", "br"):
            left, bottom = W - self.margin - iw, self.margin
        elif self.anchor in ("topright", "tr"):
            left, bottom = W - self.margin - iw, H - self.margin - ih
        elif self.anchor in ("topleft", "tl"):
            left, bottom = self.margin, H - self.margin - ih
        else:  # left / bottomleft
            left, bottom = self.margin, self.margin
        left = max(0, min(int(left), max(0, W - iw)))
        bottom = max(0, min(int(bottom), max(0, H - ih)))
        return mujoco.MjrRect(left, bottom, iw, ih), img

    def render_ms(self) -> float:
        return float(self.last_ms)


# ====================================================================== TextHud
def _num(x: float) -> str:
    """压缩数字（每多一个字符就多一次字形光栅化）。

    * 接近 0 直接写 0（别写 -6e-17）；
    * 一位有效数字；
    * |x|<1 时去掉前导 0：0.05 -> .05、-0.4 -> -.4。
    """
    x = float(x)
    if abs(x) < 1e-9:
        x = 0.0
    s = "%.1g" % x
    if s.startswith("0."):
        s = s[1:]
    elif s.startswith("-0."):
        s = "-" + s[2:]
    return s


def _rng_txt(v: np.ndarray) -> str:
    """一个 obs 分段的"当前值范围" min~max，压到最短。"""
    return "%s~%s" % (_num(np.min(v)), _num(np.max(v)))


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return "%.3g" % v
    return str(v)


class TextHud:
    """文字 HUD（契约第 4 节）。

    texts() 返回 [(font, gridpos, text1, text2)]：
      * font 是 mujoco.mjtFontScale，gridpos 是 mujoco.mjtGridPos（viewer API 就是这两个枚举）；
      * text1 / text2 一定是 str，且支持多行（反斜杠 n）。
    """

    def __init__(self, font: Any = None, font_small: Any = None):
        self._font = font if font is not None else mujoco.mjtFontScale.mjFONTSCALE_150
        self._font_small = (font_small if font_small is not None
                            else mujoco.mjtFontScale.mjFONTSCALE_100)
        self._frame: Any = None
        self._extra: Dict[str, Any] = {}
        self._rows: List[Tuple[Any, Any, str, str]] = []
        self._compose()

    def _compose(self) -> None:
        fr = self._frame
        lvl = str(getattr(fr, "level_name", None) or "L0") if fr is not None else "L0"
        mode = str(getattr(fr, "mode", "?")) if fr is not None else "?"
        obs_ok = bool(getattr(fr, "obs_valid", True)) if fr is not None else True
        sources = (getattr(fr, "sources", None) or {}) if fr is not None else {}
        tilt = float(np.linalg.norm(_get(fr, "rpy", 3)[:2])) * 180.0 / math.pi
        z = float(_get(fr, "base_pos", 3)[2])
        vel = _get(fr, "base_lin_vel_world", 3)
        tau = _get(fr, "joint_tau", NUM_JOINT)
        act = _get(fr, "action", NUM_JOINT)
        tstr, estr = "n/a", "n/a"
        if fr is not None and getattr(fr, "base_lin_vel_truth", None) is not None:
            tv = _get(fr, "base_lin_vel_truth", 3)
            tstr = "%+.2f" % tv[0]
            estr = "%.3f" % abs(vel[0] - tv[0])

        left1 = ("%s  %s  obs:%s\n"
                 "z %.3f m  tilt %.1f deg\n"
                 "vx %+.2f / %s  err %s\n"
                 "tau_max %.1f Nm  |a| max %.2f"
                 % (lvl, mode, "ok" if obs_ok else "LOST", z, tilt, vel[0], tstr, estr,
                    float(np.max(np.abs(tau))), float(np.max(np.abs(act)))))
        right1 = "linvel: %s\nscan:   %s" % (sources.get("linvel", "-"),
                                              sources.get("scan", "-"))
        left2 = "rtf %.2f  overrun %d  n_cmd %d  n_state %d" % (
            _scalar(fr, "rtf"), int(_scalar(fr, "overruns")),
            int(_scalar(fr, "n_cmd")), int(_scalar(fr, "n_state")))
        right2 = "  ".join("%s=%s" % (k, v) for k, v in sorted(sources.items())) or "sources: -"
        extra = "  ".join("%s=%s" % (k, _fmt(v)) for k, v in sorted(self._extra.items()))

        self._rows = [
            (self._font, mujoco.mjtGridPos.mjGRID_TOPLEFT, str(left1), str(right1)),
            (self._font_small, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, str(left2), str(right2)),
        ]
        if extra:
            self._rows.append((self._font_small, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                               str(extra), ""))

    def update(self, frame: Any, extra: Optional[dict] = None) -> None:
        self._frame = frame
        self._extra = dict(extra) if extra else {}
        self._compose()

    def texts(self) -> List[tuple]:
        return list(self._rows)
