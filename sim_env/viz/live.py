"""独立窗口的实时曲线 + 仪表盘（matplotlib QtAgg）。

为什么不塞进 MuJoCo 窗口：MuJoCo 的窗口尺寸是它自己创建的（本机 740x800），
塞进去的曲线每块只有 89x161 px —— **根本看不清**；要放大就必然糊住机器人。
所以拆成两个窗口，各司其职：

    MuJoCo 窗口   只放机器人和 3D 标记（机器人始终看得见）
    本模块窗口    13x7.5 英寸的 6 宫格，可自由缩放、可拖走

没有 Qt 后端时 LivePlots.available() 返回 False，调用方退化成旧的内嵌面板。

契约见 docs/viz-interfaces.md；数据只消费 VizFrame。
"""

from __future__ import annotations

import time
from collections import deque
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .frame import VizFrame

LEG_COLORS = ("#e6194b", "#3cb44b", "#4363d8", "#f58231")
JOINT_NAMES = [f"{leg}_{part}" for leg in ("FR", "FL", "RR", "RL")
               for part in ("hip", "thigh", "calf")]


def available() -> bool:
    """能不能开独立窗口（需要 Qt 后端）。"""
    try:
        import matplotlib
        matplotlib.use("QtAgg", force=True)
        import matplotlib.pyplot as plt
        # 只 import pyplot 不够: 后端是懒加载的, 真正建画布时才 import Qt。
        # 这里用一个最小 figure 探一下(然后关掉), 否则 available() 会误报 True。
        fig = plt.figure(figsize=(0.5, 0.5))
        plt.close(fig)
        return True
    except Exception:
        return False


class LivePlots:
    """6 宫格实时面板：力矩 / 高度倾斜 / 速度 / 高度扫描 / 步态相位 / 动作。"""

    def __init__(self, which: Sequence[str] = ("tau", "height", "vel", "scan", "phase", "act"),
                 window_s: float = 6.0, dpi: int = 100, figsize: Tuple[float, float] = (13.0, 7.5),
                 title: str = ""):
        import matplotlib
        matplotlib.use("QtAgg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec

        # 中文标题要指定 CJK 字体，否则全是豆腐块
        matplotlib.rcParams["font.sans-serif"] = [
            "Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei", "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        self._plt = plt
        self.which = tuple(which)
        self.window_s = float(window_s)
        self.n_draw = 0
        self.t_draw = 0.0

        self.fig = plt.figure(figsize=figsize, dpi=dpi)
        self.fig.canvas.manager.set_window_title(title or "Go2 观察器")
        gs = GridSpec(2, 3, figure=self.fig, hspace=0.42, wspace=0.30,
                      left=0.055, right=0.985, top=0.930, bottom=0.085)
        self.ax_tau = self.fig.add_subplot(gs[0, 0])
        self.ax_h = self.fig.add_subplot(gs[0, 1])
        self.ax_v = self.fig.add_subplot(gs[0, 2])
        self.ax_scan = self.fig.add_subplot(gs[1, 0])
        # 相位用普通 axes 画 4 根横条，而不是 polar —— polar 每次重绘要多花 ~24 ms
        self.ax_ph = self.fig.add_subplot(gs[1, 1])
        self.ax_act = self.fig.add_subplot(gs[1, 2])

        self._tau_lines = []
        for i in range(12):
            (ln,) = self.ax_tau.plot([], [], lw=1.0, color=LEG_COLORS[i // 3], alpha=0.55 + 0.45 * (i % 3) / 2)
            self._tau_lines.append(ln)
        self.ax_tau.set_title("关节力矩 (N·m)", fontsize=9)
        self.ax_tau.set_xlabel("t (s)", fontsize=8)
        self.ax_tau.tick_params(labelsize=7)
        self.ax_tau.grid(alpha=0.25)

        (self._h_z,) = self.ax_h.plot([], [], lw=1.2, color="#1f77b4", label="z (m)")
        (self._h_t,) = self.ax_h.plot([], [], lw=1.2, color="#d62728", label="tilt (deg)")
        self.ax_h.set_title("躯干高度 / 倾斜", fontsize=9)
        self.ax_h.tick_params(labelsize=7)
        self.ax_h.grid(alpha=0.25)
        self.ax_h.legend(fontsize=7, loc="upper left")

        (self._v_ex,) = self.ax_v.plot([], [], lw=1.2, color="#1f77b4", label="|v| 估计")
        (self._v_tr,) = self.ax_v.plot([], [], lw=1.2, color="#ff7f0e", ls="--", label="|v| 真值")
        (self._v_ez,) = self.ax_v.plot([], [], lw=1.0, color="#7f7f7f", ls=":", label="估计误差")
        self.ax_v.set_title("基座线速度 (m/s)", fontsize=9)
        self.ax_v.tick_params(labelsize=7)
        self.ax_v.grid(alpha=0.25)
        self.ax_v.legend(fontsize=7, loc="upper left")

        self._scan_img = self.ax_scan.imshow(np.zeros((7, 5)), cmap="coolwarm",
                                             vmin=-1.75, vmax=1.75, origin="lower", aspect="auto")
        self.ax_scan.set_title("height scan 7x5（负=上坡) ", fontsize=9)
        self.ax_scan.set_xlabel("y: 右 -> 左", fontsize=7)
        self.ax_scan.set_ylabel("x: 前 ->", fontsize=7)
        self.ax_scan.tick_params(labelsize=7)
        # 不放 colorbar（省 ~8 ms/帧），直接把量程写进标题

        self.ax_ph.set_title("步态相位 sin/cos", fontsize=9)
        self.ax_ph.tick_params(labelsize=7)
        self.ax_ph.set_yticks([0.5, 1.5, 2.5, 3.5])
        self.ax_ph.set_yticklabels(["FR", "FL", "RR", "RL"], fontsize=7)
        self.ax_ph.set_xlim(-1.15, 1.15)
        self.ax_ph.grid(alpha=0.25, axis="x")
        self._ph_cos = self.ax_ph.barh([0.5, 1.5, 2.5, 3.5], np.zeros(4), height=0.5,
                                       color=list(LEG_COLORS), alpha=0.85)
        self._ph_sin = self.ax_ph.barh([0.5, 1.5, 2.5, 3.5], np.zeros(4), height=0.5,
                                       color=list(LEG_COLORS), alpha=0.35)

        self._act_bars = self.ax_act.bar(range(12), np.zeros(12),
                                         color=[LEG_COLORS[i // 3] for i in range(12)], alpha=0.85)
        self.ax_act.set_title("策略动作 (rad, 关节位置目标)", fontsize=9)
        self.ax_act.set_xticks(range(12))
        self.ax_act.set_xticklabels([n.replace("_", "\n") for n in JOINT_NAMES], fontsize=5.5)
        self.ax_act.tick_params(labelsize=7)
        self.ax_act.grid(alpha=0.25, axis="y")

        self._t: deque = deque()
        self._tau: deque = deque()
        self._z: deque = deque()
        self._tilt: deque = deque()
        self._ve: deque = deque()
        self._vt: deque = deque()
        self._maxlen = 4096

        self.fig.canvas.draw()
        self._plt.ion()
        self._plt.show(block=False)

    # ------------------------------------------------------------------
    def update(self, frame: VizFrame) -> None:
        self._t.append(float(frame.t))
        self._tau.append(np.asarray(frame.joint_tau, dtype=np.float64).copy())
        self._z.append(float(frame.base_pos[2]))
        self._tilt.append(float(np.degrees(np.hypot(frame.rpy[0], frame.rpy[1]))))
        self._ve.append(float(np.linalg.norm(frame.base_lin_vel_world)))
        vtf = frame.base_lin_vel_truth
        self._vt.append(float("nan") if vtf is None else float(np.linalg.norm(vtf)))
        self._scan = np.asarray(frame.height_scan, dtype=np.float64).reshape(7, 5)
        self._phase = np.asarray(frame.phase, dtype=np.float64).reshape(-1)[:4]
        self._act = np.asarray(frame.action, dtype=np.float64).reshape(-1)[:12]
        self._level = frame.level_name
        self._mode = frame.mode
        self._sources = dict(frame.sources)
        self._rtf = frame.rtf
        # 滚动窗口
        if self._t:
            t_end = self._t[-1]
            while self._t and (t_end - self._t[0]) > self.window_s:
                self._t.popleft(); self._tau.popleft(); self._z.popleft()
                self._tilt.popleft(); self._ve.popleft(); self._vt.popleft()

    def pump(self, force: bool = False) -> float:
        """画一帧。返回耗时 (s)。调用方按自己的频率调，别每拍都调。"""
        if not self._t:
            return 0.0
        t0 = time.perf_counter()
        t = np.asarray(self._t)
        tau = np.asarray(self._tau)
        for i, ln in enumerate(self._tau_lines):
            ln.set_data(t, tau[:, i])
        self.ax_tau.set_xlim(t[0], max(t[-1], t[0] + 1e-3))
        m = float(np.abs(tau).max()) * 1.15 + 1e-6
        self.ax_tau.set_ylim(-m, m)

        self._h_z.set_data(t, np.asarray(self._z))
        self._h_t.set_data(t, np.asarray(self._tilt))
        self.ax_h.set_xlim(t[0], max(t[-1], t[0] + 1e-3))
        self.ax_h.set_ylim(0, max(0.6, float(np.max(self._z)) * 1.3,
                                  float(np.max(self._tilt)) * 1.1 + 1e-3))
        self.ax_h.set_title("躯干高度 / 倾斜   z=%.3f m  tilt=%.1f°  %s / %s"
                            % (self._z[-1], self._tilt[-1], self._level, self._mode), fontsize=9)

        ve = np.asarray(self._ve)
        vt = np.asarray(self._vt)
        err = np.abs(ve - vt)
        self._v_ex.set_data(t, ve)
        self._v_tr.set_data(t, vt)
        self._v_ez.set_data(t, err)
        self.ax_v.set_xlim(t[0], max(t[-1], t[0] + 1e-3))
        vm = max(0.3, float(np.nanmax(ve)) * 1.3, float(np.nanmax(vt)) * 1.3)
        self.ax_v.set_ylim(0, vm)
        self.ax_v.set_title("基座线速度 估计=%.3f 真值=%.3f 误差=%.3f m/s"
                            % (ve[-1], vt[-1], err[-1]), fontsize=9)

        self._scan_img.set_data(self._scan)
        self._scan_img.set_clim(-1.75, 1.75)
        self.ax_scan.set_title("height scan 7x5  min=%.2f max=%.2f（负=上坡）"
                               % (float(self._scan.min()), float(self._scan.max())), fontsize=9)

        ph = np.asarray(self._phase)
        for b, v in zip(self._ph_cos, np.cos(ph)):
            b.set_width(float(v))
        for b, v in zip(self._ph_sin, np.sin(ph)):
            b.set_width(float(v))

        act = self._act
        for b, v in zip(self._act_bars, act):
            b.set_height(float(v))
        self.ax_act.set_ylim(min(-0.2, float(act.min()) * 1.2), max(0.2, float(act.max()) * 1.2))
        self.ax_act.set_title("策略动作  mean|a|=%.3f  max|a|=%.3f  rtf=%.2f"
                              % (float(np.abs(act).mean()), float(np.abs(act).max()), self._rtf), fontsize=9)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        dt = time.perf_counter() - t0
        self.n_draw += 1
        self.t_draw += dt
        return dt

    def stats(self) -> Dict[str, object]:
        return {"window": "matplotlib/QtAgg", "n_draw": self.n_draw,
                "draw_ms_mean": round(self.t_draw / max(1, self.n_draw) * 1e3, 1),
                "figsize": list(self.fig.get_size_inches()), "dpi": self.fig.dpi,
                "window_s": self.window_s}

    def close(self) -> None:
        try:
            self._plt.close(self.fig)
        except Exception:
            pass


__all__ = ["LivePlots", "available", "JOINT_NAMES", "LEG_COLORS"]
