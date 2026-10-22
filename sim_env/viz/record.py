#!/usr/bin/env python3
"""记录与终端仪表盘：没有图形界面时的兜底 + 落盘证据。

两个组件：

    Recorder           每帧一行 CSV + 可选关键帧 PNG；close() 写 summary.png 与 report.md
    TerminalDashboard  纯字符串 + ANSI 转义原地刷新，不依赖任何 GUI（无 tkinter/Qt/GL/display）

为什么要有 TerminalDashboard：用户可能在没有 DISPLAY 的机器上跑观察脚本（ssh、CI、容器）。
这时 MuJoCo 窗口开不出来，但闭环到底有没有跑对必须还能看见，所以这里是纯 str 拼接 +
ANSI 转义。两条硬约束：

  1. render() 每次返回固定行数的字符串，并显式 \033[H\033[J 清屏；
  2. 行内按显示列宽（东亚宽字符算 2 列）截断/补齐，绝不因为某行太长而折行 ——
     一旦折行就会滚屏，把历史输出冲掉，等于没有仪表盘。

落盘部分是能降级的：matplotlib 缺失时 summary.png 退化成一张合法的小占位 PNG，
report.md 永远写；任何渲染异常都记进 warnings，close() 不抛异常。
"""

from __future__ import annotations

import csv
import math
import os
import re
import struct
import sys
import time
import unicodedata
import zlib
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - 只给类型检查器看，运行时不 import frame
    from .frame import VizFrame

__all__ = [
    "Recorder", "TerminalDashboard", "CSV_COLUMNS", "TAU_COLUMNS", "TAU_LABELS",
    "LEG_ORDER", "JOINT_NAMES", "tilt_deg", "display_width",
]

LEG_ORDER: Tuple[str, ...] = ("FR", "FL", "RR", "RL")
JOINT_NAMES: Tuple[str, ...] = ("hip", "thigh", "calf")
TAU_LABELS: Tuple[str, ...] = tuple("%s-%s" % (leg, j) for leg in LEG_ORDER for j in JOINT_NAMES)
TAU_COLUMNS: Tuple[str, ...] = tuple("tau_%s_%s" % (leg, j) for leg in LEG_ORDER for j in JOINT_NAMES)

#: CSV 表头（顺序即列顺序，永远不要改顺序，只允许在末尾追加）
CSV_COLUMNS: Tuple[str, ...] = (
    "t", "wall", "z", "tilt", "level", "mode", "obs_valid",
    "vx_est", "vy_est", "vz_est",
    "vx_truth", "vy_truth", "vz_truth", "vx_err",
) + TAU_COLUMNS + (
    "tau_max_abs",
    "action_mean", "action_max", "action_rms",
    "scan_mean", "scan_min", "scan_max",
    "foot_0", "foot_1", "foot_2", "foot_3",
    "rtf", "overruns", "n_cmd", "n_state",
    # 末尾追加 (v2 诊断): 姿态角与**机体系**速度, 用来区分"航向漂移"和"横向滑移"
    "roll", "pitch", "yaw", "vx_body", "vy_body", "vz_body",
)


# --------------------------------------------------------------------------- 小工具
def _f(v, default: float = 0.0) -> float:
    """宽松取 float：None/字符串/NaN/Inf 一律回落到 default。"""
    try:
        x = float(v)
    except (TypeError, ValueError):
        return float(default)
    return x if math.isfinite(x) else float(default)


def _vec(v, n: int, default: float = 0.0) -> np.ndarray:
    """取成定长 (n,) float64：短了补 default，长了截断，非有限值替换成 default。"""
    if v is None:
        return np.full(n, float(default), dtype=np.float64)
    a = np.asarray(v, dtype=np.float64).reshape(-1)
    out = np.full(n, float(default), dtype=np.float64)
    k = min(n, a.size)
    if k:
        out[:k] = np.nan_to_num(a[:k], nan=float(default),
                                posinf=float(default), neginf=float(default))
    return out


def _vec_any(v, default: float = 0.0) -> np.ndarray:
    """取成**任意长度**的有限值数组（不截断）; None/空 -> 长度 1 的 [default]。

    v6: 大扫描档有 187 个点, 统计量 (mean/min/max) 要用整条而不是前 35 个。
    """
    if v is None:
        return np.full(1, float(default), dtype=np.float64)
    a = np.asarray(v, dtype=np.float64).reshape(-1)
    if a.size == 0:
        return np.full(1, float(default), dtype=np.float64)
    return np.nan_to_num(a, nan=float(default), posinf=float(default),
                         neginf=float(default))


def tilt_deg(rpy) -> float:
    """roll/pitch 合成的倾角（度）：arccos(cos(roll)*cos(pitch))，平地 = 0。"""
    r = _vec(rpy, 3)
    c = max(-1.0, min(1.0, math.cos(r[0]) * math.cos(r[1])))
    return math.degrees(math.acos(c))


def _sign(x, nd: int = 3) -> str:
    x = None if x is None else float(x)
    if x is None or not math.isfinite(x):
        return "n/a"
    return "%+.*f" % (nd, x)


def _clean(v):
    """CSV 友好：numpy 标量 -> python 标量；float 保留 6 位小数。"""
    if isinstance(v, (bool, np.bool_)):
        return int(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return round(float(v), 6)
    return v


# --------------------------------------------------------------------------- 显示宽度
_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _char_width(ch: str) -> int:
    if unicodedata.combining(ch):
        return 0
    if unicodedata.east_asian_width(ch) in ("W", "F"):
        return 2
    return 1


def display_width(s: str) -> int:
    """去掉 ANSI 转义后的终端显示列宽（东亚宽/全角字符按 2 列算）。"""
    return sum(_char_width(c) for c in _ANSI_RE.sub("", s))


def _cut_cols(s: str, cols: int) -> str:
    out: List[str] = []
    used = 0
    for ch in s:
        cw = _char_width(ch)
        if used + cw > cols:
            break
        out.append(ch)
        used += cw
    return "".join(out)


_STYLES = {
    "reset": "\033[0m", "bold": "\033[1m", "dim": "\033[2m",
    "red": "\033[31m", "green": "\033[32m", "yellow": "\033[33m",
    "blue": "\033[34m", "magenta": "\033[35m", "cyan": "\033[36m",
    "gray": "\033[90m", "bright_red": "\033[91m", "bright_green": "\033[92m",
    "bright_yellow": "\033[93m", "bright_cyan": "\033[96m",
}
_LEVEL_STYLE = {0: "bright_green", 1: "bright_yellow", 2: "red", 3: "bright_red"}
_LEVEL_NAME = {0: "正常", 1: "降级", 2: "保护", 3: "急停"}

CLEAR = "\033[H\033[J"


def _pick_cjk_font() -> Optional[str]:
    """找一个能画中文的字体；没有就返回 None（调用方改用英文标题，避免豆腐块）。"""
    try:
        from matplotlib import font_manager
        have = {f.name for f in font_manager.fontManager.ttflist}
    except Exception:
        return None
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Sans SC",
                 "Source Han Sans SC", "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
                 "Droid Sans Fallback", "AR PL UMing CN"):
        if cand in have:
            return cand
    return None


# =========================================================================== Recorder
class Recorder:
    """落盘：CSV + 关键帧 PNG + 结尾汇总图。

        rec = Recorder("logs/viz", fps=10.0)
        for frame in stream:
            rec.write(frame, render_fn=viewer.render_rgb)   # render_fn 返回 HxWx3 uint8 就存帧
        info = rec.close(summary={"duration_s": 60.0})

    产物（都在 out_dir 下）：
        frames.csv   每帧一行，列见 module 级 CSV_COLUMNS
        frames/      关键帧 PNG（最多 fps 张/秒；save_frames=False 时不建）
        summary.png  close() 时画的汇总图
        report.md    close() 时写的文字报告
    """

    def __init__(self, out_dir: str, fps: float = 10.0, save_frames: bool = True,
                 flush_every: int = 1):
        self.out_dir = os.path.abspath(str(out_dir))
        os.makedirs(self.out_dir, exist_ok=True)
        self.frames_dir = os.path.join(self.out_dir, "frames")
        self.csv_path = os.path.join(self.out_dir, "frames.csv")
        self.summary_png = os.path.join(self.out_dir, "summary.png")
        self.report_md = os.path.join(self.out_dir, "report.md")

        self.fps = max(1e-6, _f(fps, 10.0))
        self.save_frames = bool(save_frames)
        self.flush_every = max(1, int(flush_every))
        self.warnings: List[str] = []

        self._fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(CSV_COLUMNS)

        self._rows = 0
        self._saved = 0
        self._last_kf_t: Optional[float] = None
        self._closed = False
        self._result: Optional[dict] = None

        self._series: Dict[str, List[float]] = {
            k: [] for k in ("t", "z", "tilt", "vx_est", "vx_truth", "vx_err",
                            "tau_max", "action_max", "level")
        }
        self._rtf: List[float] = []
        self._overruns_max = 0
        self._last_sources: Dict[str, str] = {}
        self._last_level = 0

    # -------------------------------------------------- 属性
    @property
    def rows(self) -> int:
        return self._rows

    @property
    def frames_saved(self) -> int:
        return self._saved

    @property
    def closed(self) -> bool:
        return self._closed

    def flush(self) -> None:
        """把 CSV 缓冲刷到磁盘（默认每写一行就刷，便于运行中 tail）。"""
        if not self._closed:
            self._fh.flush()

    # -------------------------------------------------- 写一帧
    def write(self, frame, render_fn: Optional[Callable[[], np.ndarray]] = None) -> None:
        """写一行 CSV；给了 render_fn 且到关键帧时刻就存一张 PNG。"""
        if self._closed:
            raise RuntimeError("Recorder 已关闭，不能再 write()")
        self._w.writerow([_clean(x) for x in self._row_of(frame)])
        self._rows += 1
        if self._rows % self.flush_every == 0:
            self._fh.flush()

        t = _f(getattr(frame, "t", 0.0))
        rpy = _vec(getattr(frame, "rpy", None), 3)
        pos = _vec(getattr(frame, "base_pos", None), 3)
        v_est = _vec(getattr(frame, "base_lin_vel_world", None), 3)
        v_raw = getattr(frame, "base_lin_vel_truth", None)
        tau = _vec(getattr(frame, "joint_tau", None), 12)
        act = _vec(getattr(frame, "action", None), 12)

        s = self._series
        s["t"].append(t)
        s["z"].append(float(pos[2]))
        s["tilt"].append(tilt_deg(rpy))
        s["vx_est"].append(float(v_est[0]))
        if v_raw is None:
            s["vx_truth"].append(float("nan"))
            s["vx_err"].append(float("nan"))
        else:
            vt = _vec(v_raw, 3)
            s["vx_truth"].append(float(vt[0]))
            s["vx_err"].append(float(v_est[0] - vt[0]))
        s["tau_max"].append(float(np.abs(tau).max()))
        s["action_max"].append(float(np.abs(act).max()))
        s["level"].append(float(getattr(frame, "level", 0)))
        self._rtf.append(_f(getattr(frame, "rtf", 0.0)))
        self._overruns_max = max(self._overruns_max, int(_f(getattr(frame, "overruns", 0))))
        self._last_sources = dict(getattr(frame, "sources", {}) or {})
        self._last_level = int(_f(getattr(frame, "level", 0)))

        if render_fn is not None and self.save_frames:
            due = self._last_kf_t is None or (t - self._last_kf_t) >= (1.0 / self.fps - 1e-9)
            if due:
                img = None
                try:
                    img = render_fn()
                except Exception as exc:  # 渲染失败不能打断记录
                    self.warnings.append("render_fn 异常: %r" % (exc,))
                if img is not None:
                    if self._save_png(img, self._saved):
                        self._saved += 1
                        self._last_kf_t = t

    # -------------------------------------------------- 行内容
    def _row_of(self, frame) -> List:
        pos = _vec(getattr(frame, "base_pos", None), 3)
        rpy = _vec(getattr(frame, "rpy", None), 3)
        v_est = _vec(getattr(frame, "base_lin_vel_world", None), 3)
        v_raw = getattr(frame, "base_lin_vel_truth", None)
        tau = _vec(getattr(frame, "joint_tau", None), 12)
        act = _vec(getattr(frame, "action", None), 12)
        scan = _vec_any(getattr(frame, "height_scan", None))
        foot = _vec(getattr(frame, "foot_force", None), 4)

        if v_raw is None:
            vt = np.full(3, float("nan"))
            vx_err = float("nan")
        else:
            vt = _vec(v_raw, 3)
            vx_err = float(v_est[0] - vt[0])

        row: List = [
            _f(getattr(frame, "t", 0.0)), _f(getattr(frame, "wall", 0.0)),
            float(pos[2]), tilt_deg(rpy),
            int(_f(getattr(frame, "level", 0))), str(getattr(frame, "mode", "-")),
            int(bool(getattr(frame, "obs_valid", True))),
            float(v_est[0]), float(v_est[1]), float(v_est[2]),
            float(vt[0]), float(vt[1]), float(vt[2]), vx_err,
        ]
        row += [float(x) for x in tau]
        row += [
            float(np.abs(tau).max()),
            float(act.mean()), float(np.abs(act).max()), float(np.sqrt(np.mean(act ** 2))),
            float(scan.mean()), float(scan.min()), float(scan.max()),
        ]
        row += [float(x) for x in foot]
        row += [
            _f(getattr(frame, "rtf", 0.0)), int(_f(getattr(frame, "overruns", 0))),
            int(_f(getattr(frame, "n_cmd", 0))), int(_f(getattr(frame, "n_state", 0))),
        ]
        # v2 诊断: 姿态角 + 机体系真值速度 (qvel[0:3] 是世界系, 用 base_quat 转机体系)
        quat = _vec(getattr(frame, "base_quat", None), 4, default=0.0)
        if not np.any(quat):
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        nq = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        w, x, y, z = w / nq, x / nq, y / nq, z / nq
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        v_body = R.T @ np.asarray(vt, dtype=np.float64)
        row += [float(rpy[0]), float(rpy[1]), float(rpy[2]),
                float(v_body[0]), float(v_body[1]), float(v_body[2])]
        return row

    # -------------------------------------------------- 关键帧
    def _save_png(self, img, index: int) -> bool:
        a = np.asarray(img)
        if a.dtype != np.uint8:
            m = float(np.nanmax(a)) if a.size else 0.0
            a = np.clip(a * (255.0 if m <= 1.0 else 1.0), 0, 255).astype(np.uint8)
        if a.ndim == 3 and a.shape[2] == 4:
            a = a[:, :, :3]
        if a.ndim != 3 or a.shape[2] != 3 or a.shape[0] < 1 or a.shape[1] < 1:
            self.warnings.append("关键帧 #%d 形状不支持: %r" % (index, a.shape))
            return False
        os.makedirs(self.frames_dir, exist_ok=True)
        path = os.path.join(self.frames_dir, "frame_%05d.png" % index)
        try:
            from PIL import Image  # 首选：快，且不需要 plt
            Image.fromarray(a).save(path)
            return True
        except Exception:
            pass
        try:
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.image as mpimg
            mpimg.imsave(path, a)
            return True
        except Exception as exc:
            self.warnings.append("关键帧 #%d 保存失败: %r" % (index, exc))
            return False

    # -------------------------------------------------- 汇总
    def close(self, summary: Optional[dict] = None) -> dict:
        """收尾：关 CSV、写 summary.png + report.md，返回产物与统计。可重复调用。"""
        if self._closed:
            return dict(self._result or {})
        self._closed = True
        try:
            self._fh.flush()
        finally:
            self._fh.close()

        stats = self._stats()
        png_ok = self._write_summary_png(summary or {})
        self._write_report(summary or {}, stats)
        result = {
            "out_dir": self.out_dir,
            "csv": self.csv_path,
            "rows": int(self._rows),
            "frames_dir": self.frames_dir if self._saved else None,
            "frames_saved": int(self._saved),
            "summary_png": self.summary_png,
            "report_md": self.report_md,
            "summary_png_ok": bool(png_ok),
            "stats": stats,
            "sources": dict(self._last_sources),
            "level": int(self._last_level),
            "warnings": list(self.warnings),
        }
        if summary:
            result["summary"] = dict(summary)
        self._result = result
        return dict(result)

    @staticmethod
    def _safe_stat(a, fn, default: float = float("nan")) -> float:
        a = np.asarray(a, dtype=np.float64)
        if a.size == 0:
            return float(default)
        try:
            return float(fn(a))
        except Exception:
            return float(default)

    def _stats(self) -> dict:
        s = self._series
        t = np.asarray(s["t"], dtype=np.float64)
        z = np.asarray(s["z"], dtype=np.float64)
        tilt = np.asarray(s["tilt"], dtype=np.float64)
        vx = np.asarray(s["vx_est"], dtype=np.float64)
        err = np.asarray(s["vx_err"], dtype=np.float64)
        finite = err[np.isfinite(err)]
        tau_max = np.asarray(s["tau_max"], dtype=np.float64)
        act_max = np.asarray(s["action_max"], dtype=np.float64)
        lv = np.asarray(s["level"], dtype=np.float64)
        rtf = np.asarray(self._rtf, dtype=np.float64)
        hist = {k: int((lv == k).sum()) for k in range(4)} if lv.size else {k: 0 for k in range(4)}
        return {
            "rows": int(self._rows),
            "frames_saved": int(self._saved),
            "duration_s": float(t[-1] - t[0]) if t.size >= 2 else 0.0,
            "z_mean": self._safe_stat(z, np.mean),
            "z_min": self._safe_stat(z, np.min),
            "z_max": self._safe_stat(z, np.max),
            "tilt_max_deg": self._safe_stat(tilt, np.max),
            "tilt_rms_deg": self._safe_stat(tilt, lambda a: np.sqrt(np.mean(a ** 2))),
            "vx_est_mean": self._safe_stat(vx, np.mean),
            "vx_est_final": float(vx[-1]) if vx.size else float("nan"),
            "vx_err_mean_abs": float(np.abs(finite).mean()) if finite.size else float("nan"),
            "vx_err_max_abs": float(np.abs(finite).max()) if finite.size else float("nan"),
            "tau_absmax": self._safe_stat(tau_max, np.max),
            "action_absmax": self._safe_stat(act_max, np.max),
            "rtf_mean": self._safe_stat(rtf, np.mean),
            "overruns": int(self._overruns_max),
            "level_hist": hist,
            "level_last": int(lv[-1]) if lv.size else 0,
        }

    def _write_summary_png(self, summary: dict) -> bool:
        try:
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt
            _font = _pick_cjk_font()
            _zh = _font is not None
            if _font is not None:
                plt.rcParams["font.sans-serif"] = [_font, "DejaVu Sans"]
                plt.rcParams["axes.unicode_minus"] = False
        except Exception as exc:
            self.warnings.append("matplotlib 不可用，summary.png 退化为占位图: %r" % (exc,))
            return self._minimal_png_or_fail()

        try:
            s = self._series
            t = np.asarray(s["t"], dtype=np.float64)
            z = np.asarray(s["z"], dtype=np.float64)
            tilt = np.asarray(s["tilt"], dtype=np.float64)
            vx = np.asarray(s["vx_est"], dtype=np.float64)
            vxt = np.asarray(s["vx_truth"], dtype=np.float64)
            err = np.asarray(s["vx_err"], dtype=np.float64)
            tau_max = np.asarray(s["tau_max"], dtype=np.float64)
            act_max = np.asarray(s["action_max"], dtype=np.float64)
            lv = np.asarray(s["level"], dtype=np.float64)

            fig, ax = plt.subplots(3, 2, figsize=(11.0, 9.0), sharex=True)
            ax[0][0].plot(t, z, color="tab:blue", lw=1.2)
            ax[0][0].set_ylabel("z [m]")
            ax[0][0].set_title("躯干高度" if _zh else "trunk height z")
            ax[0][0].grid(alpha=0.3)

            ax[0][1].plot(t, tilt, color="tab:red", lw=1.2)
            ax[0][1].set_ylabel("tilt [deg]")
            ax[0][1].set_title("倾角" if _zh else "tilt (roll/pitch)")
            ax[0][1].grid(alpha=0.3)

            ax[1][0].plot(t, vx, color="tab:blue", lw=1.2, label=("估计 vx" if _zh else "est vx"))
            if vxt.size and np.isfinite(vxt).any():
                ax[1][0].plot(t, vxt, color="tab:orange", lw=1.2, label=("真值 vx" if _zh else "truth vx"))
            ax[1][0].set_ylabel("vx [m/s]")
            ax[1][0].set_title("基座前向速度" if _zh else "base forward velocity")
            ax[1][0].legend(fontsize=8)
            ax[1][0].grid(alpha=0.3)

            ax[1][1].plot(t, err, color="tab:purple", lw=1.2)
            ax[1][1].axhline(0.0, color="k", lw=0.6, alpha=0.5)
            ax[1][1].set_ylabel("vx_err [m/s]")
            ax[1][1].set_title("速度估计误差 (est - truth)" if _zh else "vx error (est - truth)")
            ax[1][1].grid(alpha=0.3)

            ax[2][0].plot(t, tau_max, color="tab:green", lw=1.2, label="|tau|max")
            ax[2][0].plot(t, act_max, color="tab:gray", lw=1.0, label="|action|max")
            ax[2][0].set_ylabel("N*m / -")
            ax[2][0].set_title("力矩与动作幅值" if _zh else "|tau|max and |action|max")
            ax[2][0].legend(fontsize=8)
            ax[2][0].grid(alpha=0.3)

            if lv.size:
                ax[2][1].step(t, lv, where="post", color="tab:red", lw=1.4)
            ax[2][1].set_ylim(-0.2, 3.4)
            ax[2][1].set_yticks([0, 1, 2, 3])
            ax[2][1].set_ylabel("level")
            ax[2][1].set_title("健康等级" if _zh else "health level")
            ax[2][1].grid(alpha=0.3)
            for a in ax[2]:
                a.set_xlabel("t [s]")

            if _zh:
                head = "Go2 观察汇总 · %d 帧 · %.1f s" % (
                    self._rows, (t[-1] - t[0]) if t.size >= 2 else 0.0)
            else:
                head = "Go2 summary | %d frames | %.1f s" % (
                    self._rows, (t[-1] - t[0]) if t.size >= 2 else 0.0)
            extra = ", ".join("%s=%s" % (k, summary[k]) for k in sorted(summary))
            if extra:
                head += "  |  " + extra[:120]
            fig.suptitle(head, fontsize=10)
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            fig.savefig(self.summary_png, dpi=110)
            plt.close(fig)
            return True
        except Exception as exc:
            self.warnings.append("summary.png 渲染失败，退化为占位图: %r" % (exc,))
            return self._minimal_png_or_fail()

    def _minimal_png_or_fail(self) -> bool:
        try:
            _write_minimal_png(self.summary_png)
            return False
        except Exception as exc:
            self.warnings.append("占位 PNG 也失败: %r" % (exc,))
            return False

    def _write_report(self, summary: dict, stats: dict) -> None:
        def num(x, nd: int = 4) -> str:
            x = float(x)
            return "n/a" if not math.isfinite(x) else ("%%.%dg" % nd) % x

        lines: List[str] = []
        lines.append("# 观察记录报告")
        lines.append("")
        lines.append("- 输出目录: " + self.out_dir)
        lines.append("- CSV: %s（%d 行数据 + 1 行表头，共 %d 列）"
                     % (os.path.basename(self.csv_path), self._rows, len(CSV_COLUMNS)))
        if self._saved:
            frames_txt = "frames/ 共 %d 张" % self._saved
        elif not self.save_frames:
            frames_txt = "未启用（save_frames=False）"
        else:
            frames_txt = "无"
        lines.append("- 关键帧: " + frames_txt)
        lines.append("- 汇总图: " + os.path.basename(self.summary_png))
        lines.append("")
        if summary:
            lines.append("## 调用方汇总 summary")
            lines.append("")
            lines.append("| key | value |")
            lines.append("| --- | --- |")
            for k in sorted(summary):
                lines.append("| %s | %s |" % (k, summary[k]))
            lines.append("")
        lines.append("## 统计")
        lines.append("")
        lines.append("| 指标 | 值 |")
        lines.append("| --- | --- |")
        for k in ("rows", "frames_saved", "duration_s", "z_mean", "z_min", "z_max",
                  "tilt_max_deg", "tilt_rms_deg", "vx_est_mean", "vx_est_final",
                  "vx_err_mean_abs", "vx_err_max_abs", "tau_absmax", "action_absmax",
                  "rtf_mean", "overruns", "level_last"):
            v = stats.get(k, float("nan"))
            lines.append("| %s | %s |" % (k, num(v) if isinstance(v, float) else v))
        lines.append("")
        hist = stats.get("level_hist", {})
        lines.append("等级停留帧数: " + ", ".join("L%d=%d" % (k, hist.get(k, 0)) for k in range(4)))
        lines.append("")
        lines.append("## CSV 列")
        lines.append("")
        lines.append(", ".join(CSV_COLUMNS))
        lines.append("")
        lines.append("## 每路来源 sources")
        lines.append("")
        if self._last_sources:
            for k in sorted(self._last_sources):
                lines.append("- %s = %s" % (k, self._last_sources[k]))
        else:
            lines.append("- （调用方未提供）")
        lines.append("")
        lines.append("## 降级 / 告警")
        lines.append("")
        if self.warnings:
            for wn in self.warnings:
                lines.append("- " + wn)
        else:
            lines.append("- 无")
        lines.append("")
        with open(self.report_md, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")


def _write_minimal_png(path: str, w: int = 4, h: int = 4,
                       rgb: Tuple[int, int, int] = (240, 240, 240)) -> None:
    """不依赖任何图像库，手写一张合法的 RGB PNG（降级占位用）。"""
    row = b"\x00" + bytes(rgb) * int(w)
    raw = row * int(h)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", int(w), int(h), 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)


# ==================================================================== TerminalDashboard
class TerminalDashboard:
    """无界面时的替代：ANSI 原地刷新（ESC[H ESC[J），10 Hz，不需要任何 GUI。

        dash = TerminalDashboard(width=100)
        while running:
            if dash.due():
                sys.stdout.write(dash.render(frame))
        dash.write(frame)          # 等价于 due()+render()+flush 的便捷写法

    行数固定（line_count = 26：12 行信息 + 1 行 tau 表头 + 12 条 tau + 1 行页脚），
    每行按显示列宽截断/补齐到 width，所以终端不会因为折行而滚屏。
    """

    N_TAU = 12

    def __init__(self, width: int = 100, use_color: bool = True, clear_screen: bool = True,
                 fps: float = 10.0, tau_scale: float = 20.0, contact_thresh: float = 1.0,
                 bar_width: Optional[int] = None):
        self.width = max(40, int(width))
        self.use_color = bool(use_color)
        self.clear_screen = bool(clear_screen)
        self.fps = max(1e-6, _f(fps, 10.0))
        self.tau_scale = _f(tau_scale, 20.0)
        self.contact_thresh = _f(contact_thresh, 1.0)
        self.bar_width = int(bar_width) if bar_width else max(8, min(24, self.width // 4))
        # 12 行信息 + 1 行 tau 表头 + 12 条 tau + 1 行页脚
        self.line_count = 12 + 1 + self.N_TAU + 1
        self._last_emit: Optional[float] = None

    # -------------------------------------------------- 限频
    def due(self, now: Optional[float] = None) -> bool:
        """距上次 write() 是否已过一个刷新周期（默认 10 Hz）。"""
        now = time.monotonic() if now is None else _f(now, time.monotonic())
        if self._last_emit is None:
            return True
        return (now - self._last_emit) >= (1.0 / self.fps - 1e-9)

    def write(self, frame, extra: Optional[dict] = None, stream=None,
              now: Optional[float] = None) -> bool:
        """到点就原地重画一屏，返回是否真的写了。"""
        if not self.due(now):
            return False
        stream = sys.stdout if stream is None else stream
        stream.write(self.render(frame, extra=extra))
        try:
            stream.flush()
        except Exception:
            pass
        self._last_emit = time.monotonic() if now is None else _f(now, time.monotonic())
        return True

    # -------------------------------------------------- 行装配
    def _fit(self, segs, width: int):
        out: List[Tuple[str, Optional[str]]] = []
        used = 0
        for text, style in segs:
            text = "" if text is None else str(text)
            if not text:
                continue
            remain = width - used
            if remain <= 0:
                break
            cut = _cut_cols(text, remain)
            if cut:
                out.append((cut, style))
                used += display_width(cut)
        return out

    def _render_line(self, segs) -> str:
        segs = self._fit(segs, self.width)
        used = sum(display_width(t) for t, _ in segs)
        parts: List[str] = []
        for text, style in segs:
            if self.use_color and style:
                parts.append(_STYLES.get(style, "") + text + _STYLES["reset"])
            else:
                parts.append(text)
        parts.append(" " * max(0, self.width - used))
        return "".join(parts)

    def _bar(self, value: float, scale: float) -> str:
        """零居中的横向条：正 = 右侧 '='，负 = 左侧 '-'，中间 '|'。"""
        n = max(2, self.bar_width)
        half = max(1, n // 2)
        ratio = 0.0 if scale <= 0 else float(np.clip(float(value) / float(scale), -1.0, 1.0))
        pos = int(round(abs(ratio) * half))
        if ratio >= 0.0:
            return " " * half + "|" + "=" * pos + " " * (half - pos)
        return " " * (half - pos) + "-" * pos + "|" + " " * half

    def _lines(self, frame, extra: dict) -> List[List[Tuple[str, Optional[str]]]]:
        level = int(_f(getattr(frame, "level", 0)))
        lname = str(getattr(frame, "level_name", "") or ("L%d" % level))
        mode = str(getattr(frame, "mode", "-"))
        obs_valid = bool(getattr(frame, "obs_valid", True))
        pos = _vec(getattr(frame, "base_pos", None), 3)
        rpy = _vec(getattr(frame, "rpy", None), 3)
        v_est = _vec(getattr(frame, "base_lin_vel_world", None), 3)
        v_raw = getattr(frame, "base_lin_vel_truth", None)
        v_true = None if v_raw is None else _vec(v_raw, 3)
        tau = _vec(getattr(frame, "joint_tau", None), 12)
        act = _vec(getattr(frame, "action", None), 12)
        scan = _vec_any(getattr(frame, "height_scan", None))
        foot = _vec(getattr(frame, "foot_force", None), 4)
        phase = _vec(getattr(frame, "phase", None), 4)
        obs = _vec_any(getattr(frame, "obs", None))
        sources = dict(getattr(frame, "sources", {}) or {})
        if extra.get("sources"):
            try:
                sources.update(dict(extra["sources"]))
            except Exception:
                pass
        thr = _f(extra.get("contact_thresh", self.contact_thresh), self.contact_thresh)
        scale = _f(extra.get("tau_scale", self.tau_scale), self.tau_scale)
        clip = int(_f(extra.get("clip_count", extra.get("action_clipped", 0))))
        note = str(extra.get("note", "") or "")
        t = _f(getattr(frame, "t", 0.0))
        wall = _f(getattr(frame, "wall", 0.0))
        lstyle = _LEVEL_STYLE.get(level, "gray")

        contact = " ".join("%s%s" % (LEG_ORDER[i], "*" if foot[i] > thr else ".")
                           for i in range(4))
        forces = " ".join("%6.1f" % foot[i] for i in range(4))
        if v_true is None:
            vx_line = "vx   估计 %s   真值 n/a           误差 n/a" % _sign(v_est[0])
            vyvz = "vy/vz 估计 %s / %s   真值 n/a" % (_sign(v_est[1]), _sign(v_est[2]))
        else:
            vx_line = ("vx   估计 %s   真值 %s   误差 %s m/s"
                       % (_sign(v_est[0]), _sign(v_true[0]), _sign(v_est[0] - v_true[0])))
            vyvz = ("vy/vz 估计 %s / %s   真值 %s / %s"
                    % (_sign(v_est[1]), _sign(v_est[2]), _sign(v_true[1]), _sign(v_true[2])))
        phase_s = " ".join("%s %+.2f" % (LEG_ORDER[i], phase[i]) for i in range(4))
        if sources:
            src_s = "来源 " + " ".join("%s=%s" % (k, sources[k]) for k in sorted(sources))
        else:
            src_s = "来源 （调用方未提供）"
        if note:
            src_s += "  |  " + note

        L: List[List[Tuple[str, Optional[str]]]] = []
        L.append([("Go2 终端仪表盘", "bold"), ("   headless / no-GUI fallback", "dim")])
        L.append([("t %-9s wall %-9s rtf %6.3f    overrun %-5d n_cmd %-7d n_state %-7d"
                   % ("%.3fs" % t, "%.3fs" % wall, _f(getattr(frame, "rtf", 0.0)),
                      int(_f(getattr(frame, "overruns", 0))),
                      int(_f(getattr(frame, "n_cmd", 0))),
                      int(_f(getattr(frame, "n_state", 0)))), None)])
        L.append([("健康 ", None), ("[%s %s]" % (lname, _LEVEL_NAME.get(level, "?")), lstyle),
                  ("   模式 ", None), (mode, "cyan"),
                  ("   obs_valid=", None), (str(obs_valid), "green" if obs_valid else "red")])
        L.append([("姿态  tilt %+7.2f°   z %6.3f m   |v_est| %6.3f m/s"
                   % (tilt_deg(rpy), float(pos[2]), float(np.linalg.norm(v_est))), None)])
        L.append([(vx_line, None)])
        L.append([(vyvz, None)])
        L.append([("接触  ", None), (contact, "green"), ("   力[N] ", None), (forces, None)])
        L.append([("动作  mean %s  max %.3f  rms %.3f   clip %d"
                   % (_sign(act.mean()), float(np.abs(act).max()),
                      float(np.sqrt(np.mean(act ** 2))), clip), None)])
        L.append([("高度特征  mean %s  min %s  max %s   (负=地面更高)"
                   % (_sign(scan.mean()), _sign(scan.min()), _sign(scan.max())), None)])
        L.append([("obs %d 维 = base48 + scan%d + gait8   |obs| %.3f"
                   % (obs.size, max(int(obs.size) - 48 - 8, 0),
                      float(np.linalg.norm(obs))), None)])
        L.append([("相位  " + phase_s, "magenta")])
        L.append([(src_s, "dim")])
        L.append([("tau [N*m]   左 '-' = 负   右 '=' = 正   量程 +-%.1f" % scale, "bold")])
        for i in range(self.N_TAU):
            label = "tau %-3s %-5s" % (LEG_ORDER[i // 3], JOINT_NAMES[i % 3])
            style = "cyan" if tau[i] >= 0 else "magenta"
            L.append([(label + " ", None), ("%+7.2f" % tau[i], style),
                      (" [", "dim"), (self._bar(tau[i], scale), style), ("]", "dim")])
        L.append([("10 Hz 原地刷新 | ANSI \\033[H\\033[J | 固定 %d 行 | 无 GUI 依赖"
                   % self.line_count, "dim")])
        return L

    # -------------------------------------------------- 出屏
    def render(self, frame, extra: Optional[dict] = None) -> str:
        """返回一整屏字符串：ANSI 清屏 + 固定 line_count 行（无尾随换行）。"""
        lines = self._lines(frame, dict(extra or {}))
        blank: List[Tuple[str, Optional[str]]] = [("", None)]
        while len(lines) < self.line_count:
            lines.append(blank)
        lines = lines[:self.line_count]
        body = "\n".join(self._render_line(segs) for segs in lines)
        return (CLEAR if self.clear_screen else "") + body
