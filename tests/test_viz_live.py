#!/usr/bin/env python3
"""独立曲线窗口（viz/live.py）的自测。

没有 Qt 后端时不算失败，只打印 skip —— 这个模块是"可选增强"，
但只要有 Qt 就必须能建窗口、能吃 30 帧、能关掉且不抛异常。

    python tests/test_viz_live.py
"""

from __future__ import annotations

import os
import sys
import warnings

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
warnings.filterwarnings("ignore")

OK = 0


def check(name, cond, detail=""):
    global OK
    if cond:
        OK += 1
        print("  [OK]   %s %s" % (name, detail))
    else:
        print("  [FAIL] %s %s" % (name, detail))
        os._exit(1)


def fake(k):
    from sim_env.viz import VizFrame
    f = VizFrame(t=k * 0.02)
    f.joint_tau = np.sin(np.arange(12) + k * 0.1)
    f.base_pos = np.array([k * 0.01, 0.0, 0.29])
    f.rpy = np.array([0.02, 0.03, 0.1])
    f.base_lin_vel_world = np.array([0.5, 0.01, 0.0])
    f.base_lin_vel_truth = np.array([0.48, 0.02, 0.0])
    f.height_scan = np.linspace(-1.0, 1.0, 35)
    f.phase = np.array([0.1, 1.2, 3.3, 4.4])
    f.action = np.random.default_rng(k).standard_normal(12) * 0.5
    f.level_name, f.mode, f.rtf = "L0", "track", 1.0
    f.sources = {"linvel": "estimator", "scan": "leg_fk"}
    return f


def main() -> int:
    from sim_env.viz.live import JOINT_NAMES, LivePlots, available
    check("JOINT_NAMES 是 12 个", len(JOINT_NAMES) == 12)
    if not available():
        print("  [skip] 这台机器没有 Qt 后端 —— live.py 只在 window 模式下用到，跳过")
        print("OK test_viz_live: 跳过（无 Qt）")
        return 0

    lp = LivePlots(window_s=1.0)
    check("窗口建起来了", lp.fig is not None, "figsize=%s" % list(lp.fig.get_size_inches()))
    check("6 个 axes", len(lp.fig.axes) == 6, "实际 %d" % len(lp.fig.axes))

    for k in range(30):
        lp.update(fake(k))
    check("吃了 30 帧", len(lp._t) == 30)
    dt = lp.pump()
    check("pump() 返回耗时", isinstance(dt, float) and dt > 0, "%.1f ms" % (dt * 1e3))

    # 滚动窗口：window_s=1.0 + dt=0.02 -> 最多 ~51 个点
    for k in range(300):
        lp.update(fake(k))
    check("滚动窗口有界", len(lp._t) <= 60, "留了 %d 个点" % len(lp._t))

    st = lp.stats()
    check("stats 有 draw_ms_mean", "draw_ms_mean" in st and st["n_draw"] >= 1, str(st))
    lp.close()
    print("OK test_viz_live: 全部通过（共 %d 项）" % OK)
    # Qt 在解释器退出时可能挂住，这里直接退
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
