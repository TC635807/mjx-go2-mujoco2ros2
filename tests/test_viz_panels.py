#!/usr/bin/env python3
"""tests/test_viz_panels.py —— FigurePanel / ImagePanel / TextHud（契约第 4 节）。

直接跑:
    .venv/bin/python tests/test_viz_panels.py
不用 pytest。全部通过打印 "OK ..." 并以 0 退出，否则 SystemExit(1)。

这里刻意**不**复用任何"记忆中的" MuJoCo 字段名：MjvFigure 的真实字段先在运行时 dir()
出来核对，再断言（例如没有 figuretitle，标题字段叫 title）。
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from sim_env.viz import panels  # noqa: E402
from sim_env.viz.frame import VizFrame  # noqa: E402
from sim_env.viz.panels import (ImagePanel, FigurePanel, TextHud,  # noqa: E402
                                   JOINT_NAMES, OBS_SEGMENTS)

RESULT = []
SKIPPED = []

CONTRACT_FIELDS = (
    "t", "wall", "base_pos", "base_quat", "base_lin_vel_world", "base_lin_vel_truth",
    "rpy", "joint_q", "joint_dq", "joint_q_des", "joint_tau", "foot_pos_world",
    "foot_force", "scan_xy_body", "scan_xy_world", "height_scan", "phase", "action",
    "action_applied", "obs", "level", "level_name", "mode", "obs_valid", "sources",
    "rtf", "overruns", "n_cmd", "n_state",
)


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULT.append((name, bool(ok)))
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    return bool(ok)


def skip(name: str, why: str) -> None:
    SKIPPED.append(name)
    print("  [SKIP] %s %s" % (name, why))


def median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def make_scan_body() -> np.ndarray:
    gx = np.linspace(-0.3, 0.9, 7)
    gy = np.linspace(-0.4, 0.4, 5)
    GX, GY = np.meshgrid(gx, gy, indexing="ij")
    return np.stack([GX.ravel(), GY.ravel()], axis=-1)


def make_frame(i: int = 0, dt: float = 0.02, truth: bool = True) -> VizFrame:
    """按契约第 2 节的字段造一帧假数据（数值随 i 变化，便于断言滚动窗口）。"""
    f = VizFrame(t=i * dt, wall=100.0 + i * dt)
    f.base_pos = np.array([0.01 * i, -0.02 * i, 0.281 + 0.001 * i])
    f.base_quat = np.array([1.0, 0.0, 0.0, 0.0])
    f.base_lin_vel_world = np.array([0.5, 0.02 * np.sin(i), -0.01])
    f.base_lin_vel_truth = np.array([0.51, 0.01 * np.sin(i), -0.02]) if truth else None
    f.rpy = np.array([0.03, -0.04, 0.1])
    f.joint_q = np.linspace(-1.0, 1.0, 12) + 0.001 * i
    f.joint_dq = np.linspace(-2.0, 2.0, 12)
    f.joint_q_des = np.linspace(-0.9, 0.9, 12)
    f.joint_tau = np.linspace(-6.0, 6.0, 12) * (1.0 + 0.01 * (i % 7))
    f.foot_pos_world = np.zeros((4, 3))
    f.foot_force = np.array([12.0, 0.0, 11.0, 0.5])
    f.scan_xy_body = make_scan_body()
    f.scan_xy_world = np.zeros((35, 3))
    f.height_scan = np.linspace(-1.75, 1.75, 35)
    f.phase = np.array([0.1 + 0.01 * i, 3.2, 3.1, 0.2])
    f.action = np.linspace(-1.0, 1.0, 12)
    f.action_applied = np.clip(f.action, -0.8, 0.8)
    f.obs = np.tile(np.linspace(-1.0, 1.0, 91), 1) * (1.0 + 0.01 * (i % 5))
    f.level = i % 4
    f.level_name = "L%d" % (i % 4)
    f.mode = "track" if i % 2 == 0 else "crouch"
    f.obs_valid = True
    f.sources = {"linvel": "estimator", "scan": "lidar", "leg_fk": "oracle"}
    f.rtf = 1.0
    f.overruns = i
    f.n_cmd = 100 + i
    f.n_state = 99 + i
    return f


# ------------------------------------------------------------------ 1. 契约字段
def test_contract_fields() -> None:
    f = VizFrame()
    missing = [n for n in CONTRACT_FIELDS if not hasattr(f, n)]
    check("VizFrame 契约字段齐全", not missing, "缺失=%s" % missing)
    check("joint_tau 长度 = 12", np.asarray(f.joint_tau).size == 12)
    check("obs 长度 = 91", np.asarray(f.obs).size == 91)
    check("height_scan 长度 = 35", np.asarray(f.height_scan).size == 35)
    check("scan_xy_body 形状 = (35,2)", tuple(np.asarray(f.scan_xy_body).shape) == (35, 2))
    check("关节名 12 个且按 FR/FL/RR/RL 分组",
          len(JOINT_NAMES) == 12 and JOINT_NAMES[0] == "FR_hip" and JOINT_NAMES[-1] == "RL_calf",
          str(JOINT_NAMES[:3]))
    check("obs 分 10 段且首尾覆盖 0..91",
          len(OBS_SEGMENTS) == 10 and OBS_SEGMENTS[0][1] == 0 and OBS_SEGMENTS[-1][2] == 91,
          str([s[0] for s in OBS_SEGMENTS]))


# ------------------------------------------------------------------ 2. 滚动窗口
def test_figure_buffer_window() -> None:
    window = 0.5
    fp = FigurePanel(which=("tau", "height", "vel", "action", "err"), window=window)
    for i in range(100):
        fp.update(make_frame(i))
    lens = fp.n_samples()
    cap = int(window / 0.02) + 2
    check("5 张图都建好", len(fp.figures()) == 5 and set(lens) == set(fp.which))
    check("100 帧后缓冲受 window 限制（< 100）",
          all(n < 100 for n in lens.values()), str(lens))
    check("缓冲长度 <= window/dt + 2 = %d" % cap,
          all(n <= cap for n in lens.values()), str(lens))
    check("缓冲长度 = 窗口内样本数（约 25）",
          all(abs(n - 25) <= 2 for n in lens.values()), str(lens))
    # 重复 which 去重
    fp2 = FigurePanel(which=("tau", "tau"))
    check("which 去重", fp2.which == ("tau",))
    # 未知 key 必须报错
    try:
        FigurePanel(which=("nope",))
        ok = False
    except ValueError:
        ok = True
    check("未知 which 抛 ValueError", ok)


# ------------------------------------------------------------------ 3. viewports
def test_figure_viewports() -> None:
    W, H = 1280, 720
    fp = FigurePanel(which=("tau", "height", "vel", "action", "err"), window=2.0)
    for i in range(60):
        fp.update(make_frame(i))
    vps = fp.viewports(W, H)
    check("viewports 数量 = 5", len(vps) == 5, str(len(vps)))
    inside = all(0 <= r.left and 0 <= r.bottom and r.left + r.width <= W
                 and r.bottom + r.height <= H and r.width > 0 and r.height > 0
                 for r, _f in vps)
    check("MjrRect 全部落在 (W,H) 内且非空", inside,
          str([(r.left, r.bottom, r.width, r.height) for r, _ in vps]))
    # 互不重叠
    overlap = False
    for i in range(len(vps)):
        for j in range(i + 1, len(vps)):
            a, b = vps[i][0], vps[j][0]
            if (a.left < b.left + b.width and b.left < a.left + a.width
                    and a.bottom < b.bottom + b.height and b.bottom < a.bottom + a.height):
                overlap = True
    check("5 个 rect 互不重叠", not overlap)

    figs = [f for _r, f in vps]
    check("返回的是真 MjvFigure", all(isinstance(f, mujoco.MjvFigure) for f in figs))
    tau_fig = figs[0]
    names = [bytes(n).decode() for n in tau_fig.linename[:12]]
    check("tau 12 条线都设了 linename",
          all(n.strip() for n in names) and names[0] == "FR_hip" and names[-1] == "RL_calf",
          str(names[:3] + names[-1:]))
    check("linewidth 已设置 (>0)", float(tau_fig.linewidth) > 0.0, str(tau_fig.linewidth))
    check("legend 打开", int(tau_fig.flg_legend) == 1)
    check("title 用 title 字段（没有 figuretitle）",
          tau_fig.title == "joint torque [Nm]" and not hasattr(tau_fig, "figuretitle"),
          repr(tau_fig.title))

    n = int(tau_fig.linepnt[0])
    check("linepnt 落在 (0, mjMAXLINEPNT]", 0 < n <= mujoco.mjMAXLINEPNT, str(n))
    row = tau_fig.linedata[0]
    xs = row[0:2 * n:2]
    ys = row[1:2 * n:2]
    # linedata 按**推入顺序**（旧->新）存放；窗口 2.0s > 1.18s，60 个样本全在窗口里
    start = 60 - n
    expect_x = np.array([make_frame(i).t for i in range(start, 60)], dtype=np.float32)
    expect_y = np.array([make_frame(i).joint_tau[0] for i in range(start, 60)],
                        dtype=np.float32)
    check("linedata 是 (x,y) 交错存放：x 正确",
          np.allclose(xs, expect_x, atol=1e-6), "x[:3]=%s" % xs[:3])
    check("linedata 是 (x,y) 交错存放：y 正确",
          np.allclose(ys, expect_y, atol=1e-5), "y[:3]=%s" % ys[:3])
    # range 是 float32，不能用 == 比
    check("range[0] 是 x 轴且随窗口滑动",
          np.isclose(float(tau_fig.range[0][1]), make_frame(59).t, atol=1e-6)
          and np.isclose(float(tau_fig.range[0][0]),
                         make_frame(59).t - fp.window, atol=1e-5)
          and float(tau_fig.range[0][0]) < float(tau_fig.range[0][1]),
          str(tau_fig.range[0]))
    check("range[1] 是 y 轴且覆盖数据",
          float(tau_fig.range[1][0]) < float(np.min(expect_y))
          and float(tau_fig.range[1][1]) > float(np.max(expect_y)),
          str(tau_fig.range[1]))
    # origin：把曲线挪到指定区域（默认 (0,0) 时就是契约原行为）
    vps_off = fp.viewports(600, 400, origin=(700, 300))
    ok_off = all(700 <= r.left and r.left + r.width <= 1300
                 and 300 <= r.bottom and r.bottom + r.height <= 700
                 for r, _ in vps_off)
    check("origin 生效：曲线落在指定区域", ok_off,
          str([(r.left, r.bottom, r.width, r.height) for r, _ in vps_off]))

    # 和右下角仪表盘不重叠（这就是 origin 的用途）
    ip = ImagePanel()
    for i in range(3):
        ip.update(make_frame(i))
    img_rect, _img = ip.viewport_image(1280, 720)
    vps_top = fp.viewports(1280, 340, origin=(0, 380))
    clash = any(img_rect.left < r.left + r.width and r.left < img_rect.left + img_rect.width
                and img_rect.bottom < r.bottom + r.height
                and r.bottom < img_rect.bottom + img_rect.height for r, _ in vps_top)
    check("曲线区域与右下角仪表盘不重叠", not clash,
          "img=%s figs=%s" % ((img_rect.left, img_rect.bottom, img_rect.width, img_rect.height),
                              [(r.left, r.bottom, r.width, r.height) for r, _ in vps_top]))

    # 小视口也要优雅退化，不能越界
    vps_small = fp.viewports(200, 120)
    ok_small = all(0 <= r.left and 0 <= r.bottom and r.left + r.width <= 200
                   and r.bottom + r.height <= 120 and r.width >= 1 and r.height >= 1
                   for r, _ in vps_small)
    check("小视口 (200x120) 仍不越界", ok_small,
          str([(r.left, r.bottom, r.width, r.height) for r, _ in vps_small]))


# ------------------------------------------------------------------ 4. 缺真值 / 字段缺失
def test_figure_truth_absent() -> None:
    fp = FigurePanel(which=("vel", "err"), window=2.0)
    for i in range(20):
        fp.update(make_frame(i, truth=False))
    vps = fp.viewports(800, 400)
    vel, err = vps[0][1], vps[1][1]
    check("无真值时 est 4 条线有数据", all(int(vel.linepnt[i]) > 0 for i in range(4)),
          str([int(vel.linepnt[i]) for i in range(4)]))
    check("无真值时 true 4 条线不画 (linepnt=0)",
          all(int(vel.linepnt[i]) == 0 for i in range(4, 8)),
          str([int(vel.linepnt[i]) for i in range(4, 8)]))
    check("无真值时 err 图 4 条线都不画",
          all(int(err.linepnt[i]) == 0 for i in range(4)),
          str([int(err.linepnt[i]) for i in range(4)]))
    # 有真值时 err 有数据
    fp2 = FigurePanel(which=("err",), window=2.0)
    for i in range(20):
        fp2.update(make_frame(i, truth=True))
    e2 = fp2.viewports(400, 200)[0][1]
    check("有真值时 err 图 4 条线有数据", all(int(e2.linepnt[i]) > 0 for i in range(4)),
          str([int(e2.linepnt[i]) for i in range(4)]))


def test_bare_object_frames() -> None:
    """字段全缺的 frame 不能把观察脚本弄崩（只允许退化成 0/nan）。"""
    ok = True
    detail = ""
    try:
        fp = FigurePanel(which=("tau", "vel", "err"), window=1.0)
        fp.update(object())
        vps = fp.viewports(640, 480)
        ok = len(vps) == 3 and all(isinstance(f, mujoco.MjvFigure) for _r, f in vps)
    except Exception as e:
        ok, detail = False, "%s: %s" % (type(e).__name__, e)
    check("FigurePanel.update(object()) 不抛异常", ok, detail)

    ok, detail = True, ""
    try:
        ip = ImagePanel()
        ip.update(object())
        got = ip.viewport_image(640, 480)
        ok = got is None or (got[1].ndim == 3 and got[1].dtype == np.uint8)
    except Exception as e:
        ok, detail = False, "%s: %s" % (type(e).__name__, e)
    check("ImagePanel.update(object()) 不抛异常", ok, detail)

    ok, detail = True, ""
    try:
        hud = TextHud()
        hud.update(object())
        ts = hud.texts()
        ok = all(isinstance(t, tuple) and len(t) == 4 for t in ts)
    except Exception as e:
        ok, detail = False, "%s: %s" % (type(e).__name__, e)
    check("TextHud.update(object()) 不抛异常", ok, detail)


# ------------------------------------------------------------------ 5. ImagePanel
def test_image_panel() -> None:
    ip = ImagePanel()
    for i in range(5):
        ip.update(make_frame(i))
    sizes = [(1280, 720), (800, 480), (600, 320), (400, 300), (300, 200), (64, 48)]
    all_ok = True
    detail = []
    for W, H in sizes:
        got = ip.viewport_image(W, H)
        if got is None:
            all_ok = False
            detail.append("%dx%d -> None" % (W, H))
            continue
        rect, img = got
        good = (isinstance(img, np.ndarray) and img.dtype == np.uint8 and img.ndim == 3
                and img.shape[2] == 3
                and img.shape[:2] == (rect.height, rect.width)
                and 0 <= rect.left and 0 <= rect.bottom
                and rect.left + rect.width <= W and rect.bottom + rect.height <= H
                and rect.width > 0 and rect.height > 0)
        all_ok = all_ok and good
        detail.append("%dx%d rect=%s img=%s" % (W, H, (rect.left, rect.bottom, rect.width,
                                                       rect.height), img.shape))
    check("viewport_image 返回 (MjrRect, uint8 HxWx3) 且 shape 与 rect 一致、rect 在视口内",
          all_ok, " | ".join(detail))

    rect, img = ip.viewport_image(1280, 720)
    check("set_images 的硬校验能过：img.shape[:2] == (rect.height, rect.width)",
          img.shape[:2] == (rect.height, rect.width), str((img.shape, rect.height, rect.width)))
    check("图像不是空白（std > 0）", float(img.std()) > 0.0, "std=%.2f mean=%.1f"
          % (float(img.std()), float(img.mean())))
    check("默认像素尺寸 = size*dpi = (512,288)",
          ip._want_px == (512, 288), str(ip._want_px))

    # 面板内容：热图 7x5、相位 4 点、obs 91 条、文字含等级/高度均值/动作统计
    ip2 = ImagePanel()
    f = make_frame(7)
    ip2.update(f)
    arr = np.asarray(ip2._im.get_array())
    check("热图数据形状 = (5,7)（x 外层 y 内层，转置后 [y,x]）", arr.shape == (5, 7),
          str(arr.shape))
    check("热图 (0,0) 对应 grid (x0,y0) 的 height_scan[0]",
          abs(float(arr[0, 0]) - float(f.height_scan[0])) < 1e-6,
          "%.3f vs %.3f" % (float(arr[0, 0]), float(f.height_scan[0])))
    ext = ip2._im.get_extent()
    check("热图 extent 来自 scan_xy_body 且 +x 向右（前）",
          abs(ext[0] - (-0.3)) < 1e-9 and abs(ext[1] - 0.9) < 1e-9
          and abs(ext[2] - (-0.4)) < 1e-9 and abs(ext[3] - 0.4) < 1e-9, str(ext))
    offs = np.asarray(ip2._phase_scat.get_offsets())
    check("相位图 4 个点、角度 = frame.phase",
          offs.shape == (4, 2) and np.allclose(offs[:, 0], f.phase), str(offs[:, 0]))
    segs = np.asarray(ip2._obs_lc.get_segments())
    check("obs 条形 = 91 根", segs.shape == (91, 2, 2), str(segs.shape))
    txt = ip2._txt.get_text()
    check("面板文字含健康等级 / hscan 均值 / 动作 mean,max",
          f.level_name in txt and "hscan" in txt and " a " in txt, repr(txt[:60]))
    check("面板文字含 10 段当前值范围（每段一个 min~max）",
          "rng " in txt and txt.count("~") == 10, "~x%d" % txt.count("~"))
    check("10 个段名作为静态刻度标注",
          [t.get_text() for t in ip2._ax_o.get_xticklabels()] == [s[0] for s in OBS_SEGMENTS])

    # 退化尺寸与自定义色标
    small = ImagePanel(size=(2.0, 1.0))
    small.update(make_frame(0))
    got_s = small.viewport_image(1280, 720)
    check("极小面板 (160x80 px) 也能出图",
          got_s is not None and got_s[1].shape == (got_s[0].height, got_s[0].width, 3),
          None if got_s is None else str(got_s[1].shape))
    hl = ImagePanel(hlim=2.5)
    hl.update(make_frame(0))
    check("hlim 可覆盖热图色标", tuple(hl._im.get_clim()) == (-2.5, 2.5), str(hl._im.get_clim()))

    # 自动档位：明显超档会升档并锁住，峰值变小要等迟滞才降
    ip3 = ImagePanel()
    ip3.update(make_frame(0))
    z0 = ip3.obs_lim
    big = make_frame(0)
    big.obs = np.full(91, 40.0)
    ip3.update(big)
    check("obs 峰值超档立即升档", ip3.obs_lim[1] > z0[1],
          "%s -> %s" % (z0, ip3.obs_lim))
    small = make_frame(0)
    small.obs = np.full(91, 0.5)
    before = ip3.obs_lim
    for _ in range(10):
        ip3.update(small)
    check("降档有迟滞（10 帧内不降）", ip3.obs_lim == before,
          "%s -> %s" % (before, ip3.obs_lim))
    for _ in range(30):
        ip3.update(small)
    check("持续小峰值 40 帧后降档", ip3.obs_lim[1] < before[1],
          "%s -> %s" % (before, ip3.obs_lim))
    # 固定档位不自动变
    ipf = ImagePanel(obs_ylim=(-3.0, 3.0))
    ipf.update(big)
    check("obs_ylim 给定时锁死不自动换档", ipf.obs_lim == (-3.0, 3.0), str(ipf.obs_lim))


def test_image_perf() -> None:
    ip = ImagePanel()
    frames = [make_frame(i) for i in range(60)]
    for f in frames:                      # 预热 + 让自动档位稳定下来
        ip.update(f)
    ts = []
    for f in frames:
        t0 = time.perf_counter()
        ip.update(f)
        ts.append((time.perf_counter() - t0) * 1000.0)
    med, lo, hi = median(ts), min(ts), max(ts)
    check("ImagePanel.update() 中位耗时 < 15 ms（契约第 4 节）", med < 15.0,
          "min=%.2f med=%.2f max=%.2f ms" % (lo, med, hi))
    check("无异常降级（_ok 仍为 True 且 error 为空）", ip._ok and not ip.error,
          "ok=%s error=%s" % (ip._ok, ip.error))
    check("渲染器真的在出图", ip.n_render >= 60, str(ip.n_render))

    ip2 = ImagePanel(ranges=False)
    for f in frames:
        ip2.update(f)
    ts2 = []
    for f in frames:
        t0 = time.perf_counter()
        ip2.update(f)
        ts2.append((time.perf_counter() - t0) * 1000.0)
    check("ranges=False（不带 10 段范围文字）更快", median(ts2) < med,
          "med=%.2f vs %.2f ms" % (median(ts2), med))


def test_image_degrade() -> None:
    saved = panels._HAVE_MPL
    panels._HAVE_MPL = False
    try:
        ip = ImagePanel()
        ok = (not ip._ok) and ip.error is not None
        ip.update(make_frame(0))            # 不能抛
        got = ip.viewport_image(1280, 720)
        check("无 matplotlib 时 _ok=False 且 viewport_image() 返回 None",
              ok and got is None, "ok=%s error=%s" % (ip._ok, ip.error))
    finally:
        panels._HAVE_MPL = saved
    # 恢复后还能正常用
    ip2 = ImagePanel()
    ip2.update(make_frame(0))
    check("恢复 matplotlib 后重新可用", ip2._ok and ip2.viewport_image(640, 480) is not None)


# ------------------------------------------------------------------ 6. TextHud
def test_text_hud() -> None:
    hud = TextHud()
    empty = hud.texts()
    check("未 update 也能 texts()", len(empty) >= 1 and all(len(t) == 4 for t in empty))
    f = make_frame(2)
    hud.update(f, extra={"gc": 0.31, "hz": 19.8})
    ts = hud.texts()
    check("texts() 每条都是 4 元组", all(isinstance(t, tuple) and len(t) == 4 for t in ts),
          str([len(t) for t in ts]))
    check("text1/text2 都是 str", all(isinstance(t[2], str) and isinstance(t[3], str) for t in ts))
    check("font 是 mjtFontScale",
          all(isinstance(t[0], mujoco.mjtFontScale) for t in ts),
          str([str(t[0]) for t in ts]))
    check("gridpos 是 mjtGridPos（viewer.set_texts 要的就是它）",
          all(isinstance(t[1], mujoco.mjtGridPos) for t in ts),
          str([str(t[1]) for t in ts]))
    blob = "\n".join(t[2] + "\n" + t[3] for t in ts)
    check("含健康等级", f.level_name in blob, f.level_name)
    check("含 mode / z / tilt", f.mode in blob and "z " in blob and "tilt" in blob)
    check("含 vx 估计与真值、tau_max", "vx" in blob and "tau_max" in blob)
    check("含每一路来源", all(k in blob for k in f.sources))
    check("extra 诊断出现在 HUD 里", "gc=0.31" in blob and "hz=19.8" in blob)
    check("多行文字用反斜杠 n 分隔（mjr_text 支持）", "\n" in ts[0][2])
    # 没有 extra 时不多出空条目
    hud.update(f)
    check("不带 extra 时只有需要的条目", len(hud.texts()) == len(ts) - 1,
          "%d -> %d" % (len(ts), len(hud.texts())))


# ------------------------------------------------------------------ 7. MuJoCo 字段事实
def test_mjvfigure_facts() -> None:
    fig = mujoco.MjvFigure()
    check("没有 figuretitle 字段（不能凭记忆写）", not hasattr(fig, "figuretitle"))
    check("标题字段叫 title", hasattr(fig, "title") and fig.title == "")
    check("linedata 形状 = (mjMAXLINE, 2*mjMAXLINEPNT)",
          fig.linedata.shape == (mujoco.mjMAXLINE, 2 * mujoco.mjMAXLINEPNT),
          str(fig.linedata.shape))
    check("mjMAXLINEPNT = 1001", int(mujoco.mjMAXLINEPNT) == 1001, str(mujoco.mjMAXLINEPNT))
    check("linename 是 bytes (|S100)", fig.linename.dtype.kind == "S", str(fig.linename.dtype))
    check("range 形状 = (2,2) 且 [0]=x, [1]=y",
          fig.range.shape == (2, 2), str(fig.range.shape))
    check("linergb 形状 = (100,3)", fig.linergb.shape == (100, 3), str(fig.linergb.shape))
    r = mujoco.MjrRect(11, 22, 33, 44)
    check("MjrRect 是 (left,bottom,width,height)",
          (r.left, r.bottom, r.width, r.height) == (11, 22, 33, 44))


# ------------------------------------------------------------------ 8. 可选：真渲染
def test_gl_render_optional() -> None:
    """有 GL 上下文就真渲染一张图并检查非空；没有就如实 SKIP（本机 egl/osmesa 都不可用）。"""
    try:
        m = mujoco.MjModel.from_xml_string(
            "<mujoco><worldbody><geom type='plane' size='1 1 .1'/></worldbody></mujoco>")
        d = mujoco.MjData(m)
        ctx = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
        scn = mujoco.MjvScene(m, maxgeom=10)
        cam, opt = mujoco.MjvCamera(), mujoco.MjvOption()
        mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    except Exception as e:
        skip("真实 MjrFigure 光栅化", "无可用 GL 上下文: %s: %s" % (type(e).__name__, str(e)[:60]))
        return
    fp = FigurePanel(which=("tau",), window=1.0)
    for i in range(20):
        fp.update(make_frame(i))
    rect, fig = fp.viewports(320, 160)[0]
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
    scn.ngeom = 0
    mujoco.mjr_render(rect, scn, ctx)
    mujoco.mjr_figure(rect, fig, ctx)
    buf = np.zeros((rect.height, rect.width, 3), np.uint8)
    dep = np.zeros((rect.height, rect.width), np.float32)
    mujoco.mjr_readPixels(buf, dep, rect, ctx)
    check("MjrFigure 光栅化出非空像素", int(buf.sum()) > 0,
          "sum=%d" % int(buf.sum()))


def main() -> int:
    print("test_viz_panels.py —— FigurePanel / ImagePanel / TextHud（mujoco %s, mpl %s）"
          % (mujoco.__version__,
             __import__("matplotlib").__version__ if panels._HAVE_MPL else "N/A"))
    test_contract_fields()
    test_figure_buffer_window()
    test_figure_viewports()
    test_figure_truth_absent()
    test_bare_object_frames()
    test_image_panel()
    test_image_perf()
    test_image_degrade()
    test_text_hud()
    test_mjvfigure_facts()
    test_gl_render_optional()
    n_fail = sum(1 for _n, ok in RESULT if not ok)
    print("\n===== test_viz_panels 汇总 =====")
    print("  %d/%d 通过，%d 项 SKIP" % (len(RESULT) - n_fail, len(RESULT), len(SKIPPED)))
    if n_fail:
        for n, ok in RESULT:
            if not ok:
                print("  FAIL: %s" % n)
        raise SystemExit(1)
    print("OK test_viz_panels.py 全部 %d 项通过" % len(RESULT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
