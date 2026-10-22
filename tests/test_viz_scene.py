#!/usr/bin/env python3
"""tests/test_viz_scene.py —— viz/scene.py 验收（契约 docs/viz-interfaces.md §3）。

直接运行（不需要 pytest）：.venv/bin/python tests/test_viz_scene.py
成功打印 "OK test_viz_scene: ..." 并以 0 退出。

覆盖：
  1. draw() 后 ngeom 增量 == 35 扫描点 + 4 足端 + 3 箭头 + 拖尾，且 <= maxgeom
  2. 颜色随 height_scan 符号变化（负=暖/红, 正=冷/蓝），半径随 |height_scan| 增大
  3. 连续 3 次 draw() 同一帧：ngeom 相等、geom 内容逐位相同（不累积）
  4. 基址保护：scene 里已有别人的 geom 时不会覆盖、不会丢
  5. 容量退化：maxgeom=20 / maxgeom=0 都不越界、不抛异常、丢弃数记账
  6. 拖尾：随 t 推进增长、上限 trail_n、预算内 stride 抽样、geom_budget 封顶
  7. frame=None 能安全清空自己那一层
  8. 足端接触 绿球 / 非接触 灰球；箭头标签 v_est/v_truth/cmd 与真值偏移
  9. set_camera：真模型走 TRACKING，model=None 退化 FREE，top 模式方位角正确
 10. stats()/legend() 字段齐全；draw() 单帧耗时（打印实测）
 11. cfg 开关全关时 ngeom 归零且记 disabled
 12. 真 Renderer 离屏渲染冒烟：加叠加层后像素真的变化（可选，无 GL 时 SKIP）
"""
from __future__ import annotations

import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from sim_env.viz.frame import VizFrame  # noqa: E402
from sim_env.viz.scene import (  # noqa: E402
    NUM_FEET, SCAN_COLOR_LEGEND, SCAN_N, SceneOverlay,
)

MODEL_PATH = os.path.join(ROOT, "sim_env", "models", "go2", "scene_flat.xml")
FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def try_check(name: str, fn) -> None:
    try:
        fn()
    except AssertionError as exc:
        check(name, False, str(exc))
    except Exception as exc:  # noqa: BLE001
        check(name, False, "%s: %s" % (type(exc).__name__, exc))


# ------------------------------------------------------------------ 夹具
def load_model():
    return mujoco.MjModel.from_xml_path(MODEL_PATH)


def make_scene(model, maxgeom: int = 600):
    return mujoco.MjvScene(model, maxgeom)


def make_frame(t: float = 0.0, n_arrow: int = 3) -> VizFrame:
    """假 VizFrame：非零速度、两条腿接触、扫描特征从 -1.75 线性到 +1.75。"""
    f = VizFrame(t=float(t), wall=float(t))
    f.base_pos = np.array([1.0, 2.0, 0.30 + 0.01 * t])
    f.base_quat = np.array([1.0, 0.0, 0.0, 0.0])
    f.base_lin_vel_world = np.array([0.40, 0.10, 0.0]) if n_arrow >= 1 else np.zeros(3)
    f.base_lin_vel_truth = np.array([0.35, 0.12, 0.0]) if n_arrow >= 2 else None
    f.foot_pos_world = np.array([[0.30, 0.15, 0.0], [0.30, -0.15, 0.0],
                                 [-0.30, 0.15, 0.0], [-0.30, -0.15, 0.0]])
    f.foot_force = np.array([12.0, 0.0, 11.0, 0.0])
    f.height_scan = np.linspace(-1.75, 1.75, SCAN_N)
    xs = np.linspace(-0.8, 0.8, 7)
    ys = np.linspace(-0.5, 0.5, 5)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    xy = np.stack([gx.ravel(), gy.ravel()], axis=-1)[:SCAN_N]
    f.scan_xy_body = xy.copy()
    # 世界系点 = 机体系 + base_pos(xy)，z 用地面高度（有起伏，便于看颜色）
    f.scan_xy_world = np.concatenate(
        [xy + f.base_pos[None, :2], np.zeros((SCAN_N, 1))], axis=-1)
    f.obs = np.zeros(91)
    return f


def rgba_snapshot(scn):
    return np.array([np.array(scn.geoms[i].rgba, dtype=np.float64)
                     for i in range(int(scn.ngeom))])


def pos_snapshot(scn):
    return np.array([np.array(scn.geoms[i].pos, dtype=np.float64)
                     for i in range(int(scn.ngeom))])


# ------------------------------------------------------------------ 1. 基本增量
def t_draw_counts():
    model = load_model()
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    f = make_frame(0.0)
    ov.draw(scn, f)
    st = ov.stats()
    c = st["counts"]
    check("draw: ngeom == 35+4+3+1(拖尾首点) = 43",
          int(scn.ngeom) == 43, "ngeom=%d counts=%s" % (scn.ngeom, c))
    check("draw: 分项计数自洽 (scan=35, feet=4, arrows=3, trail=1)",
          c["scan_drawn"] == 35 and c["feet_drawn"] == 4 and c["arrows_drawn"] == 3
          and c["trail_drawn"] == 1 and c["drawn"] == 43, "counts=%s" % c)
    check("draw: 不丢任何 geom (dropped 全 0)",
          c["scan_dropped"] == 0 and c["feet_dropped"] == 0
          and c["arrows_dropped"] == 0 and c["trail_dropped"] == 0, "counts=%s" % c)
    check("draw: ngeom <= maxgeom", scn.ngeom <= scn.maxgeom)
    check("draw: stats counts[ngeom] == 实际 scn.ngeom",
          int(c["ngeom"]) == int(scn.ngeom) and int(c["ngeom"]) == 43,
          "counts.ngeom=%s scn.ngeom=%d" % (c["ngeom"], scn.ngeom))
    check("draw: maxgeom=0 的 scene 不炸", _maxgeom_zero(model))
    check("draw: cfg 全关时 ngeom=0 且记 disabled", _all_off(model))


def _all_off(model):
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None, {"scan_on": False, "feet_on": False,
                                    "arrows_on": False, "trail_on": False})
    ov.draw(scn, make_frame(0.0))
    c = ov.stats()["counts"]
    return (int(scn.ngeom) == 0
            and c.get("scan_disabled") and c.get("feet_disabled")
            and c.get("arrows_disabled") and c.get("trail_disabled"))


def _maxgeom_zero(model):
    scn = mujoco.MjvScene()          # maxgeom == 0
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    return int(scn.ngeom) == 0 and int(ov.stats()["maxgeom"]) == 0


# ------------------------------------------------------------------ 2. 颜色 / 半径
def t_colors():
    model = load_model()
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    # 索引 0 = 最负(-1.75, 上坡) -> 暖/红; 索引 34 = 最正(+1.75, 下陷) -> 冷/蓝
    neg = np.array(scn.geoms[0].rgba, dtype=np.float64)
    mid = np.array(scn.geoms[17].rgba, dtype=np.float64)
    pos = np.array(scn.geoms[34].rgba, dtype=np.float64)
    check("color: height_scan<0 -> 暖色 (R > B)", neg[0] > neg[2],
          "neg rgba=%s" % np.round(neg, 3))
    check("color: height_scan>0 -> 冷色 (B > R)", pos[2] > pos[0],
          "pos rgba=%s" % np.round(pos, 3))
    check("color: 两个极值颜色明显不同", abs(float(neg[0] - pos[0])) > 0.5,
          "R_neg=%.3f R_pos=%.3f" % (neg[0], pos[0]))
    check("color: 零值点接近灰白 (|R-B| 小)", abs(float(mid[0] - mid[2])) < 0.15,
          "mid rgba=%s" % np.round(mid, 3))
    r_neg, r_mid, r_pos = (float(scn.geoms[i].size[0]) for i in (0, 17, 34))
    check("radius: 半径 ∝ |height_scan| (极值 > 零点)",
          r_neg > r_mid and r_pos > r_mid,
          "r_neg=%.4f r_mid=%.4f r_pos=%.4f" % (r_neg, r_mid, r_pos))


# ------------------------------------------------------------------ 3. 不累积
def t_no_accumulate():
    model = load_model()
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    f = make_frame(0.0)
    ov.draw(scn, f)
    n1 = int(scn.ngeom)
    s1, p1 = rgba_snapshot(scn), pos_snapshot(scn)
    ov.draw(scn, f)
    n2 = int(scn.ngeom)
    ov.draw(scn, f)
    n3 = int(scn.ngeom)
    check("no-accumulate: 连续 3 次 draw() ngeom 相等", n1 == n2 == n3,
          "n1=%d n2=%d n3=%d" % (n1, n2, n3))
    check("no-accumulate: geom 内容逐位相同（真正重画，不是叠加）",
          np.allclose(s1, rgba_snapshot(scn)) and np.allclose(p1, pos_snapshot(scn)))
    check("no-accumulate: 同一 t 不重复 push 拖尾", ov.stats()["trail_len"] == 1,
          "trail_len=%d" % ov.stats()["trail_len"])


# ------------------------------------------------------------------ 4. 基址保护
def t_base_offset():
    model = load_model()
    scn = make_scene(model, 600)
    # 先在 scene 里放 5 个"别人的" geom
    for i in range(5):
        mujoco.mjv_initGeom(scn.geoms[i], int(mujoco.mjtGeom.mjGEOM_SPHERE),
                            np.array([0.123, 0.0, 0.0]), np.zeros(3),
                            np.eye(3).reshape(-1),
                            np.array([0.0, 0.0, 1.0, 1.0], dtype=np.float32))
    scn.ngeom = 5
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    st = ov.stats()
    check("base: 已有 5 个 geom 时基址 = 5", st["base_ngeom"] == 5, "base=%d" % st["base_ngeom"])
    check("base: 总 ngeom = 5 + 43", int(scn.ngeom) == 48, "ngeom=%d" % scn.ngeom)
    check("base: 别人的 geom 未被改动",
          abs(float(scn.geoms[0].size[0]) - 0.123) < 1e-6
          and int(scn.geoms[0].type) == int(mujoco.mjtGeom.mjGEOM_SPHERE),
          "size=%r type=%d" % (float(scn.geoms[0].size[0]), int(scn.geoms[0].type)))

    # 外部清空 scene 后应自动重新对齐基址（t 推进 -> 拖尾多 1 个点: 35+4+3+2=44）
    scn.ngeom = 0
    ov.draw(scn, make_frame(1.0))
    check("base: 外部清空后基址自动回 0", ov.stats()["base_ngeom"] == 0
          and int(scn.ngeom) == 44, "base=%d ngeom=%d" % (ov.stats()["base_ngeom"], scn.ngeom))


# ------------------------------------------------------------------ 4b. 两个 scene 交替（view_policy 的真实用法）
def t_two_scenes():
    """scripts/view_policy.py 会轮流往 renderer.scene（基址=模型 geom 数）和
    viewer.user_scn（基址=0）里画；一个 overlay 实例必须按 scene 分别记基址，
    否则第二帧就会把 renderer.scene 里机器人的 geom 覆盖掉。"""
    model = load_model()
    scn_render = make_scene(model, 600)
    scn_user = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    ok = True
    detail = ""
    for k in range(5):
        # 模拟 Renderer.update_scene: 每帧把 ngeom 重置回"模型 geom 数" 20
        for i in range(20):
            mujoco.mjv_initGeom(scn_render.geoms[i], int(mujoco.mjtGeom.mjGEOM_SPHERE),
                                np.array([0.123, 0.0, 0.0]), np.zeros(3),
                                np.eye(3).reshape(-1),
                                np.array([0.0, 0.0, 1.0, 1.0], dtype=np.float32))
        scn_render.ngeom = 20
        ov.draw(scn_render, make_frame(k * 0.01))
        n_r = int(scn_render.ngeom)
        ov.draw(scn_user, make_frame(k * 0.01))
        n_u = int(scn_user.ngeom)
        marker_ok = abs(float(scn_render.geoms[0].size[0]) - 0.123) < 1e-6
        if not (n_r == 20 + 42 + (k + 1) and n_u == 42 + (k + 1) and marker_ok):
            ok = False
            detail = "k=%d render=%d user=%d marker_ok=%s base_render=%s" % (
                k, n_r, n_u, marker_ok, ov.stats()["base_ngeom"])
            break
    check("two-scenes: renderer.scene 与 user_scn 分别记基址、互不破坏", ok, detail)


# ------------------------------------------------------------------ 5. 容量退化
def t_capacity():
    model = load_model()
    scn = make_scene(model, 20)
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    st = ov.stats()
    c = st["counts"]
    check("capacity(20): ngeom == 20 == maxgeom", int(scn.ngeom) == 20,
          "ngeom=%d" % scn.ngeom)
    check("capacity(20): 越界不发生且丢 15 个扫描点",
          c["scan_drawn"] == 20 and c["scan_dropped"] == 15 and c.get("degraded") == 1,
          "counts=%s" % c)
    check("capacity(20): 足端/箭头全丢并记账",
          c["feet_drawn"] == 0 and c["arrows_drawn"] == 0, "counts=%s" % c)

    # 长时间小容量跑：每一帧都不越界
    scn2 = make_scene(model, 33)      # 连 35 个扫描点都放不下
    ov2 = SceneOverlay(model, None)
    ok = True
    for k in range(300):
        ov2.draw(scn2, make_frame(k * 0.01))
        if int(scn2.ngeom) > int(scn2.maxgeom):
            ok = False
            break
    check("capacity(33): 连续 300 帧 draw() 从不越界", ok, "ngeom=%d" % scn2.ngeom)

    # geom_budget 封顶
    scn3 = make_scene(model, 600)
    ov3 = SceneOverlay(model, None)
    for k in range(600):
        ov3.draw(scn3, make_frame(k * 0.01))
    st3 = ov3.stats()
    check("budget: ngeom 被 geom_budget 封顶",
          int(scn3.ngeom) <= st3["geom_budget"] and int(scn3.ngeom) == st3["geom_budget"],
          "ngeom=%d budget=%d" % (scn3.ngeom, st3["geom_budget"]))
    check("budget: 拖尾上限 trail_n=400 生效",
          st3["trail_len"] == 400, "trail_len=%d" % st3["trail_len"])
    check("budget: 拖尾 stride 抽样并记账",
          st3["counts"]["trail_drawn"] + st3["counts"]["trail_dropped"] == 400,
          "counts=%s" % st3["counts"])


# ------------------------------------------------------------------ 6. frame=None
def t_none_frame():
    model = load_model()
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    n_before = int(scn.ngeom)
    ov.draw(scn, None)
    check("frame=None: 清空自己那一层且不抛异常",
          int(scn.ngeom) == 0 and n_before == 43, "n_before=%d n_after=%d" % (n_before, scn.ngeom))
    ov.draw(scn, make_frame(0.5))   # t 前进 -> 拖尾 2 个点 = 44
    check("frame=None 之后还能正常画", int(scn.ngeom) == 44 and int(scn.ngeom) <= int(scn.maxgeom),
          "ngeom=%d" % scn.ngeom)


# ------------------------------------------------------------------ 7. 足端 / 箭头
def t_feet_arrows():
    model = load_model()
    scn = make_scene(model, 600)
    ov = SceneOverlay(model, None)
    ov.draw(scn, make_frame(0.0))
    # 足端在扫描点之后：索引 35..38
    fr = np.array(scn.geoms[35].rgba, dtype=np.float64)
    fl = np.array(scn.geoms[36].rgba, dtype=np.float64)
    check("feet: force>阈值 -> 绿球 (G 最大)", fr[1] > fr[0] and fr[1] > fr[2],
          "FR rgba=%s" % np.round(fr, 3))
    check("feet: force=0 -> 灰点 (R≈G≈B)",
          abs(fr[0] - fl[0]) > 0.2 and abs(fl[0] - fl[2]) < 1e-6 and fl[3] < 0.5,
          "FL rgba=%s" % np.round(fl, 3))
    check("feet: 接触球半径 > 抬起点半径",
          float(scn.geoms[35].size[0]) > float(scn.geoms[36].size[0]),
          "r_on=%.4f r_off=%.4f" % (scn.geoms[35].size[0], scn.geoms[36].size[0]))

    labels = {}
    for i in range(int(scn.ngeom)):
        lab = str(scn.geoms[i].label)
        if lab:
            labels[lab] = i
    check("arrows: 三个箭头标签齐全 (v_est/v_truth/cmd)",
          set(labels) == {"v_est", "v_truth", "cmd"}, "labels=%s" % sorted(labels))
    base_pos = np.array([1.0, 2.0, 0.30])
    est = np.array(scn.geoms[labels["v_est"]].pos, dtype=np.float64)
    truth = np.array(scn.geoms[labels["v_truth"]].pos, dtype=np.float64)
    check("arrows: v_est 起点 = base_pos", np.allclose(est, base_pos, atol=1e-9),
          "est=%s" % np.round(est, 3))
    check("arrows: v_truth 挪到旁边 (+0.35 y)",
          np.allclose(truth, base_pos + np.array([0.0, 0.35, 0.0]), atol=1e-9),
          "truth=%s" % np.round(truth, 3))
    check("arrows: 箭头是 ARROW1 类型",
          int(scn.geoms[labels["cmd"]].type) == int(mujoco.mjtGeom.mjGEOM_ARROW1))
    # ARROW1: pos=起点, size[2]=长度, mat 第 3 列 = 单位方向
    g_est = scn.geoms[labels["v_est"]]
    v_est = np.array([0.40, 0.10, 0.0])
    want_len = float(np.linalg.norm(v_est)) * 0.5
    got_len = float(g_est.size[2])
    mat = np.array(g_est.mat, dtype=np.float64).reshape(3, 3)
    axis = mat[:, 2]
    check("arrows: 箭头长度 = |v| * vel_scale", abs(got_len - want_len) < 1e-3,
          "got=%.4f want=%.4f" % (got_len, want_len))
    check("arrows: 箭头方向 = v/|v|",
          float(np.linalg.norm(axis - v_est / np.linalg.norm(v_est))) < 1e-6,
          "axis=%s" % np.round(axis, 4))

    # 零速度不画退化箭头
    scn2 = make_scene(model, 600)
    ov2 = SceneOverlay(model, None)
    f = make_frame(0.0)
    f.base_lin_vel_world = np.zeros(3)
    f.base_lin_vel_truth = None
    f.extra = {"cmd_vx": 0.0, "cmd_vy": 0.0}
    ov2.draw(scn2, f)
    check("arrows: 零速度/零指令时退化箭头被跳过 (只留扫描+足端+拖尾)",
          int(scn2.ngeom) == 35 + 4 + 1, "ngeom=%d" % scn2.ngeom)


# ------------------------------------------------------------------ 8. 相机
def t_camera():
    model = load_model()
    ov = SceneOverlay(model, None)
    cam = mujoco.MjvCamera()
    ov.set_camera(cam, make_frame(0.0))
    check("camera: follow -> TRACKING 且 trackbodyid 有效",
          int(cam.type) == int(mujoco.mjtCamera.mjCAMERA_TRACKING)
          and int(cam.trackbodyid) >= 0 and ov.stats()["base_body"] >= 0,
          "type=%d trackbodyid=%d" % (cam.type, cam.trackbodyid))
    check("camera: lookat 跟随 base_pos",
          np.allclose(np.array(cam.lookat), [1.0, 2.0, 0.30], atol=1e-9),
          "lookat=%s" % np.round(np.array(cam.lookat), 3))

    ov_top = SceneOverlay(model, None, {"camera_mode": "top"})
    cam_top = mujoco.MjvCamera()
    ov_top.set_camera(cam_top, make_frame(0.0))
    check("camera: top 模式 -> FREE + 俯视角度",
          int(cam_top.type) == int(mujoco.mjtCamera.mjCAMERA_FREE)
          and abs(float(cam_top.azimuth) + 90.0) < 1e-9
          and abs(float(cam_top.elevation) + 89.0) < 1e-9,
          "type=%d az=%.1f el=%.1f" % (cam_top.type, cam_top.azimuth, cam_top.elevation))

    ov_none = SceneOverlay(None, None)   # model=None -> 找不到躯干
    cam_none = mujoco.MjvCamera()
    ov_none.set_camera(cam_none, make_frame(0.0))
    check("camera: model=None 退化 FREE 不抛异常",
          int(cam_none.type) == int(mujoco.mjtCamera.mjCAMERA_FREE)
          and ov_none.stats()["base_body"] == -1,
          "type=%d base_body=%d" % (cam_none.type, ov_none.stats()["base_body"]))


# ------------------------------------------------------------------ 9. stats / legend / 性能
def t_stats_and_perf():
    model = load_model()
    ov = SceneOverlay(model, None)
    keys = set(ov.stats())
    need = {"frames", "last_ms", "mean_ms", "max_ms", "base_ngeom", "maxgeom",
            "trail_len", "trail_n", "geom_budget", "base_body", "counts"}
    check("stats: 字段齐全", need <= keys, "missing=%s" % sorted(need - keys))
    check("legend: 颜色语义说明非空且提到上坡/下陷",
          bool(SCAN_COLOR_LEGEND) and ("上坡" in SCAN_COLOR_LEGEND) and ("下陷" in SCAN_COLOR_LEGEND))

    scn = make_scene(model, 600)
    for k in range(20):        # 预热
        ov.draw(scn, make_frame(k * 0.01))
    times = []
    ngeoms = []
    for k in range(520):       # 拖尾灌满 400 -> 预算 320 全部用上
        f = make_frame(10.0 + k * 0.01)
        t0 = time.perf_counter()
        ov.draw(scn, f)
        times.append((time.perf_counter() - t0) * 1e3)
        ngeoms.append(int(scn.ngeom))
    arr = np.asarray(times)
    st = ov.stats()
    over = int(np.count_nonzero(arr > 1.0))
    # 契约要求单帧 < 1 ms：用中位数当硬门槛（对共享机器的调度抖动稳健），
    # 均值当回归护栏；两者和 p95/max 都打印出来接受审查。
    check("perf: 单帧中位数 < 1.0 ms（预算内 %d geom，均值护栏 < 1.5 ms）" % scn.ngeom,
          float(np.median(arr)) < 1.0 and float(arr.mean()) < 1.5,
          "mean=%.3f ms  p50=%.3f ms  p95=%.3f ms  max=%.3f ms  >1ms 帧数=%d/%d"
          % (arr.mean(), np.median(arr), np.percentile(arr, 95), arr.max(), over, arr.size))
    # 前 20 预热 + 520 帧里，拖尾到第 380 帧左右才灌满 400，之后 ngeom 必须恒定
    tail = ngeoms[420:]
    check("perf: 拖尾灌满后每帧 ngeom 恒定 = geom_budget（无漂移）",
          int(scn.ngeom) == st["geom_budget"] and min(tail) == max(tail) == st["geom_budget"],
          "ngeom=%d budget=%d min/max(last100)=%d/%d" % (scn.ngeom, st["geom_budget"],
                                                         min(tail), max(tail)))


# ------------------------------------------------------------------ 10. 真渲染冒烟（像素级证据）
def t_render_smoke():
    """把叠加层塞进真 Renderer 的 scene 渲染两帧对比: 有/无叠加层的像素差 > 0 才算真的画出来了。

    离屏 GL 不可用时打印 SKIP 而不是失败（契约要求的是 ngeom 级验收）。
    """
    os.environ.setdefault("MUJOCO_GL", "egl")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg

    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    try:
        ren = mujoco.Renderer(model, 240, 320)
    except Exception as exc:  # noqa: BLE001 - 无离屏 GL 就跳过
        check("render: 离屏渲染冒烟（SKIP: 无 GL）", True,
              "%s: %s" % (type(exc).__name__, exc))
        return

    f = make_frame(0.0)
    f.base_pos = np.asarray(data.qpos[0:3], dtype=np.float64).copy()
    # 给扫描点一点起伏，颜色/半径才有东西看
    f.height_scan = 1.75 * np.sin(np.linspace(0.0, 2.0 * np.pi, SCAN_N))
    f.scan_xy_world = np.concatenate(
        [f.scan_xy_body + f.base_pos[None, :2],
         0.06 * np.sin(np.linspace(0.0, 3.0 * np.pi, SCAN_N))[:, None]], axis=-1)
    f.foot_pos_world = f.base_pos[None, :] + np.array([
        [0.30, 0.15, -0.28], [0.30, -0.15, -0.05],
        [-0.30, 0.15, -0.28], [-0.30, -0.15, -0.05]])
    f.foot_force = np.array([12.0, 0.0, 11.0, 0.0])

    ov = SceneOverlay(model, data)
    cam = mujoco.MjvCamera()
    ov.set_camera(cam, f)
    ren.update_scene(data, camera=cam)
    n_model = int(ren.scene.ngeom)
    img_before = ren.render().copy()
    ov.draw(ren.scene, f)
    n_after = int(ren.scene.ngeom)
    img_after = ren.render().copy()

    d = np.abs(img_after.astype(np.int32) - img_before.astype(np.int32))
    changed = int(np.count_nonzero(d.sum(axis=2) > 12))
    check("render: 叠加层让渲染像素真的变了（不是只改 ngeom）",
          n_after == n_model + 43 and changed > 200,
          "ngeom %d -> %d, changed_px=%d, mean|diff|=%.3f"
          % (n_model, n_after, changed, float(d.mean())))

    out = os.path.join(tempfile.gettempdir(), "viz_scene_overlay_smoke.png")
    mpimg.imsave(out, img_after)
    check("render: 冒烟图已写出（非空 PNG）",
          os.path.isfile(out) and os.path.getsize(out) > 1000,
          out + " (%d bytes)" % (os.path.getsize(out) if os.path.isfile(out) else 0))
    try:
        ren.close()
    except Exception:  # noqa: BLE001 - EGL 析构在部分驱动上会报错，不影响结果
        pass


def main() -> int:
    print("== test_viz_scene: SceneOverlay 验收 (model=%s) ==" % os.path.relpath(MODEL_PATH, ROOT))
    try_check("1. draw 增量 / 分项计数", t_draw_counts)
    try_check("2. 颜色映射 / 半径缩放", t_colors)
    try_check("3. 每帧重画不累积", t_no_accumulate)
    try_check("4. 基址保护", t_base_offset)
    try_check("4b. renderer.scene / user_scn 交替", t_two_scenes)
    try_check("5. 容量退化 / 预算封顶", t_capacity)
    try_check("6. frame=None", t_none_frame)
    try_check("7. 足端接触 / 箭头", t_feet_arrows)
    try_check("8. 相机", t_camera)
    try_check("9. stats / legend / 性能", t_stats_and_perf)
    try_check("10. 离屏渲染冒烟（像素级）", t_render_smoke)

    if FAILS:
        print("\nFAILED %d: %s" % (len(FAILS), "; ".join(FAILS)))
        return 1
    print("OK test_viz_scene: 10 组检查全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
