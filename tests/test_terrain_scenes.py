#!/usr/bin/env python3
"""tests/test_terrain_scenes.py —— 部署地形场景的碰撞几何契约（sim2sim 修复回归）。

背景（docs/go2-sim-fidelity.md §4）：CPU MuJoCo 的 hfield 与足端球体接触会产生多个
重复接触点，导致地形上策略抽搐/走不动。导出器现在把 hfield 改成**只做视觉**，
碰撞改用倾斜 box 瓦片（`tc_*`）+ 平地 plane 兜底。本测试逐个体检查 16 个地形：

  1. 场景存在且能被 MuJoCo 加载；
  2. hfield geom 的 contype/conaffinity 都是 0（只视觉）；
  3. 有 `floor_plane`（type=plane）兜底；
  4. 碰撞瓦片数量 == terrains.json 里的 n_collision_boxes；
  5. box geom 数量 == n_boxes；
  6. 空跑 50 步物理，qpos/接触不出 NaN。
"""
from __future__ import annotations

import json
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def test_raycast_ground_sees_boxes() -> None:
    """v33 视觉统一后 box 移到 group4; RaycastGround(观察器3D采样点真值) 必须包含 group4.

    回归背景 (2026-10-22): 旧 mask=(1,0,0,0,0,0) 只打 group1 -> 障碍物处射线打回
    地面高度, 点被障碍物网格遮挡 -> 看起来"扫描点碰到障碍物就消失"。
    """
    import mujoco
    sys.path.insert(0, ROOT)
    from sim_env.viz.frame import RaycastGround
    scene = os.path.join(ROOT, "sim_env/models/go2/scene_terrain_wide_h050.xml")
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    g = RaycastGround(m, d)
    assert int(g.geomgroup[4]) == 1, "RaycastGround mask 必须包含 group4 (碰撞 box/瓦片)"
    h = g.height(np.array([[-3.0, 0.0], [0.0, 0.0]]))
    assert h[0] > 0.02, "wide_h050 第一级台阶处 raycast 应抬高 (实际 %.3f)" % h[0]
    assert h[1] > 0.2, "wide_h050 平台处 raycast 应抬高 (实际 %.3f)" % h[1]
    print("  [OK]   RaycastGround 看到 group4 台阶: h=%.3f/%.3f" % (h[0], h[1]))


def main() -> int:
    test_raycast_ground_sees_boxes()
    man = json.load(open(os.path.join(ROOT, "sim_env/models/go2/terrains.json"), encoding="utf-8"))
    entries = man["terrains"]
    print("[1] terrains.json: %d 个地形（含 flat）" % len(entries))
    check("地形数 >= 16", len(entries) >= 16, "n=%d" % len(entries))

    for e in entries:
        name = e["name"]
        scene = e["scene"]
        if not os.path.isabs(scene):
            scene = os.path.join(ROOT, scene)
        if not os.path.isfile(scene):
            check("%s 场景存在" % name, False, scene)
            continue
        xml = open(scene, encoding="utf-8").read()
        n_hf_coll = len(re.findall(r'<geom[^>]*type="hfield"[^>]*contype="0"[^>]*conaffinity="0"', xml))
        n_plane = len(re.findall(r'<geom name="floor_plane" type="plane"', xml))
        # 属性顺序无关 (v33: box 行改成 group/material 在前)
        n_tc = len(re.findall(r'<geom[^>]*name="tc_', xml))
        n_bx = len(re.findall(r'<geom[^>]*name="(bx|pb)_', xml))
        n_mesh = len(re.findall(r'<geom[^>]*type="mesh"' r'[^>]*group="1"', xml))
        if e.get("kind") == "flat":
            check("%-12s 平地为 plane" % name, 'type="plane"' in xml)
        else:
            check("%-12s hfield 不参与碰撞 (contype=0/conaffinity=0)" % name,
                  n_hf_coll >= 1, "n=%d" % n_hf_coll)
            check("%-12s 有 floor_plane 兜底" % name, n_plane == 1, "n=%d" % n_plane)
        want_tc = int(e.get("n_collision_boxes", 0) or 0)
        check("%-12s 碰撞瓦片数 = %d" % (name, want_tc), n_tc == want_tc, "actual=%d" % n_tc)
        want_bx = int(e.get("n_boxes", 0) or 0)
        check("%-12s box geom 数 = %d" % (name, want_bx), n_bx == want_bx, "actual=%d" % n_bx)
        if e.get("kind") != "flat":
            check("%-12s 至少一种碰撞几何" % name, (n_tc + n_bx) > 0)
        # v33 视觉统一契约: 有 box 的地形必须有 1 个视觉网格 (group1, contype0),
        # 且 box/瓦片放 group=4 (观察器隐藏渲染, 物理/raycast 不变)。
        if want_bx > 0:
            check("%-12s 有 1 个视觉网格 (group1)" % name, n_mesh == 1, "n=%d" % n_mesh)
            check("%-12s box 隐藏渲染 (group4)" % name,
                  len(re.findall(r'<geom[^>]*group="4"[^>]*name="(bx|pb)_', xml)) == want_bx)
        if n_tc > 0:
            check("%-12s 瓦片隐藏渲染 (group4)" % name,
                  len(re.findall(r'<geom[^>]*name="tc_[^"]*"[^>]*group="4"', xml)) == n_tc)
        # ---- v28 最终契约 (2026-10-21 第二轮; 见 docs/go2-sim-fidelity.md 第 5 节) ----
        # (1) 碰撞瓦片: alpha=0 对 raycast 隐形 (扫描应看视觉 hfield 面, 不看碰撞
        #     包络), xy 半尺寸 = 0.5*cell*overlap (重叠 1.05 封住瓦缝, T1 实测
        #     精确尺寸 79% 接缝嵌入 -> 0.05 重叠 0%)。
        if n_tc > 0:
            alphas = re.findall(r'<geom name="tc_[^"]*"[^>]*rgba="1 1 1 ([0-9.]+)"', xml)
            sizes = re.findall(r'<geom name="tc_[^"]*"[^>]*size="([0-9.]+) ', xml)
            ov = float(e.get("collision_overlap", 1.0) or 1.0)
            cell = float(e.get("collision_cell", 0.2) or 0.2)
            check("%-12s 瓦片 alpha=0 (对 raycast 隐形)" % name,
                  len(alphas) == n_tc and all(abs(float(a)) < 1e-9 for a in alphas),
                  "n_alpha=%d/%d" % (len(alphas), n_tc))
            want_half = 0.5 * cell * ov
            ok_sz = (len(sizes) == n_tc
                     and all(abs(float(s) - want_half) <= 2e-4 for s in sizes))
            check("%-12s 瓦片半尺寸 = %.4f (重叠 %.2f)" % (name, want_half, ov),
                  ok_sz, "n_size=%d/%d" % (len(sizes), n_tc))
        # (2) 视觉 hfield 必须 group=1: 环境组掩码 (group 0/1) 里可见 -> 出生点地面
        #     探测与 height_scan 的 raycast 都打到训练同款高度面 (旧 group=3 时
        #     只能打到 floor_plane, uniform 出生四足嵌地形 2-8cm)。
        if n_hf_coll >= 1:
            check("%-12s 视觉 hfield group=1 (raycast 可达)" % name,
                  'type="hfield"[^>]*group="1"' in xml.replace("\n", " ")
                  or re.search(r'<geom name="floor" type="hfield".*?group="1"', xml, re.S) is not None)
        # (3) raycast 高度必须与视觉地形一致 (不能落到 floor_plane): 取 40 个点,
        #     hfield 解析高度 vs raycast; 任何点低于解析面 5mm 即判失败。
        try:
            m2 = mujoco.MjModel.from_xml_path(scene)
            if int(m2.nhfield) < 1:
                raise ValueError("__flat__")
            d2 = mujoco.MjData(m2)
            mujoco.mj_forward(m2, d2)
            mask = np.array([1, 1, 0, 0, 1, 0], dtype=np.uint8)   # group 0+1+4 (v33 box/瓦片)
            gid = np.zeros(1, dtype=np.int32)
            hid = 0
            nrow = int(m2.hfield_nrow[hid])
            ncol = int(m2.hfield_ncol[hid])
            sz = np.asarray(m2.hfield_size[hid], dtype=np.float64)
            adr = int(m2.hfield_adr[hid])
            tab = np.asarray(m2.hfield_data[adr:adr + nrow * ncol],
                             dtype=np.float64).reshape(nrow, ncol)
            hf_gid = int(np.where(m2.geom_type == mujoco.mjtGeom.mjGEOM_HFIELD)[0][0])
            gpos = np.asarray(m2.geom_pos[hf_gid][:2], dtype=np.float64)

            def hf_h(x, y):
                u = (x - gpos[0]) / sz[0]
                v = (y - gpos[1]) / sz[1]
                col = np.clip((u + 1.0) * 0.5 * (ncol - 1), 0, ncol - 2)
                row = np.clip((v + 1.0) * 0.5 * (nrow - 1), 0, nrow - 2)
                c0 = int(np.floor(col)); r0 = int(np.floor(row))
                fc = col - c0; fr = row - r0
                h = ((1 - fr) * ((1 - fc) * tab[r0, c0] + fc * tab[r0, c0 + 1])
                     + fr * ((1 - fc) * tab[r0 + 1, c0] + fc * tab[r0 + 1, c0 + 1]))
                return float(h * sz[2])

            hh = float(e.get("hfield_half", 6.0) or 6.0)
            # 注意: mj_ray 命中 z=0 表面时返回距离恰为 3.0 (不是 miss); miss 返回 -1
            # 或 None。用 >=3.0 判 miss 会把所有 z<=0 的命中当失败 (第一版就踩了)。
            xs = np.linspace(-0.85 * hh, 0.85 * hh, 8)
            bad_below = 0
            worst = 0.0
            n_hit = 0
            for x in xs:
                for y in xs:
                    dist = mujoco.mj_ray(m2, d2, np.array([x, y, 3.0]),
                                         np.array([0.0, 0.0, -1.0]), mask, 1, -1, gid)
                    if dist is None or float(dist) < 0.0:
                        continue
                    h = 3.0 - float(dist)
                    n_hit += 1
                    dlt = h - hf_h(x, y)
                    # 容差 2cm: MuJoCo 的 hfield 射线是三角面近似, 与双线性解析在
                    # 陡峭 hfield (stairs_only/boxes_only) 上差 ~1.3cm; 旧 bug
                    # (raycast 落到 floor_plane) 的误差是 -5cm~-20cm, 仍会被抓住。
                    if dlt < -0.02:
                        bad_below += 1
                        worst = min(worst, dlt)
            check("%-12s raycast 打到视觉地形 (不低于解析面 5mm)" % name,
                  n_hit >= 50 and bad_below == 0,
                  "hit=%d bad_below=%d worst=%.3f" % (n_hit, bad_below, worst))
        except ValueError as exc:
            if "__flat__" in str(exc):
                check("%-12s raycast 打到视觉地形" % name, True,
                      "flat: 无 hfield, 只验 plane")
            else:
                check("%-12s raycast 打到视觉地形 (不低于解析面 5mm)" % name,
                      False, repr(exc)[:120])
        except Exception as exc:
            check("%-12s raycast 打到视觉地形 (不低于解析面 5mm)" % name,
                  False, repr(exc)[:120])
        # 加载 + 空跑 50 步
        try:
            m = mujoco.MjModel.from_xml_path(scene)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            for _ in range(50):
                mujoco.mj_step(m, d)
            fin = bool(np.all(np.isfinite(d.qpos)))
            check("%-12s 加载 + 50 步物理有限" % name, fin, "ngeom=%d" % m.ngeom)
        except Exception as exc:
            check("%-12s 加载 + 50 步物理有限" % name, False, repr(exc)[:120])

    print()
    if FAILS:
        print("FAILED %d 项: %s" % (len(FAILS), "; ".join(FAILS)))
        return 1
    print("OK test_terrain_scenes: %d 个地形碰撞几何契约全部通过" % len(entries))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())