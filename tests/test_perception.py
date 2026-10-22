#!/usr/bin/env python3
"""tests/test_perception.py —— Go2 感知链路 (SimLidar ray-cast + HeightMapper) 验收。

    python tests/test_perception.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from signal_io.config import load_config, model_xml  # noqa: E402
from signal_io.perception import (  # noqa: E402
    HeightMapper, SimLidar, scan_grid_body,
)
from sim_env.robot import Go2Sim  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def plane_points(h, x0=-0.6, x1=1.4, y0=-0.8, y1=0.8, step=0.02):
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([X.ravel(), Y.ravel(), np.full(X.size, float(h))], axis=-1)


def main() -> int:
    cfg = load_config(os.path.join(ROOT, "config", "go2_sim.json"))
    scene_flat = os.path.join(ROOT, cfg["robots"]["go2"]["scene_flat"])
    scene = scene_flat

    print("[1] scan 网格: Go2 17x11")
    grid = scan_grid_body()
    check("scan_grid_body 形状 (187,2)", grid.shape == (187, 2), "shape=%r" % (grid.shape,))
    check("第一点 = (-0.4,-0.5) / 最后 = (1.2,0.5)",
          np.allclose(grid[0], [-0.4, -0.5]) and np.allclose(grid[-1], [1.2, 0.5]),
          "%s / %s" % (grid[0], grid[-1]))
    m0 = HeightMapper()
    check("HeightMapper 默认与 scan_grid_body 一致",
          np.array_equal(m0.scan_grid(), grid), "shape=%r" % (m0.scan_grid().shape,))

    base_cfg = float(cfg.get("scan_base") or 0.268)
    print("[2] 平地: SimLidar ray-cast -> HeightMapper.scan 应 ≈ (z_base-scan_base)*5")
    sim = Go2Sim(scene)
    sim.reset(height=float(cfg.get("stand_height") or 0.32))
    base = np.asarray(sim.data.qpos[:3], dtype=np.float64).copy()
    quat = np.asarray(sim.data.qpos[3:7], dtype=np.float64).copy()
    lidar = SimLidar(sim, n_rays=3000, self_occlusion=True)
    pts = lidar.sample()
    st = lidar.stats()
    check("点云 (N,3)", pts.ndim == 2 and pts.shape[1] == 3, "N=%d" % len(pts))
    check("射线命中率 > 0.4", st["hit_rate"] > 0.4,
          "hit=%d/%d rate=%.3f" % (st["n_hits"], st["n_rays_cast"], st["hit_rate"]))
    m = HeightMapper(fill_local=True)
    m.update(pts, 0.0)
    scan = np.asarray(m.scan(base, quat), dtype=np.float64)
    # Go2: 出生站高 0.32, 训练 scan_base=0.268 -> 平地特征 = (0.32-0.268)*5 = 0.26
    expect = (float(base[2]) - base_cfg) * 5.0
    frac = float(np.mean(np.abs(scan - expect) <= 0.05))
    check("平地上 scan ≈ (z_base-scan_base)*5 = %.2f (>=80%% 格点, 其余未观测=0)" % expect,
          frac >= 0.8 and float(np.max(scan)) <= expect + 0.05 and float(np.min(scan)) >= -0.05,
          "frac=%.2f max=%.3f min=%.3f" % (frac, float(np.max(scan)), float(np.min(scan))))
    check("有真实观测到的扫描点", m.last_scan_observed >= 5,
          "observed=%d/187" % m.last_scan_observed)

    print("[3] 特征定义: scan = clip(z_base - h_ground - base, ±0.35)*5")
    ident = np.array([1.0, 0.0, 0.0, 0.0])
    m_flat = HeightMapper(base=base_cfg, fill_local=True)
    m_flat.update(plane_points(0.0), 0.0)
    s_hi = np.asarray(m_flat.scan(np.array([0.0, 0.0, base_cfg + 0.07]), ident))
    check("平地 + z=base+0.07 -> scan ≈ +0.35 (clip)",
          float(np.max(np.abs(s_hi - 0.35))) < 0.02, "scan=%.4f" % float(s_hi[0]))
    m_high = HeightMapper(base=base_cfg, fill_local=True)
    m_high.update(plane_points(0.1), 0.0)
    s_lo = np.asarray(m_high.scan(np.array([0.0, 0.0, base_cfg]), ident))
    check("地面抬高 0.1m -> scan ≈ -0.5",
          float(np.max(np.abs(s_lo + 0.5))) < 0.03, "scan=%.4f" % float(s_lo[0]))

    print("[4] 台阶可见性: 抬高的台阶在 scan 里出现负值")
    pts_step = np.concatenate([plane_points(0.0), plane_points(0.2, -0.4, 0.4, -0.5, 0.5, 0.02)])
    m_step = HeightMapper(base=base_cfg, fill_local=True)
    m_step.update(pts_step, 0.0)
    scan_step = np.asarray(m_step.scan(np.array([0.0, 0.0, base_cfg]), ident))
    check("台阶区域 scan 出现负值", float(np.min(scan_step)) < -0.5,
          "min=%.3f max=%.3f" % (float(np.min(scan_step)), float(np.max(scan_step))))

    print()
    if FAILS:
        print("FAILED %d 项: %s" % (len(FAILS), "; ".join(FAILS)))
        return 1
    print("OK test_perception: Go2 感知链路全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
