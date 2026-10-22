#!/usr/bin/env python
"""检查控制器的 height_scan (SimLidar+HeightMapper) 与地面真值 (全组 raycast) 是否一致。

用法: python scripts/probe_height_scan_ground_truth.py --terrain wide_h050 --x -3.6 --y 0
输出: 187 个采样点里按 x 排列的 (世界xy, 高程图高度, 真值高度, height_scan 特征) 摘要;
      以及"观测到的格点"上的 max|h_map-h_true|。

背景 (2026-10-22): 观察器里 3D 叠加点用的是 RaycastGround 真值, 不是模型输入;
v33 把 box 从 group0 移到 group4 后, RaycastGround 的旧 mask=(1,0,0,0,0,0) 看不到
box -> 点掉进障碍物/被遮挡, 看起来"扫描点碰到障碍物就消失"。本脚本用于区分:
* 模型输入 (SimLidar -> HeightMapper -> scan) 是否正确;
* 观察器真值 raycast 的 group mask 是否正确。
"""
import argparse
import os
import sys

import numpy as np
import mujoco

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from sim_env.robot import Go2Sim  # noqa: E402
from signal_io.perception import HeightMapper, SimLidar, scan_grid_body  # noqa: E402
from sim_env.viz.frame import RaycastGround  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain", default="wide_h050")
    ap.add_argument("--x", type=float, default=-3.6)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.32)
    ap.add_argument("--steps", type=int, default=5)
    args = ap.parse_args()
    scene = os.path.join(ROOT, "sim_env/models/go2/scene_terrain_%s.xml" % args.terrain)
    sim = Go2Sim(scene, sim_dt=0.002)
    sim.reset()
    sim.data.qpos[0] = args.x
    sim.data.qpos[1] = args.y
    sim.data.qpos[2] = args.z
    sim.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(sim.model, sim.data)

    lidar = SimLidar(sim, n_rays=2000, self_occlusion=True)
    mapper = HeightMapper(fill_local=True)
    for k in range(args.steps):
        mapper.update(lidar.sample(), t=0.1 * k)
    base = np.asarray(sim.data.qpos[:3]); quat = np.asarray(sim.data.qpos[3:7])
    scan = mapper.scan(base, quat)

    # 真值: 全环境组 raycast (group0/1/4), 排除机器人
    truth = RaycastGround(sim.model, sim.data).height
    g = scan_grid_body(mapper.nx, mapper.ny, mapper.x_range, mapper.y_range)
    yaw = float(np.arctan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                           1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2)))
    c, s = float(np.cos(yaw)), float(np.sin(yaw))
    xw = c * g[:, 0] - s * g[:, 1] + base[0]
    yw = s * g[:, 0] + c * g[:, 1] + base[1]
    hm = mapper.ground_height(np.stack([xw, yw], axis=-1), fallback_height=base[2] - 0.268)
    ht = truth(np.stack([xw, yw], axis=-1))
    known = mapper.observed(np.stack([xw, yw], axis=-1))
    err = np.abs(hm - ht)[known]
    print("[probe] terrain=%s robot=(%.2f,%.2f,%.2f) lidar_mask=%s hits=%.2f"
          % (args.terrain, args.x, args.y, args.z, lidar.geom_group_mask,
             lidar.stats()["hit_rate"]))
    print("[probe] 187 点中直接观测 %d 个; 观测点上 max|h_map-h_true|=%.3f m, mean=%.3f m"
          % (int(known.sum()), float(err.max()) if err.size else -1.0,
             float(err.mean()) if err.size else -1.0))
    print("   i  x_world  y_world   h_map  h_true  scan_feat")
    ny = mapper.ny
    for i in range(0, g.shape[0], ny):
        print("  %2d %+7.3f %+7.3f  %+6.3f  %+6.3f  %+6.2f"
              % (i, xw[i], yw[i], hm[i], ht[i], scan[i]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
