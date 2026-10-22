#!/usr/bin/env python3
"""height_scan 来源对照: leg_fk (部署框架默认) vs 引擎真值 h_ground (训练口径) (task-5 §4)。

训练口径 (envs/go2_walk.py:714-745 / eval_go2.py:height_scan):
    feature_i = clip(z_base - h_ground(xy_i) - base, ±0.35) * 5.0     i = 0..34 逐点
    xy_i = 基座 xy + Rz(yaw) @ 网格点   (nx=7, ny=5, x∈[-0.3,0.9], y∈[-0.4,0.4], x 外层)

部署框架 leg_fk (ros2_controller/controller.py:205-251):
    zc = median( -foot_z_body + foot_radius )  取**接触腿** (foot_force>thresh) 的 FK 结果
    feature = clip(zc - 0, ±0.35) * 5.0          <<< 35 个点填**同一个值**
    (代码注释: "平地假设: 35 个采样点的地面高度都是 0 (真地形走 perception.HeightMapper)")

本脚本把机器人在同一片 hfield 地形上摆到一批位置, 每个位置用 kp=20/kd=1 站 0.2 s 让足端真正
接触, 然后同时算两套特征并比较。**只做数值对照, 不改任何东西。**

    python scripts/probe_height_scan_sources.py --grid 11 --json /tmp/go2_scan_sources.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GETUP = "/home/tc63/mujoco/mjx-go2-training"
for _p in (PROJECT_ROOT, os.path.join(PROJECT_ROOT, "scripts"),
           os.path.join(GETUP, "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco  # noqa: E402

import eval_go2 as E  # noqa: E402                      # 训练口径的引擎真值实现
from compare_deploy_vs_train_physics import build_deploy_terrain  # noqa: E402
from signal_io.config import load_config, robot_spec_of  # noqa: E402
from signal_io.estimator import LegKinematics  # noqa: E402
from ros2_controller.obs import WalkObsBuilder, WalkObsConfig  # noqa: E402
from signal_io.perception import HeightMapper, SimLidar  # noqa: E402
from sim_env.robot import Go2Sim, make_stand_cmd  # noqa: E402

DEPLOY_BODY = os.path.join(PROJECT_ROOT, "sim_env", "models", "go2", "go2.xml")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=int, default=11, help="地形上的采样格 n x n")
    ap.add_argument("--extent", type=float, default=4.0, help="采样半边长 (m); hfield 半径 6")
    ap.add_argument("--settle-s", type=float, default=0.2, help="每个位置先站多久让足端接触")
    ap.add_argument("--workdir", default=os.path.join(PROJECT_ROOT, "logs", "go2_parity_eval", "_scenes"))
    ap.add_argument("--raycast", action="store_true", default=True,
                    help="同时算 perception.HeightMapper (SimLidar 点云) 那一档，作为可用的替代")
    ap.add_argument("--no-raycast", dest="raycast", action="store_false")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    cfg = load_config(os.path.join(PROJECT_ROOT, "config", "go2_sim.json"),
                      overrides={"robot": "go2"})
    scan_base = float(cfg.get("_train_scan_base", 0.268))
    home_q = np.asarray(cfg.get("_train_home_q"), dtype=np.float64)
    xml = build_deploy_terrain(os.path.join(args.workdir, "deploy_terrain.xml"))
    print("地形场景     :", os.path.relpath(xml, PROJECT_ROOT), "(部署 body + 训练 hfield)")
    print("obs home     :", np.round(home_q, 3).tolist())
    print("scan_base    : %.4f (= 策略训练机型 go2 的值)" % scan_base)

    # ---- 训练口径 (引擎真值) ----
    pa = E.build_parser().parse_args([
        "--xml", xml, "--solver", "cpu", "--home-pose", "unitree_rl_gym",
        "--spawn", "origin", "--spawn-z", "0.32", "--reset-noise", "0",
        "--foot-contact", "xml", "--seeds", "1", "--duration", "1", "--quiet", "--no-plot"])
    ev = E.Go2Sim(xml, E.finalize_args(pa))

    # ---- 部署框架路径 (motor + PD + config 站姿) ----
    sim = Go2Sim(xml, spec=robot_spec_of(cfg), sim_dt=0.002,
                 foot_soft_contact=bool(cfg["foot_soft_contact"]),
                 solver_iterations=cfg["solver_iterations"],
                 solver_ls_iterations=cfg["solver_ls_iterations"])
    sim.stand_pose = np.asarray(home_q, dtype=np.float64)
    if sim.spec is not None:
        sim.spec.stand_pose = sim.stand_pose
    kin = LegKinematics(DEPLOY_BODY)
    obs_b = WalkObsBuilder(WalkObsConfig(scan_base=scan_base), home_q=home_q)
    hold = make_stand_cmd(home_q, kp=20.0, kd=1.0)

    # ---- 可选: perception.HeightMapper (SimLidar 射线 -> 世界系点云 -> 稀疏高程图) ----
    #      这就是"lidar 可用时切到 raycast"的那条路: height_scan_fn 一挂, PolicyController
    #      的 _height_scan 就用它 (source 标 "external")，不再走 leg_fk。
    perc = dict(cfg.get("perception") or {})
    mapper = lidar = None
    if args.raycast:
        mapper = HeightMapper(
            base=scan_base,
            resolution=float(perc.get("map_resolution", 0.05)),
            decay_s=float(perc.get("map_decay_s", 1.0)),
            fill_local=bool(perc.get("fill_local", True)))
        lidar = SimLidar(sim, n_rays=int(perc.get("n_rays", 2000)),
                         vfov=tuple(perc.get("vfov", (-0.5, 0.2))),
                         max_range=float(perc.get("max_range", 8.0)),
                         height_above_ground=float(perc.get("height_above_ground", 0.30)),
                         nominal_base_height=scan_base,
                         mount_body=perc.get("mount_body"),
                         min_range=float(perc.get("min_range", 0.05)),
                         self_occlusion=bool(perc.get("self_occlusion", False)))
        print("raycast 档    : SimLidar n_rays=%d vfov=%s max_range=%.1f 安装高=%.2f -> "
              "HeightMapper(res=%.2f, base=%.3f)" % (
                  lidar.n_rays, lidar.vfov, lidar.max_range, lidar.height_above_ground,
                  mapper.resolution, mapper.base))

    g = np.linspace(-args.extent, args.extent, int(args.grid))
    n_settle = int(round(args.settle_s / sim.dt))
    truth_rows, legfk_rows, ray_rows, meta = [], [], [], []
    observed_rows = []
    for x in g:
        for y in g:
            h = float(ev.terrain_height(np.array([[x, y]]))[0])
            sim.stand_height = h + 0.32
            sim.reset()
            sim.data.qpos[0], sim.data.qpos[1] = x, y
            mujoco.mj_forward(sim.model, sim.data)
            for _ in range(n_settle):
                sim.write_cmd(hold)
                sim.step(1)
            # (a) 引擎真值: eval_go2.height_scan 用同一份 hfield, 逐点算
            truth = ev.height_scan(data=sim.data)
            # (b) 部署框架 leg_fk: 接触腿 FK 中位数 -> 单个标量 -> 35 个点填同一个值
            st = sim.read_state()
            P = kin.foot_positions_body(st.q12())
            ff = np.asarray(st.foot_force, dtype=np.float64)
            zs = -P[:, 2] + 0.023
            zc = float(np.median(zs[ff > 30.0])) if (ff > 30.0).any() else float(zs.min())
            legfk = obs_b.scan_feature(np.full(35, zc), np.zeros(35))
            if mapper is not None:
                mapper.reset()
                # 让高程图先被观测几拍（真机上也是一直在更新），再取一次扫描
                for _ in range(3):
                    mapper.update(lidar.sample(), float(sim.time))
                ray = mapper.scan(sim.data.qpos[:3], sim.data.qpos[3:7])
                ray_rows.append(np.asarray(ray, dtype=np.float64))
                observed_rows.append(int(mapper.last_scan_observed))
            rpy = sim.orientation_rpy()
            truth_rows.append(np.asarray(truth, dtype=np.float64))
            legfk_rows.append(np.asarray(legfk, dtype=np.float64))
            meta.append({"x": round(float(x), 3), "y": round(float(y), 3),
                         "h_ground": round(h, 4), "zc_leg_fk": round(zc, 4),
                         "z_base": round(float(sim.data.qpos[2]), 4),
                         "tilt_deg": round(float(np.degrees(max(abs(rpy[0]), abs(rpy[1])))), 2),
                         "n_contact_leg": int((ff > 30.0).sum())})
    T = np.array(truth_rows)
    L = np.array(legfk_rows)
    D = T - L
    print()
    print("采样 %d 个位置 (x,y ∈ [%.1f,%.1f], 每处先站 %.2f s)" % (
        len(meta), -args.extent, args.extent, args.settle_s))
    print("站位倾角 max %.1f°  中位 %.1f°   接触腿数中位 %d" % (
        max(m["tilt_deg"] for m in meta),
        float(np.median([m["tilt_deg"] for m in meta])),
        int(np.median([m["n_contact_leg"] for m in meta]))))
    print()
    print("引擎真值特征 (35 维): mean %.3f  std %.3f  min %.3f  max %.3f" % (
        T.mean(), T.std(), T.min(), T.max()))
    print("leg_fk   特征 (35 维): mean %.3f  std %.3f  min %.3f  max %.3f" % (
        L.mean(), L.std(), L.min(), L.max()))
    print("  -> leg_fk 在**同一位置**的 35 个分量互相之间的最大极差 = %.3e "
          "(= 常数向量, 没有任何空间信息)" % float((L.max(axis=1) - L.min(axis=1)).max()))
    print()
    print("逐点差值 (真值 - leg_fk), 单位 = 特征值 (1.0 特征 ≈ 0.2 m):")
    print("  mean %+.3f  std %.3f   p50 |Δ| %.3f   p95 |Δ| %.3f   max |Δ| %.3f" % (
        D.mean(), D.std(), float(np.percentile(np.abs(D), 50)),
        float(np.percentile(np.abs(D), 95)), float(np.abs(D).max())))
    over2cm = float((np.abs(D) / 5.0 > 0.02).mean())
    print("  |Δ| 折成米 = |Δ|/5:  p50 %.4f m   p95 %.4f m   max %.4f m" % (
        float(np.percentile(np.abs(D), 50)) / 5, float(np.percentile(np.abs(D), 95)) / 5,
        float(np.abs(D).max()) / 5))
    print("  超过 2 cm 的 (位置 x 维) 占比 = %.1f%%   (阈值: 部署侧给策略的扫描特征已经废掉)" % (
        100 * over2cm))
    if ray_rows:
        R = np.array(ray_rows)
        DR = T - R
        print()
        print("raycast 档 (SimLidar + HeightMapper) 对照真值:")
        print("  特征 mean %.3f  std %.3f  min %.3f  max %.3f" % (
            R.mean(), R.std(), R.min(), R.max()))
        print("  |Δ| = |真值 - raycast|:  p50 %.3f  p95 %.3f  max %.3f  (折成米 %.4f / %.4f / %.4f)" % (
            float(np.percentile(np.abs(DR), 50)), float(np.percentile(np.abs(DR), 95)),
            float(np.abs(DR).max()), float(np.percentile(np.abs(DR), 50)) / 5,
            float(np.percentile(np.abs(DR), 95)) / 5, float(np.abs(DR).max()) / 5))
        print("  超过 2 cm 的占比 = %.1f%%   (leg_fk 是 %.1f%%)" % (
            100 * float((np.abs(DR) / 5.0 > 0.02).mean()), 100 * over2cm))
        if observed_rows:
            print("  高程图: 每帧直接观测到的格点 = %.1f / %d (%.1f%%); "
                  "lidar 命中率 %.2f, 自遮挡射线 %d"
                  % (float(np.mean(observed_rows)), 35,
                     100.0 * float(np.mean(observed_rows)) / 35.0,
                     lidar.stats()["hit_rate"], lidar.stats()["n_occluded"]))
    else:
        R = DR = None

    print()
    per_dim = [{"dim": i, "truth_mean": float(T[:, i].mean()), "truth_std": float(T[:, i].std()),
                "legfk_mean": float(L[:, i].mean()), "legfk_std": float(L[:, i].std()),
                "d_mean": float(D[:, i].mean()), "d_std": float(D[:, i].std()),
                "d_absmax": float(np.abs(D[:, i]).max())} for i in range(35)]
    print("  逐维 (前 10 维; 网格 index = i*ny + j, ny=5 -> 第 i 行是 x 方向的第 i 个采样环):")
    print("  %4s %10s %10s %10s %10s %10s" % ("dim", "真值mean", "真值std", "legfk", "Δmean", "|Δ|max"))
    for r in per_dim[:10]:
        print("  %4d %10.3f %10.3f %10.3f %+10.3f %10.3f" % (
            r["dim"], r["truth_mean"], r["truth_std"], r["legfk_mean"], r["d_mean"], r["d_absmax"]))
    print("  (其余 %d 维同形, 见 JSON)" % (35 - 10))

    out = {"n_positions": len(meta), "scan_base": scan_base, "home_q": home_q.tolist(),
           "raycast": (None if R is None else {
               "stats": {"mean": float(R.mean()), "std": float(R.std()),
                         "min": float(R.min()), "max": float(R.max())},
               "diff": {"p50_abs": float(np.percentile(np.abs(DR), 50)),
                        "p95_abs": float(np.percentile(np.abs(DR), 95)),
                        "max_abs": float(np.abs(DR).max()),
                        "p95_abs_m": float(np.percentile(np.abs(DR), 95)) / 5.0,
                        "frac_gt_2cm": float((np.abs(DR) / 5.0 > 0.02).mean())},
               "mapper": mapper.stats(), "lidar": lidar.stats()}),
           "truth_stats": {"mean": float(T.mean()), "std": float(T.std()),
                           "min": float(T.min()), "max": float(T.max())},
           "legfk_stats": {"mean": float(L.mean()), "std": float(L.std()),
                           "min": float(L.min()), "max": float(L.max()),
                           "const_across_dims": float((L.max(axis=1) - L.min(axis=1)).max())},
           "diff": {"mean": float(D.mean()), "std": float(D.std()),
                    "p50_abs": float(np.percentile(np.abs(D), 50)),
                    "p95_abs": float(np.percentile(np.abs(D), 95)),
                    "max_abs": float(np.abs(D).max()),
                    "p95_abs_m": float(np.percentile(np.abs(D), 95)) / 5.0,
                    "frac_gt_2cm": over2cm},
           "per_dim": per_dim, "samples": meta}
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2, default=float)
        print("\n[json]", args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
