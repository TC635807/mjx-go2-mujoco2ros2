#!/usr/bin/env python3
"""Go2 标称站姿可站立性自检 (CPU MuJoCo)。

背景 (task-5 §2)
----------------
部署模型 models/go2/go2.xml 的 keyframe home = 每腿 (0, 0.9, -1.8) 是**真机出厂站立姿态**,
不是训练姿态。实测该姿态 + PD(kp=20, kd=1) 在 CPU MuJoCo 下站不住: 3 s 内 z 掉到 0.19 m、
倾角 8-18 且继续下沉。Go2 开源惯例 (unitree_rl_gym) 的 home 才稳:

    FR -0.1 / FL +0.1 / RR -0.1 / RL +0.1, 前腿 thigh 0.8, 后腿 thigh 1.0, calf 全 -1.5

本脚本用**部署侧的同一套 LowCmd + Go2Sim.write_cmd** 路径, 对若干姿态 × 接触档位各跑一段
纯站姿闭环, 打印 z / 倾角时间序列与是否满足 eval_go2 的健康判据。

    python scripts/check_go2_stand.py                    # 默认: 配置里的新姿态 + 新旧对比
    python scripts/check_go2_stand.py --duration 10 --json /tmp/go2_stand.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

import mujoco  # noqa: E402

from signal_io.config import load_config, model_xml, robot_spec_of  # noqa: E402
from signal_io.msg import NUM_JOINT  # noqa: E402
from sim_env.robot import Go2Sim, make_stand_cmd  # noqa: E402

# 部署模型 keyframe home (真机出厂站立姿态)
OFFICIAL_HOME = np.array([0.0, 0.9, -1.8] * 4)
# unitree_rl_gym 惯例 home; 顺序 = FR,FL,RR,RL x (hip,thigh,calf)
UNITREE_HOME = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
                         -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
# eval_go2.py 的健康判据 (Go2)
HEALTHY_Z = (0.19, 0.65)
HEALTHY_TILT_DEG = 10.0


def tilt_deg(model, data, body: int) -> float:
    q = np.asarray(data.xquat[body], dtype=np.float64)
    w, x, y, z = q
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return float(np.degrees(np.arccos(np.clip(r22, -1.0, 1.0))))


def run_stand(xml, home, z, duration, kp, kd, soft_feet, iters, ls_iters=None, label=""):
    spec = robot_spec_of({"robot": "go2", "robots": {}})
    spec.stand_pose = np.asarray(home, dtype=np.float64)
    spec.stand_pose_from_xml = False
    spec.stand_height = float(z)
    sim = Go2Sim(xml, spec=spec, sim_dt=0.002, foot_soft_contact=soft_feet,
                 solver_iterations=iters,
                 **({"solver_ls_iterations": ls_iters} if ls_iters is not None else {}))
    cmd = make_stand_cmd(sim.stand_pose, kp=kp, kd=kd)     # 与 SimNode 的 hold 指令同一条路径
    n = int(round(duration / sim.dt))
    samples, tmax = [], 0.0
    for k in range(n):
        sim.write_cmd(cmd)
        sim.step(1)
        tl = tilt_deg(sim.model, sim.data, sim.base_body)
        tmax = max(tmax, tl)
        if k % int(round(0.5 / sim.dt)) == 0:
            samples.append({"t": round(k * sim.dt, 2), "z": round(float(sim.data.qpos[2]), 4),
                            "tilt_deg": round(tl, 2),
                            "x": round(float(sim.data.qpos[0]), 4)})
    z_end = float(sim.data.qpos[2])
    tl_end = tilt_deg(sim.model, sim.data, sim.base_body)
    ok = (HEALTHY_Z[0] <= z_end <= HEALTHY_Z[1]) and tl_end <= HEALTHY_TILT_DEG
    return {"label": label, "xml": os.path.relpath(xml, PROJECT_ROOT), "home": [float(v) for v in home],
            "spawn_z": float(z), "kp": kp, "kd": kd, "soft_feet": bool(soft_feet),
            "iters": iters, "ls_iters": ls_iters, "duration_s": duration,
            "z_end": round(z_end, 4), "tilt_end_deg": round(tl_end, 2),
            "z_min": round(float(min(s["z"] for s in samples)), 4),
            "tilt_max_deg": round(tmax, 2),
            "drift_x": round(float(sim.data.qpos[0]), 4),
            "vx_end": round(float(sim.data.qvel[0]), 4),
            "healthy_at_end": bool(ok), "samples": samples}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(PROJECT_ROOT, "config", "go2_sim.json"))
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config, overrides={"robot": "go2"})
    xml = model_xml(cfg)
    spec = robot_spec_of(cfg)
    print("部署模型      :", os.path.relpath(xml, PROJECT_ROOT))
    print("config 站姿   :", np.round(spec.stand_pose, 3).tolist(), " z =", spec.stand_height)
    print("config 接触档 : soft_feet=%s iters=%s/%s" % (
        cfg["foot_soft_contact"], cfg["solver_iterations"], cfg["solver_ls_iterations"]))
    print("kp/kd         : %g / %g   时长 %g s" % (args.kp, args.kd, args.duration))
    print()

    cases = [
        # (标签, 姿态, 出生 z, 软接触, iters, ls)
        ("部署 keyframe home (0,0.9,-1.8) + 硬接触 [旧默认]",
         OFFICIAL_HOME, 0.27, False, 100, 50),
        ("unitree_rl_gym home + 硬接触",
         UNITREE_HOME, 0.32, False, 100, 50),
        ("unitree_rl_gym home + 软接触 [新默认]",
         UNITREE_HOME, 0.32, True, 100, 50),
        ("unitree_rl_gym home + 软接触 + 2/5 迭代 (MJX warp 档)",
         UNITREE_HOME, 0.32, True, 2, 5),
        ("config 实际取值 (照抄 cfg)",
         spec.stand_pose, spec.stand_height or 0.32, bool(cfg["foot_soft_contact"]),
         cfg["solver_iterations"], cfg["solver_ls_iterations"]),
    ]
    out = []
    print("%-52s %8s %8s %8s %9s %8s %s" % (
        "姿态 / 接触档", "z_end", "z_min", "tilt_end", "tilt_max", "drift_x", "10s 内站住?"))
    print("-" * 118)
    for label, home, z, soft, it, ls in cases:
        r = run_stand(xml, home, z, args.duration, args.kp, args.kd, soft, it, ls, label=label)
        out.append(r)
        print("%-52s %8.4f %8.4f %8.2f %9.2f %8.4f %s" % (
            label, r["z_end"], r["z_min"], r["tilt_end_deg"], r["tilt_max_deg"],
            r["drift_x"], "是" if r["healthy_at_end"] else "**否**"))

    print()
    print("新默认档 (unitree_rl_gym + 软接触) 时间序列:")
    for r in out:
        if "新默认" in r["label"]:
            for s in r["samples"]:
                print("   t=%5.1fs  z=%.4f  tilt=%5.2f°  x=%+.4f" % (
                    s["t"], s["z"], s["tilt_deg"], s["x"]))
            break

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"cases": out, "healthy_z": HEALTHY_Z,
                       "healthy_tilt_deg": HEALTHY_TILT_DEG}, f, ensure_ascii=False, indent=2)
        print("\n[json]", args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
