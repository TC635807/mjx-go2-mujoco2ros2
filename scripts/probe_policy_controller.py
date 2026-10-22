#!/usr/bin/env python3
"""把 **PolicyController 本体**放进单进程闭环跑（不经过任何传输层）。

目的：把"策略/观测/控制器的实现问题"和"传输层（ROS2 丢帧、延迟、时钟）问题"分开。
同一套 config + 同一个控制器构建函数（直接复用 scripts/run_controller.py 的
build_policy_controller），所以和 ROS2 回路里跑的是**同一份代码**。

    python scripts/probe_policy_controller.py --robot go2 --duration 12
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from signal_io.config import load_config, model_xml, robot_spec_of  # noqa: E402
from signal_io.msg import NUM_JOINT, LowCmd  # noqa: E402
from sim_env.robot import Go2Sim  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="go2", choices=["go2"])
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--control-dt", type=float, default=0.02)
    ap.add_argument("--scan", default=None, choices=["nominal", "leg_fk"])
    ap.add_argument("--prepose-s", type=float, default=None)
    ap.add_argument("--policy-profile", default=None, help="策略档: auto(默认, 按 --robot 选)/go2/go2")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    from ros2_controller import run_controller as rc

    cfg = load_config(os.path.join(ROOT, "config", "go2_sim.json"),
                      overrides={"robot": args.robot, "policy_profile": args.policy_profile})
    if args.scan:
        cfg["policy"]["height_scan_source"] = args.scan
    if args.prepose_s is not None:
        cfg["policy"]["prepose_s"] = args.prepose_s
    xml = model_xml(cfg)
    flat = os.path.join(os.path.dirname(xml), "scene_flat.xml")
    if os.path.isfile(flat):
        xml = flat
    # go2 按 config 的 robots.go2 档
    if args.robot == "go2":
        soft = bool(cfg.get("foot_soft_contact", False))
        iters, ls_iters = cfg.get("solver_iterations"), cfg.get("solver_ls_iterations")
    else:
        soft, iters, ls_iters = True, None, None
    sim = Go2Sim(xml, spec=robot_spec_of(cfg), sim_dt=cfg["sim_dt"], foot_soft_contact=soft,
                 solver_iterations=iters, solver_ls_iterations=ls_iters)
    print("[probe] 站姿 =", np.round(sim.stand_pose, 3).tolist(), "| z =", sim.stand_height,
          "| 足端 =", "soft" if soft else "xml", "| 求解器 =",
          sim.model.opt.iterations, "/", sim.model.opt.ls_iterations)
    ctrl = rc.build_policy_controller(cfg, sim)
    ctrl.reset()

    decim = int(round(args.control_dt / sim.dt))
    n = int(round(args.duration / sim.dt))
    trace = []
    for k in range(n):
        if k % decim == 0:
            st = sim.read_state()
            cmd = ctrl.update(st, float(sim.time))
        sim.write_cmd(cmd)
        sim.step(1)
        if k % 100 == 0:
            rpy = np.degrees(sim.orientation_rpy())
            trace.append({"t": round(sim.time, 2), "x": round(float(sim.data.qpos[0]), 3),
                          "z": round(sim.base_height, 3),
                          "tilt": round(float(max(abs(rpy[0]), abs(rpy[1]))), 2)})
    rpy = np.degrees(sim.orientation_rpy())
    out = {"robot": args.robot, "scan": cfg["policy"].get("height_scan_source"),
           "prepose_s": cfg["policy"].get("prepose_s"),
           "final": {"x": round(float(sim.data.qpos[0]), 3), "z": round(sim.base_height, 3),
                     "tilt_deg": round(float(max(abs(rpy[0]), abs(rpy[1]))), 2),
                     "upright": bool(sim.is_upright(30.0))},
           "controller": ctrl.stats(), "trace": trace}
    print(json.dumps({k: v for k, v in out.items() if k != "trace"}, ensure_ascii=False, indent=1))
    print("trace:", [ (r["t"], r["x"], r["z"], r["tilt"]) for r in trace[::4] ])
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
