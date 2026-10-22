#!/usr/bin/env python3
"""可行性探针：把训练好的 Go2 策略放到 **Go2** 模型上闭环跑，看它到底站不站得住。

这是"先量后改"：移植模型之前，先知道域差有多大，别把"策略本来就不适配"误判成
"我们的观测/接口搭错了"。

观测按 docs/interfaces.md §1 的 91 维定义构造（height scan 在平地场景上恒为 0）。
线速度这里用**仿真真值**（oracle），用来隔离"速度估计误差"这一个变量。

    python scripts/probe_policy_go2.py --duration 8
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from signal_io.config import load_config, model_xml, robot_spec_of  # noqa: E402
from signal_io.msg import NUM_JOINT, LowCmd  # noqa: E402
from ros2_controller.policy import NumpyMLPPolicy, load_policy_meta  # noqa: E402
from sim_env.robot import Go2Sim, _quat_to_rpy  # noqa: E402

TRAINING_HOME_Q = np.array([0.0, 0.8, -1.5, 0.0, 0.8, -1.5,
                            0.0, 1.0, -1.5, 0.0, 1.0, -1.5])
TROT_PHASE0 = np.array([0.0, np.pi, np.pi, 0.0])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="go2", choices=["go2"],
                    help="go2 = 部署侧真机模型")
    ap.add_argument("--model", default=None, help="覆盖模型路径（默认按机型解析）")
    ap.add_argument("--scene", default="flat", choices=["flat", "obstacles"],
                    help="flat=纯平面（策略对比用，排除地形变量）；obstacles=带台阶的完整场景")
    ap.add_argument("--policy", default=os.path.join(PROJECT_ROOT, "ros2_controller", "policies", "go2_walk_v25b.npz"))
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--cmd", type=float, nargs=3, default=[0.5, 0.0, 0.0])
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=0.5, help="先用标称站姿 PD 稳定多久再交给策略")
    ap.add_argument("--soft-feet", dest="soft_feet", action="store_const", const=True, default=None,
                    help="足端软接触（复刻训练模型 condim=3 + solimp 0.9/0.95/0.023）；"
                         "默认取 config 的 foot_soft_contact（go2=true, go2=false）")
    ap.add_argument("--hard-feet", dest="soft_feet", action="store_const", const=False,
                    help="强制用模型 XML 原始足端接触参数")
    ap.add_argument("--iters", type=int, default=None,
                    help="MuJoCo solver iterations（默认取 config；都没给就是 100。"
                         "不要用 2：那是 MJX warp 的档，CPU Newton 下 2 次迭代会发散）")
    ap.add_argument("--ls-iters", type=int, default=None, help="MuJoCo solver ls_iterations")
    ap.add_argument("--linvel", default="oracle", choices=["oracle", "estimator"],
                    help="oracle=仿真真值 qvel[0:3]；estimator=LegOdometry（真机可用的那条路）")
    ap.add_argument("--scan", default="oracle", choices=["oracle", "leg_fk"],
                    help="oracle=直接读基座高度；leg_fk=接触腿正运动学（ROS2 回路里用的那条）")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    pol = NumpyMLPPolicy.from_npz(args.policy)
    meta = load_policy_meta(args.policy)
    cfg = load_config(os.path.join(PROJECT_ROOT, "config", "go2_sim.json"),
                      overrides={"robot": args.robot})
    xml = args.model or model_xml(cfg)
    if args.model is None and args.scene == "flat":
        cand = os.path.join(os.path.dirname(xml), "scene_flat.xml")
        if os.path.isfile(cand):
            xml = cand
    spec = robot_spec_of(cfg)
    soft_feet = (args.soft_feet if args.soft_feet is not None
                 else bool(cfg.get("foot_soft_contact", False)))
    iters = args.iters if args.iters is not None else cfg.get("solver_iterations")
    ls_iters = args.ls_iters if args.ls_iters is not None else cfg.get("solver_ls_iterations")
    sim = Go2Sim(xml, spec=spec, sim_dt=0.002, foot_soft_contact=soft_feet,
                 solver_iterations=iters, solver_ls_iterations=ls_iters)
    print("[probe] 模型=%s 站姿=%s z=%s 足端=%s 求解器=%d/%d" % (
        os.path.relpath(xml, PROJECT_ROOT), [round(float(x), 3) for x in sim.stand_pose],
        sim.stand_height, "soft" if soft_feet else "xml", sim.model.opt.iterations,
        sim.model.opt.ls_iterations))
    dt_ctrl = 0.02
    decim = int(round(dt_ctrl / sim.dt))
    cmd = np.array(args.cmd, dtype=np.float64)

    # 直接放在"训练 home"上：否则策略一上来就看到一个训练中没见过的初始偏差
    # 观测的 12:24 段减的是**策略训练机型**的 home (config 的 robots.<train_robot>.home_q)
    HOME_Q = np.asarray(cfg.get("_train_home_q", TRAINING_HOME_Q), dtype=np.float64)
    if HOME_Q.size != NUM_JOINT:
        raise SystemExit("config 的 home_q 维度不对: %d" % HOME_Q.size)
    sim.reset()
    # 出生姿态用当前被控机型的标称站姿 (config 的 stand_pose)
    sim.data.qpos[7:7 + NUM_JOINT] = np.asarray(sim.stand_pose, dtype=np.float64)
    sim.data.qvel[:] = 0.0
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)

    def projected_gravity():
        q = np.asarray(sim.data.xquat[sim.base_body], dtype=np.float64)
        w, x, y, z = q
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R.T @ np.array([0.0, 0.0, -1.0])

    from signal_io.estimator import LegKinematics, LegOdometry
    kin = LegKinematics(os.path.join(PROJECT_ROOT, "sim_env", "models", "go2", "go2.xml"))
    est = LegOdometry(kin, contact_force_thresh=30.0)

    def build_obs(last_act, phase):
        st = sim.read_state()
        # height scan：平地假设 h_ground = 0。
        # 注意**不是全 0**：= clip(离地高度 - 0.28, ±0.35) * 5.0，见 docs/interfaces.md §1.2。
        if args.scan == "oracle":
            zc = float(sim.data.qpos[2])
        else:
            P = kin.foot_positions_body(st.q12())
            ff = np.asarray(st.foot_force, dtype=np.float64)
            c = ff > 30.0
            z = -P[:, 2] + 0.023
            zc = float(np.median(z[c])) if c.any() else float(z.min())
        # height_scan 的 base 必须等于**策略训练机型**的值: go2 0.268
        base_z = float(cfg.get("_train_scan_base", 0.28))
        scan = np.full(35, np.clip(zc - base_z, -0.35, 0.35) * 5.0)
        last_scan[0] = float(zc)
        last_scan[1] = float(scan[0])
        q = np.asarray(sim.data.qpos[sim.qpos_adr], dtype=np.float64)
        dq = np.asarray(sim.data.qvel[sim.dof_adr], dtype=np.float64)
        if args.linvel == "oracle":
            lin = np.asarray(sim.data.qvel[0:3], dtype=np.float64)
        else:
            lin = est.update(st, 0.02, float(sim.time))
        ang = np.asarray(sim.data.qvel[3:6], dtype=np.float64)
        obs = np.concatenate([
            lin * 2.0, ang * 0.25, projected_gravity(), cmd * 2.0,
            q - HOME_Q, dq, last_act,
            scan,
            np.cos(TROT_PHASE0 + phase), np.sin(TROT_PHASE0 + phase),
        ])
        return np.clip(obs, -100.0, 100.0).astype(np.float32)

    # 训练侧动作语义：motor_targets = clip(action, actuator_ctrlrange[:,0], [:,1])，
    # 而 <position> 的 ctrlrange（autolimits=true）= 关节范围。不夹会和训练差一大截：
    # 例如 knee 范围是 [-2.818, -0.888]，策略若输出 -0.3，训练夹到 -0.888，不夹就顶死在限位上。
    jnt_lo = np.array([sim.model.jnt_range[sim.model.jnt_qposadr[0:0].size + 0]] if False else
                      [sim.model.jnt_range[mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, j)][0]
                       for j in sim.spec.motor_joints])
    jnt_hi = np.array([sim.model.jnt_range[mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, j)][1]
                       for j in sim.spec.motor_joints])
    n_sat = 0
    n_ctrl = 0

    def make_cmd(action):
        nonlocal n_sat, n_ctrl
        a = np.clip(np.asarray(action, dtype=np.float64), jnt_lo, jnt_hi)
        n_ctrl += 1
        n_sat += int(np.any(np.abs(a - np.asarray(action)) > 1e-9))
        c = LowCmd()
        for i in range(NUM_JOINT):
            m = c.motor_cmd[i]
            m.mode, m.q, m.dq = 0x01, float(a[i]), 0.0
            m.kp, m.kd, m.tau = args.kp, args.kd, 0.0
        return c

    n_steps = int(round(args.duration / sim.dt))
    n_settle = int(round(args.settle / sim.dt))
    stand = np.zeros(NUM_JOINT)
    last_act = np.zeros(NUM_JOINT)
    last_scan = [0.0, 0.0]
    phase = 0.0
    log = []
    first_action = None
    for k in range(n_steps):
        if k < n_settle:
            # 标称站姿 PD 稳定（对应我们框架里的 hold_until_first_cmd）
            c = LowCmd()
            for i in range(NUM_JOINT):
                m = c.motor_cmd[i]
                m.mode, m.q, m.dq, m.kp, m.kd, m.tau = 0x01, float(sim.stand_pose[i]), 0.0, 30.0, 1.0, 0.0
            sim.write_cmd(c)
        else:
            if k % decim == 0:
                obs = build_obs(last_act, phase)
                if first_action is None:
                    first_action = obs.copy()
                a = pol(obs)
                last_act = a.astype(np.float64)
                phase = (phase + 2 * np.pi * 1.8 * dt_ctrl)
                phase = (phase + np.pi) % (2 * np.pi) - np.pi
            sim.write_cmd(make_cmd(last_act))
        sim.step(1)
        if k % 50 == 0:
            rpy = np.degrees(sim.orientation_rpy())
            log.append({"t": round(sim.time, 2), "z": round(sim.base_height, 3),
                        "x": round(float(sim.data.qpos[0]), 3),
                        "tilt": round(float(max(abs(rpy[0]), abs(rpy[1]))), 2)})

    rpy = np.degrees(sim.orientation_rpy())
    out = {
        "robot": args.robot,
        "soft_feet": bool(soft_feet), "iters": iters, "ls_iters": ls_iters,
        "stand_pose": [round(float(x), 4) for x in sim.stand_pose],
        "stand_height": sim.stand_height,
        "obs_home_q": [round(float(x), 4) for x in HOME_Q],
        "scan_base": float(cfg.get("_train_scan_base", 0.28)),
        "torque_limits": [float(x) for x in spec.torque_limits],
        "model": os.path.relpath(xml, PROJECT_ROOT),
        "policy": os.path.relpath(args.policy, PROJECT_ROOT),
        "cmd": list(cmd), "kp": args.kp, "kd": args.kd, "duration": args.duration,
        "final": {"x": round(float(sim.data.qpos[0]), 3), "y": round(float(sim.data.qpos[1]), 3),
                  "z": round(sim.base_height, 3),
                  "rpy_deg": [round(float(v), 2) for v in rpy],
                  "tilt_deg": round(float(max(abs(rpy[0]), abs(rpy[1]))), 2),
                  "vx": round(float(sim.data.qvel[0]), 3)},
        "mean_z": round(float(np.mean([r["z"] for r in log])), 4),
        "max_tilt_deg": round(float(np.max([r["tilt"] for r in log[1:]])), 2) if len(log) > 1 else None,
        "trace": log,
        "first_obs": None if first_action is None else [round(float(v), 4) for v in first_action],
        "obs_dim": int(pol.obs_dim),
        "linvel_source": args.linvel, "scan_source": args.scan,
        "ground_clearance": round(last_scan[0], 4), "height_scan": round(last_scan[1], 4),
        "linvel_confidence": round(float(est.confidence), 3),
        "n_ctrl": n_ctrl, "n_action_clamped": n_sat,
        "action_clamp_rate": round(n_sat / max(1, n_ctrl), 4),
        "meta_equivalence": (meta.get("equivalence") or {}).get("passed"),
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
