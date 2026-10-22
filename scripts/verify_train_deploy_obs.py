#!/usr/bin/env python3
"""训练侧 vs 部署侧 **观测/动作逐段对账** (信号链的最终验证)。

回答一个问题: "部署闭环喂给策略的 243 维 obs / 策略输出, 和训练时是不是同一个东西?"

做法:
  * 训练侧参考 = mjx-go2-training/sim/eval_go2.py (CPU 版 go2_walk.py 的 obs 复刻,
    projected_gravity 用训练 env 的 site_xmat[imu].T @ [0,0,-1]);
  * 部署侧 = ros2_controller/obs.py 的 WalkObsBuilder + policies/*.npz;
  * 把两台机器人摆到**同一个物理状态** (同一 base 位姿/速度/关节角/关节速度 + 同一相位),
    逐段比较 243 维 obs; 再用 jax(pkl) 与 numpy(npz) 各自前向, 比较 12 维动作。

通过标准: 每段 max|diff| = 0 (数值上逐位一致), 动作 max|diff| < 1e-5。

历史: 2026-10-20 用本脚本发现 obs[6:9] projected_gravity 的第三列/第三行写反 ——
纯 yaw 时看不出来, 一旦有 roll/pitch 重力 xy 的符号是反的 (max|diff|=0.627)。
修复后全段 0。以后改 obs / 换策略都必须跑这个脚本。

用法:
    cd mjx-go2-mujoco2ros2 && python scripts/verify_train_deploy_obs.py
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

# 单线程 BLAS(与部署入口一致): 1024-512 前向在多线程 OpenBLAS 下 12ms/次
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

TRAIN = os.environ.get("TRAIN_REPO", "/home/tc63/mujoco/mjx-go2-training")
DEP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(TRAIN, "sim")); sys.path.insert(0, DEP)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
import mujoco
import eval_go2 as E
from signal_io.config import load_config, robot_spec_of
from sim_env.robot import Go2Sim, MOTOR_JOINTS
from ros2_controller import run_controller as rc

E.SCAN.update(dict(nx=17, ny=11, x_min=-0.4, x_max=1.2, y_min=-0.5, y_max=0.5))
argv = ["--policy", "none", "--dummy", "hold_home", "--terrain", "flat", "--seeds", "1",
        "--duration", "0.2", "--scan_nx", "17", "--scan_ny", "11",
        "--scan_x", "-0.4", "1.2", "--scan_y", "-0.5", "0.5", "--vel-frame", "body"]
args = E.finalize_args(E.build_parser().parse_args(argv))
E.SCAN.update(dict(nx=17, ny=11, x_min=-0.4, x_max=1.2, y_min=-0.5, y_max=0.5))
tsim = E.Go2Sim(os.path.join(TRAIN, "models/go2/scene_mjx_position.xml"), args)

cfg = load_config(os.path.join(DEP, "config", "go2_sim.json"),
                  overrides={"robot": "go2", "policy_profile": "go2_v25b"})
cfg.setdefault("policy", {})["height_scan_source"] = "nominal"
dsim = Go2Sim(os.path.join(DEP, "sim_env/models/go2/scene_flat.xml"),
              spec=robot_spec_of(cfg), sim_dt=0.002)
ctrl = rc.build_controller("policy", cfg, dsim, cli_cmd=[0.5, 0, 0], linvel="oracle")
b = ctrl.obs

HOME = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
TROT = np.array([0.0, np.pi, np.pi, 0.0])

def set_state(base, quat, v_world, w_body, q_act, dq_act):
    tq = tsim.key_qpos.copy()
    tq[0:3] = base; tq[3:7] = quat
    tq[tsim.act_joint_posadr] = q_act
    tsim.data.qpos[:] = tq; tsim.data.qvel[:] = 0.0
    tsim.data.qvel[0:3] = v_world; tsim.data.qvel[3:6] = w_body
    tsim.data.qvel[tsim.act_joint_dofadr] = dq_act
    mujoco.mj_forward(tsim.model, tsim.data)
    dq = dsim.data.qpos
    dq[0:3] = base; dq[3:7] = quat
    dq[dsim.qpos_adr] = q_act
    dsim.data.qvel[:] = 0.0
    dsim.data.qvel[0:3] = v_world; dsim.data.qvel[3:6] = w_body
    dsim.data.qvel[dsim.dof_adr] = dq_act
    mujoco.mj_forward(dsim.model, dsim.data)
    return dsim.read_state()

def quat_axis(axis, ang):
    axis = np.asarray(axis, dtype=float); axis = axis / np.linalg.norm(axis)
    return np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * axis])

cases = [
    dict(name="home静止", base=[0.0, 0.0, 0.32], quat=[1.0, 0, 0, 0],
         v=[0, 0, 0], w=[0, 0, 0], q=HOME, dq=np.zeros(12), cmd=[0.5, 0, 0], phase=0.7),
    dict(name="随机姿态+速度", base=[1.13, -0.42, 0.35], quat=quat_axis([0.3, 1.0, 0.2], 0.34),
         v=[0.42, -0.13, 0.07], w=[0.21, -0.34, 0.12],
         q=HOME + np.linspace(-0.2, 0.25, 12), dq=np.linspace(-1.5, 1.2, 12),
         cmd=[0.5, 0.0, 0.0], phase=-1.9),
    dict(name="yaw90+侧向指令", base=[-0.7, 2.05, 0.29], quat=quat_axis([0, 0, 1], np.pi / 2),
         v=[0.05, 0.33, -0.02], w=[-0.05, 0.08, -0.44],
         q=HOME + 0.15 * np.sin(np.arange(12)), dq=0.4 * np.cos(np.arange(12)),
         cmd=[0.4, 0.2, -0.5], phase=2.4),
]

print("=" * 70)
print("obs 分段对账: 训练侧(eval_go2/CPU 复刻) vs 部署侧(ros2_controller/obs.py)")
print("=" * 70)
segs = [("0:3 linvel_body", 0, 3), ("3:6 angvel_body", 3, 6), ("6:9 gravity", 6, 9),
        ("9:12 cmd", 9, 12), ("12:24 qpos-home", 12, 24), ("24:36 qvel", 24, 36),
        ("36:48 last_act", 36, 48), ("48:235 height_scan", 48, 235), ("235:243 phase", 235, 243)]
for cs in cases:
    state = set_state(np.array(cs["base"], float), np.array(cs["quat"], float),
                      np.array(cs["v"], float), np.array(cs["w"], float),
                      np.array(cs["q"], float), np.array(cs["dq"], float))
    st = {"cmd": np.array(cs["cmd"], float), "last_act": np.zeros(12),
          "phase": TROT + cs["phase"]}
    scan_train = tsim.height_scan()
    obs_t = tsim.obs(st, True, True)
    b.reset(); b.set_phase(cs["phase"]); b._cmd = np.array(cs["cmd"], float)
    b.last_action = np.zeros(12)
    obs_d, meta = b.observe(state, 0.02, base_lin_vel_world=np.array(cs["v"], float),
                            height_scan=scan_train)
    d = np.asarray(obs_d, dtype=np.float64) - np.asarray(obs_t, dtype=np.float64)
    print("---- case:", cs["name"], " obs_dim train", obs_t.size, "dep", obs_d.size,
          " valid", meta.valid, meta.reasons)
    worst = 0.0
    for nm, a, z in segs:
        dd = np.abs(d[a:z])
        mx = float(dd.max()) if dd.size else 0.0
        rms = float(np.sqrt(np.mean(dd ** 2))) if dd.size else 0.0
        worst = max(worst, mx)
        print("   %-18s max|diff| %10.3e  rms %10.3e  train范围[%+.3f,%+.3f]"
              % (nm, mx, rms, float(np.min(obs_t[a:z])), float(np.max(obs_t[a:z]))))
    print("   >>> 全分段 max|diff| = %.3e" % worst)

# ---- 动作对账: 训练 pkl(jax) vs 部署 npz(numpy) ----
print()
print("=" * 70)
print("动作对账: pkl(jax/brax) vs npz(numpy)")
print("=" * 70)
from ros2_controller.policy import NumpyMLPPolicy
pol = NumpyMLPPolicy.from_npz(os.path.join(DEP, "ros2_controller/policies/go2_walk_v25b.npz"))
jp = E.Policy(os.path.join(TRAIN, "policies/go2_walk_v25b.pkl"), action_size=12, normalize="off", dist="auto")
rng = np.random.default_rng(1)
worst = 0.0
for k in range(5):
    obs = rng.normal(0, 1.0, jp.ns).astype(np.float32)
    a_t = np.asarray(jp(obs), dtype=np.float64)
    a_d = np.asarray(pol(obs), dtype=np.float64)
    e = float(np.max(np.abs(a_t - a_d)))
    worst = max(worst, e)
    print("   random obs #%d  max|daction| = %.3e" % (k, e))
# 用真实 obs 再对一次 (home 静止态)
state = set_state(np.array([0, 0, 0.32], float), np.array([1., 0, 0, 0], float),
                  np.zeros(3), np.zeros(3), HOME, np.zeros(12))
st = {"cmd": np.array([0.5, 0, 0]), "last_act": np.zeros(12), "phase": TROT + 0.7}
obs_t = tsim.obs(st, True, True)
a_t = np.asarray(jp(obs_t), dtype=np.float64); a_d = np.asarray(pol(obs_t), dtype=np.float64)
print("   真实观测          max|daction| = %.3e" % float(np.max(np.abs(a_t - a_d))))
print("   训练动作 =", np.round(a_t, 3).tolist())
print("   部署动作 =", np.round(a_d, 3).tolist())
print("   >>> 动作全对账 max|daction| = %.3e" % worst)