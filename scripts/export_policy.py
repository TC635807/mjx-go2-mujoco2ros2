#!/usr/bin/env python3
"""把 mjx-go2-getup 训练好的 brax PPO 策略导出成 **纯 numpy** 可推理的 npz。

为什么导出而不是直接依赖 jax：
  - 部署侧（ROS 2 控制节点 / 真机）不该背 jax + brax + CUDA；
  - 导出时把 numpy 前向与 jax 前向在**同一批输入**上逐元素比对，等价性留证据；
  - 比对样本一起落盘（.equiv.npz），于是**没有 jax 的项目 venv 里**也能回归。

前向语义（逐条从 brax 源码读出，并用 f64 数值验证到 1e-14）:
    brax/training/agents/ppo/networks.py::make_inference_fn -> deterministic -> distribution.mode(logits)
    brax/training/networks.py::PolicyModuleWithStd -> MLP(hidden, activate_final=True) -> Dense(param_size)
    brax/training/distribution.py::NormalDistribution.mode() == loc（**没有 tanh**）
    brax/training/acme/running_statistics.py::normalize(x) = (x - mean) / std   （std_eps = 0.0）
  =>  a = W_out @ tanh(W1 @ tanh(W0 @ ((x-mean)/std) + b0) + b1) + b_out

⚠️ 关键事实 1（**必须按它来，否则策略完全跑不对**）：
这个策略是 **normalize_observations=False** 训出来的 —— 训练侧的
/home/tc63/mujoco/mjx-go2-training/train/train_go2.py 明确置 False（experiment-log.md:230 也记着"无 obs 归一化"），
brax 的默认值也是 False（ppo/train.py:201）。所以参数里那个
RunningStatisticsState（params[0]，count≈6e7）**在推理时不被使用**，
策略吃的是**原始未归一化观测**。本脚本因此以 identity 前向为准。

实测证据（在训练环境 envs.go2_walk.Go2Walk 里，MJX warp + GPU，各 4 个环境跑 200 步）：
    identity  -> 存活 4/4，末段 |vx_body| = 0.474 m/s（指令 0.5），平均步奖励 4.03
    normalize -> 存活 0/4，末段 |vx_body| = 0.171，平均步奖励 1.44
所以"给策略套上 running_statistics.normalize"是**错的**（我一开始就是这么错的，
所有闭环探针都会倒）。脚本仍会把两套前向都导出来并记录差异，方便复核。

⚠️ 坑 2（会被误判成"移植错了"）：jax 的 **默认 matmul 精度不是 f32 数学精度**。
同一批观测，jax(f32, 默认精度) 与 numpy(f64) 相差 max 1.78e-2 / 中位数 2.7e-4；
把 jax_default_matmul_precision 设成 "highest" 后立刻降到 7.4e-6。
所以本脚本同时记录三个数字，避免把"jax 的精度设置"误读成"公式不对"。

用法（必须用带 jax 的 venv）:
    /home/tc63/mujoco/.venv/bin/python scripts/export_policy.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pickle
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

# 观测布局：与 /home/tc63/mujoco/mjx-go2-training/envs/go2_walk.py::_get_obs 逐字段对应
# 基础段 (Go2 地形行走策略): 0:48 本体
BASE_LAYOUT = [
    {"name": "base_lin_vel_body", "start": 0, "len": 3, "scale": 2.0,
     "source": "R^T @ data.qvel[0:3] * obs_scale.linear_velocity (body 系)"},
    {"name": "base_ang_vel_body", "start": 3, "len": 3, "scale": 0.25,
     "source": "data.qvel[3:6] * obs_scale.angular_velocity"},
    {"name": "projected_gravity", "start": 6, "len": 3, "scale": 1.0,
     "source": "site_xmat[imu].T @ [0,0,-1]"},
    {"name": "command", "start": 9, "len": 3, "scale": 2.0,
     "source": "info['command'] * obs_scale.linear_velocity"},
    {"name": "dofs_position", "start": 12, "len": 12, "scale": 1.0,
     "source": "qpos[7:] - key_ctrl[0]（obs_scale.dofs_position 没被乘上）"},
    {"name": "dofs_velocity", "start": 24, "len": 12, "scale": 1.0,
     "source": "qvel[6:]（不缩放）"},
    {"name": "last_action", "start": 36, "len": 12, "scale": 1.0,
     "source": "info['last_act']"},
]

# Go2 训练 home (go2_mjx_position.xml keyframe ctrl)
TRAINING_HOME_Q = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
                   -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]


def make_obs_layout(obs_dim: int):
    """按 obs 维度拼布局: 48 base + scan(17x11=187) + phase8 + history。"""
    obs_dim = int(obs_dim)
    lay = [dict(x) for x in BASE_LAYOUT]
    rest = obs_dim - 48
    scan_n = 187 if rest >= 187 else max(0, rest)
    lay.append({"name": "height_scan", "start": 48, "len": scan_n, "scale": 1.0,
                "source": "clip(z-h-0.268, ±0.35)*5.0, 17x11 yaw 对齐网格, x 外层 y 内层"})
    pos = 48 + scan_n
    if obs_dim - pos >= 8:
        lay.append({"name": "gait_phase_cos", "start": pos, "len": 4, "scale": 1.0,
                    "source": "cos(trot_i + Phi), trot=[0,pi,pi,0] (FR,FL,RR,RL)"})
        lay.append({"name": "gait_phase_sin", "start": pos + 4, "len": 4, "scale": 1.0,
                    "source": "sin(trot_i + Phi)"})
        pos += 8
    if obs_dim > pos:
        lay.append({"name": "history_frames", "start": pos, "len": obs_dim - pos,
                    "scale": 1.0, "source": "历史帧 (base48+phase8), 最近帧在前"})
    return lay


def infer_layers_and_dims(params):
    """与 /home/tc63/mujoco/mjx-go2-training/sim/common.py 同名函数同逻辑（从参数形状反推层宽与 obs 维度）。"""
    roots = [params[1]] if isinstance(params, (tuple, list)) and len(params) > 1 else [params]
    found = {}

    def walk(node):
        if isinstance(node, (tuple, list)):
            for v in node:
                walk(v)
            return
        if not hasattr(node, "items"):
            return
        for k, v in node.items():
            if k.startswith("hidden_") and hasattr(v, "get") and "kernel" in v and hasattr(v["kernel"], "shape"):
                found[int(k.split("_")[1])] = int(v["kernel"].shape[1])
            walk(v)

    for r in roots:
        walk(r)
    layers = tuple(found[i] for i in sorted(found)) if found else (128, 128)

    def first_dim(node):
        best = {}

        def w(n):
            if isinstance(n, (tuple, list)):
                for v in n:
                    w(v)
                return
            if not hasattr(n, "items"):
                return
            for k, v in n.items():
                if k == "hidden_0" and hasattr(v, "get") and "kernel" in v and hasattr(v["kernel"], "shape"):
                    best.setdefault("dim", int(v["kernel"].shape[0]))
                w(v)

        w(node)
        return best.get("dim")

    if isinstance(params, (tuple, list)) and len(params) >= 3:
        ns, nv = first_dim(params[1]) or 48, first_dim(params[2]) or 119
    else:
        ns, nv = (first_dim(params) or 48), 119
    return layers, ns, nv


def flatten_policy_params(policy_params, n_layers):
    """flax 参数树 -> 扁平 numpy 权重 dict（统一 f64，部署时精度更高）。"""
    p = policy_params["params"]
    out = {}
    for i in range(n_layers):
        out["w_hidden_%d" % i] = np.asarray(p["MLP_0"]["hidden_%d" % i]["kernel"], dtype=np.float64)
        out["b_hidden_%d" % i] = np.asarray(p["MLP_0"]["hidden_%d" % i]["bias"], dtype=np.float64)
    out["w_out"] = np.asarray(p["Dense_0"]["kernel"], dtype=np.float64)
    out["b_out"] = np.asarray(p["Dense_0"]["bias"], dtype=np.float64)
    return out


def numpy_forward(weights, norm_mean, norm_std, obs, apply_normalizer: bool = False):
    """与 policy/net.py::NumpyMLPPolicy.__call__ 逐字同实现的参考函数（保持同步）。

    apply_normalizer 默认 False —— 这个策略是 normalize_observations=False 训的。
    """
    x = np.asarray(obs, dtype=np.float64)
    if apply_normalizer:
        x = (x - norm_mean) / norm_std
    n_layers = sum(1 for k in weights if k.startswith("w_hidden_"))
    for i in range(n_layers):
        x = np.tanh(x @ weights["w_hidden_%d" % i] + weights["b_hidden_%d" % i])
    return x @ weights["w_out"] + weights["b_out"]


def make_sample_obs(mean, std, n, seed):
    """两批输入：① 归一化空间高斯（工作点附近）② 归一化空间 ±2 均匀（饱和区）。"""
    rng = np.random.default_rng(seed)
    n1 = n // 2
    obs1 = mean + std * rng.standard_normal((n1, mean.size))
    n2 = n - n1
    obs2 = mean + std * rng.uniform(-2.0, 2.0, size=(n2, mean.size))
    obs = np.concatenate([obs1, obs2], axis=0)
    obs[:, 9:12] = np.clip(obs[:, 9:12], -2.0, 2.0)     # 命令（×2 后 ±2）
    obs[:, 48:] = np.clip(obs[:, 48:], -2.0, 2.0)         # height scan(±1.75) + 相位(±1) + 历史
    return np.clip(obs, -100.0, 100.0).astype(np.float32)


def _versions():
    import brax, flax, jax
    return {"brax": brax.__version__, "jax": jax.__version__, "flax": flax.__version__}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pkl", default="/home/tc63/mujoco/mjx-go2-training/policies/go2_walk_v25b.pkl")
    ap.add_argument("--out", default=os.path.join(PROJECT_ROOT, "ros2_controller", "policies", "go2_walk_v25b.npz"))
    ap.add_argument("--name", default="go2_walk_v25b")
    ap.add_argument("--n-samples", type=int, default=4096)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--exact-atol", type=float, default=1e-9, help="numpy(f64) vs jax(f64) 判据：公式是否正确")
    ap.add_argument("--f32-atol", type=float, default=1e-4, help="numpy(f64) vs jax(f32,highest) 判据：同精度等价")
    args = ap.parse_args()

    import jax
    import jax.numpy as jp
    from brax.training import types
    from brax.training.agents.ppo import networks as ppo_networks
    from brax.training.acme import running_statistics

    with open(args.pkl, "rb") as f:
        params = pickle.load(f)
    sha = hashlib.sha256(open(args.pkl, "rb").read()).hexdigest()
    layers, ns, nv = infer_layers_and_dims(params)
    count = params[0].count
    print("[export] pkl      =", args.pkl)
    print("[export] sha256   =", sha)
    print("[export] obs_dim  = %d (privileged %d), layers = %s" % (ns, nv, list(layers)))
    print("[export] 归一化    = brax running_statistics, 训练样本数 %d" % (int(count.hi) * 2**32 + int(count.lo)))

    norm_mean = np.asarray(params[0].mean["state"], dtype=np.float64)
    norm_std = np.asarray(params[0].std["state"], dtype=np.float64)
    weights = flatten_policy_params(params[1], len(layers))
    for k in sorted(weights):
        print("[export]   %-16s %s" % (k, tuple(weights[k].shape)))

    obs_batch = make_sample_obs(norm_mean, norm_std, args.n_samples, args.seed)
    act_np = np.stack([numpy_forward(weights, norm_mean, norm_std, o, False) for o in obs_batch])
    act_np_norm = np.stack([numpy_forward(weights, norm_mean, norm_std, o, True) for o in obs_batch])

    def build(pre):
        return ppo_networks.make_ppo_networks(
            observation_size={"state": (ns,), "privileged_state": (nv,)},
            action_size=12, policy_hidden_layer_sizes=layers,
            value_hidden_layer_sizes=layers, policy_obs_key="state",
            value_obs_key="privileged_state", distribution_type="normal",
            activation=jax.nn.tanh, preprocess_observations_fn=pre)

    def run(pre, dtype):
        ifn = ppo_networks.make_inference_fn(build(pre))(params, deterministic=True)
        obs_j = jp.asarray(obs_batch, dtype=dtype)
        return np.asarray(jax.jit(lambda o: ifn({"state": o}, None)[0])(obs_j))

    IDENT = types.identity_observation_preprocessor
    # ① 训练时实际用的设置：identity 预处理器 + 默认 matmul 精度
    act_default_prec = run(IDENT, jp.float32)
    # ② identity + f32 + 最高 matmul 精度：与 numpy 的可比对象
    jax.config.update("jax_default_matmul_precision", "highest")
    act_f32 = run(IDENT, jp.float32)
    # ③ identity + f64 参考：验证公式本身
    jax.config.update("jax_enable_x64", True)
    act_f64 = run(IDENT, jp.float64)
    # ④ 对照组：错误地套上 normalize 会偏多少
    act_norm = run(running_statistics.normalize, jp.float64)

    err_exact = float(np.abs(act_f64 - act_np).max())
    err_f32 = float(np.abs(act_f32.astype(np.float64) - act_np).max())
    d_prec = np.abs(act_default_prec.astype(np.float64) - act_np)
    d_identity = float(np.abs(act_np_norm - act_f64).max())   # numpy 套 normalize 后与 identity jax 的差
    print("[export] numpy(f64) vs jax(f64)            max|diff| = %.3e   <- 公式正确性" % err_exact)
    print("[export] numpy(f64) vs jax(f32,highest)    max|diff| = %.3e   <- 同精度等价" % err_f32)
    print("[export] jax(f32,默认精度) vs numpy(f64)   max %.3e / p99 %.3e / 中位 %.3e   <- 坑 2"
          % (d_prec.max(), np.percentile(d_prec, 99), np.median(d_prec)))
    print("[export] 错误地套上 normalize 的偏离        max|diff| = %.3e   <- 千万别这么做" % d_identity)
    print("[export] 动作量级: |a| max %.3f  p99 %.3f  mean %.3f"
          % (np.abs(act_np).max(), np.percentile(np.abs(act_np), 99), np.abs(act_np).mean()))
    ok = bool(err_exact < args.exact_atol and err_f32 < args.f32_atol)
    print("[export] 等价性    :", "PASS" if ok else "FAIL")

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    payload = dict(weights)
    payload["norm_mean"] = norm_mean
    payload["norm_std"] = norm_std
    np.savez_compressed(args.out, **payload)

    meta = {
        "name": args.name,
        "source_pkl": os.path.abspath(args.pkl),
        "source_sha256": sha,
        "obs_dim": int(ns), "privileged_obs_dim": int(nv), "action_dim": 12,
        "hidden_sizes": [int(x) for x in layers],
        "activation": "tanh", "output": "loc (deterministic mode, 无 tanh)",
        "apply_normalizer": False,
        "normalizer_note": ("训练时 normalize_observations=False（/home/tc63/mujoco/mjx-go2-training/train/train_go2.py；"
                            "experiment-log.md:230；brax ppo/train.py 默认也是 False），"
                            "所以 params[0] 的 RunningStatisticsState 推理时**不使用**，"
                            "策略吃原始观测。npz 里仍然保存了 mean/std 以备复核，"
                            "但 NumpyMLPPolicy 默认 apply_normalizer=False。"),
        "normalizer": "不启用（identity）。误用 (x-mean)/std 会造成 max 偏差见 pitfalls.wrong_normalizer",
        "pitfalls": {
            "wrong_normalizer": {
                "what": "若错误地套上 running_statistics.normalize，动作偏差有多大",
                "measured_drift": d_identity,
                "evidence": ("训练环境 4 env × 200 步实测：identity 存活 4/4，|vx_body|=0.474；"
                             "normalize 存活 0/4，|vx_body|=0.171"),
            },
            "jax_default_matmul_precision": {
                "what": "jax 默认 matmul 精度低于 f32 数学精度；设为 highest 后与 numpy(f64) 只差 1e-6 量级",
                "max_abs_err": float(d_prec.max()),
                "p99_abs_err": float(np.percentile(d_prec, 99)),
                "median_abs_err": float(np.median(d_prec)),
            },
        },
        "distribution_type": "normal",
        "brax_jax_flax": _versions(),
        "obs_layout": make_obs_layout(ns),
        "training_home_q": TRAINING_HOME_Q,
        "action_semantics": {
            "kind": "joint_position_target_rad",
            "lowcmd_mapping": {"q": "action[i]", "dq": 0.0, "kp": 20.0, "kd": 1.0, "tau": 0.0},
            "training_actuator": "<position kp=20 kv=1 forcerange=±23.7 (knee ±45.43)>",
            "note": "训练时 MuJoCo 按 actuator ctrlrange 内部 clamp；部署侧由关节 limit 兜底",
        },
        "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "equivalence": {
            "n_samples": int(obs_batch.shape[0]),
            "numpy_f64_vs_jax_f64_max_abs_err": err_exact,
            "numpy_f64_vs_jax_f32_highest_max_abs_err": err_f32,
            "exact_atol": float(args.exact_atol),
            "f32_atol": float(args.f32_atol),
            "passed": ok,
        },
    }
    meta_path = os.path.splitext(args.out)[0] + ".json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    equiv_path = os.path.splitext(args.out)[0] + ".equiv.npz"
    np.savez_compressed(equiv_path, obs=obs_batch, action_jax_f64=act_f64,
                        action_jax_f32_highest=act_f32, action_jax_default_prec=act_default_prec,
                        action_jax_wrong_normalize=act_norm)

    print("[export] 写出 npz   =", args.out, "(%d B)" % os.path.getsize(args.out))
    print("[export] 写出 meta  =", meta_path)
    print("[export] 写出等价性 =", equiv_path, "(没有 jax 也能回归)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
