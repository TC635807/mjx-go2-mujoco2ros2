"""纯 numpy 的策略前向推理（不依赖 jax / brax / torch）。

来源：scripts/export_go2_policy.py 从 brax PPO 的 flax 参数树导出的 .npz。
前向与 brax 逐位对齐（对 f64 参考实现 max|diff| 1.4e-14），导出时的等价性证据存在
<policy>.equiv.npz 里，测试 tests/test_policy_port.py 在没有 jax 的环境里回归。

    a = W_out @ tanh(W1 @ tanh(W0 @ ((x-mean)/std) + b0) + b1) + b_out

内部一律用 float64：比 jax 的 f32 更接近数学真值，也避免部署侧为了"复刻 jax 的舍入"
而引入无意义的复杂度。实测 numpy(f64) 与 jax(f32, highest matmul) 差 7e-6，
与 jax(f32, 默认 matmul 精度) 差中位 2.7e-4 / 最大 1.8e-2 —— 见导出 meta 的 pitfalls。
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Sequence

import numpy as np

__all__ = ["NumpyMLPPolicy", "load_policy_meta"]


class NumpyMLPPolicy:
    """brax PPO 的 deterministic 策略：归一化 + MLP(tanh) + 线性输出头。"""

    def __init__(self, weights: Dict[str, np.ndarray], norm_mean: np.ndarray,
                 norm_std: np.ndarray, name: str = "policy",
                 apply_normalizer: bool = False):
        self.name = name
        # ⚠️ 默认 **不** 归一化：go2_walk 是 normalize_observations=False 训出来的
        # （train/train_go2.py:346、experiment-log.md:230；brax 默认也是 False）。
        # 套上 (x-mean)/std 会让动作偏差 max 5.4，闭环直接倒 —— 见导出 meta 的 pitfalls。
        self.apply_normalizer = bool(apply_normalizer)
        self.norm_mean = np.asarray(norm_mean, dtype=np.float64).ravel()
        self.norm_std = np.asarray(norm_std, dtype=np.float64).ravel()
        n_layers = sum(1 for k in weights if k.startswith("w_hidden_"))
        if n_layers == 0:
            raise ValueError("权重里没有 w_hidden_*，不是有效的策略 npz")
        self.weights: List[Dict[str, np.ndarray]] = []
        for i in range(n_layers):
            self.weights.append({
                "w": np.asarray(weights["w_hidden_%d" % i], dtype=np.float64),
                "b": np.asarray(weights["b_hidden_%d" % i], dtype=np.float64),
            })
        self.w_out = np.asarray(weights["w_out"], dtype=np.float64)
        self.b_out = np.asarray(weights["b_out"], dtype=np.float64)
        self.obs_dim = int(self.weights[0]["w"].shape[0])
        self.action_dim = int(self.w_out.shape[1])
        self.hidden_sizes = tuple(int(l["w"].shape[1]) for l in self.weights)
        if self.norm_mean.shape != (self.obs_dim,) or self.norm_std.shape != (self.obs_dim,):
            raise ValueError("normalizer 形状 %s/%s 与 obs_dim %d 不一致"
                             % (self.norm_mean.shape, self.norm_std.shape, self.obs_dim))
        if not np.all(np.isfinite(self.norm_std)) or np.any(self.norm_std <= 0.0):
            raise ValueError("normalizer std 含非法值（<=0 或非有限）")
        self._n_calls = 0

    # ------------------------------------------------------------------ 构造
    @classmethod
    def from_npz(cls, path: str, name: Optional[str] = None,
                 apply_normalizer: Optional[bool] = None) -> "NumpyMLPPolicy":
        """apply_normalizer=None 时读 <policy>.json 的 apply_normalizer（缺失则按 False）。"""
        with np.load(path) as z:
            weights = {k: z[k] for k in z.files}
        if name is None:
            name = os.path.splitext(os.path.basename(path))[0]
        if apply_normalizer is None:
            meta = load_policy_meta(path)
            apply_normalizer = bool(meta.get("apply_normalizer", False))
        return cls(weights, weights["norm_mean"], weights["norm_std"], name=name,
                   apply_normalizer=apply_normalizer)

    # ------------------------------------------------------------------ 推理
    def _preprocess(self, x: np.ndarray) -> np.ndarray:
        if not self.apply_normalizer:
            return x
        return (x - self.norm_mean) / self.norm_std

    def batch(self, obs: np.ndarray) -> np.ndarray:
        """(N, obs_dim) -> (N, action_dim)，float64。"""
        x = self._preprocess(np.asarray(obs, dtype=np.float64))
        if x.ndim != 2 or x.shape[1] != self.obs_dim:
            raise ValueError("obs 形状 %s，期望 (N, %d)" % (x.shape, self.obs_dim))
        for layer in self.weights:
            x = np.tanh(x @ layer["w"] + layer["b"])
        out = x @ self.w_out + self.b_out
        self._n_calls += int(x.shape[0])
        return out

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        """(obs_dim,) -> (action_dim,)。"""
        x = np.asarray(obs, dtype=np.float64).ravel()
        if x.shape[0] != self.obs_dim:
            raise ValueError("obs 长度 %d，期望 %d" % (x.shape[0], self.obs_dim))
        if not np.all(np.isfinite(x)):
            raise ValueError("obs 含 NaN/Inf，拒绝推理")
        self._n_calls += 1
        x = self._preprocess(x)
        for layer in self.weights:
            x = np.tanh(x @ layer["w"] + layer["b"])
        return x @ self.w_out + self.b_out

    def reset(self) -> None:
        pass

    def summary(self) -> Dict[str, object]:
        n_params = sum(l["w"].size + l["b"].size for l in self.weights) \
            + self.w_out.size + self.b_out.size
        return {"name": self.name, "obs_dim": self.obs_dim, "action_dim": self.action_dim,
                "hidden_sizes": list(self.hidden_sizes), "activation": "tanh",
                "apply_normalizer": self.apply_normalizer,
                "dtype": "float64", "n_params": int(n_params), "n_calls": self._n_calls}


def load_policy_meta(npz_path: str) -> Dict[str, object]:
    """读导出脚本写在 <policy>.json 的元数据（动作语义、观测布局、等价性证据）。"""
    meta_path = os.path.splitext(npz_path)[0] + ".json"
    if not os.path.isfile(meta_path):
        return {}
    with open(meta_path, "r", encoding="utf-8") as f:
        return json.load(f)
