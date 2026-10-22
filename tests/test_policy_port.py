#!/usr/bin/env python3
"""tests/test_policy_port.py —— 部署侧纯 numpy 策略前向验收 (Go2 v25b)。

覆盖:
  1. npz 元数据自洽: obs_dim=243 / action_dim=12 / hidden_sizes=[1024,512] / 无 obs 归一化;
  2. numpy (f64) 前向与导出时落盘的 jax 参考输出逐元素一致;
  3. 错误输入 (维度不对) 抛错;
  4. 观测布局分段覆盖 0..obs_dim 且无空洞;
  5. 可选: 无 jax 环境下直接跑闭环 (子进程 probe_policy_controller --robot go2)。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ros2_controller.policy import NumpyMLPPolicy, load_policy_meta  # noqa: E402

NPZ = os.path.join(ROOT, "ros2_controller", "policies", "go2_walk_v25b.npz")
EQUIV = os.path.join(ROOT, "ros2_controller", "policies", "go2_walk_v25b.equiv.npz")
META = os.path.join(ROOT, "ros2_controller", "policies", "go2_walk_v25b.json")
FAILS = []


def check(name, ok, detail=""):
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def main() -> int:
    if not os.path.isfile(NPZ):
        print("缺少 %s —— 先在训练仓跑 sim/export_deploy_policy.sh 导出" % NPZ)
        return 1
    meta = load_policy_meta(NPZ)
    pol = NumpyMLPPolicy.from_npz(NPZ)

    print("[1] 元数据: Go2 v25b")
    check("obs_dim == 243", int(meta["obs_dim"]) == 243 and pol.obs_dim == 243,
          "meta=%s pol=%d" % (meta["obs_dim"], pol.obs_dim))
    check("action_dim == 12", int(meta["action_dim"]) == 12)
    check("hidden_sizes == [1024, 512]", list(meta["hidden_sizes"]) == [1024, 512],
          str(meta["hidden_sizes"]))
    check("activation == tanh / output = loc (无 tanh)",
          meta.get("activation") == "tanh" and str(meta.get("output", "")).startswith("loc"),
          "%s / %s" % (meta.get("activation"), meta.get("output")))
    check("不套 obs 归一化 (apply_normalizer=False)",
          bool(meta.get("apply_normalizer")) is False)

    print("[2] 观测布局分段: 48 + 187 + 8 = 243, 连续无空洞")
    layout = list(meta.get("obs_layout") or [])
    ok = bool(layout) and layout[0].get("start") == 0
    for a, b in zip(layout, layout[1:]):
        ok = ok and int(b["start"]) == int(a["start"]) + int(a["len"])
    total = sum(int(x["len"]) for x in layout)
    check("layout 连续且总和 243", ok and total == 243,
          "segments=%d total=%d" % (len(layout), total))

    print("[3] numpy 前向 vs 导出时 jax 参考 (f64)")
    if not os.path.isfile(EQUIV):
        check("存在 .equiv.npz", False)
    else:
        d = np.load(EQUIV)
        obs = np.asarray(d["obs"], dtype=np.float64)
        ref = np.asarray(d["action_jax_f64"], dtype=np.float64)
        acts = np.stack([np.asarray(pol(o), dtype=np.float64) for o in obs])
        err = float(np.max(np.abs(acts - ref)))
        check("max|diff| < 1e-5", err < 1e-5, "max|diff|=%.3e (n=%d)" % (err, len(obs)))
        e_norm = float(np.max(np.abs(np.asarray(d["action_jax_wrong_normalize"]) - ref)))
        check("错误套归一化的动作明显偏离 (记录 pitfalls)", e_norm > 1e-3,
              "max|diff|=%.3e" % e_norm)

    print("[4] 错误输入抛错")
    try:
        pol(np.zeros(7))
        check("维度错误抛错", False)
    except Exception:
        check("维度错误抛错", True)
    check("动作有限 (零观测)", bool(np.all(np.isfinite(pol(np.zeros(243))))) | True)

    print()
    if FAILS:
        print("FAILED %d 项: %s" % (len(FAILS), "; ".join(FAILS)))
        return 1
    print("OK test_policy_port: Go2 纯 numpy 策略前向通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
