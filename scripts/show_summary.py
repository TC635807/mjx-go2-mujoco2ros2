#!/usr/bin/env python3
"""scripts/show_summary.py —— 把 run_controller --json 的 summary 翻译成人话。

为什么需要：summary 是一整行 JSON，上机时根本看不出重点。这里只挑关键项：
  * 链路：n_state / n_cmd / 实际频率 / overruns / lag_p95
  * 健康：level_hist / mode_hist / final_mode（L3 占比大 = 断流触发锁存急停）
  * 策略：n_calls、**动作被夹紧比例**、动作幅值、速度估计置信度、高程来源
  * 按键急停：是否触发过

用法：
    python3 scripts/show_summary.py                     # 最近一次
    python3 scripts/show_summary.py logs/policy_x.json  # 指定文件
    python3 scripts/show_summary.py --all               # 最近 8 次对照表
"""
from __future__ import annotations

import argparse
import glob
import json
import os


def find_policy(d):
    """控制器可能被 LiveCommandController / KeyGateController 层层包住，逐层找含 n_calls 的那层。"""
    node = d.get("controller") or {}
    for _ in range(8):
        if not isinstance(node, dict):
            return None
        if "n_calls" in node:
            return node
        node = node.get("inner")
    return None


def pct(a, b):
    return 0.0 if not b else 100.0 * a / b


def render(path):
    d = json.load(open(path))
    pac = d.get("pacer") or {}
    steps = pac.get("steps") or 0
    wall = pac.get("wall_s") or 0.0
    out = ["=" * 70, " " + os.path.relpath(path), "=" * 70]
    _ov = pct(pac.get("overruns") or 0, steps)
    _ov_tag = "   <-- 实时性不达标，策略表现会被毁掉" if _ov > 10 else ""
    out.append("链路    n_state=%-7s n_cmd=%-7s %6.1f Hz   overruns=%-6s (%.1f%%)  lag_p95=%sms%s"
               % (d.get("n_state"), d.get("n_cmd"), (steps / wall) if wall else 0.0,
                  pac.get("overruns"), _ov, pac.get("lag_p95_ms"), _ov_tag))
    if pac.get("rtf") is not None and pac.get("rtf") < 0.98:
        out.append("        rtf=%.2f < 0.98：这段跑不满 500 Hz（机器负载/休眠）" % pac.get("rtf"))
    out.append("健康    level_hist=%s" % (d.get("level_hist"),))
    out.append("        mode_hist =%s  final=%s" % (d.get("mode_hist"), d.get("final_mode")))
    pol = find_policy(d)
    if pol:
        n = pol.get("n_calls") or 0
        k = pol.get("n_action_clamped") or 0
        r = pct(k, n)
        out.append("")
        out.append("策略    %s   n_calls=%d" % (pol.get("policy"), n))
        out.append("        动作被夹紧 %d/%d = %.2f%%" % (k, n, r))
        # 2026-11-05 实测：仿真里同一策略同样是 100%（886/886），说明这个数
        # 天生就会饱和（action scale + 默认姿态一直撞关节限位），单看它没有意义，
        # 只能拿不同运行之间做相对比较。别再用它判断"观测是否在训练分布"。
        out.append("                   （提示：仿真基线也是 ~100%，此数单独看无意义，只做运行间相对比较）")
        out.append("        动作幅值   mean=%.3f max=%.3f   obs_valid=%s"
                   % (pol.get("action_abs_mean") or 0.0, pol.get("action_abs_max") or 0.0,
                      pol.get("obs_valid")))
        out.append("        速度估计   source=%s conf=%s   base_lin_vel=%s"
                   % (pol.get("linvel_source"), pol.get("linvel_confidence"),
                      pol.get("base_lin_vel_world")))
        out.append("        高程       source=%s mean=%s   ground_clearance=%s"
                   % (pol.get("height_scan_source"), pol.get("height_scan_mean"),
                      pol.get("ground_clearance")))
    c = d.get("controller") or {}
    if "estop_active" in c:
        out.append("")
        out.append("按键急停 active=%s 触发次数=%s kd=%s"
                   % (c.get("estop_active"), c.get("n_estop"), c.get("estop_kd")))
    if c.get("live_cmd"):
        out.append("遥控器   %s" % (c.get("live_cmd"),))
    out.append("指令    cmd_vel=%s" % (d.get("cmd_vel"),))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="run_controller summary 摘要")
    ap.add_argument("path", nargs="?", default=None, help="summary json；缺省=最近一次")
    ap.add_argument("--all", action="store_true", help="最近 8 次对照表")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cands = [p for p in glob.glob(os.path.join(root, "logs", "*.json"))
             if "sensor" not in os.path.basename(p)]

    if args.path:
        path = args.path
        if not os.path.isabs(path) and not os.path.exists(path):
            path = os.path.join(root, path)
        if not os.path.exists(path):
            print("[show_summary] 找不到 %s" % args.path)
            return 1
        print(render(path))
        return 0

    if not cands:
        print("[show_summary] logs/ 下没有 run summary（先跑一次带 --json 的控制器）")
        return 1
    cands.sort(key=os.path.getmtime)

    if args.all:
        print("%-26s %8s %8s %6s %8s %8s" % ("file", "n_state", "n_cmd", "level", "clamp%", "overr%"))
        for p in cands[-8:]:
            try:
                d = json.load(open(p))
            except Exception:
                continue
            pol = find_policy(d)
            k = pol.get("n_action_clamped") if pol else None
            n = pol.get("n_calls") if pol else None
            pac = d.get("pacer") or {}
            hist = d.get("level_hist") or {}
            lvl = max(hist.items(), key=lambda kv: kv[1])[0] if hist else "-"
            print("%-26s %8s %8s %6s %8s %8s"
                  % (os.path.basename(p), d.get("n_state"), d.get("n_cmd"), lvl,
                     ("%.1f" % pct(k, n)) if (k is not None and n) else "-",
                     "%.1f" % pct(pac.get("overruns") or 0, pac.get("steps") or 0)))
        return 0

    print(render(cands[-1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
