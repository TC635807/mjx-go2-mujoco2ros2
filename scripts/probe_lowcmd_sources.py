#!/usr/bin/env python3
"""scripts/probe_lowcmd_sources.py —— 只读探针：/lowcmd 上到底有几个指令源、各占多少。

为什么要它（2026-11-05 实机）：停掉 sport_mode 之后，/lowcmd 上仍然有 500 Hz 的
kd=3 阻尼帧在发（Publisher count=1）。所以"停 sport_mode"并不等于"我们独占指令通道"。
本探针把 /lowcmd 的每一帧按"签名"分类，用来回答三个问题：
  1. 总帧率是 500 还是 1000？（1000 = 两个源在交替）
  2. 我们的帧（kd=0、kp=0）占多少比例？
  3. 机载阻尼源（kd=3）有没有停下来？

我们的帧和固件的帧 head/level_flag 完全一样（都是 FE EF / 0xFF），所以唯一的
区分特征就是 motor_cmd 的 kp/kd/tau：
  固件阻尼帧: mode=1, q=0, dq=0, tau=0, kp=0, kd=3.0
  我们的 zero: mode=1, q=0, dq=0, tau=0, kp=0, kd=0.0

全程只订阅，绝不发布。

用法：
    bash scripts/run_with_ros2.sh scripts/probe_lowcmd_sources.py \
        --iface enp0s31f6 --duration 10
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from collections import Counter
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="/lowcmd 指令源构成探针（只读）")
    ap.add_argument("--iface", default="enp0s31f6", help="接狗的有线网卡")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--topic", default="/lowcmd")
    return ap.parse_args(argv)


def _sig(msg) -> Tuple[int, float, float, float, int]:
    """一帧 LowCmd 的签名（取第 0 个电机槽 + level_flag）。"""
    try:
        mc = msg.motor_cmd[0]
        return (int(getattr(msg, "level_flag", -1)), round(float(mc.kp), 3),
                round(float(mc.kd), 3), round(float(mc.tau), 3), int(mc.mode))
    except Exception:  # noqa: BLE001
        return (-1, 0.0, 0.0, 0.0, -1)


def describe(sig: Tuple[int, float, float, float, int]) -> str:
    lf, kp, kd, tau, mode = sig
    if kp == 0.0 and kd == 0.0 and tau == 0.0:
        return "零力矩(kp=0,kd=0,tau=0)  <- 我们的 zero 控制器"
    if kp == 0.0 and kd > 0.0 and tau == 0.0:
        return "纯阻尼(kp=0,kd=%.1f)      <- 固件/机载桥在维持" % kd
    if kp > 0.0:
        return "位置控制(kp=%.1f,kd=%.1f)  <- 有控制器在做位置环" % (kp, kd)
    return "其他(level_flag=%d,mode=%d,kp=%.2f,kd=%.2f,tau=%.2f)" % (lf, mode, kp, kd, tau)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    os.environ["CYCLONEDDS_URI"] = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="%s" priority="default" multicast="default" />'
        '</Interfaces></General></Domain></CycloneDDS>' % args.iface)
    os.environ.setdefault("ROS_DOMAIN_ID", str(args.domain))

    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import LowCmd

    rclpy.init(args=None)
    node = Node("go2_probe_lowcmd")
    for p in getattr(node, "publishers", []):
        t = str(getattr(p, "topic_name", "") or "")
        assert not t or t in ("/rosout", "/parameter_events") or t.startswith("/rosout"), \
            "探针必须只读"

    counts: Counter = Counter()
    timeline: List[Tuple[float, int, Tuple]] = []   # (t, n_pub, sig)
    t0 = time.monotonic()

    def on_cmd(msg):
        counts[_sig(msg)] += 1
        if len(timeline) < 200000:
            timeline.append((round(time.monotonic() - t0, 3), n_pub_now[0], _sig(msg)))

    n_pub_now = [0]
    node.create_subscription(LowCmd, args.topic, on_cmd, 10)

    print("[probe] 只读监听 %s，时长 %.1f s（不发布任何帧）" % (args.topic, args.duration))
    print("[probe] 现在去启动/停掉控制器，我在看帧的构成 ...", flush=True)
    nxt = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= nxt:
            nxt = time.monotonic() + 0.25
            try:
                n_pub_now[0] = len(node.get_publishers_info_by_topic(args.topic) or [])
            except Exception:  # noqa: BLE001
                n_pub_now[0] = -1
    dur = time.monotonic() - t0
    total = sum(counts.values())

    print()
    print("=" * 72)
    print(" /lowcmd 指令源构成")
    print("=" * 72)
    print("采集 %.2f s，共 %d 帧，总帧率 %.1f Hz，当前发布者 %d 个"
          % (dur, total, total / max(dur, 1e-6), n_pub_now[0]))
    print()
    print("-- 按帧签名分类 --")
    for sig, n in counts.most_common():
        print("  %5d 帧 (%5.1f%%)  %s" % (n, 100.0 * n / max(total, 1), describe(sig)))
    print()
    hz = total / max(dur, 1e-6)
    kinds = len([s for s, n in counts.items() if n > max(3, 0.01 * total)])
    print("-- 判定 --")
    if kinds >= 2 and hz > 700:
        print("  总帧率 %.0f Hz 且出现 %d 种签名 -> 【两个指令源在交替】。" % (hz, kinds))
        print("  控制器发的帧确实上了总线，但它和机载桥各占一部分；电机每 1 ms 收到哪条看谁后到。")
        print("  这就是「喀哒喀哒」颗粒感的来源。")
    elif kinds >= 2:
        print("  出现 %d 种签名，但总帧率只有 %.0f Hz -> 有一个源在丢帧/被限速。" % (kinds, hz))
    elif hz > 700:
        print("  总帧率 %.0f Hz 但只有 1 种签名 -> 单一源在超频发（异常，查是否两个发布者同内容）" % hz)
    else:
        only = counts.most_common(1)[0][0] if counts else None
        print("  只有一个源，帧率 %.0f Hz：%s" % (hz, describe(only) if only else "-"))
        if only is not None and only[2] == 0.0:
            print("  -> 机载阻尼源已经停了，现在 /lowcmd 上只有我们。")
        else:
            print("  -> 现在 /lowcmd 上只有固件/机载桥，没有我们的帧（控制器没在跑？）")
    print()
    print("注意：本探针全程只订阅，任何时刻都没有发出 LowCmd。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
