#!/usr/bin/env python3
"""scripts/probe_remote.py —— 只读探针：把遥控器按键实时解出来（验证"机器人到底有没有收到我按的键"）。

为什么要它（2026-11-05 实机）：按 L2+R2 进调试模式时，不确定按键是没按对、遥控器没连上、
还是机器人收到了但没响应。/wirelesscontroller 这个话题直接把按键位图发出来，
所以可以先在软件侧确认"键真的到了"，再去看固件的反应，避免瞎猜。

用法：
    bash scripts/run_with_ros2.sh scripts/probe_remote.py --iface enp0s31f6 --duration 30
    # 期间你按任意键，屏幕上会实时打印位图变化

位图含义（unitree 官方 WirelessController.keys，16 位）：
    0:R1 1:L1 2:start 3:select 4:R2 5:L2 6:F1 7:F2
    8:A 9:B 10:X 11:Y 12:up 13:right 14:down 15:left
如果实际按键和名字对不上，把打印出来的位图发我，我们按实测重新标定（--raw 只打原始值）。
全程只订阅，绝不发布。
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

BUTTONS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
           "A", "B", "X", "Y", "up", "right", "down", "left"]


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="遥控器按键只读探针")
    ap.add_argument("--iface", default="enp0s31f6")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--topic", default="/wirelesscontroller")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--raw", action="store_true", help="只打原始位图，不翻译")
    return ap.parse_args(argv)


def decode(keys: int) -> List[str]:
    return [BUTTONS[b] for b in range(16) if keys & (1 << b)]


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    os.environ["CYCLONEDDS_URI"] = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="%s" priority="default" multicast="default" />'
        '</Interfaces></General></Domain></CycloneDDS>' % args.iface)
    os.environ.setdefault("ROS_DOMAIN_ID", str(args.domain))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import WirelessController

    rclpy.init(args=None)
    node = Node("go2_probe_remote")
    for p in getattr(node, "publishers", []):
        t = str(getattr(p, "topic_name", "") or "")
        assert not t or t in ("/rosout", "/parameter_events") or t.startswith("/rosout"), \
            "探针必须只读"

    state = {"keys": None, "n": 0}
    t0 = time.monotonic()

    def on_msg(msg):
        k = int(msg.keys) & 0xFFFF
        ax = (round(float(msg.lx), 3), round(float(msg.ly), 3),
              round(float(msg.rx), 3), round(float(msg.ry), 3))
        state["n"] += 1
        moved = state.get("ax") is not None and any(
            abs(ax[i] - state["ax"][i]) > 0.05 for i in range(4))
        if k != state["keys"] or moved or state.get("ax") is None:
            state["keys"] = k
            state["ax"] = ax
            dt = time.monotonic() - t0
            names = decode(k)
            print("[%6.2fs] keys=0x%04X %-22s lx=%+.2f ly=%+.2f rx=%+.2f ry=%+.2f"
                  % (dt, k, ("+".join(names) if names else "(无按键)"),
                     ax[0], ax[1], ax[2], ax[3]), flush=True)

    node.create_subscription(WirelessController, args.topic, on_msg, qos_profile_sensor_data)
    print("[probe] 只读监听 %s，%.1f s。现在按遥控器按键 ..." % (args.topic, args.duration), flush=True)
    end = t0 + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    print()
    print("共收到 %d 帧；最后 keys=0x%04X" % (state["n"], state["keys"] or 0))
    if state["n"] == 0:
        print("[!] 一帧都没收到：遥控器没连上/没唤醒，或话题名不对")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
