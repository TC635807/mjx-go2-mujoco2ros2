#!/usr/bin/env python3
"""用【官方 unitree_sdk2_python】订阅 DDS 话题 rt/lowstate，验证 DDS 互通。

用途：在容器里同时跑本仓 ROS 2 仿真（rmw_cyclonedds_cpp 发布 /lowstate）
和这个脚本（官方 Python SDK 直接订阅 rt/lowstate），如果两边能收到同一批
LowState，说明：
  1) ROS 2 的 /lowstate -> DDS rt/lowstate 映射正确；
  2) ROS 2 生成的 unitree_go 消息与官方 SDK 的 IDL 在 DDS 线上类型兼容。

用法：
    python3 dds_sub_lowstate.py --domain 1 --iface lo --duration 5
退出码：收到 >= --min-msgs 条则 0，否则 1。
"""

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=1,
                    help="DDS domain（真机固件固定 0；本仓仿真约定 1）")
    ap.add_argument("--iface", default="lo", help="网卡名（仿真 lo / 真机有线网卡）")
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--min-msgs", type=int, default=100)
    ap.add_argument("--topic", default="rt/lowstate")
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    ChannelFactoryInitialize(args.domain, args.iface)

    stats = {"n": 0, "first": None, "last": None}

    def on_state(msg) -> None:
        stats["n"] += 1
        if stats["first"] is None:
            stats["first"] = msg
        stats["last"] = msg

    sub = ChannelSubscriber(args.topic, LowState_)
    sub.Init(on_state, 10)

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < args.duration:
        time.sleep(0.05)
    wall = time.perf_counter() - t0

    n = stats["n"]
    print("[sdk-sub] topic=%s domain=%d iface=%s  rx=%d  (%.1f Hz over %.2f s)"
          % (args.topic, args.domain, args.iface, n, n / wall, wall))
    if stats["last"] is not None:
        m = stats["last"]
        q = [round(float(m.motor_state[i].q), 4) for i in range(3)]
        print("[sdk-sub] tick=%d  motor_state[0:3].q=%s  imu.rpy=%s"
              % (int(m.tick), q, [round(float(x), 4) for x in m.imu_state.rpy]))
        print("[sdk-sub] motor_state[0].mode=0x%02X  bit_flag=0x%02X  foot_force=%s"
              % (int(m.motor_state[0].mode) & 0xFF, int(m.bit_flag) & 0xFF,
                 [int(x) for x in m.foot_force]))
    if n >= args.min_msgs:
        print("[sdk-sub] PASS：官方 SDK 收到了 ROS 2 侧的 rt/lowstate")
        return 0
    print("[sdk-sub] FAIL：rx=%d < min_msgs=%d（domain/网卡/rmw 不一致？）" % (n, args.min_msgs))
    return 1


if __name__ == "__main__":
    sys.exit(main())
