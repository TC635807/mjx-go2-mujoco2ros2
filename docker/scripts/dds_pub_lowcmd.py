#!/usr/bin/env python3
"""用【官方 unitree_sdk2_python】向 DDS 话题 rt/lowcmd 发合法 LowCmd。

用途：让本仓 ROS 2 仿真（订阅 /lowcmd，crc_check=True）校验官方 SDK 发出的
LowCmd —— 这条链路等价于“真机固件会收到什么”，因此可以提前证明：
  1) 812 字节 IDL 布局 + crc32_core 与官方 SDK 逐位一致；
  2) ROS 2 /lowcmd -> DDS rt/lowcmd 映射正确；
  3) 固定头 head=[0xFE,0xEF] / level_flag=0xFF / motor mode=0x01 都对。

用法：
    python3 dds_pub_lowcmd.py --domain 1 --iface lo --duration 6 --rate 500
"""

import argparse
import sys
import time

# 标称站姿：与官方部署惯例一致（FR, FL, RR, RL x hip/thigh/calf）
STAND_POSE = [-0.1, 0.8, -1.5,
              0.1, 0.8, -1.5,
              -0.1, 1.0, -1.5,
              0.1, 1.0, -1.5]

# 官方 unitree_sdk2_python/example/go2/low_level/unitree_legged_const.py 的哨兵值
POS_STOP_F = 2.146e9
VEL_STOP_F = 1.0e9


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=1)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--rate", type=float, default=500.0, help="发布频率 Hz（固件按 500 Hz 用）")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--no-crc", action="store_true", help="故意发 crc=0（负向测试用）")
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    ChannelFactoryInitialize(args.domain, args.iface)

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01   # FOC / 伺服使能
        cmd.motor_cmd[i].q = POS_STOP_F
        cmd.motor_cmd[i].dq = VEL_STOP_F
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
    for i in range(12):
        cmd.motor_cmd[i].q = float(STAND_POSE[i])
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = args.kp
        cmd.motor_cmd[i].kd = args.kd
        cmd.motor_cmd[i].tau = 0.0

    crc = CRC()
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    dt = 1.0 / args.rate
    n = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < args.duration:
        cmd.crc = 0 if args.no_crc else crc.Crc(cmd)
        pub.Write(cmd)
        n += 1
        # 简单节拍：落后就跳过 sleep（不追帧）
        target = t0 + n * dt
        now = time.perf_counter()
        if target > now:
            time.sleep(target - now)
    wall = time.perf_counter() - t0
    print("[sdk-pub] topic=rt/lowcmd domain=%d iface=%s  tx=%d (%.1f Hz)  crc=%s"
          % (args.domain, args.iface, n, n / wall, "off(0)" if args.no_crc else "on"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
