#!/usr/bin/env python3
"""真机 Go2 上电自检：**只订阅、不发布任何指令的只读探针**。

第一次连真机时先跑它，确认：话题在不在、频率对不对、坐标系是什么、
LowCmd 有没有别的进程在发、CRC 是否合法、点云轴向如何（用于标定）。

    scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
    # 仿真自检（验证脚本本身）
    scripts/run_with_ros2.sh scripts/go2_real_check.py --target sim_ros --duration 3 \
        --cloud-topic /go2/lidar/points --state-topic /lowstate

检查项（对应 real_env/README.md 的上电 checklist）：
  1. 环境：ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI / ROS_LOCALHOST_ONLY
  2. /lowstate：频率、tick 增量、电机 mode（低层控制下应为 0x01）、foot_force
  3. /lowcmd：是否有别的发布者；有的话逐条校验 812B 布局 CRC（signal_io/go2_crc.py）
  4. /utlidar/cloud：frame_id、频率、点数、x/y/z 分位（判断雷达系轴向/离地高度）
  5. /utlidar/robot_pose、/utlidar/robot_odom、/sportmodestate、/wirelesscontroller 是否有数据
     （官方文档：释放 sport mode 后 sportmodestate 不可读；robot_odom/pose 可能长期为空）
  6. 结论速判：低层链路是否 ready、还缺什么
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from signal_io.config import apply_io_env, load_config  # noqa: E402


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default="real_ros", choices=["real_ros", "real_sdk", "sim_ros"])
    ap.add_argument("--duration", type=float, default=5.0, help="观察时长（秒）")
    ap.add_argument("--state-topic", default="/lowstate")
    ap.add_argument("--cmd-topic", default="/lowcmd")
    ap.add_argument("--cloud-topic", default="/utlidar/cloud")
    ap.add_argument("--pose-topic", default="/utlidar/robot_pose")
    ap.add_argument("--odom-topic", default="/utlidar/robot_odom")
    ap.add_argument("--sport-topic", default="/sportmodestate")
    ap.add_argument("--no-cmd-check", action="store_true", help="不订阅 /lowcmd（避免误判）")
    args = ap.parse_args()

    cfg = load_config(args.config, target=args.target)
    env = apply_io_env(cfg)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    stats = {
        "state": {"n": 0, "t": [], "ticks": [], "modes": None, "foot": None, "rpy": None},
        "cmd": {"n": 0, "bad_crc": 0, "crc0": 0, "last": None},
        "cloud": {"n": 0, "t": [], "frames": set(), "npts": [], "ranges": []},
        "pose": {"n": 0, "last": None, "frames": set()},
        "odom": {"n": 0, "last": None},
        "sport": {"n": 0},
    }

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("go2_real_check")

    from unitree_go.msg import LowCmd, LowState
    from unitree_go.msg import SportModeState, WirelessController
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    def on_state(m: LowState) -> None:
        s = stats["state"]
        s["n"] += 1
        s["t"].append(time.monotonic())
        s["ticks"].append(int(m.tick))
        if s["modes"] is None and len(m.motor_state) >= 12:
            s["modes"] = [int(m.motor_state[i].mode) for i in range(12)]
            s["foot"] = [int(x) for x in list(m.foot_force)[:4]]
            s["rpy"] = [float(x) for x in list(m.imu_state.rpy)[:3]]

    def on_cmd(m: LowCmd) -> None:
        c = stats["cmd"]
        c["n"] += 1
        msg_crc = int(getattr(m, "crc", 0)) & 0xFFFFFFFF
        # 把 ROS 消息按 812B 布局重新打包后校验（与 firmware 同口径）。
        # 注意：本脚本只还原本项目发布方会填的字段（head/level/frame_reserve/gpio + 电机）；
        # 若别的发布方填了 sn/version/bandwidth/bms/wireless/led/fan/reserve，会误报 bad_crc。
        from signal_io.msg import LowCmd as PyLowCmd, MotorCmd
        py = PyLowCmd(head=[int(m.head[0]), int(m.head[1])], level_flag=int(m.level_flag),
                      frame_reserve=int(m.frame_reserve), gpio=int(m.gpio))
        for i in range(20):
            src = m.motor_cmd[i]
            py.motor_cmd[i] = MotorCmd(mode=int(src.mode), q=float(src.q), dq=float(src.dq),
                                       tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                                       reserve=int(src.reserve[0]))
        if msg_crc == 0:
            c["crc0"] += 1
        else:
            from signal_io.go2_crc import lowcmd_crc
            if msg_crc != lowcmd_crc(py):
                c["bad_crc"] += 1
        c["last"] = {"head": [int(m.head[0]), int(m.head[1])],
                     "level_flag": int(m.level_flag), "crc": msg_crc,
                     "kp": [round(float(m.motor_cmd[i].kp), 2) for i in range(12)]}

    def on_cloud(m: PointCloud2) -> None:
        s = stats["cloud"]
        s["n"] += 1
        s["t"].append(time.monotonic())
        s["frames"].add(m.header.frame_id)
        n = int(m.width) * int(m.height)
        s["npts"].append(n)
        if len(s["ranges"]) < 5:
            try:
                from sensor_msgs_py import point_cloud2
                pts = np.array(list(point_cloud2.read_points(
                    m, field_names=("x", "y", "z"), skip_nans=True)))
                if pts.size:
                    p = np.stack([pts[name] for name in ("x", "y", "z")], -1) \
                        if pts.dtype.names else pts.reshape(-1, 3)
                    s["ranges"].append(np.percentile(np.asarray(p, np.float64),
                                                     [1, 50, 99], axis=0).round(3).tolist())
            except Exception:
                pass

    def on_pose(m: PoseStamped) -> None:
        stats["pose"]["n"] += 1
        stats["pose"]["frames"].add(m.header.frame_id)
        stats["pose"]["last"] = [round(m.pose.position.x, 3), round(m.pose.position.y, 3),
                                 round(m.pose.position.z, 3)]

    def on_odom(m: Odometry) -> None:
        stats["odom"]["n"] += 1
        stats["odom"]["last"] = [round(m.pose.pose.position.x, 3),
                                 round(m.pose.pose.position.y, 3),
                                 round(m.pose.pose.position.z, 3)]

    def on_sport(m: SportModeState) -> None:
        stats["sport"]["n"] += 1

    # 低层状态/指令用默认 QoS（可靠）：unitree_ros2 官方例程就是 depth=10 默认；
    # 点云是传感器数据，机器人侧多为 BEST_EFFORT，必须用 sensor_data，否则收不到。
    node.create_subscription(LowState, args.state_topic, on_state, 10)
    if not args.no_cmd_check:
        node.create_subscription(LowCmd, args.cmd_topic, on_cmd, 10)
    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, args.pose_topic, on_pose, 10)
    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    node.create_subscription(SportModeState, args.sport_topic, on_sport, 10)

    print("[check] 环境 =", env or "(未额外设置，沿用 shell)")
    print("[check] domain=%s iface=%s | 观察 %.1fs | %s / %s / %s"
          % (os.environ.get("ROS_DOMAIN_ID"), cfg["transport"].get("interface"),
             args.duration, args.state_topic, args.cloud_topic, args.cmd_topic))
    print("[check] ROS_LOCALHOST_ONLY =", os.environ.get("ROS_LOCALHOST_ONLY", "(unset)"))

    t_end = time.monotonic() + max(args.duration, 0.5)
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    def hz(ts):
        if len(ts) < 2:
            return 0.0
        dt = ts[-1] - ts[0]
        return (len(ts) - 1) / dt if dt > 1e-9 else 0.0

    print()
    print("=== 1. /lowstate ===")
    s = stats["state"]
    print("  条数=%d  频率=%.1f Hz" % (s["n"], hz(s["t"])))
    if s["ticks"]:
        d = np.diff(np.asarray(s["ticks"], dtype=np.int64))
        print("  tick: first=%d last=%d  Δtick(中位)=%s  (仿真 tick=2ms/步，真机固件 tick=1ms)"
              % (s["ticks"][0], s["ticks"][-1],
                 int(np.median(d)) if d.size else "-"))
        print("  电机 mode[0:12] =", s["modes"], "(低层控制应为 0x01，0 表示未使能)")
        print("  foot_force =", s["foot"], " rpy =", [round(x, 3) for x in (s["rpy"] or [])])

    print()
    print("=== 2. /lowcmd（是否有别的进程在发） ===")
    c = stats["cmd"]
    if args.no_cmd_check:
        print("  (跳过)")
    else:
        print("  条数=%d  crc=0 的条数=%d  CRC 校验失败=%d" % (c["n"], c["crc0"], c["bad_crc"]))
        if c["last"]:
            print("  最后一条: head=%s level_flag=0x%02X crc=0x%08X kp[0:12]=%s"
                  % (c["last"]["head"], c["last"]["level_flag"], c["last"]["crc"],
                     c["last"]["kp"]))
        if c["n"] and c["bad_crc"]:
            print("  !! CRC 校验失败：发布方布局/算法不对，固件很可能静默丢帧")
        if c["n"] and c["crc0"] == c["n"]:
            print("  !! 所有 LowCmd 都没填 crc -> 固件不会执行（除非它不校验）")

    print()
    print("=== 3. L1 点云 ===")
    cl = stats["cloud"]
    print("  条数=%d  频率=%.1f Hz  frame_id=%s" % (cl["n"], hz(cl["t"]), sorted(cl["frames"])))
    if cl["npts"]:
        print("  每帧点数: 中位=%d 最大=%d" % (int(np.median(cl["npts"])), max(cl["npts"])))
    for i, r in enumerate(cl["ranges"]):
        print("  第%d帧 xyz 分位 [1%%,50%%,99%%] = %s" % (i + 1, r))
    if cl["n"]:
        print("  提示: frame_id 应为 utlidar_lidar（雷达系）。上面 z 的 50%% 分位≈雷达离地高度；")
        print("        x/y 的分布可判断轴向：本项目机体系 = x 前 / y 左 / z 上。")

    print()
    print("=== 4. 里程计 / 高层状态 ===")
    print("  /utlidar/robot_pose 条数=%d frame=%s last=%s"
          % (stats["pose"]["n"], sorted(stats["pose"]["frames"]), stats["pose"]["last"]))
    print("  /utlidar/robot_odom 条数=%d last=%s" % (stats["odom"]["n"], stats["odom"]["last"]))
    print("  /sportmodestate     条数=%d" % stats["sport"]["n"])
    print("  说明: 官方 unitree_mujoco 文档明确——释放 sport mode 后 sportmodestate 不可读；")
    print("        robot_odom/robot_pose 在部分固件上长期为空，不能作为 odom 的唯一来源。")

    print()
    print("=== 5. 结论 ===")
    ready = s["n"] > 0
    print("  低层状态链路:", "OK" if ready else "FAIL（收不到 /lowstate：检查网线/网段/domain/网卡/CYCLONEDDS_URI）")
    if s["modes"] is not None and ready:
        all_on = all(m == 0x01 for m in s["modes"])
        print("  电机伺服使能:", "OK (0x01)" if all_on else "未使能/被覆盖（先确认已 ReleaseMode 或 L2+R2 debug mode）")
    print("  点云链路:", "OK" if cl["n"] > 0 else "无数据（L1 未开或话题名不同：先发 rt/utlidar/switch ON）")
    print("  odom:", "有数据" if (stats["pose"]["n"] or stats["odom"]["n"])
          else "无数据（预期内：走 --scan leg_fk，或接 LIO/EKF）")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0 if ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
