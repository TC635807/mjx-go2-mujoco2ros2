#!/usr/bin/env python3
"""真机 Go2 上电自检：只订阅、不发布任何指令的只读探针。

===============================================================================
1. 什么时候跑它
===============================================================================

  第一次连真机、改网络配置、换固件、或"控制器能起但机器人不动"时，先跑这个：

      scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

  它不会发 LowCmd、不会改机器人状态，**可以放心在吊起/落地前运行**。
  输出是给人看的诊断 + 退出码（0=低层状态链路 OK，1=没收到 /lowstate）。

  仿真自检（验证脚本本身 / 对比输出）：
      scripts/run_with_ros2.sh scripts/go2_real_check.py --target sim_ros --duration 3 \
          --state-topic /lowstate --cloud-topic /go2/lidar/points

===============================================================================
2. 检查哪些项、期望值是什么
===============================================================================

  (1) 环境
      ROS_DOMAIN_ID=0（真机）   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      CYCLONEDDS_URI 绑定有线网卡    ROS_LOCALHOST_ONLY 未设置
      这些由 config.apply_io_env() 设置；必须发生在 rclpy.init 之前。

  (2) /lowstate（unitree_go/LowState，机器人 -> PC）
      - 频率：官方例程 500 Hz；tick 增量：真机固件按 1 ms 递增
      - motor_state[0:12].mode：低层控制下应为 0x01；全 0 = 电机未使能
        （常见原因：没释放 sport mode / CRC 错 / 没进 debug mode）
      - foot_force[4]：顺序 FR,FL,RR,RL；Go2 AIR/PRO 可能没有该传感器
      - imu_state.rpy：姿态，用于确认 IMU 正常

  (3) /lowcmd（unitree_go/LowCmd，PC -> 机器人）
      - 条数 > 0 说明有别的进程在向机器人发指令（可能是另一个控制器/示例）
      - 逐条校验 812 字节布局 CRC：bad_crc > 0 = 发布方布局/算法不对
      - crc=0 = 没填 CRC；真机固件大概率丢弃
      注意：本脚本只还原本项目发布方会填的字段（head/level/frame_reserve/gpio +
      电机）；若别的发布方填了 sn/version/bandwidth/bms/wireless/led/fan/reserve，
      会误报 bad_crc。

  (4) /utlidar/cloud（sensor_msgs/PointCloud2，L1 -> PC）
      - frame_id 应为 utlidar_lidar（雷达系）
      - 频率约 11 Hz（L1 周向扫描频率），单帧点数约几千
      - x/y/z 分位用于**标定**：z 的 50% 分位 ≈ 雷达离地高度；
        x/y 的分布可判断轴向（本项目机体系 = x 前 / y 左 / z 上）。
        官方 URDF 与社区 URDF 的外参互相矛盾（见 perception_node.py §3），
        必须用这里的实测点云 + 已知平面/台阶做标定。

  (5) 位姿 / 高层状态
      /utlidar/robot_pose（PoseStamped）、/utlidar/robot_odom（Odometry）、
      /sportmodestate（SportModeState）：
      - 官方 unitree_mujoco readme 明确：释放 sport mode 后 sportmodestate 不可读；
      - robot_odom/robot_pose 在部分固件上长期为空（厂商建议接 LIO）。
      所以"没有 odom"通常是预期现象，控制器应回退 leg_fk 而不是报错。

===============================================================================
3. QoS（容易踩）
===============================================================================

  低层状态/指令用默认可靠 QoS（depth=10）：unitree_ros2 官方例程就是这样。
  点云必须用 qos_profile_sensor_data（BEST_EFFORT）：机器人侧 L1 驱动通常是
  BEST_EFFORT，用 RELIABLE 订阅会一条都收不到。

===============================================================================
4. 退出码
===============================================================================

  0  收到过 /lowstate（低层状态链路 OK；其余项只是 warning）
  1  整个观察期没有 /lowstate（链路不通，先查网线/网段/domain/网卡/CYCLONEDDS_URI）

相关文件：
  real_env/README.md             真机完整部署 checklist
  signal_io/go2_crc.py           812 字节 CRC 布局（本脚本用它校验 /lowcmd）
  signal_io/config.py            apply_io_env（domain / RMW / 网卡）
  signal_io/perception_node.py   L1 点云坐标系与外参说明
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
    """ROS 2 builtin_interfaces/Time -> 秒（float）。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    """订阅所有相关话题 duration 秒，打印诊断并给出结论。"""
    # ------------------------------------------------------------ 1) 参数
    ap = argparse.ArgumentParser(
        description="Go2 真机只读自检（不发送任何指令）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default="real_ros", choices=["real_ros", "real_sdk", "sim_ros"],
                    help="IO profile；真机用 real_ros")
    ap.add_argument("--duration", type=float, default=5.0, help="观察时长（秒）")
    ap.add_argument("--state-topic", default="/lowstate")
    ap.add_argument("--cmd-topic", default="/lowcmd")
    ap.add_argument("--cloud-topic", default="/utlidar/cloud")
    ap.add_argument("--pose-topic", default="/utlidar/robot_pose")
    ap.add_argument("--odom-topic", default="/utlidar/robot_odom")
    ap.add_argument("--sport-topic", default="/sportmodestate")
    ap.add_argument("--no-cmd-check", action="store_true",
                    help="不订阅 /lowcmd（避免把别的控制器误判成异常）")
    args = ap.parse_args()

    # ------------------------------------------------------------ 2) 环境
    cfg = load_config(args.config, target=args.target)
    env = apply_io_env(cfg)   # 必须在 rclpy.init 之前

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from unitree_go.msg import LowCmd, LowState, SportModeState
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    # 统计容器：回调里只累加，最后统一打印。
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

    # ------------------------------------------------------------ 3) 回调
    def on_state(m: LowState) -> None:
        """/lowstate：统计频率/tick/电机 mode/足端力/姿态（只取首帧的静态量）。"""
        s = stats["state"]
        s["n"] += 1
        s["t"].append(time.monotonic())
        s["ticks"].append(int(m.tick))
        if s["modes"] is None and len(m.motor_state) >= 12:
            s["modes"] = [int(m.motor_state[i].mode) for i in range(12)]
            s["foot"] = [int(x) for x in list(m.foot_force)[:4]]
            s["rpy"] = [float(x) for x in list(m.imu_state.rpy)[:3]]

    def on_cmd(m: LowCmd) -> None:
        """/lowcmd：统计条数，并按 812 字节布局校验 CRC（判断别的发布方是否正确）。"""
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
        """/utlidar/cloud：统计频率/点数/frame_id，并取前 5 帧的 xyz 分位用于标定。"""
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

    # ------------------------------------------------------------ 4) 订阅
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

    # ------------------------------------------------------------ 5) 观察
    t_end = time.monotonic() + max(args.duration, 0.5)
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    def hz(ts):
        """由时间戳列表估算频率（Hz）。"""
        if len(ts) < 2:
            return 0.0
        dt = ts[-1] - ts[0]
        return (len(ts) - 1) / dt if dt > 1e-9 else 0.0

    # ------------------------------------------------------------ 6) 报告
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
