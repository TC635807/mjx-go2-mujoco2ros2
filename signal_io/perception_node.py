#!/usr/bin/env python3
"""ROS 2 感知节点：点云 + 位姿 -> 187 点高度扫描（仿真 / 真机同一份）。

===============================================================================
1. 数据流
===============================================================================

  仿真:
      go2_sim_sensors  --/go2/lidar/points-->  （世界系 PointCloud2）
                       --/go2/odom--------->  （nav_msgs/Odometry 真值）
                                                   │
  真机:                                            ▼
      Unitree L1 驱动 --/utlidar/cloud---->  本节点 go2_perception
                      --/utlidar/robot_pose>   │
                                                   ▼
                                          /perception/height_scan
                                          （Float32MultiArray，187 点）
                                                   │
                                                   ▼
                                          go2_controller（--scan topic）

  控制器完全不知道数据来自仿真还是真机；它只认 187 维高度扫描这个契约。

===============================================================================
2. 高度扫描是怎么算出来的（与训练侧唯一真相对齐）
===============================================================================

  训练侧 mjx-go2-training/envs/go2_walk.py::height_scan_features：
      grid      : 17x11 机体系采样点，x∈[-0.4,1.2], y∈[-0.5,0.5]
      xy_world  = Rz(yaw) @ xy_local + base_xy     （只按 yaw，不含 roll/pitch）
      d_m       = base_z - h_ground(xy_world) - 0.268
      feature   = clip(d_m, ±0.35) * 5.0

  本节点的实现分两步：
      1) HeightMapper.update(points_world, t)：把世界系点云投到 5 cm 稀疏格网，
         每格存 max z（地面/障碍物的最高点），并按 decay_s 衰减旧观测。
      2) HeightMapper.scan(base_pos, base_quat)：把 17x11 机体系网格按 yaw 旋到世界系，
         双线性插值取地面高度 h，再算上面的 feature。
  所以本节点必须把输入点云统一到**世界系**再喂给 HeightMapper。

===============================================================================
3. 坐标系与真机 L1（最容易错的地方）
===============================================================================

  三种 --points-frame：
      world  点云已经是世界系（仿真 MuJoCo raycast 用这个）
      body   点云已在机体系（需要 odom 提供 base 位姿转到世界系）
      lidar  L1 雷达系（真机 /utlidar/cloud 的 frame_id 是 utlidar_lidar）

  雷达系 -> 机体系（刚体外参）：
      p_base = R(rpy) @ p_lidar + t
      其中 t = --lidar-mount（默认 config.perception.mount_body）
           R = --lidar-rpy  （默认 config.perception.mount_rpy_body）

  WARN: 外参来源互相矛盾，**必须真机标定**：
      * 官方 unitree_ros go2_description.urdf 的 radar_joint：
            xyz=(0.28945, 0, -0.046825), rpy=(0, 2.8782, 0)  ← 前装、倒装约 165 度
      * 社区 go2_ros2_sdk 的 URDF：顶装 xyz≈(0.0018, 0.0001, 0.2536), rpy=0
    两者不可能同时成立。当前默认取官方值（前装 + 2.8782 rad 旋转），
    但标定前不要用真感知跑策略，先在平地/已知台阶上验证扫描是否与真实地形一致。

  WARN: 真机上 /utlidar/robot_pose、/utlidar/robot_odom 很多固件没有数据
        （厂商建议接 LIO）。没有位姿时本节点会一直 skip 并打警告，
        控制器会自动回退 leg_fk（单标量，平地近似正确、地形不可信）。

===============================================================================
4. 输入输出 / QoS
===============================================================================

  in  --points-topic   sensor_msgs/PointCloud2
      仿真 /go2/lidar/points（世界系）；真机 /utlidar/cloud（lidar 系，约 11 Hz）
  in  --odom-topic     nav_msgs/Odometry（--odom-type odom）
      或 geometry_msgs/PoseStamped（--odom-type pose，真机 /utlidar/robot_pose）
  out --scan-topic     std_msgs/Float32MultiArray（187 个 float，顺序 x 外层 y 内层）

  QoS：点云用 qos_profile_sensor_data（BEST_EFFORT），机器人侧 L1 驱动多为
  BEST_EFFORT，用默认 RELIABLE 会收不到。位姿/扫描用默认 reliable。

===============================================================================
5. 排障
===============================================================================

  * 仿真没有 height_scan：看 /go2/odom、/go2/lidar/points 是否都有数据。
  * 真机没有 height_scan：先跑 scripts/go2_real_check.py 看 /utlidar/cloud 和
    /utlidar/robot_pose 是否有数据；没有 pose 属于预期（见上）。
  * 扫描值与真实地形不符：先标定外参（mount/rpy），再检查点云世界系变换是否正确。
  * 频率：默认取 config.perception.rate_hz（15 Hz），真机 L1 只有 ~11 Hz，
    不需要更高；高程图有 decay 记忆，控制 50 Hz 用最新扫描即可。

用法：
    scripts/run_with_ros2.sh signal_io/perception_node.py --target sim_ros
    scripts/run_with_ros2.sh signal_io/perception_node.py --target real_ros \
        --points-topic /utlidar/cloud --points-frame lidar \
        --odom-topic /utlidar/robot_pose --odom-type pose

相关文件：
  signal_io/perception.py    HeightMapper / SimLidar / 训练侧契约常量
  signal_io/estimator.py     quat_to_mat
  ros2_controller/controller.py  PolicyController._height_scan（消费 187 点，缺失时回退）
  ros2_ws/launch/sim.launch.py / real.launch.py
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
from signal_io.estimator import quat_to_mat  # noqa: E402
from signal_io.perception import HeightMapper  # noqa: E402


def _stamp_seconds(stamp) -> float:
    """ROS 2 builtin_interfaces/Time -> 秒（float）。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _cloud_to_xyz(msg) -> np.ndarray:
    """PointCloud2 -> (N,3) float64；只取 x/y/z，丢弃 NaN/Inf。

    不同版本 sensor_msgs_py.read_points 的返回类型可能是结构化数组或普通 (N,3)，
    这里两种都兼容。
    """
    from sensor_msgs_py import point_cloud2

    raw = np.array(list(point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True)))
    if raw.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if raw.dtype.names:
        pts = np.stack([raw[name] for name in ("x", "y", "z")], axis=-1)
    else:
        pts = raw.reshape(-1, 3)
    return np.asarray(pts, dtype=np.float64).reshape(-1, 3)


def _rpy_to_mat(rpy) -> np.ndarray:
    """rpy (roll, pitch, yaw) -> 3x3 旋转矩阵（R = Rz(yaw) @ Ry(pitch) @ Rx(roll)）。

    官方 unitree_ros 的 Go2 radar_joint 是 rpy=(0, 2.8782, 0)（约 165 度，L1 倒装），
    所以「雷达系 -> 机体系」不是简单平移；真机必须带旋转，否则整帧点云错位。
    """
    r, p, y = (float(v) for v in rpy)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def main() -> int:
    """入口：配置 -> 环境 -> 订阅点云/位姿 -> 定时生成并发布 height_scan。"""
    # ------------------------------------------------------------ 1) 参数
    ap = argparse.ArgumentParser(
        description="点云 + 位姿 -> 187 点高度扫描（仿真/真机同一份节点）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default=None,
                    help="IO profile（决定 ROS_DOMAIN_ID / 网卡；仿真 sim_ros，真机 real_ros）")
    ap.add_argument("--points-topic", default="/go2/lidar/points",
                    help="PointCloud2 话题；仿真 /go2/lidar/points，真机 /utlidar/cloud")
    ap.add_argument("--odom-topic", default="/go2/odom",
                    help="位姿话题；仿真 /go2/odom，真机可能是 /utlidar/robot_pose")
    ap.add_argument("--odom-type", default="odom", choices=["odom", "pose"],
                    help="odom=nav_msgs/Odometry（仿真）；pose=geometry_msgs/PoseStamped"
                         "（真机 /utlidar/robot_pose）")
    ap.add_argument("--scan-topic", default="/perception/height_scan",
                    help="输出的高度扫描话题（std_msgs/Float32MultiArray）")
    ap.add_argument("--points-frame", default="world", choices=["world", "body", "lidar"],
                    help="world=点云已在世界系（仿真）；body=机体系；"
                         "lidar=L1 雷达系（真机 /utlidar/cloud），按 --lidar-mount/--lidar-rpy 变换")
    ap.add_argument("--lidar-mount", type=float, nargs=3, default=None,
                    help="雷达相对 base 的安装位置 (x y z)，默认取 config.perception.mount_body")
    ap.add_argument("--lidar-rpy", type=float, nargs=3, default=None,
                    help="雷达相对 base 的安装姿态 (roll pitch yaw)，默认取 "
                         "config.perception.mount_rpy_body；官方 Go2 radar_joint 是 0 2.8782 0")
    ap.add_argument("--rate-hz", type=float, default=None,
                    help="发布 height_scan 的频率（默认取 config.perception.rate_hz）")
    args = ap.parse_args()

    # ------------------------------------------------------------ 2) 配置 + 环境
    cfg = load_config(args.config, target=args.target)
    apply_io_env(cfg)                       # 必须在 rclpy.init 之前设 domain / RMW

    # 扫描契约（训练侧唯一真相；17x11=187、x/y 范围、scan.base=0.268）
    p = dict(cfg.get("policy") or {})
    sc = dict(p.get("scan") or {})
    nx = int(sc.get("nx", 17))
    ny = int(sc.get("ny", 11))
    x_range = tuple(sc.get("x", (-0.4, 1.2)))
    y_range = tuple(sc.get("y", (-0.5, 0.5)))
    scan_base = float(cfg.get("scan_base", 0.268))

    perc = dict(cfg.get("perception") or {})
    rate_hz = float(args.rate_hz if args.rate_hz is not None
                    else (perc.get("rate_hz", 15.0) or 15.0))

    # 雷达外参：p_base = R(rpy) @ p_lidar + t。外参来源有冲突（见模块 docstring §3），
    # 默认取官方 unitree_ros radar_joint。
    # TODO(real): 用实测点云标定 mount/rpy（地面/已知台阶），标定前不要用真感知跑策略。
    mount = np.asarray(
        args.lidar_mount if args.lidar_mount is not None
        else perc.get("mount_body", (0.28945, 0.0, -0.046825)), dtype=np.float64)
    lidar_rpy = np.asarray(
        args.lidar_rpy if args.lidar_rpy is not None
        else perc.get("mount_rpy_body", (0.0, 0.0, 0.0)), dtype=np.float64)
    lidar_R = _rpy_to_mat(lidar_rpy)

    # 高程图：世界系 5 cm 格网，每格取最高点，decay_s 秒后过期。
    mapper = HeightMapper(
        nx=nx, ny=ny, x_range=x_range, y_range=y_range,
        base=scan_base,
        resolution=float(perc.get("map_resolution", 0.05)),
        decay_s=float(perc.get("map_decay_s", 1.0)),
        fill_local=bool(perc.get("fill_local", True)),
    )

    # ------------------------------------------------------------ 3) 建节点/订阅
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32MultiArray

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("go2_perception")

    # 回调只更新缓存；真正的计算放在 15 Hz 定时器里（避免点云回调里做重活）。
    state = {"odom": None, "points": None, "points_t": None, "seq": 0, "done_seq": -1,
             "n_scan": 0, "n_points": 0, "n_skip": 0}

    def _set_pose(position, orientation) -> None:
        state["odom"] = (
            np.array([position.x, position.y, position.z], dtype=np.float64),
            np.array([orientation.w, orientation.x, orientation.y, orientation.z],
                     dtype=np.float64),
        )

    def on_odom(msg: Odometry) -> None:
        _set_pose(msg.pose.pose.position, msg.pose.pose.orientation)

    def on_pose(msg: PoseStamped) -> None:
        _set_pose(msg.pose.position, msg.pose.orientation)

    def on_points(msg: PointCloud2) -> None:
        state["points"] = _cloud_to_xyz(msg)
        state["points_t"] = _stamp_seconds(msg.header.stamp)
        state["seq"] += 1
        state["n_points"] += int(state["points"].shape[0])

    if args.odom_type == "pose":
        node.create_subscription(PoseStamped, args.odom_topic, on_pose, 10)
    else:
        node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    # 点云用 sensor_data（BEST_EFFORT）：L1 驱动/仿真桥多按 best-effort 发。
    node.create_subscription(PointCloud2, args.points_topic, on_points,
                             qos_profile_sensor_data)
    pub = node.create_publisher(Float32MultiArray, args.scan_topic, 10)

    # ------------------------------------------------------------ 4) 15 Hz 计算/发布
    def tick() -> None:
        # 没位姿或没点云就不发布；5 s 打一次警告（真机最常见的是没有 pose）。
        if state["odom"] is None or state["points"] is None:
            state["n_skip"] += 1
            missing = "odom" if state["odom"] is None else "points"
            if state.get("last_warn_t") is None or \
                    time.monotonic() - state["last_warn_t"] > 5.0:
                state["last_warn_t"] = time.monotonic()
                node.get_logger().warn(
                    "perception 缺 %s：%s=%s pts=%s（有雷达没 pose 时 height_scan 不会发布，"
                    "控制器会回退 leg_fk）"
                    % (missing, args.odom_topic, state["odom"] is not None,
                       state["points"] is not None))
            return
        # 一帧点云只算一次（点云 11 Hz、本定时器 15 Hz）。
        if state["done_seq"] == state["seq"]:
            return
        state["done_seq"] = state["seq"]

        pos, quat = state["odom"]
        pts = state["points"]
        # (a) 雷达系 -> 机体系：p_base = R(rpy) @ p_lidar + t
        if args.points_frame == "lidar" and pts.size:
            pts = pts @ lidar_R.T + mount[None, :]
        # (b) 机体系 -> 世界系：p_world = R(base) @ p_base + base_pos
        if args.points_frame in ("body", "lidar") and pts.size:
            pts = pts @ quat_to_mat(quat).T + pos[None, :]

        t = state["points_t"]
        if t is None or t <= 0.0:
            t = node.get_clock().now().nanoseconds * 1e-9
        mapper.update(pts, t)
        scan = mapper.scan(pos, quat)

        msg = Float32MultiArray()
        msg.data = [float(v) for v in np.asarray(scan, dtype=np.float64).reshape(-1)]
        pub.publish(msg)
        state["n_scan"] += 1

    timer = node.create_timer(1.0 / max(rate_hz, 1e-3), tick)
    node.get_logger().info(
        "perception: %s(%s) + %s(%s) -> %s | %dx%d base=%.3f mount=%s rpy=%s rate=%.1f Hz"
        % (args.points_topic, args.points_frame, args.odom_topic, args.odom_type,
           args.scan_topic, nx, ny, scan_base, mount.tolist(), lidar_rpy.tolist(), rate_hz))

    # ------------------------------------------------------------ 5) 运行/退出
    try:
        from rclpy.executors import ExternalShutdownException
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("[perception] points=%d scan=%d skip=%d points_total=%d"
              % (state["seq"], state["n_scan"], state["n_skip"], state["n_points"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
