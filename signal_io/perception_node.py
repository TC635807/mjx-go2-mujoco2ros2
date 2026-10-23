#!/usr/bin/env python3
"""ROS 2 感知节点：PointCloud2 + Odometry -> 187 点 height_scan。

这是仿真 / 真机**同一份**节点：

    仿真:  go2_sim_sensors 发布 /go2/lidar/points（MuJoCo raycast）+ /go2/odom（真值）
    真机:  Unitree L1 驱动发布 /utlidar/cloud（PointCloud2，lidar 系）
           + /utlidar/robot_pose（PoseStamped，odom 系）
    下游:  go2_controller 订阅 /perception/height_scan，完全不知道数据来自哪边。

真机点云坐标系（重要）:
    /utlidar/cloud 的 frame_id 是 `utlidar_lidar`，点在 **L1 雷达系**里，
    不是机体（base）系。雷达刚性装在机头（config.perception.mount_body，
    来自 unitree_ros URDF 的 radar_joint），所以用 --points-frame lidar 时
    先加 mount 平移再按 --points-frame body 的方式转到世界系。

话题：
    in  <points-topic>         sensor_msgs/PointCloud2
        仿真 /go2/lidar/points（世界系）  真机 /utlidar/cloud（lidar 系）
    in  <odom-topic>           nav_msgs/Odometry 或 geometry_msgs/PoseStamped
        仿真 /go2/odom                    真机 /utlidar/robot_pose（PoseStamped）
    out /perception/height_scan std_msgs/Float32MultiArray

用法（在 ROS 2 环境里）：
    scripts/run_with_ros2.sh signal_io/perception_node.py --target sim_ros
    scripts/run_with_ros2.sh signal_io/perception_node.py --target real_ros \
        --points-topic /utlidar/cloud --points-frame lidar \
        --odom-topic /utlidar/robot_pose --odom-type pose
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
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _cloud_to_xyz(msg) -> np.ndarray:
    """PointCloud2 -> (N,3) float64；只读取 x/y/z，丢弃 NaN。"""
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

    官方 unitree_ros 的 Go2 radar_joint 是 rpy="0 2.8782 0"（≈165°，L1 倒装），
    所以"雷达系 → 机体系"不是简单平移；真机必须带旋转，否则整帧点云错位。
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default=None,
                    help="IO profile（决定 ROS_DOMAIN_ID / 网卡；仿真 sim_ros，真机 real_ros）")
    ap.add_argument("--points-topic", default="/go2/lidar/points")
    ap.add_argument("--odom-topic", default="/go2/odom")
    ap.add_argument("--odom-type", default="odom", choices=["odom", "pose"],
                    help="odom = nav_msgs/Odometry（仿真）；pose = geometry_msgs/PoseStamped"
                         "（真机 /utlidar/robot_pose）")
    ap.add_argument("--scan-topic", default="/perception/height_scan")
    ap.add_argument("--points-frame", default="world", choices=["world", "body", "lidar"],
                    help="world = 点云已在世界系（仿真默认）；body = 已在机体系；"
                         "lidar = L1 雷达系（真机 /utlidar/cloud），按 --lidar-mount 平移到机体系")
    ap.add_argument("--lidar-mount", type=float, nargs=3, default=None,
                    help="雷达相对 base 的安装位置 (x y z)，默认取 config.perception.mount_body")
    ap.add_argument("--lidar-rpy", type=float, nargs=3, default=None,
                    help="雷达相对 base 的安装姿态 (roll pitch yaw)，默认取 "
                         "config.perception.mount_rpy_body；官方 Go2 radar_joint 是 0 2.8782 0")
    ap.add_argument("--rate-hz", type=float, default=None,
                    help="发布 height_scan 的频率（默认取 config.perception.rate_hz）")
    args = ap.parse_args()

    cfg = load_config(args.config, target=args.target)
    apply_io_env(cfg)                       # 必须在 rclpy.init 之前设 domain / RMC

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
    # 雷达外参：p_base = R(rpy) @ p_lidar + t。外参来源有冲突（官方 unitree_ros
    # radar_joint 前装 rpy=2.8782 vs 社区 URDF 顶装 rpy=0），真机必须用点云标定确认。
    mount = np.asarray(
        args.lidar_mount if args.lidar_mount is not None
        else perc.get("mount_body", (0.28945, 0.0, -0.046825)), dtype=np.float64)
    lidar_rpy = np.asarray(
        args.lidar_rpy if args.lidar_rpy is not None
        else perc.get("mount_rpy_body", (0.0, 0.0, 0.0)), dtype=np.float64)
    lidar_R = _rpy_to_mat(lidar_rpy)

    mapper = HeightMapper(
        nx=nx, ny=ny, x_range=x_range, y_range=y_range,
        base=scan_base,
        resolution=float(perc.get("map_resolution", 0.05)),
        decay_s=float(perc.get("map_decay_s", 1.0)),
        fill_local=bool(perc.get("fill_local", True)),
    )

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
    node.create_subscription(PointCloud2, args.points_topic, on_points,
                             qos_profile_sensor_data)
    pub = node.create_publisher(Float32MultiArray, args.scan_topic, 10)

    def tick() -> None:
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
        if state["done_seq"] == state["seq"]:
            return
        state["done_seq"] = state["seq"]
        pos, quat = state["odom"]
        pts = state["points"]
        if args.points_frame == "lidar" and pts.size:
            # L1 点云在 utlidar_lidar（雷达系）：p_base = R_lidar_to_base @ p_lidar + t。
            pts = pts @ lidar_R.T + mount[None, :]
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
