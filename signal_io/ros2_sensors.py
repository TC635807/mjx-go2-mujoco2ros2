#!/usr/bin/env python3
"""仿真侧 ROS 2 传感器桥：把 MuJoCo 真值发布成与真机同名的话题。

设计目标：仿真 / 真机共用同一张 ROS 2 节点图，区别只在数据源节点：

    仿真:  go2_sim_sensors  --/go2/odom-->  go2_perception  --/perception/height_scan-->  go2_controller
                            --/go2/lidar/points-->

    真机:  Unitree 状态估计  --/go2/odom-->  go2_perception  --/perception/height_scan-->  go2_controller
           Unitree L1 雷达   --/go2/lidar/points-->

本模块只在仿真侧使用；真机侧由硬件桥 / 雷达驱动发布同名话题即可，下游节点一行不改。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from .perception import SimLidar


def _stamp(t: float):
    """秒 -> builtin_interfaces/Time（仿真时间，不依赖 ROS /clock）。"""
    from builtin_interfaces.msg import Time

    sec = int(np.floor(float(t)))
    nanosec = int(round((float(t) - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


class SimSensorPublisher:
    """把 Go2Sim 的状态发布成 /go2/odom 与 /go2/lidar/points。

    只读使用 sim.model / sim.data；不推进物理、不修改数据。
    """

    def __init__(self, sim, cfg: Dict[str, Any],
                 odom_topic: str = "/go2/odom",
                 points_topic: str = "/go2/lidar/points",
                 odom_hz: float = 50.0,
                 sensor_hz: Optional[float] = None,
                 node_name: str = "go2_sim_sensors") -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node(node_name)
        self.odom_topic = odom_topic
        self.points_topic = points_topic
        self.odom_hz = max(float(odom_hz), 1e-3)
        perc = dict(cfg.get("perception") or {})
        if sensor_hz is None:
            sensor_hz = float(perc.get("rate_hz", 15.0) or 0.0)
        self.sensor_hz = max(float(sensor_hz), 0.0)

        self.odom_pub = self.node.create_publisher(Odometry, odom_topic, 10)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, points_topic, qos_profile_sensor_data)

        self.lidar = SimLidar(
            sim,
            n_rays=int(perc.get("n_rays", 900)),
            hfov=tuple(perc.get("hfov", (0.0, 2.0 * np.pi))),
            vfov=tuple(perc.get("vfov", (-1.0472, 0.5236))),
            max_range=float(perc.get("max_range", 6.0)),
            height_above_ground=float(perc.get("height_above_ground", 0.30)),
            nominal_base_height=float(perc.get("nominal_base_height",
                                               cfg.get("scan_base", 0.268))),
            mount_body=perc.get("mount_body"),
            min_range=float(perc.get("min_range", 0.10)),
            self_occlusion=bool(perc.get("self_occlusion", True)),
            seed=int(perc.get("seed", 0)),
            multiray=bool(perc.get("use_multiray", True)),
        )
        self._next_odom = -1.0e18
        self._next_cloud = -1.0e18
        self.n_odom = 0
        self.n_cloud = 0
        self.n_points = 0
        self.node.get_logger().info(
            "sim sensors: %s (%s) + %s (%.1f Hz) -> %s"
            % (odom_topic, "Go2Sim", points_topic, self.sensor_hz,
               "%.1f Hz" % self.odom_hz))

    # ------------------------------------------------------------ 发布
    def _publish_odom(self, sim, t: float) -> None:
        from nav_msgs.msg import Odometry

        msg = Odometry()
        msg.header.stamp = _stamp(t)
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        q = np.asarray(sim.data.qpos[3:7], dtype=np.float64).reshape(4)  # wxyz
        msg.pose.pose.position.x = float(sim.data.qpos[0])
        msg.pose.pose.position.y = float(sim.data.qpos[1])
        msg.pose.pose.position.z = float(sim.data.qpos[2])
        # ROS 四元数是 x,y,z,w；MuJoCo 是 w,x,y,z
        msg.pose.pose.orientation.x = float(q[1])
        msg.pose.pose.orientation.y = float(q[2])
        msg.pose.pose.orientation.z = float(q[3])
        msg.pose.pose.orientation.w = float(q[0])
        v = np.asarray(sim.data.qvel[0:3], dtype=np.float64)
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = float(v[2])
        self.odom_pub.publish(msg)
        self.n_odom += 1

    def _publish_cloud(self, sim, t: float) -> None:
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

        pts = self.lidar.sample()
        msg = PointCloud2()
        msg.header.stamp = _stamp(t)
        msg.header.frame_id = "world"
        cloud = create_cloud_xyz32(msg.header, np.asarray(pts, dtype=np.float32))
        self.cloud_pub.publish(cloud)
        self.n_cloud += 1
        self.n_points += int(len(pts))

    def publish(self, sim) -> None:
        """每个 500 Hz 物理拍之后调用；内部按频率分频。"""
        t = float(sim.time)
        if t + 1e-9 >= self._next_odom:
            self._publish_odom(sim, t)
            self._next_odom = t + 1.0 / self.odom_hz
        if self.sensor_hz > 0.0 and t + 1e-9 >= self._next_cloud:
            self._publish_cloud(sim, t)
            self._next_cloud = t + 1.0 / self.sensor_hz

    def stats(self) -> Dict[str, Any]:
        return {"odom_topic": self.odom_topic, "points_topic": self.points_topic,
                "n_odom": self.n_odom, "n_cloud": self.n_cloud, "n_points": self.n_points,
                "sensor_hz": self.sensor_hz, "odom_hz": self.odom_hz,
                "lidar": self.lidar.stats()}

    def close(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass


__all__ = ["SimSensorPublisher"]
