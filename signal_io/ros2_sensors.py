#!/usr/bin/env python3
"""仿真侧 ROS 2 传感器桥：把 MuJoCo 真值发布成下游认识的话题。

===============================================================================
1. 它在节点图里的位置
===============================================================================

  仿真:
      SimNode（MuJoCo 500 Hz） --每拍--> SimSensorPublisher.publish(sim)
          -> /lowstate（由 transport 发，500 Hz，与真机同名）
          -> /go2/odom（50 Hz，真值：位置/姿态/线速度）
          -> /go2/lidar/points（15 Hz，raycast 点云，世界系）
                    │
                    ▼
      go2_perception（/go2/lidar/points + /go2/odom -> /perception/height_scan）

  真机时没有这个类：机器人固件直接发 /lowstate；L1 驱动发 /utlidar/cloud；
  位姿可能来自 LIO/EKF（也可能是空话题）。下游 go2_perception 一行不改。

===============================================================================
2. 为什么要单独发布 odom / 点云
===============================================================================

  * /lowstate 只含本体传感器（关节/IMU/足端力），没有世界位置和点云；
  * 高度扫描需要 world 系点云 + base 位姿；
  * 真机上 L1 是独立驱动、位姿是独立状态估计，所以仿真也拆成独立话题，
    这样"换数据源不换下游代码"才成立。

===============================================================================
3. 话题/QoS/坐标系
===============================================================================

  /go2/odom            nav_msgs/Odometry，frame_id=world，child=base_link，
                       twist 为世界系线速度（观察器 --truth 用它对照）
  /go2/lidar/points    sensor_msgs/PointCloud2，frame_id=world，xyz float32，
                       BEST_EFFORT（与真机 L1 一致）

  SimLidar 的射线几何（FOV/量程/安装位置/自遮挡）来自 config.perception；
  它只读 sim.model/sim.data，不推进物理、不修改状态。

===============================================================================
4. 频率
===============================================================================

  物理 500 Hz；本类内部按时间分频：
      odom  50 Hz（config 里可调 odom_hz）
      点云  15 Hz（config.perception.rate_hz；真机 L1 约 11 Hz）
  publish() 每个物理拍调用一次（由 sim_env/run_sim.py 驱动）。

相关文件：
  sim_env/run_sim.py                --publish-sensors 时创建本类
  signal_io/perception.py           SimLidar（raycast）
  signal_io/perception_node.py      下游消费者
  ros2_ws/launch/sim.launch.py      三节点图的启动
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from .perception import SimLidar


def _stamp(t: float):
    """秒 -> builtin_interfaces/Time（用仿真时间，不依赖 ROS /clock）。"""
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
    仿真专用；真机由固件/L1 驱动/状态估计发同名语义的话题。
    """

    def __init__(self, sim, cfg: Dict[str, Any],
                 odom_topic: str = "/go2/odom",
                 points_topic: str = "/go2/lidar/points",
                 odom_hz: float = 50.0,
                 sensor_hz: Optional[float] = None,
                 node_name: str = "go2_sim_sensors") -> None:
        """建 ROS 2 发布者并配置 SimLidar。

        Args:
            sim: Go2Sim 实例（只读）。
            cfg: load_config() 的产物（读 perception 的射线参数）。
            odom_topic/points_topic: 话题名（下游 perception_node 默认值一致）。
            odom_hz: 里程计发布频率（默认 50 Hz）。
            sensor_hz: 点云频率；None 时取 config.perception.rate_hz（默认 15 Hz）。
            node_name: rclpy 节点名（与真机桥区分）。
        """
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

        # odom 用默认 QoS（可靠）；点云用 sensor_data（BEST_EFFORT），与真机 L1 一致。
        self.odom_pub = self.node.create_publisher(Odometry, odom_topic, 10)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, points_topic, qos_profile_sensor_data)

        # 射线几何来自 config.perception；mount_body 是雷达相对 base 的安装位置
        # （仿真只用它当射线原点，不含姿态旋转；真机外参见 perception_node.py）。
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
        # 分频用"下一次允许发布的时间"（仿真时间），避免累计误差。
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
        """发布一帧真值 Odometry（位置/姿态/世界系线速度）。"""
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
        """发布一帧 raycast 点云（世界系，xyz float32）。"""
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
        """每个 500 Hz 物理拍之后调用；内部按 odom_hz/sensor_hz 分频。"""
        t = float(sim.time)
        if t + 1e-9 >= self._next_odom:
            self._publish_odom(sim, t)
            self._next_odom = t + 1.0 / self.odom_hz
        if self.sensor_hz > 0.0 and t + 1e-9 >= self._next_cloud:
            self._publish_cloud(sim, t)
            self._next_cloud = t + 1.0 / self.sensor_hz

    def stats(self) -> Dict[str, Any]:
        """给 run_sim 的 summary 用：发布条数、频率、雷达统计。"""
        return {"odom_topic": self.odom_topic, "points_topic": self.points_topic,
                "n_odom": self.n_odom, "n_cloud": self.n_cloud, "n_points": self.n_points,
                "sensor_hz": self.sensor_hz, "odom_hz": self.odom_hz,
                "lidar": self.lidar.stats()}

    def close(self) -> None:
        """销毁 rclpy 节点（不 shutdown rclpy）。"""
        try:
            self.node.destroy_node()
        except Exception:
            pass


__all__ = ["SimSensorPublisher"]
