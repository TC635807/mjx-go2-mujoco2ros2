#!/usr/bin/env python3
"""真机 Go2：与仿真**同一张** ROS 2 节点图，只把数据源换成机器人自己。

三层与仿真完全一致：
    go2_perception   /utlidar/cloud + /utlidar/robot_pose -> /perception/height_scan
    go2_controller   50 Hz 策略 -> /lowcmd（500 Hz 零阶保持，带真机 CRC）
    （可选）go2_viewer  只订阅话题

真机话题（ROS 2 侧名字，rmw 会映射成 DDS 的 rt/...）：
    /lowstate              unitree_go/msg/LowState      500 Hz  机器人发
    /lowcmd                unitree_go/msg/LowCmd        500 Hz  控制器发（含 CRC）
    /utlidar/cloud         sensor_msgs/PointCloud2      L1 点云，frame=utlidar_lidar（约 11 Hz）
    /utlidar/robot_pose    geometry_msgs/PoseStamped    **很多固件上没有数据**，不能假设可用

前提（缺一不可）：
    1. PC 有线网卡 192.168.123.xxx，ping 192.168.123.161 通；
    2. 机器人处于 debug 模式（吊起来 → 遥控器 L2+R2，关节阻尼态）；
    3. ROS_DOMAIN_ID=0，网卡名正确（real_ros profile 的 interface，默认 eth0）；
    4. 先跑只读自检：
       scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

安全默认：controller=stand、scan=leg_fk（不做地形感知；等 odom/EKF 可靠后再 scan:=topic）。
用法：
    ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=zero duration:=10
    ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
    ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
    # 已确认 /utlidar/robot_pose 有数据、且外参标定过，才开真感知：
    ... controller:=policy scan:=topic perception:=true
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[2]
RUN_WITH_ROS2 = str(ROOT / "scripts" / "run_with_ros2.sh")


def _launch(context):  # noqa: ANN001
    lc = context.launch_configurations
    duration = str(lc.get("duration", "30.0"))
    target = str(lc.get("target", "real_ros"))
    policy_profile = str(lc.get("policy_profile", "go2_v33d"))
    points_frame = str(lc.get("points_frame", "lidar"))
    points_topic = str(lc.get("points_topic", "/utlidar/cloud"))
    odom_topic = str(lc.get("odom_topic", "/utlidar/robot_pose"))
    odom_type = str(lc.get("odom_type", "pose"))
    scan = str(lc.get("scan", "leg_fk"))
    controller = str(lc.get("controller", "stand"))
    perception = str(lc.get("perception", "auto")).lower()
    viewer = str(lc.get("viewer", "false")).lower() in ("1", "true", "yes", "on")

    if target == "real_sdk":
        # 本机 Python 3.14 下 unitree_sdk2py 不可用；且 --scan topic 只支持 ros2 传输。
        print("[launch] ⚠ target=real_sdk 当前不可用（unitree_sdk2py/cyclonedds 在 Python 3.14 "
              "不可用，见 docs/ros2-setup.md §8）；强制改用 real_ros + scan=leg_fk")
        target, scan, perception = "real_ros", "leg_fk", "false"

    want_perception = perception in ("1", "true", "yes", "on") or \
        (perception == "auto" and scan == "topic")

    actions = []
    if want_perception:
        perc_cmd = ["bash", RUN_WITH_ROS2, "signal_io/perception_node.py",
                    "--target", target, "--points-frame", points_frame,
                    "--points-topic", points_topic, "--odom-topic", odom_topic,
                    "--odom-type", odom_type]
        lidar_mount = str(lc.get("lidar_mount", "") or "").strip()
        lidar_rpy = str(lc.get("lidar_rpy", "") or "").strip()
        if lidar_mount:
            perc_cmd += ["--lidar-mount", *lidar_mount.replace(",", " ").split()]
        if lidar_rpy:
            perc_cmd += ["--lidar-rpy", *lidar_rpy.replace(",", " ").split()]
        actions.append(ExecuteProcess(
            cmd=perc_cmd, output="screen", emulate_tty=True, name="go2_perception",
        ))
    else:
        print("[launch] 不启动 go2_perception（scan=%s）；控制器用 leg_fk 高度扫描" % scan)

    actions.append(ExecuteProcess(
        cmd=["bash", RUN_WITH_ROS2, "ros2_controller/run_controller.py",
             "--target", target, "--controller", controller,
             "--policy-profile", policy_profile, "--scan", scan,
             "--linvel", "estimator", "--publish-debug", "--duration", duration],
        output="screen", emulate_tty=True, name="go2_controller",
        on_exit=[Shutdown(reason="controller exited")],
    ))
    if viewer:
        actions.append(ExecuteProcess(
            cmd=["bash", RUN_WITH_ROS2, "scripts/view_ros2.py",
                 "--target", target, "--scene", "flat", "--duration", duration],
            output="screen", emulate_tty=True, name="go2_viewer",
            on_exit=[Shutdown(reason="viewer exited")],
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="30.0",
                              description="运行时长（秒）；0 = 直到 Ctrl-C"),
        DeclareLaunchArgument("target", default_value="real_ros",
                              description="real_ros（推荐）；real_sdk 当前不可用，会被强制回退"),
        DeclareLaunchArgument("policy_profile", default_value="go2_v33d",
                              description="策略档；回退 go2_v29c / go2_v25b"),
        DeclareLaunchArgument("controller", default_value="stand",
                              description="首次请用 zero；确认状态链路后 stand；最后 policy"),
        DeclareLaunchArgument("scan", default_value="leg_fk",
                              description="height scan：leg_fk（真机安全默认）/ nominal / topic"),
        DeclareLaunchArgument("perception", default_value="auto",
                              description="auto = 仅 scan:=topic 时启动感知节点；也可 true/false"),
        DeclareLaunchArgument("points_topic", default_value="/utlidar/cloud",
                              description="L1 点云话题（sensor_msgs/PointCloud2）"),
        DeclareLaunchArgument("points_frame", default_value="lidar",
                              description="点云坐标系：lidar（真机 L1）/ body / world"),
        DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_pose",
                              description="里程计话题（很多固件上没有数据；可用 /utlidar/robot_odom 或外部 EKF）"),
        DeclareLaunchArgument("odom_type", default_value="pose",
                              description="pose（PoseStamped）或 odom（Odometry）"),
        DeclareLaunchArgument("lidar_mount", default_value="",
                              description="雷达安装位置 x,y,z（留空用 config.perception.mount_body）"),
        DeclareLaunchArgument("lidar_rpy", default_value="",
                              description="雷达安装姿态 r,p,y（留空用 config.perception.mount_rpy_body）"),
        DeclareLaunchArgument("viewer", default_value="false",
                              description="是否启动 go2_viewer 观察器节点"),
        OpaqueFunction(function=_launch),
    ])
