#!/usr/bin/env python3
"""真机 Go2 launch：与仿真同一张 ROS 2 节点图，只把数据源换成机器人固件。

===============================================================================
1. 启动了什么（与 sim.launch.py 对照看）
===============================================================================

  真机（本文件）:
    go2_perception（可选）  /utlidar/cloud + 位姿 -> /perception/height_scan
    go2_controller         /lowstate + /perception/height_scan -> /lowcmd
    go2_viewer（可选）     只订阅话题，不参与控制

  机器人固件本身就是一个 ROS 2/DDS 数据源，所以**不需要自己写 lowstate/lowcmd 桥**：
      ROS 2 话题 /lowstate  <--rmw-->  DDS rt/lowstate  <-- 机器人固件
      ROS 2 话题 /lowcmd    --rmw-->  DDS rt/lowcmd    --> 机器人固件
  只有 L1 点云和位姿需要按机器人实际话题配置（下面有默认值）。

===============================================================================
2. 真机话题（ROS 2 侧名字；rmw_cyclonedds_cpp 会映射成 DDS 的 rt/...）
===============================================================================

  /lowstate              unitree_go/msg/LowState      500 Hz   机器人 -> PC
  /lowcmd                unitree_go/msg/LowCmd        500 Hz   PC -> 机器人（含 CRC）
  /utlidar/cloud         sensor_msgs/PointCloud2      L1 点云，frame=utlidar_lidar，约 11 Hz
  /utlidar/robot_pose    geometry_msgs/PoseStamped    **很多固件上没有数据**
  /utlidar/robot_odom    nav_msgs/Odometry            **很多固件上没有数据（厂商建议接 LIO）**

  WARN: 不要以为真机有现成 odom。释放 sport mode 后机器人自带 SportModeState 也不可读，
        所以真机默认 scan=leg_fk（平地近似），等接上 LIO/EKF 再 scan=topic。

===============================================================================
3. 上机前（缺一不可）
===============================================================================

  1) PC 有线网卡 192.168.123.xxx/24，ping 192.168.123.161 通；
  2) 机器人吊起来，遥控器 L2+R2 进 debug mode（关节阻尼态，释放 sport mode）；
  3) ROS_DOMAIN_ID=0，网卡名正确（real_ros profile 的 interface，默认 eth0）；
  4) 先跑只读自检（不发任何指令）：
       scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

===============================================================================
4. 启动顺序（安全默认）
===============================================================================

  本 launch 默认 controller=stand、scan=leg_fk —— 不做地形感知、只走站姿 PD。
  真机首次上机请手动按顺序：
      zero  （吊起，确认 /lowstate 和电机伺服）
      stand （仍吊起/空旷，看关节平滑到站姿）
      policy（落地，先 --cmd 0.2 0 0）

  用法：
      ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=zero duration:=10
      ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
      ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
      # 已确认位姿有数据 + L1 外参标定过，才开真感知：
      ... controller:=policy scan:=topic perception:=true lidar_rpy:="0 2.8782 0"

  退出：controller 退出 -> 整体退出；controller 在真机上会先发 100 ms 阻尼再停。
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[2]
RUN_WITH_ROS2 = str(ROOT / "scripts" / "run_with_ros2.sh")


def _launch(context):  # noqa: ANN001
    """按参数拼装真机节点（controller 必须起；perception/viewer 可选）。"""
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
        # 显式警告 + 回退，避免用户以为在跑 SDK 路径。
        print("[launch] ⚠ target=real_sdk 当前不可用（unitree_sdk2py/cyclonedds 在 Python 3.14 "
              "不可用，见 docs/ros2-setup.md §8）；强制改用 real_ros + scan=leg_fk")
        target, scan, perception = "real_ros", "leg_fk", "false"

    # perception 默认 auto：只有 scan=topic 时才启动（否则它收不到位姿也只会空转）。
    want_perception = perception in ("1", "true", "yes", "on") or \
        (perception == "auto" and scan == "topic")

    actions = []
    if want_perception:
        # ---- 可选节点 1：感知（L1 点云 -> 187 点 height_scan）----
        # 需要真机有 /utlidar/cloud 且有可用位姿；外参用 --lidar-mount/--lidar-rpy 覆盖。
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

    # ---- 节点 2（必须）：控制器（50 Hz 策略 + 500 Hz 下发 + CRC + 安全层）----
    # --linvel estimator：真机没有可信的仿真真值，只能走腿式里程计。
    # --publish-debug：给观察器提供 obs/action/info。
    actions.append(ExecuteProcess(
        cmd=["bash", RUN_WITH_ROS2, "ros2_controller/run_controller.py",
             "--target", target, "--controller", controller,
             "--policy-profile", policy_profile, "--scan", scan,
             "--linvel", "estimator", "--publish-debug", "--duration", duration],
        output="screen", emulate_tty=True, name="go2_controller",
        # controller 退出（duration 到点 / Ctrl-C / 异常）就整体退出；
        # 它在 finally 里会先发 100 ms 阻尼，然后才真正退出。
        on_exit=[Shutdown(reason="controller exited")],
    ))
    if viewer:
        # ---- 可选节点 3：观察器（只订阅话题，真机不要开 --truth）----
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
                              description="位姿话题（很多固件没有数据；可换 /utlidar/robot_odom 或外部 EKF）"),
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
