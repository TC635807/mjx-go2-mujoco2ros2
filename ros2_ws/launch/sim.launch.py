#!/usr/bin/env python3
"""仿真：一张 ROS 2 节点图，数据源是 MuJoCo。

启动 3 个真正的 ROS 2 节点：

    go2_sim_sensors    MuJoCo 500 Hz -> /lowstate, /go2/odom, /go2/lidar/points
    go2_perception     PointCloud2 + Odom -> /perception/height_scan
    go2_controller     50 Hz 策略 -> /lowcmd

用法（先 source ROS 2 与 ros2_ws/install/setup.bash）：
    ros2 launch ros2_ws/launch/sim.launch.py
    ros2 launch ros2_ws/launch/sim.launch.py duration:=20 viewer:=true
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[2]
RUN_WITH_ROS2 = str(ROOT / "scripts" / "run_with_ros2.sh")


def _terrain_scene(name: str):
    """地形名 -> 场景 XML 绝对路径（flat 返回 None）。"""
    if not name or name == "flat":
        return None
    import json
    manifest = ROOT / "sim_env" / "models" / "go2" / "terrains.json"
    if not manifest.is_file():
        return None
    data = json.loads(manifest.read_text(encoding="utf-8"))
    for item in data.get("terrains", []):
        if item.get("name") == name:
            scene = Path(item["scene"])
            return str(scene if scene.is_absolute() else (ROOT / scene))
    return None


def _launch(context):  # noqa: ANN001
    lc = context.launch_configurations
    duration = str(lc.get("duration", "30.0"))
    profile = str(lc.get("profile", "sim_clean"))
    controller = str(lc.get("controller", "policy"))
    viewer = str(lc.get("viewer", "false")).lower() in ("1", "true", "yes", "on")
    terrain = str(lc.get("terrain", "flat"))
    policy_profile = str(lc.get("policy_profile", "go2_v33d"))
    cmd = str(lc.get("cmd", "")).strip()

    sim_cmd = ["bash", RUN_WITH_ROS2, "sim_env/run_sim.py",
               "--target", "sim_ros", "--publish-sensors",
               "--duration", duration, "--profile", profile, "--terrain", terrain]

    ctrl_cmd = ["bash", RUN_WITH_ROS2, "ros2_controller/run_controller.py",
                "--target", "sim_ros", "--controller", controller,
                "--policy-profile", policy_profile,
                "--scan", "topic", "--publish-debug", "--duration", duration]
    if cmd:
        ctrl_cmd += ["--cmd"] + cmd.split()

    actions = [
        ExecuteProcess(
            cmd=sim_cmd, output="screen", emulate_tty=True,
            name="go2_sim",
            on_exit=[Shutdown(reason="sim node exited")],
        ),
        ExecuteProcess(
            cmd=["bash", RUN_WITH_ROS2, "signal_io/perception_node.py",
                 "--target", "sim_ros"],
            output="screen", emulate_tty=True, name="go2_perception",
        ),
        ExecuteProcess(
            cmd=ctrl_cmd, output="screen", emulate_tty=True, name="go2_controller",
        ),
    ]
    if viewer:
        viewer_cmd = ["bash", RUN_WITH_ROS2, "scripts/view_ros2.py",
                      "--target", "sim_ros", "--truth", "--duration", duration]
        model = _terrain_scene(terrain)
        if model:
            viewer_cmd += ["--model", model]
        actions.append(ExecuteProcess(
            cmd=viewer_cmd, output="screen", emulate_tty=True, name="go2_viewer",
            on_exit=[Shutdown(reason="viewer exited")],
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="30.0",
                              description="仿真/控制时长（秒）"),
        DeclareLaunchArgument("profile", default_value="sim_clean",
                              description="信号劣化档位 sim_clean / real_mild / real_severe"),
        DeclareLaunchArgument("controller", default_value="policy",
                              description="policy / stand / zero / lqr_stub"),
        DeclareLaunchArgument("viewer", default_value="false",
                              description="是否启动 go2_viewer 观察器节点"),
        DeclareLaunchArgument("terrain", default_value="flat",
                              description="地形名（flat / wide_h050 / hyb2 / rubble ...）"),
        DeclareLaunchArgument("policy_profile", default_value="go2_v33d",
                              description="策略档（go2_v33d / go2_v29c / go2_v25b ...）"),
        DeclareLaunchArgument("cmd", default_value="",
                              description="速度指令字符串，例如 \"0.5 0 0\""),
        OpaqueFunction(function=_launch),
    ])
