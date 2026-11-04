#!/usr/bin/env python3
"""仿真 launch：一张真正的 ROS 2 节点图，数据源是 MuJoCo。

===============================================================================
1. 启动了什么（与真机同构的三节点图）
===============================================================================

    go2_sim_sensors（sim_env/run_sim.py --publish-sensors，domain 1）
        MuJoCo 物理 500 Hz
        发布 /lowstate（unitree_go/LowState，500 Hz）    ← 与真机同一个话题名
             /go2/odom（nav_msgs/Odometry，50 Hz 真值）
             /go2/lidar/points（sensor_msgs/PointCloud2，15 Hz，raycast 世界系）
        订阅 /lowcmd（控制器指令）

    go2_perception（signal_io/perception_node.py）
        /go2/lidar/points + /go2/odom -> /perception/height_scan（187 点）

    go2_controller（ros2_controller/run_controller.py）
        /lowstate + /perception/height_scan -> /lowcmd（50 Hz 策略 + 500 Hz 下发）
        发布 /go2/controller/{obs,action,info} 给观察器（--publish-debug）

    （可选）go2_viewer（scripts/view_ros2.py）：只订阅话题，不参与控制。

  真机（real.launch.py）启动的是同样的 perception + controller；
  区别只是数据源换成机器人固件 DDS（/lowstate、/lowcmd、/utlidar/cloud）。

===============================================================================
2. 为什么用 3 个进程而不是单进程
===============================================================================

  单进程（scripts/view_go2_policy.sh --local）适合快速看效果；
  多进程是"真机同构"验证：控制器真的通过 ROS 2 话题收状态/发指令，
  这样切到 real_ros 时不会因为"单进程直连"掩盖通信/时序问题。

===============================================================================
3. 常用参数
===============================================================================

  duration      运行时长（秒）；到点后 sim 退出 -> launch 整体退出
  profile       信号劣化档位：sim_clean / real_mild / real_severe
                （在"发出去的状态"上注入延迟/丢包/噪声/偏置，压测健康层）
  controller    policy / stand / zero / lqr_stub
  terrain       地形名：flat / wide_h050 / hyb2 / rubble ...（见 terrains.json）
  policy_profile 策略档：go2_v33d（默认）/ go2_v29c / go2_v25b ...
  cmd           速度指令字符串，例如 "0.5 0 0"
  viewer        是否启动观察器（额外一个 ROS 2 节点）

用法（先 source ROS 2 与 ros2_ws/install/setup.bash）：
    ros2 launch ros2_ws/launch/sim.launch.py
    ros2 launch ros2_ws/launch/sim.launch.py duration:=20 viewer:=true
    ros2 launch ros2_ws/launch/sim.launch.py terrain:=wide_h050 policy_profile:=go2_v33d
    ros2 launch ros2_ws/launch/sim.launch.py controller:=zero   # 只看状态链路

注意：
  * 仿真固定 domain 1（真机是 0），避免和真机话题串台；
  * run_with_ros2.sh 负责 source ROS 2 / unitree_go / venv；
  * viewer 的 --truth 只在仿真有效（真机没有 qvel 真值）。
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[2]
RUN_WITH_ROS2 = str(ROOT / "scripts" / "run_with_ros2.sh")


def _terrain_scene(name: str):
    """地形名 -> 场景 XML 绝对路径（flat 返回 None，表示用默认 scene.xml）。

    地形清单在 sim_env/models/go2/terrains.json；每个地形有独立 scene.xml +
    hfield/视觉网格 + boxgrid 高度表（由训练仓导出）。
    """
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
    """按 launch 参数拼出 3~4 个 ExecuteProcess（真正启动 ROS 2 节点）。"""
    lc = context.launch_configurations
    duration = str(lc.get("duration", "30.0"))
    profile = str(lc.get("profile", "sim_clean"))
    controller = str(lc.get("controller", "policy"))
    viewer = str(lc.get("viewer", "false")).lower() in ("1", "true", "yes", "on")
    terrain = str(lc.get("terrain", "flat"))
    policy_profile = str(lc.get("policy_profile", "go2_v33d"))
    cmd = str(lc.get("cmd", "")).strip()

    # ---- 节点 1：MuJoCo 数据源（SimNode，500 Hz）----
    # --publish-sensors 打开 ROS 2 发布：/lowstate（500 Hz，与真机同名）+
    #   /go2/odom（50 Hz 真值）+ /go2/lidar/points（15 Hz raycast，世界系）。
    sim_cmd = ["bash", RUN_WITH_ROS2, "sim_env/run_sim.py",
               "--target", "sim_ros", "--publish-sensors",
               "--duration", duration, "--profile", profile, "--terrain", terrain]

    # ---- 节点 3：控制器（50 Hz 策略 + 500 Hz 下发）----
    # --scan topic 表示高度扫描从 /perception/height_scan 拿；没数据会自动回退 leg_fk。
    # --publish-debug 打开 /go2/controller/{obs,action,info}（观察器用）。
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
            # sim 退出（duration 到点 / 崩溃）就整体退出，避免控制器空转。
            on_exit=[Shutdown(reason="sim node exited")],
        ),
        # ---- 节点 2：感知（点云 + 位姿 -> 187 点高度扫描）----
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
        # ---- 可选节点 4：观察器（只订阅，不接触仿真/控制器内部对象）----
        # --truth：仿真里用 /go2/odom 的 twist 当真值对照（真机不要开）。
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
