#!/usr/bin/env python3
"""ROS 2 观察器节点：只订阅话题，不自己拉起仿真/控制器。

这是 view_policy.py 的 ROS 2 版本，用于观察“真实节点图”：

    /lowstate              unitree_go/LowState      500 Hz
    /lowcmd                unitree_go/LowCmd        500 Hz
    /go2/odom                 nav_msgs/Odometry        > 50 Hz
    /go2/controller/obs       std_msgs/Float32MultiArray 243 维（--publish-debug）
    /go2/controller/action    std_msgs/Float32MultiArray 12 维
    /go2/controller/info      std_msgs/String JSON     健康 / 模式 / 来源
    /perception/height_scan   std_msgs/Float32MultiArray 187 点

用法：
    scripts/run_with_ros2.sh scripts/view_ros2.py --target sim_ros
    scripts/run_with_ros2.sh scripts/view_ros2.py --target real_ros --headless
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from signal_io.config import apply_io_env, load_config, model_xml  # noqa: E402
from signal_io.perception import scan_grid_body  # noqa: E402
from sim_env.robot import MOTOR_JOINTS  # noqa: E402
from sim_env.viz import SceneOverlay, VizFrame  # noqa: E402
from sim_env.viz.record import Recorder, TerminalDashboard  # noqa: E402


def quat_to_rpy(q_wxyz) -> np.ndarray:
    q = np.asarray(q_wxyz, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.zeros(3)
    w, x, y, z = q / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _motor_arrays(msg):
    q = np.zeros(12, dtype=np.float64)
    dq = np.zeros(12, dtype=np.float64)
    tau = np.zeros(12, dtype=np.float64)
    for i in range(12):
        m = msg.motor_state[i]
        q[i] = float(m.q)
        dq[i] = float(m.dq)
        tau[i] = float(getattr(m, "tau_est", 0.0) or 0.0)
    ff = np.array([float(x) for x in list(getattr(msg, "foot_force", [0, 0, 0, 0]))[:4]],
                   dtype=np.float64)
    imu = msg.imu_state
    quat = np.asarray(imu.quaternion, dtype=np.float64).reshape(4)
    return q, dq, tau, ff, quat, int(getattr(msg, "tick", 0) or 0)


def _cmd_q(msg):
    return np.array([float(msg.motor_cmd[i].q) for i in range(12)], dtype=np.float64)


def _pad(vec, n: int) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.size == n:
        return arr
    out = np.zeros(n, dtype=np.float64)
    k = min(n, arr.size)
    out[:k] = arr[:k]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default="sim_ros",
                    help="IO profile：sim_ros / real_ros（只用于 domain / 网卡）")
    ap.add_argument("--model", default=None, help="可视化 MJCF；默认取 config 的 scene.xml")
    ap.add_argument("--scene", default="default", choices=["default", "flat"],
                    help="flat = scene_flat.xml（真机/平地）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="运行时长（墙钟秒）；0 = 直到 Ctrl-C / 关窗口")
    ap.add_argument("--headless", action="store_true", help="不开窗口，只出终端仪表盘/CSV")
    ap.add_argument("--plots", default="window", choices=["window", "off"],
                    help="独立 matplotlib 曲线窗口（需要 Qt）")
    ap.add_argument("--sync-hz", type=float, default=60.0, help="3D 画面刷新率")
    ap.add_argument("--plot-hz", type=float, default=3.0, help="曲线窗口刷新率")
    ap.add_argument("--follow", action="store_true", help="相机跟随躯干")
    ap.add_argument("--trail", type=int, default=120, help="轨迹拖尾点数")
    ap.add_argument("--robot-style", default="full", choices=["full", "lite"],
                    help="full=高模网格；lite=只显示碰撞几何（软件渲染提帧率）")
    ap.add_argument("--lite", default="on", choices=["on", "off"],
                    help="关掉阴影/反射等昂贵视觉效果（软件渲染建议 on）")
    ap.add_argument("--truth", action="store_true",
                    help="仿真模式：把 /go2/odom 的 twist 当成速度真值（真机不要开）")
    ap.add_argument("--record", default=None, help="落盘目录：CSV + summary.png + report.md")
    ap.add_argument("--record-every", type=float, default=10.0, help="关键帧记录频率")
    ap.add_argument("--state-topic", default="/lowstate")
    ap.add_argument("--cmd-topic", default="/lowcmd")
    ap.add_argument("--odom-topic", default="/go2/odom")
    ap.add_argument("--scan-topic", default="/perception/height_scan")
    ap.add_argument("--obs-topic", default="/go2/controller/obs")
    ap.add_argument("--action-topic", default="/go2/controller/action")
    ap.add_argument("--info-topic", default="/go2/controller/info")
    args = ap.parse_args()

    cfg = load_config(args.config, target=args.target)
    apply_io_env(cfg)

    # 观测几何（与训练/控制器同一份契约）
    p = dict(cfg.get("policy") or {})
    sc = dict(p.get("scan") or {})
    scan_nx = int(sc.get("nx", 17))
    scan_ny = int(sc.get("ny", 11))
    scan_x = tuple(sc.get("x", (-0.4, 1.2)))
    scan_y = tuple(sc.get("y", (-0.5, 0.5)))
    scan_base = float(cfg.get("scan_base", 0.268))
    scan_scale = 5.0
    grid = scan_grid_body(scan_nx, scan_ny, scan_x, scan_y)
    obs_dim = 243

    # 可视化模型（只做运动学/渲染，不步进物理）
    xml = args.model or model_xml(cfg)
    if args.scene == "flat" and args.model is None:
        cand = os.path.join(os.path.dirname(xml), "scene_flat.xml")
        if os.path.isfile(cand):
            xml = cand
    if args.model is None and args.target.startswith("real_"):
        cand = os.path.join(os.path.dirname(xml), "scene_flat.xml")
        if os.path.isfile(cand):
            xml = cand

    import mujoco

    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    if args.lite == "on":
        q = model.vis.quality
        for name, val in (("shadowsize", 256), ("offsamples", 1), ("reflections", 0)):
            if hasattr(q, name):
                setattr(q, name, val)
        g = model.vis.global_
        for name, val in (("shadow", 0), ("reflections", 0)):
            if hasattr(g, name):
                setattr(g, name, val)
    qadr = np.array([model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in MOTOR_JOINTS],
        dtype=int)
    foot_geoms = []
    for leg in ("FR", "FL", "RR", "RL"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, leg + "_calf")
        if bid < 0:
            continue
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == bid and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                foot_geoms.append(g)
                break

    # ---- ROS 2 订阅 ----
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import LowCmd, LowState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray, String

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("go2_viewer")
    st = {"lowstate": None, "lowcmd": None, "odom": None, "scan": None,
          "obs": None, "action": None, "info": {},
          "n_state": 0, "n_cmd": 0, "n_odom": 0, "n_scan": 0, "n_obs": 0, "n_action": 0}

    def on_state(msg):
        st["lowstate"] = _motor_arrays(msg)
        st["n_state"] += 1

    def on_cmd(msg):
        st["lowcmd"] = _cmd_q(msg)
        st["n_cmd"] += 1

    def on_odom(msg):
        p0 = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        st["odom"] = (np.array([p0.x, p0.y, p0.z], dtype=np.float64),
                      np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
                      np.array([v.x, v.y, v.z], dtype=np.float64))
        st["n_odom"] += 1

    def on_scan(msg):
        st["scan"] = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        st["n_scan"] += 1

    def on_obs(msg):
        st["obs"] = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        st["n_obs"] += 1

    def on_action(msg):
        st["action"] = np.asarray(msg.data, dtype=np.float64).reshape(-1)
        st["n_action"] += 1

    def on_info(msg):
        try:
            st["info"] = json.loads(msg.data)
        except Exception:
            pass

    node.create_subscription(LowState, args.state_topic, on_state, 10)
    node.create_subscription(LowCmd, args.cmd_topic, on_cmd, 10)
    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    node.create_subscription(Float32MultiArray, args.scan_topic, on_scan, 10)
    node.create_subscription(Float32MultiArray, args.obs_topic, on_obs, 10)
    node.create_subscription(Float32MultiArray, args.action_topic, on_action, 10)
    node.create_subscription(String, args.info_topic, on_info, 10)

    # 订阅回调必须独立于渲染线程：软件渲染下单帧可能 100~300 ms，
    # 如果只在渲染间隙 spin，观察器看到的状态会严重滞后。
    def _spin_loop() -> None:
        try:
            from rclpy.executors import ExternalShutdownException
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:  # pragma: no cover - 进程收尾
            print("[viewer] spin 退出:", repr(exc)[:120])

    spinner = threading.Thread(target=_spin_loop, daemon=True)
    spinner.start()

    # ---- 观察组件 ----
    live = None
    if not args.headless and args.plots == "window":
        try:
            from sim_env.viz.live import LivePlots, available as live_available
            if live_available():
                live = LivePlots(scan_nx=scan_nx, scan_ny=scan_ny,
                                 title="go2_viewer  ROS2  %s" % args.target)
            else:
                print("[viewer] 没有 Qt 后端，跳过独立曲线窗口")
        except Exception as exc:
            print("[viewer] 曲线窗口失败:", repr(exc)[:120])

    overlay = SceneOverlay(model, data, cfg={
        "geom_budget": max(160, scan_nx * scan_ny + 60),
        "trail_n": int(args.trail),
        "camera_mode": "follow",   # 观察器默认跟随躯干，保证机器人始终可见
        "vel_scale": 0.4, "arrow_width": 0.008,
    })
    hud = None
    try:
        from sim_env.viz.panels import TextHud
        hud = TextHud()
    except Exception:
        pass
    rec = Recorder(args.record, fps=args.record_every) if args.record else None
    dash = TerminalDashboard() if args.headless else None

    viewer = None
    if not args.headless:
        try:
            import mujoco.viewer
            try:
                viewer = mujoco.viewer.launch_passive(
                    model, data, show_left_ui=False, show_right_ui=False)
            except TypeError:
                viewer = mujoco.viewer.launch_passive(model, data)
            try:
                viewer.opt.geomgroup[3] = 0   # 机器人碰撞几何默认隐藏
                viewer.opt.geomgroup[4] = 0   # 地形碰撞瓦片默认隐藏
                if args.robot_style == "lite":
                    viewer.opt.geomgroup[2] = 0
                    viewer.opt.geomgroup[3] = 1
            except Exception:
                pass
        except Exception as exc:
            print("[viewer] 3D 窗口开不了（%s），退化为终端仪表盘" % repr(exc)[:120])
            dash = TerminalDashboard()
    if viewer is None:
        args.headless = True

    t0 = time.monotonic()
    last_render = -1.0e18
    last_plot = -1.0e18
    last_dash = -1.0e18
    prev_t = None
    prev_wall = None
    frames = 0
    summary = {}

    def build_frame(wall: float) -> VizFrame:
        f = VizFrame()
        f.wall = float(wall)
        f.t = float(wall)
        ls = st["lowstate"]
        od = st["odom"]
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        if ls is not None:
            q, dq, tau, ff, imu_q, tick = ls
            f.joint_q = q
            f.joint_dq = dq
            f.joint_tau = tau
            f.foot_force = ff
            quat = imu_q
            if tick > 0:
                f.t = float(tick) * 0.002
        if od is not None:
            f.base_pos = od[0]
            quat = od[1]
            if args.truth:
                f.base_lin_vel_truth = od[2]
            f.base_lin_vel_world = od[2]
        f.base_quat = np.asarray(quat, dtype=np.float64).reshape(4)
        f.rpy = quat_to_rpy(f.base_quat)
        if st["lowcmd"] is not None:
            f.joint_q_des = _pad(st["lowcmd"], 12)

        info = st["info"] or {}
        if info.get("base_lin_vel_world") is not None:
            f.base_lin_vel_world = np.asarray(info["base_lin_vel_world"], dtype=np.float64)[:3]
        if info.get("level_name") is not None:
            f.level = int(info.get("level", 0))
            f.level_name = str(info.get("level_name", "L0"))
        if info.get("mode") is not None:
            f.mode = str(info.get("mode", "track"))
        f.obs_valid = bool(info.get("obs_valid", True))
        f.sources = {
            "linvel": str(info.get("linvel_source", "-")),
            "scan": str(info.get("height_scan_source", "-")),
        }
        f.rtf = float(info.get("rtf", 0.0) or 0.0)
        f.n_state = int(st["n_state"])
        f.n_cmd = int(st["n_cmd"])

        f.obs = _pad(st["obs"] if st["obs"] is not None else np.zeros(obs_dim), obs_dim)
        f.action = _pad(st["action"] if st["action"] is not None else np.zeros(12), 12)
        f.action_applied = f.joint_q_des.copy() if st["lowcmd"] is not None else f.action.copy()
        if f.obs.size >= 243:
            f.phase = np.arctan2(f.obs[239:243], f.obs[235:239])
        elif f.obs.size >= 91:
            f.phase = np.arctan2(f.obs[87:91], f.obs[83:87])

        f.scan_nx, f.scan_ny = scan_nx, scan_ny
        scan = st["scan"]
        if scan is not None:
            f.height_scan = _pad(scan, scan_nx * scan_ny)
        else:
            f.height_scan = np.zeros(scan_nx * scan_ny)
        # 扫描点世界坐标：与 HeightMapper.scan 同一口径
        yaw = float(np.arctan2(
            2.0 * (f.base_quat[0] * f.base_quat[3] + f.base_quat[1] * f.base_quat[2]),
            1.0 - 2.0 * (f.base_quat[2] ** 2 + f.base_quat[3] ** 2)))
        c, s = math.cos(yaw), math.sin(yaw)
        xy = np.stack([c * grid[:, 0] - s * grid[:, 1],
                       s * grid[:, 0] + c * grid[:, 1]], axis=-1) + f.base_pos[None, :2]
        z = f.base_pos[2] - scan_base - f.height_scan / scan_scale
        f.scan_xy_body = grid
        f.scan_xy_world = np.concatenate([xy, (z + 0.006)[:, None]], axis=-1)
        return f

    def apply_pose(frame: VizFrame) -> None:
        data.qpos[0:3] = frame.base_pos
        data.qpos[3:7] = frame.base_quat
        data.qpos[qadr] = frame.joint_q
        mujoco.mj_forward(model, data)
        for k, g in enumerate(foot_geoms[:4]):
            if k < len(frame.foot_pos_world):
                frame.foot_pos_world[k] = np.asarray(data.geom_xpos[int(g)], dtype=np.float64)

    print("[viewer] model =", xml)
    print("[viewer] topics = state %s | cmd %s | odom %s | scan %s"
          % (args.state_topic, args.cmd_topic, args.odom_topic, args.scan_topic))
    try:
        while rclpy.ok():
            wall = time.monotonic() - t0
            if args.duration > 0.0 and wall >= args.duration:
                break
            now = time.monotonic()
            if now - last_render >= 1.0 / max(1.0, args.sync_hz):
                frame = build_frame(wall)
                # RTF 估计：用 LowState tick 的仿真时间 / 墙钟时间（真机 tick 也单调）
                if prev_t is not None and frame.t > prev_t and now > prev_wall:
                    frame.rtf = (frame.t - prev_t) / max(1e-6, now - prev_wall)
                prev_t, prev_wall = frame.t, now
                apply_pose(frame)
                if viewer is not None:
                    overlay.draw(viewer.user_scn, frame)
                    overlay.set_camera(viewer.cam, frame)
                    if hud is not None:
                        hud.update(frame)
                        viewer.set_texts(hud.texts())
                    viewer.sync()
                    if not viewer.is_running():
                        break
                if live is not None and now - last_plot >= 1.0 / max(0.2, args.plot_hz):
                    live.update(frame)
                    live.pump()
                    last_plot = now
                if rec is not None:
                    rec.write(frame)
                if dash is not None and now - last_dash >= 0.1:
                    dash.write(frame)
                    last_dash = now
                frames += 1
                last_render = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        summary = {
            "model": xml, "target": args.target, "frames": frames,
            "n_state": st["n_state"], "n_cmd": st["n_cmd"], "n_odom": st["n_odom"],
            "n_scan": st["n_scan"], "n_obs": st["n_obs"], "n_action": st["n_action"],
            "last_info": st["info"],
        }
        if rec is not None:
            try:
                rec.close(summary=summary)
            except Exception as exc:
                print("[viewer] recorder close:", repr(exc)[:120])
        if live is not None:
            live.close()
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            spinner.join(timeout=1.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        print("[viewer] summary =", json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
