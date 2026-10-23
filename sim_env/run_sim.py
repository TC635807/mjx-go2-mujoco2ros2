#!/usr/bin/env python3
"""启动 MuJoCo 仿真侧节点（500 Hz 物理 + LowState 发布 + LowCmd 接收）。

IO 层由 --target 决定（见 signal_io/config.py 的 io_profiles）：
    sim_udp  裸 UDP，不需要 ROS 2
    sim_ros  IOROS：rclpy + unitree_go，话题 /lowcmd / /lowstate
    sim_sdk  IOSDK：unitree_sdk2py（Python 3.14 下不可用）

机型由 --robot 决定（models/go2 是训练侧模型、models/go2 是部署侧真机模型）。
--profile real_mild/real_severe 会在**仿真侧**注入真机才有的信号劣化
（延迟/丢帧/噪声/偏置），用来复现健康层的降级路径 —— 详见 domain_gap.SignalDegrader。

例:
    python sim_env/run_sim.py --target sim_ros --robot go2 --duration 20 --viewer
    python sim_env/run_sim.py --target sim_udp --robot go2 --profile real_severe
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# 单线程 BLAS（策略前向 1024-512 小矩阵多线程反而慢 50 倍，见 scripts/view_policy.py 注释）
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from signal_io.config import (apply_io_env, io_summary, load_config,  # noqa: E402
                               model_xml, robot_spec_of)
from signal_io.node import SimNode  # noqa: E402
from signal_io.transport import make_transport  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--duration", type=float, default=20.0, help="仿真时长 (s, 仿真时间)")
    ap.add_argument("--robot", default=None, choices=["go2"],
                    help="被仿真的机型（只保留 go2；训练侧模型在 mjx-go2-training 仓）")
    ap.add_argument("--scene", default=None, choices=["default", "flat"],
                    help="flat = 纯平面（策略闭环对比用，排除地形变量）")
    ap.add_argument("--terrain", default="flat",
                    help="地形名（sim_env/models/go2/terrains.json；如 wide_h050/hyb2/rubble）")
    # 接触/求解器档位默认值来自 config 的 robots.<robot> 段（go2 默认软接触 + 100/50，
    # go2 不写 -> 保持本框架既有默认 = 硬接触 + MuJoCo 默认迭代）。三态：显式 flag 覆盖配置。
    ap.add_argument("--soft-feet", dest="soft_feet", action="store_const", const=True, default=None,
                    help="足端软接触（复刻训练模型 condim=3 + solimp 0.9/0.95/0.023）；"
                         "默认取 config 的 foot_soft_contact（go2=true, go2=false）")
    ap.add_argument("--hard-feet", dest="soft_feet", action="store_const", const=False,
                    help="强制用模型 XML 里的足端接触参数（go2 的既有默认行为）")
    ap.add_argument("--iters", type=int, default=None,
                    help="MuJoCo solver iterations（默认取 config 的 solver_iterations；"
                         "都没给就用 MuJoCo 默认 100）")
    ap.add_argument("--ls-iters", type=int, default=None,
                    help="MuJoCo solver ls_iterations（同上；都没给就是 50）")
    ap.add_argument("--profile", default="sim_clean",
                    choices=["sim_clean", "real_mild", "real_severe"],
                    help="信号劣化档位（在仿真侧注入，用于复现健康层降级路径）")
    ap.add_argument("--target", default=None,
                    help="IO profile: sim_udp / sim_ros / sim_sdk / real_ros / real_sdk")
    ap.add_argument("--transport", default=None, choices=["loopback", "udp", "dds", "ros2"],
                    help="[兼容] 只改 IO 名字，其余参数仍取自 profile；建议用 --target")
    ap.add_argument("--viewer", action="store_true", help="打开 MuJoCo 被动渲染窗口")
    ap.add_argument("--no-realtime", action="store_true", help="不限速，跑满 CPU（测吞吐用）")
    ap.add_argument("--no-safety", action="store_true", help="关掉看门狗/斜率限幅")
    ap.add_argument("--log", default=None, help="把状态写进 CSV")
    ap.add_argument("--json", default=None, help="把总结写进 JSON")
    ap.add_argument("--publish-sensors", action="store_true",
                    help="把 MuJoCo 真值发布成与真机同名的 ROS 2 话题："
                         "/go2/odom（nav_msgs/Odometry）+ /go2/lidar/points（PointCloud2）")
    ap.add_argument("--odom-topic", default="/go2/odom")
    ap.add_argument("--points-topic", default="/go2/lidar/points")
    ap.add_argument("--sensor-hz", type=float, default=None,
                    help="雷达点云发布频率（默认取 config.perception.rate_hz）")
    args = ap.parse_args()

    cfg = load_config(args.config, target=args.target, overrides={"robot": args.robot})
    if args.transport:
        cfg["transport"]["name"] = args.transport
    terrain_meta = None
    if args.terrain and args.terrain != "flat":
        from sim_env.terrain import load_terrain_manifest, resolve_terrain
        terrain_meta = resolve_terrain(args.terrain, load_terrain_manifest())
        xml = terrain_meta["scene"]
    else:
        xml = model_xml(cfg)
        if args.scene == "flat":
            cand = os.path.join(os.path.dirname(xml), "scene_flat.xml")
            if os.path.isfile(cand):
                xml = cand
            else:
                print("[sim] 注意：%s 不存在，仍用 %s" % (cand, os.path.basename(xml)))

    degrader = None
    if args.profile and args.profile != "sim_clean":
        from signal_io.domain_gap import SignalDegrader
        profile_cfg = dict(cfg.get("domain_gap") or {})
        profile_cfg["profile"] = args.profile
        degrader = SignalDegrader(profile_cfg)

    env = apply_io_env(cfg)          # ROS_DOMAIN_ID / CYCLONEDDS_URI 必须在 init 之前设好
    tr = make_transport(cfg["transport"], role="sim")
    # 三态 flag：显式给的值 > config 的 robots.<robot> 段 > 框架默认
    soft_feet = (args.soft_feet if args.soft_feet is not None
                 else bool(cfg.get("foot_soft_contact", False)))
    iters = args.iters if args.iters is not None else cfg.get("solver_iterations")
    ls_iters = args.ls_iters if args.ls_iters is not None else cfg.get("solver_ls_iterations")

    node = SimNode(xml, tr, sim_dt=cfg["sim_dt"],
                   watchdog_ms=cfg["watchdog_ms"], ramp_rate=cfg["ramp_rate"],
                   safety=not args.no_safety,
                   hold_until_first_cmd=cfg.get("hold_until_first_cmd", True),
                   hold_kp=cfg["controller"]["kp"], hold_kd=cfg["controller"]["kd"],
                   spec=robot_spec_of(cfg), foot_soft_contact=soft_feet,
                   solver_iterations=iters, solver_ls_iterations=ls_iters,
                   degrader=degrader)
    if terrain_meta is not None:
        from sim_env.terrain import spawn_on_terrain
        spawn_on_terrain(node.sim, terrain_meta)
        print("[sim] 地形   =", args.terrain, "| scene =", os.path.basename(xml),
              "| 出生点已按地形清单设置")
    print("[sim] model =", xml)
    print("[sim] io    =", io_summary(cfg))
    print("[sim] robot =", cfg["robot"], "| 力矩上限", list(node.sim.torque_limits[:3]), "(hip/thigh, knee)")
    print("[sim] 站姿   =", [round(float(x), 3) for x in node.sim.stand_pose],
          "| 出生 z =", node.sim.stand_height)
    print("[sim] 足端接触 =", "soft(训练同款 condim=3)" if soft_feet else "XML 原始(硬 condim=6)",
          "| 求解器 =", node.sim.model.opt.iterations, "/", node.sim.model.opt.ls_iterations,
          "| 信号劣化 =", args.profile)
    if env:
        print("[sim] env   =", env)
    print("[sim] info  =", json.dumps(node.sim.info(), ensure_ascii=False))
    print("[sim] 步长   = %.4f s (%.0f Hz)" % (node.dt, 1.0 / node.dt))

    sensors = None
    if args.publish_sensors:
        if cfg["transport"]["name"] != "ros2":
            print("[sim] --publish-sensors 需要 --target sim_ros（当前 %r）"
                  % cfg["transport"]["name"])
            return 2
        from signal_io.ros2_sensors import SimSensorPublisher
        sensors = SimSensorPublisher(node.sim, cfg,
                                     odom_topic=args.odom_topic,
                                     points_topic=args.points_topic,
                                     sensor_hz=args.sensor_hz)
        print("[sim] sensors = %s + %s" % (args.odom_topic, args.points_topic))
    try:
        summary = node.run(args.duration, realtime=not args.no_realtime, viewer=args.viewer,
                            log_path=args.log,
                            on_step=(sensors.publish if sensors is not None else None))
    except KeyboardInterrupt:
        summary = {"interrupted": True}
    if sensors is not None:
        summary["sensors"] = sensors.stats()
        sensors.close()
    summary["io"] = {"target": cfg["target"], "profile": dict(cfg["transport"])}
    print("[sim] summary =", json.dumps(summary, ensure_ascii=False))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
