#!/usr/bin/env python3
"""可视化观察器：一个 MuJoCo 窗口里检查闭环效果 + 策略/观测内部状态。

单进程跑：Go2Sim + 控制器(默认 policy) + 健康层 + 安全层，500 Hz 物理 / 50 Hz 控制，
然后**同时**用四种方式把内部状态摊开给你看（都是 mujoco passive viewer 原生能力，
不需要 tkinter/Qt）：

    3D 叠加   handle.user_scn      35 个高度采样点（按特征值着色）· 足端接触 · 速度/指令箭头 · 轨迹
    实时曲线  handle.set_figures   tau / 基座高度与倾斜 / 线速度(估计 vs 真值) / 动作幅值 / 估计误差
    仪表盘    handle.set_images    height scan 7x5 热图 · 4 腿相位 · 91 维 obs 分段条形 · 关键数字
    文字 HUD  handle.set_texts     健康等级 / 模式 / tilt / z / vx / tau_max / 每一路来源

没有图形界面时（--headless 或没有 DISPLAY）自动退化成**终端 ANSI 仪表盘**，一样能看。

用法：
    .venv/bin/python scripts/view_policy.py --robot go2 --controller policy --duration 60
    .venv/bin/python scripts/view_policy.py --robot go2 --scan nominal --scene obstacles
    .venv/bin/python scripts/view_policy.py --headless --duration 20 --record logs/obs_demo
    .venv/bin/python scripts/view_policy.py --controller stand --duration 15

窗口里键盘：r=重置 空格=暂停/继续 h=开关 3D 叠加 f=开关曲线 i=开关仪表盘 q=退出
             w/s=前进速度 ±0.1 m/s (0~1.0)  a/d=转向角速度 ±0.2 rad/s (-1~1)

地形（--terrain，读 sim_env/models/go2/terrains.json 清单；不传则平地）：
    python scripts/view_policy.py --robot go2 --terrain hyb2
    python scripts/view_policy.py --robot go2 --terrain rubble
    python scripts/view_policy.py --list-terrains
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

# 单线程 BLAS: 1024-512 策略前向在默认 OpenBLAS 线程数下实测 12 ms/次（还占满 12~16 核），
# 单线程 0.21 ms/次 —— 这是观察器 RTF 只有 0.3~0.4 的头号原因。必须在 import numpy 之前设。
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from signal_io.config import load_config, model_xml, robot_spec_of  # noqa: E402
from signal_io.msg import NUM_JOINT, LowCmd  # noqa: E402
from sim_env.robot import Go2Sim  # noqa: E402
from sim_env.viz import FrameCollector, VizFrame, make_ground  # noqa: E402


def build_state(args):
    """按命令行把 cfg / 模型 / 控制器 / 健康层 / 安全层全搭起来。"""
    from ros2_controller import run_controller as rc

    cfg = load_config(os.path.join(ROOT, "config", "go2_sim.json"),
                      overrides={"robot": args.robot,
                                 "policy_profile": args.policy_profile})
    if args.scan:
        cfg.setdefault("policy", {})["height_scan_source"] = args.scan
    if args.prepose_s is not None:
        cfg.setdefault("policy", {})["prepose_s"] = args.prepose_s

    xml = args.model or model_xml(cfg)
    if args.scene == "flat" and args.model is None:
        cand = os.path.join(os.path.dirname(xml), "scene_flat.xml")
        if os.path.isfile(cand):
            xml = cand
    spec = robot_spec_of(cfg)
    soft_feet = args.soft_feet
    if soft_feet is None:                       # 未显式指定 -> 跟随 config
        soft_feet = bool(getattr(spec, "foot_soft_contact", False))
    sim = Go2Sim(xml, spec=spec, sim_dt=cfg["sim_dt"],
                 foot_soft_contact=soft_feet)
    ctrl = rc.build_controller(args.controller, cfg, sim, cli_cmd=args.cmd,
                               linvel=args.linvel)

    from ros2_controller.health import HealthMonitor
    from ros2_controller.safety import SafetySupervisor
    health = HealthMonitor(cfg)
    sup_cfg = cfg.get("safety", {})
    limits = np.asarray(sim.torque_limits, dtype=np.float64)
    supervisor = SafetySupervisor(sim.stand_pose, limits=limits,
                                  ramp_rate=cfg.get("ramp_rate", 200.0),
                                  mode_kp=float(sup_cfg.get("mode_kp", 20.0)),
                                  mode_kd=float(sup_cfg.get("mode_kd", 1.0)),
                                  crouch_depth=float(sup_cfg.get("crouch_depth", 0.35)),
                                  crouch_s=float(sup_cfg.get("crouch_s", 2.0)))
    return cfg, xml, sim, ctrl, health, supervisor


TERRAIN_MANIFEST = os.path.join(ROOT, "sim_env", "models", "go2", "terrains.json")


def load_terrain_manifest(path: str = TERRAIN_MANIFEST) -> dict:
    """读地形清单 (/home/tc63/mujoco/mjx-go2-training/sim/export_deploy_terrains.py 生成)。"""
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as fh:
        man = json.load(fh)
    return {e["name"]: e for e in man.get("terrains", []) if "name" in e}


def resolve_terrain(name: str, manifest: dict) -> dict:
    """按名字取地形条目; scene 路径解析成绝对路径。"""
    e = manifest.get(name)
    if e is None:
        raise SystemExit(
            "未知地形 %r; 可选: %s (清单: %s)" % (name, ", ".join(sorted(manifest)),
                                              TERRAIN_MANIFEST))
    e = dict(e)
    if not os.path.isabs(e["scene"]):
        e["scene"] = os.path.join(ROOT, e["scene"])
    if not os.path.isfile(e["scene"]):
        raise SystemExit("地形场景不存在: %s (重新跑 "
                         "/home/tc63/mujoco/mjx-go2-training/sim/export_deploy_terrains.py)" % e["scene"])
    return e


def _ray_ground_height(sim: "Go2Sim", x: float, y: float, top: float = 3.0) -> float:
    """从 (x,y,top) 向下打一条射线, 返回地面高度 (米; 打不到返回 0)。"""
    import mujoco
    pnt = np.array([x, y, top], dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(sim.model, sim.data, pnt, vec, None, 1, -1, geomid)
    if dist < 0.0 or dist >= top:
        return 0.0
    return float(top - dist)


def spawn_on_terrain(sim: "Go2Sim", meta: dict, rng=None) -> None:
    """把机器人放到该地形上 (非侵入: 只是 sim.reset 后改 free joint 位姿)。

    hybrid/boxgeom 地形: 外圈平地环形出生 (场半径 +0.4~+1.3m, 朝向场地中心),
                         和训练评测口径一致 —— 能看到"从平地走进障碍场"。
    hfield-only 地形: 生成器把原点附近压平, 在中心小半径内随机出生 + 随机朝向。
    flat: 直接 sim.reset()。
    """
    rng = np.random.default_rng() if rng is None else rng
    sim.reset()
    kind = meta.get("kind", "flat")
    fh = float(meta.get("field_half") or 0.0)
    hh = float(meta.get("hfield_half") or fh)
    if kind == "flat" or (fh <= 0.0 and hh <= 0.0):
        return
    if kind == "hfield":
        r = float(rng.uniform(0.0, 0.6))
        th = float(rng.uniform(-np.pi, np.pi))
        yaw = float(rng.uniform(-np.pi, np.pi))
    else:
        r_in = min(fh + 0.4, max(0.0, hh - 0.4))
        r_out = max(r_in + 0.2, min(fh + 1.3, hh - 0.2))
        r = float(rng.uniform(r_in, r_out))
        th = float(rng.uniform(-np.pi, np.pi))
        # 朝向场地中心 (+ 随机偏航 ±30°)
        yaw = float(th + np.pi + rng.uniform(-0.5, 0.5))
    x, y = r * np.cos(th), r * np.sin(th)
    base_z = float(sim.data.qpos[2])            # keyframe 站高 (0.27)
    ground = _ray_ground_height(sim, x, y)
    sim.data.qpos[0] = x
    sim.data.qpos[1] = y
    sim.data.qpos[2] = base_z + ground + 0.01
    c, s = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    sim.data.qpos[3:7] = [c, 0.0, 0.0, s]
    sim.data.qvel[:] = 0.0
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    sim.tick = 0
    sim.time = 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="go2", choices=["go2"])
    ap.add_argument("--controller", default="policy", choices=["policy", "stand", "zero", "lqr_stub"])
    ap.add_argument("--scene", default="flat", choices=["flat", "obstacles"],
                    help="旧接口: flat/obstacles (terrain 非 flat 时忽略)")
    ap.add_argument("--terrain", default="flat",
                    help="地形名 (默认 flat); 读 sim_env/models/go2/terrains.json; "
                         "--list-terrains 看全部, 如 hyb2/rubble/big1")
    ap.add_argument("--list-terrains", action="store_true",
                    help="列出可用地形后退出")
    ap.add_argument("--model", default=None)
    # 速度来源: 仿真有真值, 用 oracle 才和训练观测同分布 (训练 obs 0:3 = qvel[0:3])
    ap.add_argument("--policy-profile", default=None,
                    help="策略档 (config policy_profiles 里的 key), 如 go2_v3; 默认 auto 按 robot")
    ap.add_argument("--linvel", default="oracle", choices=["oracle", "estimator"],
                    help="oracle=仿真真值(与训练同分布, 默认); estimator=腿式里程计(真机路径)")
    # 高程来源: nominal=按标称站高(平地上最接近训练, 特征≈0);
    #           leg_fk=接触腿正运动学(恒定标量, 无地形信息, 但有 ~-0.12 偏置)
    ap.add_argument("--scan", default="nominal",
                    choices=["nominal", "leg_fk", "raycast"],
                    help="高度扫描来源: nominal=标称站高(平地最稳); "
                         "leg_fk=接触腿正运动学(单个标量); "
                         "raycast=SimLidar+HeightMapper 真感知链路 (v6, 含自遮挡)")
    ap.add_argument("--cmd", type=float, nargs=3, default=None, help="速度指令 (m/s,m/s,rad/s)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="运行时长(s, 仿真时间); **0 = 一直跑到 Q/关窗口**")
    # 足端接触默认**跟随 config** (robots.<robot>.foot_soft_contact):
    # v2 交付策略是硬接触训的, config 里是 false; 这里再写死 True 就会用错物理。
    ap.add_argument("--soft-feet", dest="soft_feet", action="store_true",
                    default=None, help="强制足端软接触 (默认跟随 config)")
    ap.add_argument("--hard-feet", dest="soft_feet", action="store_false",
                    help="强制足端硬接触 (v2 Go2 策略用这个, 也是 config 默认)")
    ap.add_argument("--prepose-s", type=float, default=None)
    ap.add_argument("--headless", action="store_true", help="不开 3D 窗口，只出终端仪表盘")
    ap.add_argument("--no-3d", action="store_true", help="关掉 3D 叠加标记")
    ap.add_argument("--follow", action="store_true", help="相机跟随躯干")
    ap.add_argument("--plots", default="window", choices=["window", "inline", "off"],
                    help="曲线放哪：window=独立 matplotlib 大窗口（默认，能看清且不挡机器人）；"
                         "inline=塞进 MuJoCo 窗口（会挡视野，且窗口小看不清）；off=不画")
    ap.add_argument("--plot-hz", type=float, default=3.0,
                    help="独立窗口的刷新率（默认 3 Hz；Agg 每次全图重绘约 150-200 ms，"
                         "再高会拖慢 RTF，脚本还会按 RTF 自动减半）")
    ap.add_argument("--figures", default="tau,height,vel",
                    help="inline 模式下要画的曲线，逗号分隔；空字符串=不画")
    ap.add_argument("--dashboard", dest="dashboard", action="store_true", default=False,
                    help="inline 模式下画 matplotlib 仪表盘图像(obs 热图/分段条)。"
                         "单帧 50~90 ms, 会偶发卡一下, 默认关")
    ap.add_argument("--no-dashboard", dest="dashboard", action="store_false",
                    help="[兼容] 关掉仪表盘图像(默认就是关)")
    ap.add_argument("--robot-style", dest="robot_style", default="full",
                    choices=["full", "lite"],
                    help="机器人外观: full=原版高模网格(默认); lite=只显示碰撞几何(低模, fps 优先)")
    ap.add_argument("--hud", default="on", choices=["on", "off"],
                    help="左上角文字 HUD（默认 on，很小一块）")
    ap.add_argument("--lite", default="on", choices=["on", "off"],
                    help="关掉阴影/反射等昂贵视觉效果（默认 on，软件渲染下帧率差好几倍）")
    ap.add_argument("--trail", type=int, default=120, help="轨迹拖尾点数（默认 120，0=不画）")
    ap.add_argument("--record", default=None, help="落盘目录：CSV + 关键帧 + summary.png")
    ap.add_argument("--record-every", type=float, default=10.0, help="记录频率 Hz")
    ap.add_argument("--sync-hz", type=float, default=60.0,
                    help="3D 画面刷新率(仿真时间 Hz; 默认 60; 卡的话降到 30/15)。"
                         "注意这是'每隔多少物理时间画一帧', 低值会让腿看起来瞬移")
    ap.add_argument("--fig-hz", type=float, default=2.0)
    ap.add_argument("--img-hz", type=float, default=0.5)
    ap.add_argument("--hud-hz", type=float, default=2.0)
    ap.add_argument("--dash-hz", type=float, default=10.0, help="终端仪表盘刷新率（headless）")
    ap.add_argument("--monitor-hz", type=float, default=100.0,
                    help="健康/安全/状态读取的分频频率（默认 100 Hz；500 Hz 每步做这些纯属浪费，"
                         "实测每步约 0.5 ms，等于吃掉 25%% 实时性。物理仍是 500 Hz）")
    ap.add_argument("--panel-budget-ms", type=float, default=25.0,
                    help="内嵌曲线/仪表盘单帧耗时预算，超了就自动降刷新率（默认 25 ms）")
    ap.add_argument("--snapshot", default=None,
                    help="离屏渲染一张带叠加标记的 PNG（不需要窗口，用来留证/预览）")
    ap.add_argument("--snapshot-at", type=float, default=None,
                    help="在第几秒截图（默认 0.75*duration）")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    manifest = load_terrain_manifest()
    terrain = args.terrain
    tmeta = None
    if args.list_terrains:
        print("可用地形 (--terrain):")
        for name in sorted(manifest):
            e = manifest[name]
            print("  %-12s %-8s field_half=%.2fm hfield_half=%.2fm  %s"
                  % (name, e.get("kind", "?"), float(e.get("field_half") or 0.0),
                     float(e.get("hfield_half") or 0.0), e.get("desc", "")))
        return 0
    if terrain != "flat":
        if not manifest:
            raise SystemExit("找不到地形清单: %s (先跑 mjx-go2-training/sim/"
                             "export_deploy_terrains.py)" % TERRAIN_MANIFEST)
        tmeta = resolve_terrain(terrain, manifest)
        args.model = tmeta["scene"]       # 让 Go2Sim 加载地形场景 (非侵入)

    cfg, xml, sim, ctrl, health, supervisor = build_state(args)
    if tmeta is not None:
        spawn_on_terrain(sim, tmeta)
        print("[viz] terrain = %s (%s): field_half=%.2fm hfield_half=%.2fm n_boxes=%s"
              % (terrain, tmeta.get("kind", "?"), float(tmeta.get("field_half") or 0.0),
                 float(tmeta.get("hfield_half") or 0.0), tmeta.get("n_boxes", "-")))
        print("       %s" % tmeta.get("desc", ""))
    else:
        print("[viz] terrain = flat (scene=%s)" % args.scene)
    print("[viz] 键盘: W/S 速度 ±0.1 (0~1.0) | A/D 转向 ±0.2 (-1~1) | R 重置 | "
          "空格 暂停 | H/F/I 开关 | Q 退出")
    cmd_now = np.asarray(getattr(ctrl, "command", (0.5, 0.0, 0.0)),
                         dtype=np.float64).reshape(3).copy()

    def apply_cmd() -> None:
        """把当前指令写到控制器 + obs builder (观测器侧改动, 不动控制器代码)。"""
        ctrl.command[:] = cmd_now
        b = getattr(ctrl, "obs", None)
        if b is not None and hasattr(b, "_cmd"):
            b._cmd = cmd_now.copy()

    apply_cmd()
    ctrl.reset()
    health.reset(0.0)
    supervisor.reset()

    ground = make_ground(sim.model, sim.data, "auto")
    grid = None
    builder = getattr(ctrl, "obs", None)
    scan_nx = int(getattr(getattr(builder, "cfg", None), "scan_nx", 7))
    scan_ny = int(getattr(getattr(builder, "cfg", None), "scan_ny", 5))
    obs_dim = int(getattr(builder, "obs_dim", 91))
    if builder is not None and hasattr(builder, "scan_grid"):
        grid = builder.scan_grid()
    collector = FrameCollector(sim, ground=ground, scan_xy_body=grid,
                               scan_nx=scan_nx, scan_ny=scan_ny)
    if grid is None:
        collector.set_scan_grid(np.zeros((35, 2)))

    # ---- 可视化组件（队友并行开发，缺了就降级，不影响主循环）----
    scene = panels = image = hud = rec = dash = None
    try:
        from sim_env.viz.scene import SceneOverlay
        # 叠加标记**故意做小**：这东西是辅助，主角是机器人本身。
        scene = SceneOverlay(sim.model, sim.data, cfg={
            "scan_radius_min": 0.008, "scan_radius_max": 0.030,
            "foot_contact_radius": 0.026, "foot_free_radius": 0.010,
            "trail_n": int(args.trail), "trail_radius": 0.009,
            "trail_on": args.trail > 0,
            "arrow_width": 0.008, "vel_scale": 0.4,
            # v6: 大扫描档 187 点也要全画出来 -> 预算按扫描点数放大
            "geom_budget": max(160, scan_nx * scan_ny + 60),
        })
    except Exception as exc:
        print("[viz] 没有 scene.py（%s），跳过 3D 叠加" % type(exc).__name__)
    # 曲线默认走**独立窗口**：MuJoCo 窗口只有 740x800，塞进去每块只有 89x161 px 根本看不清；
    # 拆成两个窗口，各司其职（用户明确要这个形态）。
    live = None
    if args.plots == "window":
        try:
            from sim_env.viz.live import LivePlots, available as _live_ok
            if _live_ok():
                live = LivePlots(title="go2_bridge  %s / %s" % (args.robot, args.controller))
                print("[viz] 曲线/仪表盘 -> 独立窗口 (matplotlib/QtAgg)："
                      "力矩 | 高度倾斜 | 速度 | 高度扫描 | 步态相位 | 动作")
            else:
                print("[viz] 没有 Qt 后端，独立窗口开不了；改用内嵌模式（可能挡视野）")
                args.plots = "inline"
        except Exception as exc:
            print("[viz] 独立窗口失败（%s），改用内嵌模式" % type(exc).__name__)
            args.plots = "inline"
    try:
        from sim_env.viz.panels import FigurePanel, ImagePanel, TextHud
        which = [s for s in (args.figures or "").split(",") if s]
        panels = FigurePanel(which=which) if (args.plots == "inline" and which) else None
        image = ImagePanel() if (args.plots == "inline" and args.dashboard) else None
        if image is not None:
            # v6: 仪表盘按策略 obs 维度重建 (91/7x5 -> 243/17x11)
            image.configure(obs_dim=obs_dim, scan_nx=scan_nx, scan_ny=scan_ny)
        hud = TextHud() if args.hud == "on" else None
    except Exception as exc:
        print("[viz] 没有 panels.py（%s），跳过曲线/仪表盘" % type(exc).__name__)
    try:
        from sim_env.viz.record import Recorder, TerminalDashboard
        dash = TerminalDashboard()
        if args.record:
            rec = Recorder(args.record, fps=args.record_every)
    except Exception as exc:
        print("[viz] 没有 record.py（%s），跳过落盘/终端仪表盘" % type(exc).__name__)

    # ---- 窗口 ----
    viewer = None
    # 注意括号！写成 "not headless and DISPLAY or WAYLAND_DISPLAY" 会因为优先级
    # 变成 "(not headless and DISPLAY) or WAYLAND_DISPLAY"，于是 --headless 在有
    # WAYLAND_DISPLAY 的机器上照样开会话（实测把 RTF 从 1.0 拖到 0.04）。
    has_gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if (not args.headless) and has_gui:
        try:
            import mujoco.viewer
            state = {"pause": False, "quit": False, "overlay": not args.no_3d,
                     "figs": panels is not None, "img": image is not None,
                     "hud": hud is not None}

            def key_cb(key: int) -> None:
                import mujoco.viewer as V
                g = V.glfw
                if key in (getattr(g, "KEY_Q", 81), getattr(g, "KEY_ESCAPE", 256)):
                    state["quit"] = True
                elif key == getattr(g, "KEY_SPACE", 32):
                    state["pause"] = not state["pause"]
                elif key == getattr(g, "KEY_R", 82):
                    state["reset"] = True
                elif key == getattr(g, "KEY_H", 72):
                    state["overlay"] = not state["overlay"]
                elif key == getattr(g, "KEY_F", 70):
                    state["figs"] = not state["figs"]
                elif key == getattr(g, "KEY_I", 73):
                    state["img"] = not state["img"]
                elif key == getattr(g, "KEY_W", 87):
                    cmd_now[0] = min(1.0, cmd_now[0] + 0.1)
                    apply_cmd()
                    print("[viz] cmd vx=%.1f m/s wz=%+.1f rad/s" % (cmd_now[0], cmd_now[2]))
                elif key == getattr(g, "KEY_S", 83):
                    cmd_now[0] = max(0.0, cmd_now[0] - 0.1)
                    apply_cmd()
                    print("[viz] cmd vx=%.1f m/s wz=%+.1f rad/s" % (cmd_now[0], cmd_now[2]))
                elif key == getattr(g, "KEY_A", 65):
                    cmd_now[2] = min(1.0, cmd_now[2] + 0.2)
                    apply_cmd()
                    print("[viz] cmd vx=%.1f m/s wz=%+.1f rad/s" % (cmd_now[0], cmd_now[2]))
                elif key == getattr(g, "KEY_D", 68):
                    cmd_now[2] = max(-1.0, cmd_now[2] - 0.2)
                    apply_cmd()
                    print("[viz] cmd vx=%.1f m/s wz=%+.1f rad/s" % (cmd_now[0], cmd_now[2]))

            # 关掉软件渲染下最贵的三项：阴影、反射、高次采样。
            # WSL2/WSLg 上 GL 走 llvmpipe，默认设置下一帧要 ~200 ms（3~5 fps），
            # 关掉之后才有可用帧率。
            if args.lite == "on":
                # MuJoCo 3.12 把阴影/反射挪到了 scene flags，vis.global_ 里已经没有 shadow
                # 字段了，所以这里逐个 hasattr 探测，能设的都设，设不了就跳过（别让观察器崩）。
                q = sim.model.vis.quality
                for name, val in (("shadowsize", 256), ("offsamples", 1), ("reflections", 0)):
                    if hasattr(q, name):
                        setattr(q, name, val)
                g = sim.model.vis.global_
                for name, val in (("shadow", 0), ("reflections", 0)):
                    if hasattr(g, name):
                        setattr(g, name, val)
                print("[viz] lite 视觉：能关的昂贵项已关（fps 优先）")
            # v33: 关掉 viewer 左右侧 UI (只保留 3D 画面), 减少软件渲染开销
            try:
                viewer = mujoco.viewer.launch_passive(
                    sim.model, sim.data, key_callback=key_cb,
                    show_left_ui=False, show_right_ui=False)
            except TypeError:
                viewer = mujoco.viewer.launch_passive(
                    sim.model, sim.data, key_callback=key_cb)
            # v33 视觉/性能: group3 = 机器人碰撞几何 (capsule/sphere, 只在物理/ray 用),
            # group4 = 地形碰撞盒/瓦片+floor_plane (只在物理/ray 用) —— 默认隐藏渲染。
            # 可见地形只剩统一材质的地面 hfield + 1 个视觉 mesh, 帧率大幅提升。
            try:
                viewer.opt.geomgroup[3] = 0   # 机器人碰撞几何: 默认隐藏
                viewer.opt.geomgroup[4] = 0   # 地形碰撞盒/瓦片/floor_plane: 默认隐藏
                if args.robot_style == "lite":
                    # 只显示机器人碰撞几何 (低模, 软件渲染 fps 优先)
                    viewer.opt.geomgroup[2] = 0
                    viewer.opt.geomgroup[3] = 1
            except Exception:
                pass
            viewer.cam.distance = 2.2
            viewer.cam.elevation = -18.0
            viewer.cam.azimuth = 120.0
        except Exception as exc:
            print("[viz] 开不了 3D 窗口（%s: %s），退回终端模式" % (type(exc).__name__, exc))
            viewer = None

    headless = viewer is None
    if not args.headless and headless:
        print("[viz] 没有可用图形界面，改用终端仪表盘")
    if viewer is not None:
        print("[viz] 画面刷新 = %.0f Hz(仿真时间) | 面板: 曲线 %s / 仪表盘 %s / HUD %s"
              % (args.sync_hz, "开" if panels is not None and args.plots == "inline" else "关/独立窗口",
                 "开" if image is not None else "关", "开" if hud is not None else "关"))

    # ---- 主循环 ----
    sim_dt = sim.dt
    decim = max(1, int(round(cfg["control_dt"] / sim_dt)))
    n_steps = None if args.duration <= 0 else int(round(args.duration / sim_dt))
    if args.duration <= 0:
        print("[viz] 运行模式: 一直跑, 直到按 Q / 关闭窗口 (--duration N 可指定自动退出)")
    else:
        print("[viz] 运行模式: %.1f s 后自动退出 (按 Q 可提前退出; --duration 0 = 一直跑)"
              % args.duration)
    sync_every = max(1, int(round(1.0 / (args.sync_hz * sim_dt))))
    fig_every = max(1, int(round(1.0 / (max(args.fig_hz, 1e-3) * sim_dt))))
    img_every = max(1, int(round(1.0 / (max(args.img_hz, 1e-3) * sim_dt))))
    plot_every = max(1, int(round(1.0 / (max(args.plot_hz, 1e-3) * sim_dt))))
    hud_every = max(1, int(round(1.0 / (max(args.hud_hz, 1e-3) * sim_dt))))
    dash_every = max(1, int(round(1.0 / (args.dash_hz * sim_dt))))
    mon_every = max(1, int(round(1.0 / (max(args.monitor_hz, 1e-3) * sim_dt))))
    viz_cost = {"fig": 0.0, "img": 0.0}
    sync_ms: list = []
    layout: dict = {}
    degrade = 0
    plot_check = 0

    # ---- 离屏截图（不需要窗口）----
    snap = {"at": float(args.snapshot_at if args.snapshot_at is not None
                        else (0.75 * args.duration if args.duration > 0 else 5.0)),
            "done": False, "renderer": None, "path": args.snapshot}
    if snap["path"]:
        try:
            import mujoco
            os.makedirs(os.path.dirname(os.path.abspath(args.snapshot)), exist_ok=True)
            # 离屏 framebuffer 默认只有 640x480（model XML 的 offwidth/offheight），
            # 想要更大分辨率必须先把它调大，否则 Renderer 直接报
            # "Image width 1280 > framebuffer width 640"。
            sim.model.vis.global_.offwidth = max(int(sim.model.vis.global_.offwidth), 1280)
            sim.model.vis.global_.offheight = max(int(sim.model.vis.global_.offheight), 720)
            snap["renderer"] = mujoco.Renderer(sim.model, height=720, width=1280, max_geom=20000)
            snap["cam"] = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(snap["cam"])
        except Exception as exc:
            print("[viz] 离屏渲染不可用（%s: %s），跳过 --snapshot" % (type(exc).__name__, exc))
            snap["renderer"] = None

    def take_snapshot(fr):
        """把当前场景（含 3D 叠加）离屏渲染成 PNG。"""
        r = snap["renderer"]
        if r is None:
            return
        import mujoco
        cam = snap["cam"]
        cam.lookat[:] = fr.base_pos + np.array([0.0, 0.0, 0.15])
        cam.distance = 2.4
        cam.azimuth = 120.0
        cam.elevation = -18.0
        # Renderer 自己持有一个 MjvScene（r.scene）；update_scene 之后往里追加我们的标记
        r.update_scene(sim.data, camera=cam)
        scn = r.scene
        if scene is not None:
            try:
                scene.draw(scn, fr)          # 先让标准场景更新，再追加我们的标记
            except Exception as exc:
                print("[viz] 截图时叠加失败:", repr(exc)[:120])
        img = r.render()
        from PIL import Image
        Image.fromarray(img).save(snap["path"])
        print("[viz] 截图写入", snap["path"], img.shape)
        if image is not None:
            try:
                vi = image.viewport_image(img.shape[1], img.shape[0])
                if vi is not None:
                    Image.fromarray(np.asarray(vi[1], dtype=np.uint8)).save(
                        os.path.splitext(snap["path"])[0] + "_panel.png")
                    print("[viz] 仪表盘面板写入", os.path.splitext(snap["path"])[0] + "_panel.png")
            except Exception as exc:
                print("[viz] 面板截图失败:", repr(exc)[:120])

    from signal_io.node import Pacer
    pacer = Pacer(sim_dt, enabled=True)
    t0 = time.perf_counter()
    cmd = LowCmd().zero()
    st = sim.read_state()
    sim._last_state = st
    t = float(sim.time)
    hs = health.check(st, t)
    safe = LowCmd().zero()
    cmd = safe
    n_ctrl = 0
    frame = VizFrame()
    t_wall0 = time.perf_counter()
    exit_reason = ("达到 duration=%.1f s" % args.duration) if n_steps is not None else "窗口关闭/按 Q"
    try:
        for k in range(0, n_steps if n_steps is not None else 1 << 60):
            if viewer is not None:
                if not viewer.is_running() or state.get("quit"):
                    exit_reason = ("窗口被关闭 (viewer.is_running=False)"
                                   if not viewer.is_running() else "按 Q 退出")
                    print("[viz] 主循环退出: %s, t=%.2fs, 已走 %d 步"
                          % (exit_reason, float(sim.time), k))
                    break
                if state.get("pause"):
                    viewer.sync()
                    time.sleep(0.02)
                    continue
                if state.pop("reset", False):
                    if tmeta is not None:
                        spawn_on_terrain(sim, tmeta)
                    else:
                        sim.reset()
                    ctrl.reset()
                    health.reset(0.0)
                    supervisor.reset()
                    pacer = Pacer(sim_dt, enabled=True)
            # 500 Hz 物理, 但状态读取/健康/安全按 monitor_hz 分频 (默认 100 Hz):
            # 这三项每步合计约 0.5 ms, 若每步都做等于吃掉 25% 实时预算; 真机上也不需要 500 Hz。
            if k % mon_every == 0:
                st = sim.read_state()
                sim._last_state = st
                t = float(sim.time)
                hs = health.check(st, t)
                if k % decim == 0:
                    cmd = ctrl.update(st, t)
                    n_ctrl += 1
                safe = supervisor.process(cmd, hs, t, mon_every * sim_dt)
            wall = time.perf_counter() - t_wall0
            sim.write_cmd(safe)
            sim.step(1)

            if k % fig_every == 0 or k % sync_every == 0 or k % dash_every == 0:
                ps = pacer.stats()
                frame = collector.collect(t, wall, cmd=safe, health=hs, supervisor=supervisor,
                                          ctrl=ctrl, rtf=ps.get("rtf", 0.0),
                                          overruns=ps.get("overruns", 0), n_cmd=n_ctrl, n_state=n_ctrl)
                if dash is not None and k % dash_every == 0:
                    extra = {"io": "in-process", "robot": args.robot,
                             "scene": args.scene, "terrain": terrain}
                    if headless:
                        print(dash.render(frame, extra), flush=True)
                    else:
                        dash.render(frame, extra)      # 只为计数/统计，不打印
                if rec is not None and k % max(1, int(sync_every)) == 0:
                    rec.write(frame)
            if live is not None and k % plot_every == 0:
                live.update(frame)
                live.pump()
                # 自适应：Agg 每次全图重绘 ~150-200 ms，画太勤会把 RTF 拉下来。
                # 曲线是看趋势的，3 Hz 甚至 1.5 Hz 都够用，所以 RTF 一掉就自动减半。
                plot_check += 1
                if plot_check >= 250:          # 每 0.5 s 检一次
                    plot_check = 0
                    rtf_now = float(sim.time) / max(1e-6, time.perf_counter() - t0)
                    if rtf_now < 0.95 and plot_every < 1000:
                        plot_every *= 2
                        print("[viz] RTF %.3f 偏低 -> 曲线刷新率减半（约 %.1f Hz）"
                              % (rtf_now, 1.0 / (plot_every * sim_dt)))
            if snap["path"] and not snap["done"] and t >= snap["at"]:
                snap["done"] = True
                try:
                    take_snapshot(frame)
                except Exception as exc:
                    print("[viz] 截图失败:", repr(exc)[:200])
            if viewer is not None:
                if k % sync_every == 0:
                    t_sync = time.perf_counter()
                    if scene is not None and state.get("overlay", True):
                        try:
                            scene.draw(viewer.user_scn, frame)
                        except Exception as exc:
                            print("[viz] scene.draw 失败，关掉 3D 叠加:", repr(exc)[:120])
                            state["overlay"] = False
                    if hud is not None and state.get("hud", True) and k % hud_every == 0:
                        try:
                            hud.update(frame)
                            viewer.set_texts(hud.texts())
                        except Exception as exc:
                            print("[viz] HUD 失败，关掉:", repr(exc)[:120])
                            state["hud"] = False
                    if args.follow:
                        viewer.cam.lookat[:] = frame.base_pos + np.array([0.0, 0.0, 0.15])
                    try:
                        viewer.sync()
                    except Exception as exc:
                        # WSLg/llvmpipe 下 GL 上下文可能中途失效；以前这里会抛异常=看起来"闪退"。
                        exit_reason = "窗口 GL 同步异常: %r" % (exc,)
                        print("[viz] 窗口同步失败，主动退出:", repr(exc)[:200])
                        break
                    if (layout.get("figs") or layout.get("img")) and not layout.get("printed"):
                        layout["printed"] = True
                        vw, vh = viewer.viewport.width, viewer.viewport.height
                        area = 0
                        for r in (layout.get("figs") or []):
                            area += r[2] * r[3]
                        if layout.get("img"):
                            area += layout["img"][2] * layout["img"][3]
                        def fmt(r):
                            return "(%d,%d) %dx%d" % r
                        print("[viz] 面板布局：窗口 %dx%d | 曲线 %s | 仪表盘 %s | HUD 右上角 %s | 遮挡 %.1f%%"
                              % (vw, vh,
                                 " ".join(fmt(r) for r in (layout.get("figs") or [])) or "关",
                                 fmt(layout["img"]) if layout.get("img") else "关",
                                 "%dx%d" % (int(vw * 0.26), 90),
                                 100.0 * area / max(1, vw * vh)))
                    sync_ms.append(time.perf_counter() - t_sync)
                    # 画面刷新率自适应: 软件渲染/慢机器上 60 Hz 顶不住时逐级降到 30/20/15
                    if len(sync_ms) >= 20 and args.sync_hz > 15.0:
                        med_sync = float(np.median(sync_ms[-20:]))
                        if med_sync > 0.012 and sync_every < int(round(1.0 / (15.0 * sim_dt))):
                            args.sync_hz = max(15.0, args.sync_hz / 2.0)
                            sync_every = max(1, int(round(1.0 / (args.sync_hz * sim_dt))))
                            print("[viz] 单帧同步 %.1f ms 偏慢 -> 画面刷新降到 %.0f Hz"
                                  % (med_sync * 1e3, args.sync_hz))
                    # 帧率看门狗：软件渲染下每装一个面板，渲染线程的**每帧**开销就涨一截。
                    # 连续 20 帧同步中位超过阈值就逐级降级（仪表盘 -> 曲线 -> HUD），
                    # 保住"机器人看得清 + 还能操作"这个底线，并把原因打出来。
                    if len(sync_ms) >= 20 and degrade < 3:
                        med = float(np.median(sync_ms[-20:]))
                        if med > 0.12:
                            if degrade == 0 and state.get("img", False):
                                state["img"] = False
                                print("[viz] 帧时间中位 %.0f ms（≈%.1f fps）：先关掉仪表盘图像" % (med * 1e3, 1.0 / med))
                            elif degrade == 1 and state.get("figs", False):
                                state["figs"] = False
                                print("[viz] 仍然 %.0f ms：再关掉实时曲线" % (med * 1e3))
                            elif degrade == 2 and state.get("hud", True):
                                state["hud"] = False
                                print("[viz] 仍然 %.0f ms：最后关掉文字 HUD。"
                                      "要面板就得接受这个帧率，用 --sync-hz 再降。" % (med * 1e3))
                            degrade += 1
                if panels is not None and state.get("figs", False) and k % fig_every == 0:
                    try:
                        t_fig = time.perf_counter()
                        panels.update(frame)
                        # 曲线贴**左侧一竖条**（左边 26% 宽、上边 42% 高），
                        # 仪表盘在右下角；中间整片留给机器人。窗口小的时候按比例缩。
                        w, h = viewer.viewport.width, viewer.viewport.height
                        pw = max(160, int(w * 0.26))
                        ph = max(120, int(h * 0.42))
                        rects = panels.viewports(pw, ph, origin=(4, h - ph - 4))
                        viewer.set_figures(rects)
                        if not layout.get("figs"):
                            layout["figs"] = [(int(r.left), int(r.bottom), int(r.width), int(r.height))
                                              for r, _ in rects]
                        viz_cost["fig"] = 0.7 * viz_cost["fig"] + 0.3 * (time.perf_counter() - t_fig) * 1e3
                        if viz_cost["fig"] > args.panel_budget_ms and fig_every < 4000:
                            fig_every = min(4000, fig_every * 2)
                            print("[viz] 曲线面板单帧 %.0f ms > 预算 %.0f ms -> 刷新周期 x2 (约 %.1f Hz)"
                                  % (viz_cost["fig"], args.panel_budget_ms,
                                     1.0 / (fig_every * sim_dt)))
                    except Exception as exc:
                        print("[viz] FigurePanel 失败，关掉曲线:", repr(exc)[:120])
                        state["figs"] = False
                if image is not None and state.get("img", False) and k % img_every == 0:
                    try:
                        t_img = time.perf_counter()
                        image.update(frame)
                        vi = image.viewport_image(max(180, int(viewer.viewport.width * 0.26)),
                                                  max(140, int(viewer.viewport.height * 0.28)))
                        if vi is not None:
                            viewer.set_images([vi])
                            r0 = vi[0]
                            layout.setdefault("img", (int(r0.left), int(r0.bottom),
                                                      int(r0.width), int(r0.height)))
                        viz_cost["img"] = 0.7 * viz_cost["img"] + 0.3 * (time.perf_counter() - t_img) * 1e3
                        if viz_cost["img"] > args.panel_budget_ms and img_every < 8000:
                            img_every = min(8000, img_every * 2)
                            print("[viz] 仪表盘单帧 %.0f ms > 预算 %.0f ms -> 刷新周期 x2"
                                  % (viz_cost["img"], args.panel_budget_ms))
                    except Exception as exc:
                        print("[viz] ImagePanel 失败，关掉仪表盘:", repr(exc)[:120])
                        state["img"] = False
            pacer.tick()
    except KeyboardInterrupt:
        exit_reason = "Ctrl-C 中断"
        print("\n[viz] 用户中断 (Ctrl-C)")
    finally:
        if viewer is not None:
            viewer.close()
        # MuJoCo 的离屏 Renderer 在进程退出时销毁 GL context 会偶发 SIGSEGV（WSL/GLFW 已知问题），
        # 主动 close 掉，把问题挡在解释器退出之前。
        try:
            if snap.get("renderer") is not None:
                snap["renderer"].close()
                snap["renderer"] = None
        except Exception:
            pass

    print("[viz] 运行结束（%s）: 仿真 %.2f s / 墙钟 %.2f s"
          % (exit_reason, float(sim.time), time.perf_counter() - t_wall0))
    summary = {
        "robot": args.robot, "scene": args.scene, "terrain": terrain,
        "terrain_desc": (tmeta or {}).get("desc"), "controller": args.controller,
        "scan": (cfg.get("policy") or {}).get("height_scan_source"),
        "duration_s": round(float(sim.time), 2), "steps": int(sim.tick),
        "exit_reason": exit_reason,
        "final": {"x": round(float(sim.data.qpos[0]), 4), "y": round(float(sim.data.qpos[1]), 4),
                  "z": round(float(sim.base_height), 4),
                  "tilt_deg": round(float(np.degrees(max(abs(frame.rpy[0]), abs(frame.rpy[1])))), 2),
                  "upright": bool(sim.is_upright(30.0))},
        "pacer": pacer.stats(),
        "health": health.stats(),
        "supervisor": supervisor.stats(),
        "controller": ctrl.stats() if hasattr(ctrl, "stats") else {},
        "ground_source": getattr(ground, "name", "?"),
        "n_ctrl": n_ctrl,
        "viewer": None if viewer is None else {
            "sync_ms_median": round(float(np.median(sync_ms)) * 1e3, 2) if sync_ms else None,
            "sync_fps_median": round(1.0 / float(np.median(sync_ms)), 1) if sync_ms else None,
            "sync_n": len(sync_ms), "hud_on": bool(state.get("hud", False)),
            "overlay_on": bool(state.get("overlay", False)),
        },
        "overlay_rates_hz": {"sync": args.sync_hz, "hud": args.hud_hz,
                             "fig": (1.0 / (fig_every * sim_dt)) if panels is not None else None,
                             "img": (1.0 / (img_every * sim_dt)) if image is not None else None,
                             "monitor": args.monitor_hz},
        "viz_cost_ms": {k: round(v, 1) for k, v in viz_cost.items()},
        "plots": live.stats() if live is not None else {"mode": args.plots},
    }
    print("[viz] summary =", json.dumps(summary, ensure_ascii=False))
    if rec is not None:
        info = rec.close(summary)
        print("[viz] 记录写好了:", json.dumps(info, ensure_ascii=False))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
    # 装了 figures/images 之后，GLFW 窗口的 GL context 在退出时可能挂住上百秒
    # （实测 8 s 的 run 卡在退出 190 s）。summary / 记录都已经落盘并 flush，
    # 所以直接跳过解释器清理，避免"跑完了但进程不退出"。
    if live is not None:
        try:
            live.close()
        except Exception:
            pass
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
