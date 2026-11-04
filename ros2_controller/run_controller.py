#!/usr/bin/env python3
"""控制侧入口：50 Hz 算指令，500 Hz 发指令（零阶保持）+ 健康/安全分级。

===============================================================================
1. 这个进程在系统里的位置
===============================================================================

  仿真（ROS 2 模式）:
      sim_env/run_sim.py --publish-sensors（SimNode，500 Hz）
          --/lowstate-->  本进程（ControllerNode，50/500 Hz）
          <--/lowcmd----

  真机（real_ros）:
      Go2 固件 DDS rt/lowstate  --(rmw)-->  /lowstate
      本进程 --/lowcmd--> (rmw) --> DDS rt/lowcmd --> 机器人固件

  一句话：同一份控制器代码，换 --target 就能从仿真打到真机。
  IO 层怎么换见 signal_io/config.py 与 signal_io/transport/。

===============================================================================
2. 时序（为什么是 50 Hz + 500 Hz 两层）
===============================================================================

  500 Hz（每 2 ms，ControllerNode.run）:
      - recv_state() 取最新 /lowstate
      - 健康层 HealthMonitor 判 L0..L3
      - 每 10 拍（20 ms）跑一次策略推理（50 Hz）
      - 安全监督器按健康等级出指令（track/crouch/damping/estop）
      - publish_cmd() 把最近一条 LowCmd 发出去（零阶保持）
  50 Hz（每 20 ms，PolicyController.update）:
      - 243 维观测 -> numpy 策略前向 -> 12 维关节位置目标
      - 真机上电时的"起手"也在这里：pre-pose（先摆到标称站姿）-> ramp

  真机与仿真的唯一区别是数据来自哪里；50/500 Hz 时序、kp/kd、观测/动作契约完全一致。

===============================================================================
3. target 与控制器选择
===============================================================================

  --target sim_udp     裸 UDP（本机双进程，无 ROS 2 依赖）
  --target sim_ros     ROS 2 + MuJoCo（与真机同构的节点图）
  --target real_ros    ROS 2 + 真机 Go2（推荐路径；domain 0 + 有线网卡）
  --target sim_sdk / real_sdk   raw DDS（unitree_sdk2py；Python 3.14 不可用）

  --controller policy  导出的 brax PPO 策略（obs=WalkObsBuilder，action=12 维关节位置）
  --controller stand   关节 PD 顶到标称站姿（真机首次上机用这个）
  --controller zero    零力矩（吊起来确认状态链路用）
  --controller lqr_stub 占位（演示用）

  注意：策略是按训练机型（train_robot）学的，关节范围、obs home、scan base
  都必须取训练机型的常量，不能按当前仿真机型取（见 build_policy_controller）。

===============================================================================
4. 真机启动顺序（与 real_env/README.md 一致）
===============================================================================

  1) 只读自检：scripts/go2_real_check.py --target real_ros
  2) 吊起、L2+R2 进 debug mode（释放 sport mode）
  3) --controller zero  确认 /lowstate 通、电机能进伺服
  4) --controller stand 看关节平滑走到站姿
  5) 落地后 --controller policy --scan leg_fk（平地安全档）
  6) 退出/异常时本进程会先发 100 ms 阻尼再断流（--no-damping 可关）

例:
    scripts/run_with_ros2.sh ros2_controller/run_controller.py \
        --target real_ros --controller zero --scan leg_fk --duration 10
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# 单线程 BLAS: 1536-768 策略前向在默认 OpenBLAS 线程数下实测 12 ms/次（还会占满 12~16 核），
# 单线程 0.21 ms/次。50 Hz 预算 20 ms，多线程小矩阵纯属开销，这里钉死单线程。
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from signal_io.config import (PROJECT_ROOT, apply_io_env, io_summary, load_config,  # noqa: E402
                               model_xml, robot_body_xml, robot_spec_of)
from ros2_controller.controller import (PolicyController, StandController,  # noqa: E402
                                    WbcLqrStub, ZeroController)
from signal_io.node import ControllerNode  # noqa: E402
from sim_env.robot import MOTOR_JOINTS, Go2Sim  # noqa: E402
from signal_io.transport import make_transport  # noqa: E402


def joint_ranges(xml_path: str):
    """从 MJCF 里读 12 个关节的 ctrlrange（用于把策略动作 clip 回训练范围）。

    Args:
        xml_path: 带 <position> 执行器的 MJCF（用 scene，不用本体 xml ——
                  本体 xml 里的 sensor geom2="floor" 在缺平面时会报错）。
    Returns:
        (lo, hi) 两个 (12,) 数组，顺序固定为 MOTOR_JOINTS = FR,FL,RR,RL × (hip,thigh,calf)。
    """
    import mujoco

    m = mujoco.MjModel.from_xml_path(xml_path)
    lo, hi = [], []
    for j in MOTOR_JOINTS:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        r = m.jnt_range[jid]
        lo.append(float(r[0]))
        hi.append(float(r[1]))
    return np.array(lo), np.array(hi)


def build_policy_controller(cfg: dict, sim: Go2Sim, cli_cmd=None, linvel=None,
                            scan_holder=None):
    """装配策略控制器（观测构建器 + 策略网络 + 高度扫描 + 速度估计 + 安全参数）。

    Args:
        cfg: load_config() 的产物（policy/robots/perception 已合并）。
        sim: Go2Sim —— 这里只借它的模型/站姿/力矩上限做静态元数据，不步进物理。
        cli_cmd: --cmd 覆盖速度指令 (vx, vy, wz)。
        linvel: --linvel 覆盖速度来源（oracle/estimator；独立进程里 oracle 会退回 estimator）。
        scan_holder: --scan topic 时由调用方挂的订阅缓存 {"data": np.ndarray|None, "n": int}。
    Returns:
        PolicyController 实例。
    Raises:
        SystemExit: 配置缺 npz / obs_dim 与 scan 几何不匹配等无法安全继续的情况。
    """
    from signal_io.estimator import LegKinematics, LegOdometry
    from ros2_controller.obs import WalkObsBuilder, WalkObsConfig
    from ros2_controller.policy import NumpyMLPPolicy, load_policy_meta

    # ---- 1) 策略网络：npz（权重 + obs_layout 元数据）纯 numpy 前向，不需要 JAX ----
    p = dict(cfg.get("policy") or {})
    npz = p.get("npz")
    if not npz:
        raise SystemExit("--controller policy 需要 config 里的 policy.npz")
    npz_path = npz if os.path.isabs(npz) else os.path.join(PROJECT_ROOT, npz)
    policy = NumpyMLPPolicy.from_npz(npz_path)
    meta = load_policy_meta(npz_path)
    if meta and int(meta.get("obs_dim", policy.obs_dim)) != policy.obs_dim:
        raise SystemExit("策略 meta 的 obs_dim 与权重不一致")

    command = list(cli_cmd) if cli_cmd is not None else list(p.get("command", (0.5, 0, 0)))

    # ---- 2) 训练机型常量：obs 的 home / scan.base / 线速度坐标系都按它取 ----
    # 训练机型是「策略在哪个机型上学的」，与当前被控机型可能不同（移植场景）。
    # 取错会让观测整体偏置：obs 12:24 段 = q - home，scan 特征 = clip(z-h-base)*5。
    train_robot = p.get("train_robot", cfg.get("robot", "go2"))
    tr_robot_cfg = (cfg.get("robots") or {}).get(train_robot) or {}
    home_q = tr_robot_cfg.get("home_q")            # Go2 训练 home（12 维，FR,FL,RR,RL）
    scan_base = tr_robot_cfg.get("scan_base")      # Go2 -> 0.268（标称站高）

    # 观测 0:3 的线速度坐标系也必须按训练机型取（Go2 = body 系速度）。
    linvel_frame = tr_robot_cfg.get("linvel_frame")
    if linvel_frame is None:
        linvel_frame = "body"

    obs_kwargs = {"command": tuple(float(x) for x in command),
                  "linvel_frame": str(linvel_frame)}
    if scan_base is not None:
        obs_kwargs["scan_base"] = float(scan_base)

    # ---- 3) 从 obs_dim 反推 scan 几何 + 历史帧数（策略档不同，维度不同）----
    # 布局: 48 base + scan_n + 8 phase + hist((frames-1)*(48+8))
    # 例：243 = 48 + 187 + 8；某些历史帧策略会额外带 (frames-1)*56。
    phase_on = bool(p.get("include_gait_phase", True))
    _frame_dim = 48 + (8 if phase_on else 0)
    sc0 = dict(p.get("scan") or {})
    nx0, ny0 = int(sc0.get("nx", 0)), int(sc0.get("ny", 0))
    hist_frames = 1
    if nx0 > 0 and ny0 > 0:
        # 优先用 config 里显式给的 scan 几何（历史帧策略必需，否则无法区分）
        scan_n = nx0 * ny0
        _hist_dim = int(policy.obs_dim) - 48 - (8 if phase_on else 0) - scan_n
        if _hist_dim > 0 and _hist_dim % _frame_dim == 0:
            hist_frames = _hist_dim // _frame_dim + 1
        elif _hist_dim != 0:
            raise SystemExit(
                "obs_dim=%d 与 scan %dx%d + phase%s 不匹配 (剩余 %d 维既不是 0 也不是"
                "历史帧整数倍)" % (policy.obs_dim, nx0, ny0,
                                   "on" if phase_on else "off", _hist_dim))
    else:
        scan_n = int(policy.obs_dim) - 48 - (8 if phase_on else 0)
        if scan_n <= 0 and phase_on:
            # 老策略可能没相位段（v5 之前的 235 维档）-> 再试一次
            phase_on, scan_n = False, int(policy.obs_dim) - 48
    if not phase_on:
        obs_kwargs["include_gait_phase"] = False
    if hist_frames > 1:
        obs_kwargs["history_frames"] = int(hist_frames)
        print("[ctrl] obs 历史帧 = %d (obs_dim=%d, hist %d 维)"
              % (hist_frames, policy.obs_dim,
                 (hist_frames - 1) * _frame_dim))
    if scan_n > 0:
        sc = dict(p.get("scan") or {})
        nx, ny = int(sc.get("nx", 0)), int(sc.get("ny", 0))
        if nx * ny != scan_n:
            # 兼容两种历史扫描几何：v6 之前 7x5=35，当前 17x11=187。
            if scan_n == 187:
                nx, ny = 17, 11
            elif scan_n == 35:
                nx, ny = 7, 5
            else:
                raise SystemExit(
                    "策略 obs_dim=%d -> scan_n=%d, 无法从 config 推断扫描几何; "
                    "请在 policy_profiles[%s].scan 里显式给 nx/ny"
                    % (policy.obs_dim, scan_n, cfg.get("policy_profile")))
        if scan_n == 187:
            x_def, y_def = (-0.4, 1.2), (-0.5, 0.5)
        else:
            x_def, y_def = (-0.3, 0.9), (-0.4, 0.4)
        obs_kwargs.update(
            scan_nx=nx, scan_ny=ny,
            scan_x_range=tuple(sc.get("x", x_def)),
            scan_y_range=tuple(sc.get("y", y_def)))
        print("[ctrl] scan 几何 = %dx%d=%d 点, x%s y%s (由 obs_dim=%d 反推)"
              % (nx, ny, nx * ny, obs_kwargs["scan_x_range"],
                 obs_kwargs["scan_y_range"], policy.obs_dim))
    obs_cfg = WalkObsConfig(**obs_kwargs)
    builder = WalkObsBuilder(obs_cfg, home_q=None if home_q is None else np.asarray(home_q, dtype=np.float64))

    # ---- 4) 高度扫描来源（决定 obs[48:235] 从哪来）----
    #   topic   : 订阅 /perception/height_scan（真机/ROS 2 推荐）
    #   raycast : 同进程 SimLidar + HeightMapper（只适合单进程仿真）
    #   leg_fk  : 接触腿正运动学估一个标量，所有点填同一个值（平地近似，真机安全档）
    #   nominal : 全 0（标称平地先验）
    height_scan_fn = None
    hsrc = str(p.get("height_scan_source", "leg_fk"))
    if hsrc == "topic":
        if scan_holder is None:
            raise SystemExit("--scan topic 需要 ROS 2 transport 与 /perception/height_scan 订阅")

        def height_scan_fn(state, t, _h=scan_holder):  # noqa: ANN001
            data = _h.get("data")
            return None if data is None else np.asarray(data, dtype=np.float64)

        print("[ctrl] 感知链路   = topic /perception/height_scan (无数据时回退 leg_fk)")
    elif hsrc == "raycast":
        from signal_io.perception import HeightMapper, SimLidar
        perc = dict(cfg.get("perception") or {})
        mapper = HeightMapper(
            nx=obs_cfg.scan_nx, ny=obs_cfg.scan_ny,
            x_range=obs_cfg.scan_x_range, y_range=obs_cfg.scan_y_range,
            base=obs_cfg.scan_base, clip_m=obs_cfg.scan_clip_m,
            scale=obs_cfg.scan_scale,
            resolution=float(perc.get("map_resolution", 0.05)),
            decay_s=float(perc.get("map_decay_s", 1.0)),
            fill_local=bool(perc.get("fill_local", True)))
        lidar = SimLidar(
            sim, n_rays=int(perc.get("n_rays", 2000)),
            hfov=tuple(perc.get("hfov", (0.0, 2.0 * np.pi))),
            vfov=tuple(perc.get("vfov", (-1.0472, 0.5236))),
            max_range=float(perc.get("max_range", 8.0)),
            height_above_ground=float(perc.get("height_above_ground", 0.30)),
            mount_body=perc.get("mount_body"),
            min_range=float(perc.get("min_range", 0.10)),
            self_occlusion=bool(perc.get("self_occlusion", True)),
            seed=int(perc.get("seed", 0)),
            multiray=bool(perc.get("use_multiray", True)))

        # 感知频率与 50 Hz 控制解耦：真机 L1 点云也就 10~20 Hz，每拍都重扫既不符合
        # 真机，也没必要（高程图有 1 s 记忆）。rate_hz<=0 表示每拍都扫（老行为）。
        rate_hz = float(perc.get("rate_hz", 0.0) or 0.0)
        _last_scan_t = [-1.0e18]

        def height_scan_fn(state, t, _m=mapper, _l=lidar, _sim=sim):  # noqa: ANN001
            min_dt = 0.0 if rate_hz <= 0.0 else 1.0 / rate_hz
            if float(t) - _last_scan_t[0] >= min_dt - 1e-9:
                _m.update(_l.sample(), t)
                _last_scan_t[0] = float(t)
            return _m.scan(_sim.data.qpos[:3], _sim.data.qpos[3:7])

        print("[ctrl] 感知链路   = SimLidar(自遮挡=%s multiray=%s mount=%s vfov=%s) -> "
              "HeightMapper(%dx%d fill_local=%s decay=%.1fs) rate=%s"
              % (lidar.self_occlusion, lidar.multiray, lidar.mount_body, lidar.vfov,
                 mapper.nx, mapper.ny, mapper.fill_local, mapper.decay_s,
                 ("%.1f Hz" % rate_hz) if rate_hz > 0 else "每拍"))

    # ---- 5) 动作 clip 范围：按训练机型取（不是当前仿真机型）----
    # 用 scene 而不是本体 xml：本体 xml 里的 <sensor geom2="floor"> 在缺平面时会报错。
    _tr_rb = (cfg.get("robots") or {}).get(train_robot) or {}
    train_xml = os.path.join(PROJECT_ROOT, _tr_rb.get("scene_flat")
                             or os.path.join(cfg["model_dir"], "scene.xml"))
    jlo, jhi = joint_ranges(train_xml)

    # ---- 6) 状态估计：腿式里程计（真机没有可信的 base 线速度）----
    # estimator 按当前被控机型建（估的是这台机器人的速度；训练机型只影响 obs 的常量）。
    kin = LegKinematics(robot_body_xml(cfg))
    est = LegOdometry(kin, contact_force_thresh=float(p.get("contact_force_thresh", 30.0)))

    # 速度来源：仿真里 obs 0:3 训练时是真值（oracle），真机只能走 leg odometry。
    # 注意 run_controller 永远跑在独立进程里，本地 Go2Sim 不会步进；oracle 在这里是
    # 一份静止快照，喂给策略等于错误观测，所以统一退回 estimator。
    linvel_mode = str(linvel or p.get("linvel_source", "estimator"))
    if linvel_mode == "oracle":
        print("[ctrl] 注意：独立进程无法提供仿真真值，--linvel oracle -> estimator")
        linvel_mode = "estimator"
    linvel_fn = None
    if linvel_mode == "oracle":
        def linvel_fn(state, t, _sim=sim):        # noqa: ANN001
            return np.array(_sim.data.qvel[0:3], dtype=np.float64)

    # ---- 7) 装控制器：kp/kd 必须与训练执行器等价（Go2: kp=20, kd=1）----
    ctrl = PolicyController(
        policy, builder, kp=float(p.get("kp", 20.0)), kd=float(p.get("kd", 1.0)),
        command=command, estimator=est, kin=kin, linvel_fn=linvel_fn,
        joint_lo=jlo, joint_hi=jhi,
        action_clip=bool(p.get("action_clip_to_train_joint_range", True)),
        ramp_s=float(p.get("ramp_s", 0.0)), start_delay_s=float(p.get("start_delay_s", 0.0)),
        contact_force_thresh=float(p.get("contact_force_thresh", 30.0)),
        foot_radius=float(p.get("foot_radius", 0.023)),
        height_scan_fn=height_scan_fn,
        height_scan_mode=(hsrc if hsrc in ("nominal", "leg_fk") else "leg_fk"),
        clearance_hz=float(p.get("clearance_hz", 3.0)),
        prepose_s=float(p.get("prepose_s", 1.5)),
        prepose_kp=float(p.get("prepose_kp", p.get("kp", 20.0))),
        prepose_kd=float(p.get("prepose_kd", p.get("kd", 1.0))))
    if hsrc == "raycast":
        ctrl.perception = {"lidar": lidar, "mapper": mapper, "rate_hz": rate_hz}
    print("[ctrl] 策略      =", os.path.relpath(npz_path, PROJECT_ROOT),
          "| obs_dim %d | 动作 %d 维关节位置目标" % (policy.obs_dim, policy.action_dim))
    print("[ctrl] 训练机型  =", train_robot, "| 策略档 =", cfg.get("policy_profile"),
          "| 动作夹紧范围", np.round(jlo, 3), "->", np.round(jhi, 3))
    print("[ctrl] obs home  =", np.round(builder.home_q, 3),
          "| height_scan.base =", obs_cfg.scan_base)
    print("[ctrl] 速度来源  =", linvel_mode,
          "| 高程来源 =", str(p.get("height_scan_source", "leg_fk")))
    print("[ctrl] 指令      =", command)
    return ctrl


def build_controller(name: str, cfg: dict, sim: Go2Sim, cli_cmd=None, linvel=None,
                     scan_holder=None):
    """按 --controller 选择控制器实现（真机首次上机建议 zero/stand）。

    zero      : 零力矩，确认状态链路 + 电机能进伺服
    stand     : 关节 PD 顶到标称站姿（ramp 可配），确认站姿与接触
    policy    : 导出的 PPO 策略
    lqr_stub  : 占位（演示用，不参与控制质量评估）
    """
    c = cfg.get("controller", {})
    if name == "policy":
        return build_policy_controller(cfg, sim, cli_cmd, linvel=linvel,
                                       scan_holder=scan_holder)
    if name == "zero":
        return ZeroController()
    if name == "lqr_stub":
        return WbcLqrStub(sim.stand_pose)
    return StandController(sim.stand_pose, kp=c.get("kp", 60.0), kd=c.get("kd", 5.0),
                           ramp_s=c.get("ramp_s", 1.0))


def main() -> int:
    """入口：配置 -> 环境 -> 传输 -> 控制器/健康/安全 -> 主循环 -> 收尾 -> summary。"""
    # ------------------------------------------------------------ 1) 命令行
    ap = argparse.ArgumentParser(
        description="Go2 控制侧（50 Hz 策略 + 500 Hz 指令；仿真/真机同一份代码）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"),
                    help="配置文件（io_profiles / policy_profiles / health / safety）")
    ap.add_argument("--controller", default="stand", choices=["zero", "stand", "lqr_stub", "policy"],
                    help="zero=零力矩 / stand=站姿 PD / policy=策略 / lqr_stub=占位")
    ap.add_argument("--robot", default=None, choices=["go2"],
                    help="当前被控机型（真机是 go2）")
    ap.add_argument("--cmd", type=float, nargs=3, default=None,
                    help="速度指令 (vx m/s, vy m/s, wz rad/s)，覆盖配置里的 policy.command")
    ap.add_argument("--linvel", default=None, choices=["oracle", "estimator"],
                    help="速度来源：oracle=仿真真值（独立进程会自动退回 estimator）；"
                         "estimator=腿式里程计（真机路径）")
    ap.add_argument("--scan", default=None, choices=["nominal", "leg_fk", "raycast", "topic"],
                    help="height scan 来源：topic=订阅 /perception/height_scan（真机/ROS 2）；"
                         "leg_fk=接触腿运动学（真机安全档）；nominal=标称平地；"
                         "raycast=同进程 raycast（只适合单进程仿真）")
    ap.add_argument("--scan-topic", default="/perception/height_scan",
                    help="--scan topic 时订阅的高度扫描话题（std_msgs/Float32MultiArray，187 点）")
    ap.add_argument("--clearance-hz", type=float, default=None,
                    help="离地高度低通截止频率 (Hz)")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="运行时长（秒）；0 = 直到 Ctrl-C。没收到状态时也按墙钟计时退出")
    ap.add_argument("--target", default=None,
                    help="IO profile：sim_udp / sim_ros / real_ros / sim_sdk / real_sdk")
    ap.add_argument("--transport", default=None, choices=["loopback", "udp", "dds", "ros2"],
                    help="[兼容] 只改 IO 名字，其余参数仍取自 profile；建议用 --target")
    ap.add_argument("--no-realtime", action="store_true", help="不用墙钟节拍（测试用，会全速跑）")
    ap.add_argument("--no-health", action="store_true", help="关掉健康层（退回纯直通）")
    ap.add_argument("--no-supervisor", action="store_true", help="关掉安全监督器")
    ap.add_argument("--json", default=None, help="把 summary 落盘到该路径（排查真机问题用）")
    ap.add_argument("--no-damping", action="store_true",
                    help="真机退出时不发阻尼收尾（默认会发 100 ms kp=0/kd=8）")
    ap.add_argument("--publish-debug", action="store_true",
                    help="发布 /go2/controller/{obs,action,info}（观察器需要；仅 ROS 2）")
    ap.add_argument("--policy-profile", default=None,
                    help="策略档：auto（默认，按 --robot 选）/ go2 / go2_v33d / go2_v29c ...")
    args = ap.parse_args()

    # ------------------------------------------------------------ 2) 配置 + scan 来源
    cfg = load_config(args.config, target=args.target,
                      overrides={"robot": args.robot, "policy_profile": args.policy_profile})
    if args.transport:
        cfg["transport"]["name"] = args.transport
    if args.scan:
        cfg.setdefault("policy", {})["height_scan_source"] = args.scan
    else:
        # run_controller 是独立进程：raycast 只能读到本地未步进的模型快照。
        # 没有显式 --scan raycast 时，ROS 2 路径默认退回 leg_fk；launch 会显式传 topic。
        _hsrc = str((cfg.get("policy") or {}).get("height_scan_source", "leg_fk"))
        if _hsrc == "raycast" and cfg["transport"]["name"] in ("ros2", "dds"):
            print("[ctrl] 注意：独立控制器不能用 raycast（本地模型不步进），改用 leg_fk；"
                  "需要真感知请加 --scan topic")
            cfg.setdefault("policy", {})["height_scan_source"] = "leg_fk"
    if args.clearance_hz is not None:
        cfg.setdefault("policy", {})["clearance_hz"] = args.clearance_hz

    # ------------------------------------------------------------ 3) 模型元数据
    # 只借模型拿标称站姿/关节范围，不参与仿真。
    # 注意：安全监督器的「收腿下蹲姿态」= stand_pose + crouch_depth，所以它必须跟
    # config 的标称站姿一致，否则 L1/L2 会把机器人甩到另一个姿态。
    _rb_cfg = (cfg.get("robots") or {}).get(cfg["robot"]) or {}
    sim = Go2Sim(model_xml(cfg),
                 spec=(robot_spec_of(cfg) if _rb_cfg.get("stand_pose") else None),
                 sim_dt=cfg["sim_dt"])

    # ------------------------------------------------------------ 4) 环境 -> 传输
    # apply_io_env 必须在 make_transport（内部 rclpy.init）之前：
    # ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI 都是进程初始化时读取。
    env = apply_io_env(cfg)
    tr = make_transport(cfg["transport"], role="controller")

    # ------------------------------------------------------------ 5) 高度扫描订阅
    # /perception/height_scan（187 点 Float32MultiArray）由 go2_perception 发布；
    # 控制器没有数据时回退 leg_fk（PolicyController._height_scan 里实现）。
    scan_holder = None
    if (cfg.get("policy") or {}).get("height_scan_source") == "topic":
        if cfg["transport"]["name"] != "ros2":
            raise SystemExit("--scan topic 需要 --target sim_ros / real_ros（当前 %r）"
                             % cfg["transport"]["name"])
        from std_msgs.msg import Float32MultiArray
        scan_holder = {"data": None, "n": 0}

        def _on_scan(msg, _h=scan_holder):  # noqa: ANN001
            _h["data"] = np.asarray(msg.data, dtype=np.float64)
            _h["n"] += 1

        tr.create_subscription(Float32MultiArray, args.scan_topic, _on_scan)
        print("[ctrl] scan 订阅 =", args.scan_topic)

    # ------------------------------------------------------------ 6) 策略/控制器
    ctrl = build_controller(args.controller, cfg, sim, cli_cmd=args.cmd,
                            linvel=args.linvel, scan_holder=scan_holder)

    # ------------------------------------------------------------ 7) 健康层 + 安全层
    # 健康层：LowState 断流/NaN/姿态/电机 lost+温度+bit_flag -> L0..L3。
    # 安全层：按等级选 track/crouch/damping/estop，并做关节/力矩/斜率限幅。
    # 两者都可单独关闭，方便定位是"控制器"还是"安全层"的问题。
    health = supervisor = None
    if not args.no_health:
        from ros2_controller.health import HealthMonitor
        health = HealthMonitor(cfg)
    if not args.no_supervisor:
        from ros2_controller.safety import SafetySupervisor
        s = cfg.get("safety", {})
        supervisor = SafetySupervisor(
            sim.stand_pose, limits=sim.torque_limits, ramp_rate=cfg.get("ramp_rate", 200.0),
            mode_kp=float(s.get("mode_kp", 20.0)), mode_kd=float(s.get("mode_kd", 1.0)),
            crouch_depth=float(s.get("crouch_depth", 0.35)), crouch_s=float(s.get("crouch_s", 2.0)),
            damping_kd=float(s.get("damping_kd", 8.0)))
        if s.get("l2_policy"):
            supervisor.l2_policy = str(s["l2_policy"])

    # ------------------------------------------------------------ 8) 调试发布（可选）
    debug = None
    if args.publish_debug:
        if cfg["transport"]["name"] != "ros2":
            raise SystemExit("--publish-debug 需要 ROS2 transport（sim_ros / real_ros）")
        from signal_io.ros2_debug import ControllerDebugPublisher
        debug = ControllerDebugPublisher(tr)
        print("[ctrl] debug 话题 = /go2/controller/{obs,action,info}")

    # ------------------------------------------------------------ 9) 主循环
    node = ControllerNode(tr, ctrl, sim_dt=cfg["sim_dt"], control_dt=cfg["control_dt"],
                          health=health, supervisor=supervisor,
                          on_control=(debug.publish if debug is not None else None))
    print("[ctrl] io         =", io_summary(cfg))
    if env:
        print("[ctrl] env        =", env)
    print("[ctrl] controller =", ctrl.name, "| 控制周期 %.1f ms | 健康层 %s | 安全监督器 %s"
          % (cfg["control_dt"] * 1e3, "on" if health else "off", "on" if supervisor else "off"))

    # ------------------------------------------------------------ 10) 收尾
    try:
        summary = node.run(args.duration, realtime=not args.no_realtime)
    except KeyboardInterrupt:
        summary = {"interrupted": True}
    finally:
        # 真机退出收尾：先发一段阻尼再断流。官方没有公开「lowcmd 断流后固件怎么处理」，
        # 主动阻尼比「保持最后一条策略指令」安全。
        if cfg["target"].startswith("real_") and not args.no_damping:
            try:
                if node.emergency_stop():
                    print("[ctrl] 退出收尾：已发 100 ms 阻尼指令（kp=0, kd=8）")
            except Exception as exc:  # 收尾失败不能掩盖 summary
                print("[ctrl] 阻尼收尾异常:", repr(exc)[:160])
        try:
            tr.close()
        except Exception:
            pass

    # ------------------------------------------------------------ 11) summary
    if summary.get("n_state", 0) == 0:
        print("[ctrl] ⚠ 整个运行期没有收到任何 /lowstate：检查网线 / 网段 / ROS_DOMAIN_ID / "
              "CYCLONEDDS_URI 网卡 / 话题名（应为 /lowstate）")
    summary["io"] = {"target": cfg["target"], "profile": dict(cfg["transport"])}
    summary["robot"] = cfg["robot"]
    summary["cmd_vel"] = list(args.cmd) if args.cmd is not None else (cfg.get("policy") or {}).get("command")
    if debug is not None:
        summary["debug"] = debug.stats()
    print("[ctrl] summary =", json.dumps(summary, ensure_ascii=False))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
