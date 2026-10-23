"""配置加载 + IO 层切换。

借宇树 unitree_mujoco / unitree_ros2 的思路：**控制器只认 IO 层，不认环境**。
宇树是 "同一份 stand_go2，加不加网卡参数决定打仿真还是打真机"；
这里把这件事做成显式的 IO profile 表：

                  IOROS（rclpy + unitree_go）      IOSDK（unitree_sdk2py + cyclonedds）
    仿真环境      sim_ros                          sim_sdk
    真机 Go2      real_ros                         real_sdk

再加一个不依赖任何中间件的 sim_udp（裸 UDP），用于没有 ROS 2 的机器。
控制器代码一个字都不用改 —— 换的是 profile。
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- IO profiles
# 关键差异只有四个：走哪条 IO、域号、网卡、看门狗多严。
DEFAULT_IO_PROFILES: Dict[str, Dict[str, Any]] = {
    # 仿真：本机回环，domain 1（真机默认 0，别撞）
    "sim_udp": {
        "name": "udp", "host": "127.0.0.1", "cmd_port": 50001, "state_port": 50002,
        "crc_check": True, "watchdog_ms": 60.0, "realtime": True,
    },
    # ROS 2 层话题名与 unitree_ros2 官方例程一致：/lowcmd、/lowstate。
    # rmw_cyclonedds_cpp 会自动映射成 DDS 的 rt/lowcmd、rt/lowstate。
    # sim_ros 也打开 crc_check：让真机固件要求的 CRC 在仿真里就被验证到。
    # tick_dt: LowState.tick 每 +1 代表多少秒。仿真里 tick = 物理步数（2 ms/步），
    # 真机固件里 tick 按 1 ms 递增 —— 控制器的时间基（prepose/ramp/health）靠它。
    "sim_ros": {
        "name": "ros2", "domain_id": 1, "interface": "lo",
        "crc_check": True, "watchdog_ms": 60.0, "realtime": True,
        "cmd_topic": "/lowcmd", "state_topic": "/lowstate", "tick_dt": 0.002,
    },
    # raw DDS 后端（unitree_sdk2py）直接用固件话题名 rt/lowcmd、rt/lowstate。
    "sim_sdk": {
        "name": "dds", "domain_id": 1, "interface": "lo",
        "crc_check": False, "watchdog_ms": 60.0, "realtime": True,
        "cmd_topic": "rt/lowcmd", "state_topic": "rt/lowstate", "tick_dt": 0.002,
    },
    # 真机：domain 0 + 有线网卡；看门狗收紧到 20 ms（真机底层掉指令就卸力）
    "real_ros": {
        "name": "ros2", "domain_id": 0, "interface": "eth0",
        "crc_check": True, "watchdog_ms": 60.0, "realtime": True,
        "cmd_topic": "/lowcmd", "state_topic": "/lowstate", "tick_dt": 0.001,
    },
    "real_sdk": {
        "name": "dds", "domain_id": 0, "interface": "eth0",
        "crc_check": True, "watchdog_ms": 60.0, "realtime": True,
        "cmd_topic": "rt/lowcmd", "state_topic": "rt/lowstate", "tick_dt": 0.001,
    },
}

# 机型表：一个 target(IO) 只决定"打到哪"，机型决定"用哪份模型、什么力矩上限"。
# 两者正交：Go2 是训练侧模型（策略在它上面学的），Go2 是部署侧（真机）。
DEFAULT_ROBOTS: Dict[str, Dict[str, Any]] = {
    "go2": {"model_dir": "sim_env/models/go2", "scene": "scene.xml", "do_not_edit": True},
}

DEFAULTS: Dict[str, Any] = {
    "robot": "go2",
    "model_dir": "sim_env/models/go2",
    "scene": "scene.xml",
    "robots": {},
    # 以下四项由 robots.<name> 段覆盖（没写就用这里的默认值；go2 保持原有行为）
    "foot_soft_contact": False,      # 足端软接触（= 复刻 MJX 训练模型的接触参数）
    "solver_iterations": None,       # None = 沿用 MuJoCo 默认 (100)
    "solver_ls_iterations": None,    # None = 沿用 MuJoCo 默认 (50)
    "stand_height": None,            # None = 沿用模型 keyframe 的 z
    "target": "sim_udp",      # 默认：不需要 ROS 2 就能跑的仿真后端
    "io_profiles": {},        # 在 config/*.json 里覆盖或追加
    "sim_dt": 0.002,          # 500 Hz —— 真机底层也是 500 Hz
    "control_dt": 0.02,       # 50 Hz —— 上层控制器
    "ramp_rate": 200.0,       # N*m/s
    "hold_until_first_cmd": True,
    "controller": {"type": "stand", "kp": 60.0, "kd": 5.0, "ramp_s": 1.0},
}


def _merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: str | None = None, overrides: Dict[str, Any] | None = None,
                target: str | None = None) -> Dict[str, Any]:
    """读配置文件，然后把选中的 IO profile 展开成 cfg["transport"]。

    优先级（后面的覆盖前面的）: DEFAULTS < 配置文件 < IO profile < CLI 覆盖
    """
    cfg = dict(DEFAULTS)
    if path:
        with open(path, "r", encoding="utf-8") as f:
            cfg = _merge(cfg, json.load(f))

    cfg["robots"] = _merge(DEFAULT_ROBOTS, cfg.get("robots", {}))
    if overrides and overrides.get("robot"):
        cfg["robot"] = overrides["robot"]
    rb = cfg["robots"].get(cfg["robot"])
    if rb is None:
        raise ValueError("未知机型 %r，可选: %s" % (cfg["robot"], ", ".join(sorted(cfg["robots"]))))
    cfg["model_dir"] = rb.get("model_dir", cfg["model_dir"])
    cfg["scene"] = rb.get("scene", cfg["scene"])
    # 物理/姿态档位：写进 cfg 顶层，脚本直接读，不必各自去翻 robots 段
    for _key in ("foot_soft_contact", "solver_iterations", "solver_ls_iterations",
                 "stand_height", "scan_base", "home_q", "linvel_frame"):
        if _key in rb:
            cfg[_key] = rb[_key]

    cfg["io_profiles"] = _merge(DEFAULT_IO_PROFILES, cfg.get("io_profiles", {}))
    tgt = target or cfg.get("target") or "sim_udp"
    if tgt not in cfg["io_profiles"]:
        raise ValueError("未知 IO profile %r，可选: %s" % (tgt, ", ".join(sorted(cfg["io_profiles"]))))
    io = dict(cfg["io_profiles"][tgt])
    io["profile"] = tgt
    cfg["target"] = tgt
    # 配置文件里额外的 transport 段作为微调（例如换端口）
    io = _merge(io, cfg.pop("transport", {}) or {})
    cfg["transport"] = io
    cfg["watchdog_ms"] = float(io.get("watchdog_ms", 60.0))

    if overrides:
        overrides = {k: v for k, v in overrides.items() if v is not None}
        tr_over = overrides.pop("transport", None)
        cfg = _merge(cfg, overrides)
        if tr_over:
            cfg["transport"] = _merge(cfg["transport"], tr_over)

    # ---- 真机/仿真不同的健康阈值：真机 LowState 靠 DDS，偶发抖动比仿真大；
    #      按 target 覆盖 health（例如真机 stale_ms 放宽到 40 ms，避免一次 GC 就 L1 下蹲）。
    hp = cfg.get("health_profiles") or {}
    if tgt in hp and hp[tgt]:
        cfg["health"] = _merge(cfg.get("health") or {}, hp[tgt])

    # ---- 策略档：profile=auto 时按当前 robot 选；go2 档默认就是顶层 policy 块本身
    #      (所以 go2 的行为逐字不变)，go2 档必须显式写进 policy_profiles.go2。
    prof = cfg.pop("policy_profile", "auto")
    if prof == "auto":
        prof = cfg["robot"]
    profiles = cfg.get("policy_profiles") or {}
    if prof in profiles and profiles[prof]:
        cfg["policy"] = _merge(cfg.get("policy") or {}, profiles[prof])
    cfg["policy_profile"] = prof
    # train_robot 的观测/home 常量：obs 的 12:24 段必须减**策略训练机型**的 home
    tr_robot = (cfg.get("policy") or {}).get("train_robot") or cfg["robot"]
    trb = cfg["robots"].get(tr_robot) or {}
    cfg["train_robot"] = tr_robot
    for _key in ("scan_base", "home_q"):
        if _key in trb:
            cfg["_train_%s" % _key] = trb[_key]
    return cfg


def io_summary(cfg: Dict[str, Any]) -> str:
    """一行说明"这次到底在打谁、走哪条 IO"，启动时打印，省得搞混。"""
    io = cfg["transport"]
    name = io.get("name")
    if name == "ros2":
        kind = "IOROS(ROS2/cyclonedds)"
    elif name == "dds":
        kind = "IOSDK(unitree_sdk2py)"
    else:
        kind = "LOCAL(%s)" % name
    env = "真机" if cfg["target"].startswith("real") else "仿真"
    extra = ("domain=%s iface=%s" % (io.get("domain_id"), io.get("interface"))
             if name in ("ros2", "dds") else ("%s:%s" % (io.get("host"), io.get("cmd_port"))))
    return "%s -> %s  [%s]  看门狗 %.0f ms" % (cfg["target"], kind, env + " " + extra, cfg["watchdog_ms"])


def apply_io_env(cfg: Dict[str, Any]) -> Dict[str, str]:
    """ROS 2 / CycloneDDS 的环境变量必须在 rclpy.init / ChannelFactoryInitialize 之前设好。

    这里只设置、并返回一个 dict 方便调用方打印；真机那条（domain 0 + 网卡）就是靠它生效的。

    网卡写法很重要：旧写法 `<NetworkInterfaceAddress>` 在 CycloneDDS 11（ROS 2 Lyrical
    自带）里已被标记 **deprecated**（会打印警告、未来版本可能移除；本机实测中还出现过一次
    与 `AllowMulticast` 组合触发 `set_and_check_allow_multicast` assert 的情况，行为不稳定）。
    所以这里用 unitree_ros2 setup.sh 同款的新写法（无警告）：
        <Interfaces><NetworkInterface name="eth0" .../></Interfaces>
    """
    io = cfg["transport"]
    env: Dict[str, str] = {}
    if io.get("name") in ("ros2", "dds"):
        env["ROS_DOMAIN_ID"] = str(io.get("domain_id", 0))
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        iface = io.get("interface")
        if iface and iface != "lo":
            env["CYCLONEDDS_URI"] = (
                "<CycloneDDS><Domain><General><Interfaces>"
                '<NetworkInterface name="%s" priority="default" multicast="default" />'
                "</Interfaces></General></Domain></CycloneDDS>" % iface
            )
        for k, v in env.items():
            os.environ[k] = v
    return env


# 机型 -> 机器人本体 MJCF 文件名（注意 go2 是本项目转换出来的力矩版）
ROBOT_BODY_XML: Dict[str, str] = {"go2": "go2.xml"}


def robot_body_xml(cfg: Dict[str, Any], robot: str | None = None) -> str:
    """机器人本体 MJCF 的绝对路径（不含平面/障碍物，适合解析关节名与范围）。"""
    name = robot or cfg.get("robot", "go2")
    model_dir = cfg["model_dir"] if name == cfg.get("robot", "go2") else "sim_env/models/%s" % name
    return os.path.join(PROJECT_ROOT, model_dir, ROBOT_BODY_XML.get(name, name + ".xml"))


def robot_spec_of(cfg: Dict[str, Any]):
    """按 cfg["robot"] 造 RobotSpec（力矩上限 / 标称姿态因机型而异）。

    config 里的 robots.<name> 段可以覆盖**标称站姿**与**出生高度**：

        "robots": {"go2": {"stand_pose": [...12 个关节...], "stand_height": 0.32}}

    * 给了 stand_pose  -> 忽略 XML keyframe（Go2 官方 keyframe (0,0.9,-1.8) 在
      kp=20/kd=1 下站不住，见 config/go2_sim.json 的 _stand_pose_note）。
    * 没给（例如 go2） -> 完全保持原有行为：站姿取自模型 keyframe。
    关节顺序固定为 MOTOR_JOINTS = FR,FL,RR,RL × (hip,thigh,calf)。
    """
    from sim_env.robot import robot_spec
    name = cfg.get("robot", "go2")
    spec = robot_spec(name)
    rb = (cfg.get("robots") or {}).get(name) or {}
    sp = rb.get("stand_pose")
    if sp:
        spec.stand_pose = np.asarray(sp, dtype=np.float64)
        spec.stand_pose_from_xml = False
    if rb.get("stand_height") is not None:
        spec.stand_height = float(rb["stand_height"])
    return spec


def model_xml(cfg: Dict[str, Any]) -> str:
    return os.path.join(PROJECT_ROOT, cfg["model_dir"], cfg["scene"])


def robot_xml(cfg: Dict[str, Any]) -> str:
    return os.path.join(PROJECT_ROOT, cfg["model_dir"], cfg["robot"] + ".xml")
