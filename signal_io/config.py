"""配置加载 + IO 层切换（仿真 / 真机只差一个 --target）。

===============================================================================
1. 设计来源
===============================================================================

借宇树 unitree_mujoco / unitree_ros2 的思路：控制器只认 IO 层，不认环境。
宇树是「同一份 stand_go2，加不加网卡参数决定打仿真还是打真机」；本项目把这件事
做成显式的 IO profile 表：

                ROS 2 后端（rclpy + unitree_go）     raw DDS 后端（unitree_sdk2py）
  仿真环境      sim_ros                              sim_sdk
  真机 Go2      real_ros（推荐）                      real_sdk（Python 3.14 不可用）

  另外两个不依赖中间件的后端：loopback（同进程）、sim_udp（裸 UDP）。
  控制器代码一个字都不用改 —— 换的是 profile。

===============================================================================
2. 一次 load_config() 发生了什么（优先级从低到高）
===============================================================================

    DEFAULTS
      ↓ 被 config/go2_sim.json 覆盖（深度 merge）
    配置文件
      ↓ 选中的 io_profile 展开成 cfg["transport"]
    IO profile
      ↓ CLI overrides（--target / --robot / --policy-profile / transport 微调）
    CLI
      ↓ 按 target 覆盖 health_profiles（真机阈值放宽）
    health
      ↓ 按 policy_profile 合并策略块（kp/kd/obs 维度/scan 几何…）
    policy

  最终 cfg["transport"] 里已经是「可以直接喂给 make_transport()」的扁平参数：
      {"name": "ros2", "domain_id": 0, "interface": "eth0",
       "cmd_topic": "/lowcmd", "state_topic": "/lowstate",
       "tick_dt": 0.001, "crc_check": true, "profile": "real_ros", ...}

===============================================================================
3. 真机相关字段（看这一节就够）
===============================================================================

  target=real_ros：
      name=ros2            走 rclpy + rmw_cyclonedds_cpp
      domain_id=0          机器人固件固定 domain 0（仿真用 1，别串台）
      interface=eth0       连机器人的有线网卡（必须改成本机实际网卡名）
      cmd_topic=/lowcmd    ROS 2 话题名（不要写 rt/；rmw 会补）
      state_topic=/lowstate
      tick_dt=0.001        真机 LowState.tick 按 1 ms 递增
      crc_check=true       固件校验 LowCmd.crc（必须）
      watchdog_ms=60       只对仿真侧 SafetyLimiter 有意义，真机不靠它

  watchdog_ms 的真相（容易误解）：
      它只被 SimNode 的 SafetyLimiter 使用（仿真里收不到 LowCmd 就清零力矩）。
      真机方向没有「指令掉线自动卸力」的软件看门狗；保护靠
      HealthMonitor（LowState 断流按墙钟升级 L1/L2/L3）+ SafetySupervisor
      切阻尼/急停，阈值在 health_profiles.real_*。

===============================================================================
4. apply_io_env()：必须在 rclpy.init() 之前调用
===============================================================================

  ROS 2 / CycloneDDS 在进程初始化时读环境变量，之后再改无效：
      ROS_DOMAIN_ID=0                         真机 domain
      RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   必须用 CycloneDDS，才能和固件 DDS 互通
      CYCLONEDDS_URI=【Interfaces】…          多网卡时绑定有线网卡（否则可能走 Wi-Fi）
      ROS_LOCALHOST_ONLY 不能设/必须 0        否则只走回环，真机必然不通

  调用顺序（run_controller / run_sim / perception_node / view_ros2 都遵守）：
      cfg = load_config(..., target=...)
      env = apply_io_env(cfg)      # 先设环境
      tr  = make_transport(...)    # 这里才 rclpy.init()

===============================================================================
5. 常见问题
===============================================================================

  * 真机收不到 /lowstate：先看 domain（0）、网卡名、ros2 topic list；
    再跑 scripts/go2_real_check.py（只读自检）。
  * 能发 /lowcmd 但机器人不动：CRC / sport mode（L2+R2 debug mode）/
    电机 mode 三项，见 real_env/README.md。
  * 仿真和真机同时开：domain 必须分开（仿真 1 / 真机 0），否则话题串台。
  * 改网卡名只改 config 里的 interface 即可，环境变量由 apply_io_env 生成。

相关文件：
  config/go2_sim.json        真正的配置（io_profiles/policy/health/safety/perception）
  signal_io/transport/*      各后端实现
  ros2_controller/run_controller.py  控制器入口（load_config -> apply_io_env -> transport）
  real_env/README.md         真机部署 checklist
  docs/ros2-setup.md         ROS 2 / CycloneDDS 安装与排错
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ================================================================ IO profiles
# 每个 profile 描述「这次到底打谁、走哪条链路、参数是什么」。
# 字段说明见模块 docstring §3；这里逐项注释。
DEFAULT_IO_PROFILES: Dict[str, Dict[str, Any]] = {
    # ---------------------------------------------------------------- 仿真：裸 UDP
    # 不依赖 ROS 2 / DDS，本机双进程直连；报文语义与 DDS 一致（500 Hz + CRC 规则）。
    "sim_udp": {
        "name": "udp",
        "host": "127.0.0.1",
        "cmd_port": 50001,
        "state_port": 50002,
        "crc_check": True,       # 本地报文仍走 CRC，保证与真机同语义
        "watchdog_ms": 60.0,     # 只对仿真侧 SafetyLimiter 生效
        "realtime": True,
    },
    # ---------------------------------------------------------------- 仿真：ROS 2
    # 与真机同一张 ROS 2 节点图，只是数据源换成 MuJoCo。
    # 话题名与 unitree_ros2 官方例程一致：/lowcmd、/lowstate；
    # rmw_cyclonedds_cpp 会自动补 rt/，映射成 DDS 的 rt/lowcmd、rt/lowstate。
    # sim_ros 也打开 crc_check：让真机固件要求的 CRC 在仿真闭环里先被验证。
    "sim_ros": {
        "name": "ros2",
        "domain_id": 1,          # 仿真用 1，避免与真机 domain 0 串台
        "interface": "lo",
        "crc_check": True,
        "watchdog_ms": 60.0,
        "realtime": True,
        "cmd_topic": "/lowcmd",
        "state_topic": "/lowstate",
        "tick_dt": 0.002,        # 仿真 tick = 物理步数（2 ms/步）
    },
    # ---------------------------------------------------------------- 仿真：raw DDS
    # unitree_sdk2py 直接和 unitree_mujoco/真机固件同款 DDS 话题。
    # 注意：raw DDS 层没有 rmw 前缀，话题名必须写 rt/lowcmd、rt/lowstate。
    "sim_sdk": {
        "name": "dds",
        "domain_id": 1,
        "interface": "lo",
        "crc_check": False,      # 仿真 DDS 桥不校验 CRC
        "watchdog_ms": 60.0,
        "realtime": True,
        "cmd_topic": "rt/lowcmd",
        "state_topic": "rt/lowstate",
        "tick_dt": 0.002,
    },
    # ---------------------------------------------------------------- 真机：ROS 2（推荐）
    # PC 网线直连 Go2：domain 0 + 有线网卡；robot 固件就是 DDS rt/lowcmd/lowstate，
    # 经 rmw_cyclonedds_cpp 直接以 ROS 2 话题 /lowcmd、/lowstate 出现。
    # crc_check 必须 True；tick_dt=0.001（真机 tick 按 1 ms 递增）。
    # watchdog_ms 对真机没有保护作用（见模块 docstring），保留字段只是与仿真同构。
    "real_ros": {
        "name": "ros2",
        "domain_id": 0,
        "interface": "eth0",     # 改成你的有线网卡名（ip link 查，例如 enp3s0）
        "crc_check": True,
        "watchdog_ms": 60.0,
        "realtime": True,
        "cmd_topic": "/lowcmd",
        "state_topic": "/lowstate",
        "tick_dt": 0.001,
    },
    # ---------------------------------------------------------------- 真机：官方 SDK
    # 与 real_ros 等价，但走 unitree_sdk2py raw DDS（Python≤3.13；3.14 下不可用）。
    "real_sdk": {
        "name": "dds",
        "domain_id": 0,
        "interface": "eth0",
        "crc_check": True,
        "watchdog_ms": 60.0,
        "realtime": True,
        "cmd_topic": "rt/lowcmd",
        "state_topic": "rt/lowstate",
        "tick_dt": 0.001,
    },
}

# ================================================================ 机型表
# 一个 target(IO) 只决定「打到哪」；机型决定「用哪份模型、什么力矩上限」。
# 两者正交：train_robot 是策略训练机型，robot 是当前被控机型（本项目都是 go2）。
DEFAULT_ROBOTS: Dict[str, Dict[str, Any]] = {
    "go2": {"model_dir": "sim_env/models/go2", "scene": "scene.xml", "do_not_edit": True},
}

# 出厂默认配置；config/go2_sim.json 会深度覆盖。
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
    "target": "sim_udp",             # 默认：不需要 ROS 2 就能跑的仿真后端
    "io_profiles": {},               # 在 config/*.json 里覆盖或追加
    "sim_dt": 0.002,                 # 500 Hz —— 真机底层也是 500 Hz
    "control_dt": 0.02,              # 50 Hz —— 上层控制器
    "ramp_rate": 200.0,              # N*m/s，安全监督器的斜率限幅
    "hold_until_first_cmd": True,    # 仿真上电后先自己顶住站姿，等控制器接管
    "controller": {"type": "stand", "kp": 60.0, "kd": 5.0, "ramp_s": 1.0},
}


def _merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    """递归合并字典：over 覆盖 base；dict 逐键合并，其余直接替换。"""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: str | None = None, overrides: Dict[str, Any] | None = None,
                target: str | None = None) -> Dict[str, Any]:
    """读配置文件并展开成「运行时配置」（最重要的一次装配，所有入口都调用它）。

    处理流程：
      1. DEFAULTS <- 配置文件（深度 merge）
      2. 选机型：robots.<robot> 覆盖 model_dir/scene/物理与姿态档位
      3. 选 IO：io_profiles[target] 展开成 cfg["transport"]（+ 可选 transport 微调）
      4. CLI 覆盖：--robot / --policy-profile / transport 字段
      5. 按 target 覆盖 health_profiles（真机阈值放宽）
      6. 按 policy_profile 合并策略块（kp/kd/obs 维度/scan 几何/命令…）
      7. 记录 train_robot 的 home_q/scan_base（obs 用训练机型常量，不是当前机型）

    Args:
        path: config json 路径；None 表示只用 DEFAULTS。
        overrides: CLI 覆盖项（None 值会被忽略，方便 argparse 直接传）。
        target: IO profile 名；None 则用配置文件里的 target（再缺省 sim_udp）。
    Returns:
        展开后的 cfg（含 cfg["transport"] / cfg["policy"] / cfg["health"] / cfg["target"]）。
    Raises:
        ValueError: 未知机型 / 未知 IO profile。
    """
    cfg = dict(DEFAULTS)
    if path:
        with open(path, "r", encoding="utf-8") as f:
            cfg = _merge(cfg, json.load(f))

    # ---- 1) 机型：model_dir/scene/物理档位
    cfg["robots"] = _merge(DEFAULT_ROBOTS, cfg.get("robots", {}))
    if overrides and overrides.get("robot"):
        cfg["robot"] = overrides["robot"]
    rb = cfg["robots"].get(cfg["robot"])
    if rb is None:
        raise ValueError("未知机型 %r，可选: %s" % (cfg["robot"], ", ".join(sorted(cfg["robots"]))))
    cfg["model_dir"] = rb.get("model_dir", cfg["model_dir"])
    cfg["scene"] = rb.get("scene", cfg["scene"])
    # 物理/姿态档位写进 cfg 顶层，脚本直接读，不必各自去翻 robots 段。
    # scan_base/home_q 只对「训练机型」有意义，但当前机型=训练机型时也直接可用。
    for _key in ("foot_soft_contact", "solver_iterations", "solver_ls_iterations",
                 "stand_height", "scan_base", "home_q", "linvel_frame"):
        if _key in rb:
            cfg[_key] = rb[_key]

    # ---- 2) IO profile：展开成扁平的 cfg["transport"]
    cfg["io_profiles"] = _merge(DEFAULT_IO_PROFILES, cfg.get("io_profiles", {}))
    tgt = target or cfg.get("target") or "sim_udp"
    if tgt not in cfg["io_profiles"]:
        raise ValueError("未知 IO profile %r，可选: %s" % (tgt, ", ".join(sorted(cfg["io_profiles"]))))
    io = dict(cfg["io_profiles"][tgt])
    io["profile"] = tgt                 # 保留 profile 名，--json summary / 日志里可见
    cfg["target"] = tgt
    # 配置文件里额外的顶层 transport 段作为微调（例如换端口/换话题）
    io = _merge(io, cfg.pop("transport", {}) or {})
    # ---- 2.5) 环境变量微调：容器/真机上“网卡名 / domain 与配置文件不同”时不必改文件
    #   GO2_INTERFACE=eth1   覆盖 io profile 的 interface（真机有线网卡名；容器里
    #                        看到的是 WSL/docker 侧的名字，通常和宿主机不同）
    #   GO2_DOMAIN_ID=0      覆盖 domain_id（一般不用改：真机 0 / 仿真 1）
    _env_iface = os.environ.get("GO2_INTERFACE")
    if _env_iface:
        io["interface"] = _env_iface
    _env_domain = os.environ.get("GO2_DOMAIN_ID")
    if _env_domain:
        try:
            io["domain_id"] = int(_env_domain)
        except ValueError:
            pass
    # 网卡名容错：真机档默认 "eth0"，但实际机器/容器里常叫 enp0s31f6 之类。
    # 网卡名不存在时 CycloneDDS 静默收不到 /lowstate，上机现场极难排查，所以先纠正。
    if io.get("name") in ("ros2", "dds"):
        io["interface"] = resolve_interface(str(io.get("interface") or ""), tgt)
    cfg["transport"] = io
    cfg["watchdog_ms"] = float(io.get("watchdog_ms", 60.0))

    # ---- 3) CLI 覆盖（transport 字段单独深度 merge）
    if overrides:
        overrides = {k: v for k, v in overrides.items() if v is not None}
        tr_over = overrides.pop("transport", None)
        cfg = _merge(cfg, overrides)
        if tr_over:
            cfg["transport"] = _merge(cfg["transport"], tr_over)

    # ---- 4) 健康阈值按 target 覆盖
    # 真机 LowState 靠 DDS，偶发抖动比仿真大：stale_ms 太紧（20 ms）会被一次调度
    # 抖动打成 L1（下蹲）。所以 real_* 放宽到 stale 40 / protect 80 / estop 250 ms。
    hp = cfg.get("health_profiles") or {}
    if tgt in hp and hp[tgt]:
        cfg["health"] = _merge(cfg.get("health") or {}, hp[tgt])

    # ---- 5) 策略档：auto = 按当前 robot 选；go2 档默认就是顶层 policy 块本身
    prof = cfg.pop("policy_profile", "auto")
    if prof == "auto":
        prof = cfg["robot"]
    profiles = cfg.get("policy_profiles") or {}
    if prof in profiles and profiles[prof]:
        cfg["policy"] = _merge(cfg.get("policy") or {}, profiles[prof])
    cfg["policy_profile"] = prof

    # ---- 6) 训练机型的观测常量
    # obs 的 12:24 段必须减「策略训练机型」的 home_q；scan_base 同理。
    # 当前项目 train_robot == robot == go2，但保留这层是为了以后移植到其它机型。
    tr_robot = (cfg.get("policy") or {}).get("train_robot") or cfg["robot"]
    trb = cfg["robots"].get(tr_robot) or {}
    cfg["train_robot"] = tr_robot
    for _key in ("scan_base", "home_q"):
        if _key in trb:
            cfg["_train_%s" % _key] = trb[_key]
    return cfg


# ================================================================ 真机网卡容错
#: Go2 底盘内部网段（官方文档：PC 侧 192.168.123.x/24，机器人 192.168.123.161）
DOG_SUBNET = "192.168."


def _iface_exists(iface: str) -> bool:
    """网卡是否真的存在（Linux /sys/class/net；其它平台退化为 True 不做判断）。"""
    if not iface:
        return False
    if not os.path.isdir("/sys/class/net"):
        return True
    return os.path.isdir("/sys/class/net/%s" % iface)


def _iface_ipv4(iface: str) -> Optional[str]:
    """读网卡 IPv4（只依赖标准库；失败返回 None）。"""
    try:
        import fcntl
        import socket
        import struct
    except ImportError:      # 非 Linux
        return None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        req = struct.pack("256s", iface[:15].encode())
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, req)[20:24])
    except OSError:
        return None
    finally:
        s.close()


def _list_ifaces() -> List[str]:
    try:
        return sorted(os.listdir("/sys/class/net"))
    except OSError:
        return []


def resolve_interface(iface: str, target: str = "") -> str:
    """真机档的网卡名容错：配置里的 eth0 在实际机器上可能叫 enp0s31f6。

    为什么需要：真机档默认 interface="eth0"（官方例程的写法）。但在
    "docker --network host" 的容器里，看到的是**宿主机**的网卡名；
    CycloneDDS 拿到一个不存在的网卡名时不会报错，只是收不到机器人的
    /lowstate —— 表现为"话题列表空、什么都连不上"，非常难查。

    策略（保守、只在名字确实不存在时才动）：
      1) 网卡存在 -> 原样返回（不做任何猜测）；
      2) 不存在，但恰有一个网卡带 192.168.123.x 地址 -> 自动改用它并打印；
      3) 都没有 -> 原样返回，并打印本机网卡清单提示用 GO2_INTERFACE 指定。
    """
    if not iface:
        return iface
    if _iface_exists(iface):
        return iface
    if not target.startswith("real"):
        return iface
    cands = []
    for name in _list_ifaces():
        ip = _iface_ipv4(name)
        if not ip:
            continue
        if ip.startswith(DOG_SUBNET + "123."):
            cands.append((name, ip))
    if len(cands) == 1:
        name, ip = cands[0]
        print("[config] ⚠ 配置的网卡 %r 不存在；自动改用持有机器人网段地址的 %r（%s）"
              % (iface, name, ip))
        return name
    print("[config] ⚠ 配置的网卡 %r 在本机不存在（真机 DDS 会连不上机器人）。"
          % iface)
    print("[config]   本机网卡：%s" % (", ".join(_list_ifaces()) or "<读不到>"))
    print("[config]   请用 GO2_INTERFACE=<接狗的有线网卡> 或 --target 对应的 interface 指定；"
          "该网卡应已配好 192.168.123.x/24 且能 ping 通 192.168.123.161")
    return iface


def io_summary(cfg: Dict[str, Any]) -> str:
    """一行说明「这次到底在打谁、走哪条 IO」，启动时打印，避免搞混仿真/真机。

    示例：
        real_ros -> IOROS(ROS2/cyclonedds) [真机 domain=0 iface=eth0] 看门狗 60 ms
        sim_udp  -> LOCAL(udp)             [仿真 127.0.0.1:50001]        看门狗 60 ms

    注意：这里打印的「看门狗」是仿真侧 SafetyLimiter 的；真机保护见 health_profiles。
    """
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
    """把 IO profile 翻译成 ROS 2 / CycloneDDS 环境变量，并写入 os.environ。

    **必须在 rclpy.init() / ChannelFactoryInitialize() 之前调用**：
    ROS 2 在进程初始化时读取这些变量，之后再改不生效。

    设置内容：
        ROS_DOMAIN_ID         domain_id（真机 0 / 仿真 1）
        RMW_IMPLEMENTATION    rmw_cyclonedds_cpp（必须：要和机器人固件的 CycloneDDS 互通）
        CYCLONEDDS_URI        仅 interface != "lo" 时设置，把 DDS 绑到指定有线网卡
        （不设置 ROS_LOCALHOST_ONLY；如果 shell 里已设为 1，必须 unset，否则只走回环）

    网卡写法（重要）：
        旧写法 NetworkInterfaceAddress 在 CycloneDDS 11（ROS 2 Lyrical 自带）里
        已被标记 deprecated（会警告、未来可能移除；本机还遇到过一次与 AllowMulticast
        组合触发 assert 的情况，行为不稳定）。这里用 unitree_ros2 setup.sh 同款新写法：
            <CycloneDDS><Domain><General><Interfaces>
              <NetworkInterface name="eth0" priority="default" multicast="default" />
            </Interfaces></General></Domain></CycloneDDS>
        多网卡机器（有线 + Wi-Fi）必须显式绑定有线网卡，否则 DDS 可能选到 Wi-Fi，
        导致收不到 /lowstate。

    Args:
        cfg: load_config() 的产物（至少含 cfg["transport"]）。
    Returns:
        实际设置的环境变量 dict（方便调用方打印到日志/--json summary）。
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


# ================================================================ 机型相关路径/对象
# 机型 -> 机器人本体 MJCF 文件名（注意 go2 是本项目转换出来的力矩版）
ROBOT_BODY_XML: Dict[str, str] = {"go2": "go2.xml"}


def robot_body_xml(cfg: Dict[str, Any], robot: str | None = None) -> str:
    """机器人本体 MJCF 的绝对路径（不含平面/障碍物，适合解析关节名与范围）。

    和 model_xml() 的区别：model_xml 是带地形/平面/障碍物的 scene，
    robot_body_xml 只有机器人本体，用来读关节范围、构造 LegKinematics。
    """
    name = robot or cfg.get("robot", "go2")
    model_dir = cfg["model_dir"] if name == cfg.get("robot", "go2") else "sim_env/models/%s" % name
    return os.path.join(PROJECT_ROOT, model_dir, ROBOT_BODY_XML.get(name, name + ".xml"))


def robot_spec_of(cfg: Dict[str, Any]):
    """按 cfg["robot"] 造 RobotSpec（力矩上限 / 标称姿态因机型而异）。

    config 里的 robots.<name> 段可以覆盖标称站姿与出生高度：

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
    """带场景（平面/地形/障碍物）的 MJCF 绝对路径，仿真/观察器用这个。"""
    return os.path.join(PROJECT_ROOT, cfg["model_dir"], cfg["scene"])


def robot_xml(cfg: Dict[str, Any]) -> str:
    """机器人本体 MJCF 的绝对路径（scene 之外的单机器人模型）。"""
    return os.path.join(PROJECT_ROOT, cfg["model_dir"], cfg["robot"] + ".xml")
