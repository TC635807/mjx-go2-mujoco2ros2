"""传输层工厂：把 config 里的 io_profile 变成具体 Transport 对象。

===============================================================================
target / profile -> 后端 映射（config/go2_sim.json 的 io_profiles）
===============================================================================

  target      backend    domain  话题（关键区别）
  ---------   --------   ------  ---------------------------------------------
  sim_udp     udp        -       127.0.0.1:50001/50002（裸报文，与 DDS 同语义）
  sim_ros     ros2       1       /lowcmd、/lowstate（rmw 映射成 DDS rt/...）
  sim_sdk     dds        1       rt/lowcmd、rt/lowstate（raw DDS）
  real_ros    ros2       0       /lowcmd、/lowstate（**真机推荐路径**）
  real_sdk    dds        0       rt/lowcmd、rt/lowstate（官方 SDK；Python 3.14 不可用）

"仿真与真机同一套代码" 的落点就是：换 target 只改 domain/网卡/话题名，
sim 与 controller 两侧代码一行不改。

===============================================================================
profile 字段含义（哪些和真机有关）
===============================================================================

  domain_id     DDS domain。真机固件固定 0；仿真用 1，避免与真机串台。
  interface     有线网卡名（真机必须绑到连机器人的那张，例如 eth0/enp3s0）；
                仿真用 "lo"。
  cmd_topic     LowCmd 话题。ROS 2 后端写 /lowcmd；raw DDS 后端写 rt/lowcmd。
  state_topic   LowState 话题，同理。
  tick_dt       LowState.tick 每 +1 的秒数（仿真 0.002、真机 0.001）。
  crc_check     是否计算/校验真机 LowCmd CRC（real_* 必须 True）。
  watchdog_ms   仅仿真侧 SafetyLimiter 使用（掉 LowCmd 清零力矩）；
                真机侧没有这个看门狗，靠 HealthMonitor + SafetySupervisor。

注意：
  * 真正的环境变量（ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI）
    不在这里设，而是在 config.apply_io_env() 里、**rclpy.init() 之前**设好。
  * 未知的 profile 名会抛 ValueError，避免拼错 target 后静默走默认后端。

相关文件：
  signal_io/config.py         io_profiles 定义 + apply_io_env
  signal_io/transport/base.py Transport 契约
  docs/ros2-setup.md          ROS 2 / CycloneDDS 安装与环境
  real_env/README.md          真机部署 checklist
"""

from __future__ import annotations

from .base import Transport


def make_transport(cfg: dict, role: str) -> Transport:
    """按展开后的 cfg["transport"] 创建后端。

    Args:
        cfg: config.load_config() 产出的 transport 段（已合并 io_profile + CLI 覆盖）。
             至少包含 name；ros2/dds 还包含 domain_id/interface/topic 等。
        role: "sim" 或 "controller"，决定 pub/sub 方向。
    Returns:
        Transport 实例（已初始化、可立即收发）。
    Raises:
        ValueError: name 未知（配置写错 / 拼错 target 时 fail-fast）。
    """
    name = cfg.get("name", "udp")

    if name == "loopback":
        # 同进程直连：单进程验证 / tests / verify 用，不经过任何序列化。
        from .local import LoopbackTransport

        return LoopbackTransport()

    if name == "udp":
        # 裸 UDP：报文格式与 DDS 语义一致（500 Hz、CRC 规则相同），
        # 用于没有 ROS 2 / CycloneDDS 的机器上跑完整双进程链路。
        from .local import UdpTransport

        return UdpTransport(
            host=cfg.get("host", "127.0.0.1"),
            cmd_port=int(cfg.get("cmd_port", 50001)),
            state_port=int(cfg.get("state_port", 50002)),
            role=role,
            crc_check=bool(cfg.get("crc_check", True)),
        )

    if name == "dds":
        # raw DDS（unitree_sdk2py）：话题名就是机器人固件的 rt/lowcmd、rt/lowstate。
        from .dds import DdsTransport

        return DdsTransport(
            domain_id=int(cfg.get("domain_id", 1)),
            interface=cfg.get("interface", "lo"),
            crc_check=bool(cfg.get("crc_check", True)),
            cmd_topic=str(cfg.get("cmd_topic", "rt/lowcmd")),
            state_topic=str(cfg.get("state_topic", "rt/lowstate")),
            tick_dt=float(cfg.get("tick_dt", 0.002)),
            role=role,
        )

    if name == "ros2":
        # ROS 2（rclpy + rmw_cyclonedds_cpp）：ROS 2 话题 /lowcmd、/lowstate；
        # rmw 自动加上 rt/ 前缀后正好是 DDS rt/lowcmd、rt/lowstate。
        from .ros2 import Ros2Transport

        return Ros2Transport(
            domain_id=int(cfg.get("domain_id", 1)),
            crc_check=bool(cfg.get("crc_check", True)),
            cmd_topic=str(cfg.get("cmd_topic", "/lowcmd")),
            state_topic=str(cfg.get("state_topic", "/lowstate")),
            tick_dt=float(cfg.get("tick_dt", 0.002)),
            role=role,
        )

    raise ValueError("未知 transport: %r（可选 loopback/udp/ros2/dds）" % name)


__all__ = ["Transport", "make_transport"]
