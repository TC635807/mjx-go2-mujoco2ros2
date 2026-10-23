"""传输层工厂。"""

from __future__ import annotations

from .base import Transport


def make_transport(cfg: dict, role: str) -> Transport:
    """cfg = config.json 里的 "transport" 段；role = "sim" | "controller"。"""
    name = cfg.get("name", "udp")
    if name == "loopback":
        from .local import LoopbackTransport

        return LoopbackTransport()
    if name == "udp":
        from .local import UdpTransport

        return UdpTransport(
            host=cfg.get("host", "127.0.0.1"),
            cmd_port=int(cfg.get("cmd_port", 50001)),
            state_port=int(cfg.get("state_port", 50002)),
            role=role,
            crc_check=bool(cfg.get("crc_check", True)),
        )
    if name == "dds":
        from .dds import DdsTransport

        return DdsTransport(
            domain_id=int(cfg.get("domain_id", 1)),
            interface=cfg.get("interface", "lo"),
            crc_check=bool(cfg.get("crc_check", True)),
            # raw DDS 后端直接用机器人固件的话题名 rt/lowcmd、rt/lowstate
            cmd_topic=str(cfg.get("cmd_topic", "rt/lowcmd")),
            state_topic=str(cfg.get("state_topic", "rt/lowstate")),
            tick_dt=float(cfg.get("tick_dt", 0.002)),
            role=role,
        )
    if name == "ros2":
        from .ros2 import Ros2Transport

        return Ros2Transport(
            domain_id=int(cfg.get("domain_id", 1)),
            crc_check=bool(cfg.get("crc_check", True)),
            # ROS 2 层用 /lowcmd、/lowstate；rmw_cyclonedds_cpp 会映射成 DDS 的
            # rt/lowcmd、rt/lowstate，正好是机器人固件监听的话题。
            cmd_topic=str(cfg.get("cmd_topic", "/lowcmd")),
            state_topic=str(cfg.get("state_topic", "/lowstate")),
            tick_dt=float(cfg.get("tick_dt", 0.002)),
            role=role,
        )
    raise ValueError("未知 transport: %r" % name)


__all__ = ["Transport", "make_transport"]
