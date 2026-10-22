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
            role=role,
        )
    if name == "ros2":
        from .ros2 import Ros2Transport

        return Ros2Transport(
            domain_id=int(cfg.get("domain_id", 1)),
            crc_check=bool(cfg.get("crc_check", True)),
            role=role,
        )
    raise ValueError("未知 transport: %r" % name)


__all__ = ["Transport", "make_transport"]
