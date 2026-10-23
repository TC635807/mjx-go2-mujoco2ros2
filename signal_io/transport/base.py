"""传输层抽象 —— 同一个 sim 节点，四种后端可以互换。

    loopback : 同进程直连（单进程调试 / verify 用），零依赖
    udp      : 127.0.0.1 上的双 socket，与 DDS 同样的 500 Hz、同样的报文语义，
               用来在没有 ROS2/CycloneDDS 的机器上把整条链路真跑起来
    dds      : unitree_sdk2py（raw DDS），话题 rt/lowcmd / rt/lowstate，与机器人固件一致
    ros2     : rclpy + unitree_ros2 的 unitree_go/msg，ROS 2 话题 /lowcmd / /lowstate，
               rmw_cyclonedds_cpp 映射到 DDS 的 rt/lowcmd / rt/lowstate（= 真机话题）

sim 侧: publish_state() + recv_cmd()
controller 侧: recv_state() + publish_cmd()
"""

from __future__ import annotations

import abc
from typing import Optional

from ..msg import LowCmd, LowState


class Transport(abc.ABC):
    """所有后端实现这四个方法。recv_* 必须是**非阻塞**的，返回 None 表示这一拍没消息。"""

    name = "abstract"

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    # --- sim 侧 ---
    @abc.abstractmethod
    def publish_state(self, state: LowState) -> None:
        ...

    @abc.abstractmethod
    def recv_cmd(self) -> Optional[LowCmd]:
        ...

    # --- controller 侧 ---
    @abc.abstractmethod
    def recv_state(self) -> Optional[LowState]:
        ...

    @abc.abstractmethod
    def publish_cmd(self, cmd: LowCmd) -> None:
        ...

    # --- 统计 ---
    def stats(self) -> dict:
        return {}
