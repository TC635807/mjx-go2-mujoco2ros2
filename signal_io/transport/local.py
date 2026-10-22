"""loopback 与 udp 两个本地后端（零依赖，用来在没有 ROS2/DDS 的机器上先把链路跑通）。"""

from __future__ import annotations

import socket
from collections import deque
from typing import Optional

from ..msg import LowCmd, LowState, crc32_core
from .base import Transport

_UDP_MAX = 4096


class LoopbackTransport(Transport):
    """同进程直连。sim 与 controller 在同一个 Python 进程里时用它。"""

    name = "loopback"

    def __init__(self, cmd_queue: int = 200, state_queue: int = 200):
        self._to_ctrl: deque = deque(maxlen=state_queue)
        self._to_sim: deque = deque(maxlen=cmd_queue)
        self.n_cmd = 0
        self.n_state = 0

    def publish_state(self, state: LowState) -> None:
        self._to_ctrl.append(state)
        self.n_state += 1

    def recv_cmd(self) -> Optional[LowCmd]:
        return self._to_sim.popleft() if self._to_sim else None

    def recv_state(self) -> Optional[LowState]:
        return self._to_ctrl.popleft() if self._to_ctrl else None

    def publish_cmd(self, cmd: LowCmd) -> None:
        self._to_sim.append(cmd)
        self.n_cmd += 1

    def stats(self) -> dict:
        return {"transport": self.name, "n_cmd": self.n_cmd, "n_state": self.n_state}


class UdpTransport(Transport):
    """本地 UDP 双通道，报文就是 msg.py 里那 524 B / 684 B 的二进制。

    和 DDS 后端的关系: 语义、频率、CRC 完全一样，只是把 DDS 换成了裸 UDP ——
    这样在没装 ROS2 的机器上也能把「双进程 + 500 Hz + 丢包/延迟」这套结构跑起来。
    真机上换成 dds / ros2 后端即可，sim 与 controller 的代码一行不改。
    """

    name = "udp"

    def __init__(self, host: str = "127.0.0.1", cmd_port: int = 50001, state_port: int = 50002,
                 role: str = "sim", crc_check: bool = True):
        self.host = host
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.role = role            # "sim" | "controller"
        self.crc_check = crc_check
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # sim 收 cmd / controller 收 state —— 各自只 bind 自己需要的那个口
        if role == "sim":
            self._rx.bind((host, cmd_port))
        else:
            self._rx.bind((host, state_port))
        self._rx.setblocking(False)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.n_sent = 0
        self.n_recv = 0
        self.n_bad_crc = 0

    # --- 内部 ---
    def _send(self, payload: bytes, port: int) -> None:
        try:
            self._tx.sendto(payload, (self.host, port))
            self.n_sent += 1
        except OSError:
            pass

    def _recv(self, expect_magic: int):
        payload = None
        try:
            while True:
                buf, _addr = self._rx.recvfrom(_UDP_MAX)
                if len(buf) >= 4 and int.from_bytes(buf[:4], "little") == expect_magic:
                    payload = buf          # 只保留最后一帧（永远用最新的）
                else:
                    continue
        except BlockingIOError:
            pass
        return payload

    # --- Transport ---
    def publish_state(self, state: LowState) -> None:
        self._send(state.pack(), self.state_port)

    def publish_cmd(self, cmd: LowCmd) -> None:
        self._send(cmd.pack(), self.cmd_port)

    def recv_cmd(self) -> Optional[LowCmd]:
        buf = self._recv(0x474F3243)
        if buf is None:
            return None
        self.n_recv += 1
        cmd = LowCmd.unpack(buf)
        if self.crc_check and cmd.crc != crc32_core(buf, (len(buf) >> 2) - 1):
            self.n_bad_crc += 1
            return None
        return cmd

    def recv_state(self) -> Optional[LowState]:
        buf = self._recv(0x474F3253)
        if buf is None:
            return None
        self.n_recv += 1
        return LowState.unpack(buf)

    def close(self) -> None:
        for s in (self._tx, self._rx):
            try:
                s.close()
            except OSError:
                pass

    def stats(self) -> dict:
        return {"transport": self.name, "sent": self.n_sent, "recv": self.n_recv,
                "bad_crc": self.n_bad_crc}
