"""DDS 后端：unitree_sdk2py，话题 rt/lowcmd / rt/lowstate，与真机完全一致。

    sim 侧      : 订阅 rt/lowcmd, 发布 rt/lowstate
    controller 侧: 发 rt/lowcmd, 订阅 rt/lowstate
    真机切换    : domain_id 0 + 有线网卡；仿真 domain_id 1 + "lo"

注意（本机实测）：本机没有 sudo，装不了 python3.14-dev，所以 cyclonedds 的
Python 扩展编不出来 —— 这个后端在本机**没跑过**。装好 ROS 2 之后可以直接用
apt 的 rclpy，或者装了 python3-dev 之后再 pip 装 cyclonedds。
"""

from __future__ import annotations

from typing import Optional

from ..msg import NUM_MOTOR, IMUState, LowCmd, LowState, MotorCmd, MotorState
from .base import Transport

_CMD_FIELDS = ("mode", "q", "dq", "tau", "kp", "kd", "reserve")
_IMU_FIELDS = ("quaternion", "gyroscope", "accelerometer", "rpy")


class DdsTransport(Transport):
    name = "dds"

    def __init__(self, domain_id: int = 1, interface: str = "lo", crc_check: bool = True,
                 cmd_topic: str = "rt/lowcmd", state_topic: str = "rt/lowstate",
                 role: str = "controller", tick_dt: float = 0.002):
        self.role = role            # "sim" 发状态收指令; "controller" 发指令收状态
        self.domain_id = int(domain_id)
        # LowState.tick 每 +1 的秒数：仿真 = 2 ms/物理步，真机固件 = 1 ms
        self.tick_dt = float(tick_dt)
        self.interface = interface
        self.crc_check = bool(crc_check)
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self._latest_state: Optional[LowState] = None
        self._latest_cmd: Optional[LowCmd] = None
        self._crc = None
        self._load()

    def _load(self) -> None:
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        except Exception as exc:
            raise RuntimeError(
                "DDS 后端需要 unitree_sdk2py + cyclonedds。\n"
                "  pip install -e third_party/unitree_sdk2_python\n"
                "（cyclonedds 的 Python 扩展需要 python 开发头文件，本机没有 sudo 装不了；"
                "装了 ROS 2 之后可以直接用 ros2 后端）\n"
                "原始错误: " + repr(exc)
            ) from exc

        try:
            from unitree_sdk2py.utils.crc import CRC

            self._crc = CRC()
        except Exception:
            self._crc = None
        if self._crc is None and self.crc_check and self.role == "controller":
            # 真机固件校验 crc；算不了 CRC 还硬发 = 大概率静默丢帧，直接 fail-fast。
            raise RuntimeError(
                "real_sdk 需要 unitree_sdk2py.utils.crc.CRC 计算 LowCmd.crc，但 CRC 不可用；"
                "拒绝在 crc_check=True 的控制器方向静默下发 crc=0 的指令")

        ChannelFactoryInitialize(self.domain_id, self.interface)
        self._LowCmd_ = LowCmd_
        self._LowCmd_default = unitree_go_msg_dds__LowCmd_
        self._LowState_default = unitree_go_msg_dds__LowState_
        self._state_cls = LowState_
        if self.role == "sim":
            # 发 LowState、收 LowCmd
            self._pub = ChannelPublisher(self.state_topic, LowState_)
            self._pub.Init()
            self._sub = ChannelSubscriber(self.cmd_topic, LowCmd_)
            self._sub.Init(self._on_cmd, 10)
        else:
            # 发 LowCmd、收 LowState
            self._pub = ChannelPublisher(self.cmd_topic, LowCmd_)
            self._pub.Init()
            self._sub = ChannelSubscriber(self.state_topic, LowState_)
            self._sub.Init(self._on_state, 10)

    def _on_state(self, m) -> None:
        self._latest_state = self._from_idl_state(m)

    def _on_cmd(self, m) -> None:
        cmd = LowCmd()
        cmd.mode_pr = int(getattr(m, "mode_pr", 0))
        cmd.mode_machine = int(getattr(m, "mode_machine", 0))
        for i in range(NUM_MOTOR):
            src = m.motor_cmd[i]
            cmd.motor_cmd[i] = MotorCmd(
                mode=int(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                reserve=int(getattr(src, "reserve", 0)),
            )
        self._latest_cmd = cmd

    def _from_idl_state(self, m) -> LowState:
        st = LowState()
        st.tick = int(getattr(m, "tick", 0))
        st.sim_time = float(st.tick) * self.tick_dt
        st.mode_machine = int(getattr(m, "mode_machine", 0))
        imu = m.imu_state
        st.imu_state = IMUState(
            quaternion=[float(x) for x in imu.quaternion],
            gyroscope=[float(x) for x in imu.gyroscope],
            accelerometer=[float(x) for x in imu.accelerometer],
            rpy=[float(x) for x in imu.rpy],
        )
        for i in range(NUM_MOTOR):
            src = m.motor_state[i]
            st.motor_state[i] = MotorState(
                mode=int(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                ddq=float(getattr(src, "ddq", 0.0)), tau_est=float(getattr(src, "tau_est", 0.0)),
                temperature=int(getattr(src, "temperature", 0)), lost=int(getattr(src, "lost", 0)),
            )
        st.foot_force = [int(x) for x in getattr(m, "foot_force", [0, 0, 0, 0])]
        return st

    def _to_idl_cmd(self, cmd: LowCmd):
        """unitree_go::msg::dds_::LowCmd_ 没有 mode_pr/mode_machine（那是 unitree_hg）。

        Go2 固件要求 LowCmd 带固定头（head=FE EF, level_flag=FF），并且校验 crc；
        CRC 由 unitree_sdk2py 的 CRC 类按 812 字节 IDL 布局计算。
        """
        m = self._LowCmd_default()
        if hasattr(m, "mode_pr"):
            m.mode_pr = int(cmd.mode_pr)
        if hasattr(m, "mode_machine"):
            m.mode_machine = int(cmd.mode_machine)
        # 官方例程对 IDL 的 array 字段是逐元素赋值（types.array 整体赋 list 不一定可用）
        if hasattr(m, "head"):
            m.head[0] = int(cmd.head[0]) & 0xFF
            m.head[1] = int(cmd.head[1]) & 0xFF
        if hasattr(m, "level_flag"):
            m.level_flag = int(cmd.level_flag) & 0xFF
        if hasattr(m, "frame_reserve"):
            m.frame_reserve = int(getattr(cmd, "frame_reserve", 0)) & 0xFF
        if hasattr(m, "gpio"):
            m.gpio = int(cmd.gpio) & 0xFF
        for i in range(NUM_MOTOR):
            src, dst = cmd.motor_cmd[i], m.motor_cmd[i]
            for f in _CMD_FIELDS:
                if f == "reserve":              # IDL 里是 uint32[3]
                    dst.reserve[0] = int(src.reserve) & 0xFFFFFFFF
                    dst.reserve[1] = 0
                    dst.reserve[2] = 0
                else:
                    setattr(dst, f, getattr(src, f))
        if self._crc is not None:
            m.crc = self._crc.Crc(m)
        return m

    def _to_idl_state(self, st: LowState):
        m = self._LowState_default()
        m.tick = int(st.tick)
        for f in _IMU_FIELDS:
            setattr(m.imu_state, f, list(getattr(st.imu_state, f)))
        for i in range(NUM_MOTOR):
            src, dst = st.motor_state[i], m.motor_state[i]
            dst.mode, dst.q, dst.dq = src.mode, src.q, src.dq
            dst.ddq, dst.tau_est = src.ddq, src.tau_est
            dst.temperature, dst.lost = src.temperature, src.lost
        for i in range(4):
            m.foot_force[i] = st.foot_force[i]
        if self._crc is not None:
            try:
                m.crc = self._crc.Crc(m)
            except Exception:
                pass
        return m

    def publish_state(self, state: LowState) -> None:
        if self.role == "sim":
            self._pub.Write(self._to_idl_state(state))

    def recv_cmd(self) -> Optional[LowCmd]:
        cmd, self._latest_cmd = self._latest_cmd, None
        return cmd

    def recv_state(self) -> Optional[LowState]:
        st, self._latest_state = self._latest_state, None
        return st

    def publish_cmd(self, cmd: LowCmd) -> None:
        if self.role == "controller":
            self._pub.Write(self._to_idl_cmd(cmd))

    def stats(self) -> dict:
        return {"transport": self.name, "role": self.role, "domain_id": self.domain_id,
                "interface": self.interface}
