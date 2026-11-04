"""raw DDS 传输后端：unitree_sdk2py，话题 rt/lowcmd / rt/lowstate，与机器人固件一致。

===============================================================================
1. 和 ros2.py 的区别（别混）
===============================================================================

  ros2.py（推荐，真机 real_ros）
      rclpy + rmw_cyclonedds_cpp，ROS 2 话题 /lowcmd、/lowstate；
      rmw 在 DDS 层自动加 rt/ 前缀，最终也是 rt/lowcmd、rt/lowstate。
  dds.py（本文件，real_sdk / sim_sdk）
      unitree_sdk2py 直接作为 DDS participant，话题名必须原样写 rt/lowcmd、
      rt/lowstate（没有 rmw 前缀转换）。

  两条路都能到机器人固件，区别是依赖：
      ros2 路径只需要 apt 的 ROS 2（本机可用）；
      dds 路径需要 cyclonedds 的 Python 绑定 + unitree_sdk2py
      （Python 3.14 下不可用，见 docs/ros2-setup.md §8）。

===============================================================================
2. 角色（与 Transport 接口一致）
===============================================================================

  role="sim"      : 订阅 rt/lowcmd，发布 rt/lowstate（仿真侧 / DDS 桥）
  role="controller": 发布 rt/lowcmd，订阅 rt/lowstate（真机控制侧）

  domain：真机 0 + 有线网卡；仿真 1 + "lo"。
  这两个值由 config.io_profiles 的 sim_sdk/real_sdk 决定。

===============================================================================
3. 真机固件要求（与 ros2 路径相同，只是实现方式不同）
===============================================================================

  LowCmd 必须带：
      head = [0xFE, 0xEF]        level_flag = 0xFF
      motor_cmd[i].mode = 0x01   crc = CRC().Crc(msg)
  CRC 由 unitree_sdk2py 的 CRC 类按 812 字节 IDL 布局计算（与我们
  signal_io/go2_crc.py 的布局一致，但这里直接用 SDK 实现，避免重复维护）。
  unitree_go LowCmd 没有 mode_pr/mode_machine（那是 unitree_hg/G1/H1）。

===============================================================================
4. 本机状态与常见失败模式
===============================================================================

  * Python 3.14 下 import cyclonedds 会崩（PEP 649 与它的 IDL 反射冲突），
    所以 real_sdk 在本机不可用；真机请用 real_ros。
  * 若 CRC 类不可用而 crc_check=True（真机方向），__init__ 直接 fail-fast，
    避免静默发 crc=0 的指令（固件大概率丢弃）。
  * ChannelFactoryInitialize 每个进程只能调用一次；仿真/控制器不要在同一进程里
    建两个 DdsTransport。

相关文件：
  signal_io/transport/ros2.py   推荐路径（rclpy + rmw）
  signal_io/go2_crc.py          812 字节 CRC 布局（与 SDK 的 CRC 等价）
  docs/ros2-setup.md §8         Python 3.14 为什么跑不了
"""

from __future__ import annotations

from typing import Optional

from ..msg import NUM_MOTOR, IMUState, LowCmd, LowState, MotorCmd, MotorState
from .base import Transport

# MotorCmd 需要逐字段搬运的字段（reserve 是 uint32[3]，单独处理）
_CMD_FIELDS = ("mode", "q", "dq", "tau", "kp", "kd", "reserve")
_IMU_FIELDS = ("quaternion", "gyroscope", "accelerometer", "rpy")


def _scalar_or_first(v, default: int = 0) -> int:
    """把标量/IDL 数组字段都安全地读成 int。

    unitree_sdk2py 的 MotorCmd_.reserve 是 types.array[uint32, 3]（Python 里是
    list），而内部 MotorCmd.reserve 是 int；直接 int(list) 会 TypeError。
    真机固件来的 LowCmd 也可能把 reserve 填成非零，所以取首元素而不是硬编码 0。
    """
    try:
        return int(v)
    except (TypeError, ValueError):
        try:
            return int(v[0])
        except Exception:
            return default


class DdsTransport(Transport):
    """unitree_sdk2py 的 raw DDS 实现（real_sdk / sim_sdk）。

    初始化时 ChannelFactoryInitialize 会创建进程级 DDS participant；
    ChannelPublisher/ChannelSubscriber 用固件同名话题收发 IDL 对象。
    """

    name = "dds"

    def __init__(self, domain_id: int = 1, interface: str = "lo", crc_check: bool = True,
                 cmd_topic: str = "rt/lowcmd", state_topic: str = "rt/lowstate",
                 role: str = "controller", tick_dt: float = 0.002):
        """建 DDS 传输。

        Args:
            domain_id: DDS domain；真机 0、仿真 1。
            interface: 网卡名；真机用有线网卡（eth0/enp3s0），仿真 "lo"。
            crc_check: 真机方向必须 True（固件校验 LowCmd.crc）。
            cmd_topic: raw DDS 话题名，必须是 rt/lowcmd（不要写 /lowcmd）。
            state_topic: 同理 rt/lowstate。
            role: "sim" 或 "controller"。
            tick_dt: LowState.tick 每 +1 的秒数（仿真 0.002 / 真机 0.001）。
        """
        self.role = role            # "sim" 发状态收指令; "controller" 发指令收状态
        self.domain_id = int(domain_id)
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
        """导入 SDK、初始化 DDS、按 role 建 publisher/subscriber。

        Raises:
            RuntimeError: 没装 unitree_sdk2py/cyclonedds，或真机方向没有 CRC 实现。
        """
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
                "（cyclonedds 的 Python 扩展在 Python 3.14 下不可用；"
                "真机请直接用 ros2 后端 real_ros，见 docs/ros2-setup.md §8）\n"
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

        # 每个进程只能初始化一次 DDS factory；后续 DdsTransport 复用同一 participant。
        ChannelFactoryInitialize(self.domain_id, self.interface)
        self._LowCmd_ = LowCmd_
        self._LowCmd_default = unitree_go_msg_dds__LowCmd_
        self._LowState_default = unitree_go_msg_dds__LowState_
        self._state_cls = LowState_
        if self.role == "sim":
            # 仿真侧：发 LowState、收 LowCmd。
            self._pub = ChannelPublisher(self.state_topic, LowState_)
            self._pub.Init()
            self._sub = ChannelSubscriber(self.cmd_topic, LowCmd_)
            self._sub.Init(self._on_cmd, 10)
        else:
            # 控制侧：发 LowCmd、收 LowState。
            self._pub = ChannelPublisher(self.cmd_topic, LowCmd_)
            self._pub.Init()
            self._sub = ChannelSubscriber(self.state_topic, LowState_)
            self._sub.Init(self._on_state, 10)

    # ---------------------------------------------------------------- 收
    def _on_state(self, m) -> None:
        """DDS LowState 回调：只保留最新一帧（消费在 recv_state）。"""
        self._latest_state = self._from_idl_state(m)

    def _on_cmd(self, m) -> None:
        """DDS LowCmd 回调（仅 sim 角色）：IDL -> 内部 LowCmd。

        raw DDS 侧不做 CRC 校验（仿真桥通常不填；真机校验发生在固件侧）。
        """
        cmd = LowCmd()
        cmd.mode_pr = int(getattr(m, "mode_pr", 0))
        cmd.mode_machine = int(getattr(m, "mode_machine", 0))
        for i in range(NUM_MOTOR):
            src = m.motor_cmd[i]
            cmd.motor_cmd[i] = MotorCmd(
                mode=int(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                reserve=_scalar_or_first(getattr(src, "reserve", 0)),
            )
        self._latest_cmd = cmd

    def _from_idl_state(self, m) -> LowState:
        """unitree_go IDL LowState -> 内部 LowState（字段一一对应）。"""
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

    # ---------------------------------------------------------------- 发
    def _to_idl_cmd(self, cmd: LowCmd):
        """内部 LowCmd -> unitree_go::msg::dds_::LowCmd_（真机下发格式）。

        Go2 固件要求固定头（head=FE EF, level_flag=FF）并校验 crc；
        CRC 用 unitree_sdk2py 的 CRC 类按 812 字节 IDL 布局计算。
        IDL 的 array 字段（head/reserve）按官方例程逐元素赋值。
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
        """内部 LowState -> unitree_go IDL LowState（仿真侧发布用）。"""
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

    # ---------------------------------------------------------------- 接口
    def publish_state(self, state: LowState) -> None:
        """role="sim"：把 LowState 写到 rt/lowstate。"""
        if self.role == "sim":
            self._pub.Write(self._to_idl_state(state))

    def recv_cmd(self) -> Optional[LowCmd]:
        """role="sim"：取最新 LowCmd；没新消息返回 None。"""
        cmd, self._latest_cmd = self._latest_cmd, None
        return cmd

    def recv_state(self) -> Optional[LowState]:
        """role="controller"：取最新 LowState；没新消息返回 None。"""
        st, self._latest_state = self._latest_state, None
        return st

    def publish_cmd(self, cmd: LowCmd) -> None:
        """role="controller"：把 LowCmd 写到 rt/lowcmd（含固定头 + CRC）。"""
        if self.role == "controller":
            self._pub.Write(self._to_idl_cmd(cmd))

    def stats(self) -> dict:
        """运行时统计（DDS 后端目前只有静态信息；接收/CRC 统计在 ros2 后端更全）。"""
        return {"transport": self.name, "role": self.role, "domain_id": self.domain_id,
                "interface": self.interface}
