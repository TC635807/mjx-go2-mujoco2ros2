"""ROS 2 后端：rclpy + unitree_ros2 的 unitree_go/msg。

和 DDS 后端的关系: unitree_ros2 本身就是 CycloneDDS 上的 ROS 2 封装
（rmw_cyclonedds_cpp），话题名 rt/lowcmd / rt/lowstate 与真机一致，
所以仿真和真机之间只差 ROS_DOMAIN_ID 和网卡。

装好 ROS 2 之后:
    source /opt/ros/lyrical/setup.bash
    export ROS_DOMAIN_ID=1                 # 仿真；真机默认是 0
    python scripts/run_sim.py --transport ros2 --duration 20 --viewer
"""

from __future__ import annotations

from typing import Optional

from ..msg import NUM_MOTOR, IMUState, LowCmd, LowState, MotorCmd, MotorState
from .base import Transport


def _scalar(v, default: int = 0) -> int:
    """ROS 2 定长数组字段（比如 MotorCmd.reserve 是 uint32[3]）不能直接 int()。"""
    try:
        return int(v)
    except (TypeError, ValueError):
        try:
            return int(v[0])
        except Exception:
            return default


class Ros2Transport(Transport):
    name = "ros2"

    def __init__(self, domain_id: int = 1, crc_check: bool = True,
                 cmd_topic: str = "rt/lowcmd", state_topic: str = "rt/lowstate",
                 node_name: str = "go2_controller", role: str = "controller"):
        self.role = role
        self.domain_id = int(domain_id)
        self.crc_check = bool(crc_check)
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.node_name = node_name
        self._latest: Optional[LowState] = None
        self._latest_cmd: Optional[LowCmd] = None
        # 收到的消息计数：recv_* 用它判断"这一拍到底还有没有新消息"，从而把队列抽干
        self._n_state_rx = 0
        self._n_cmd_rx = 0
        self._n_drained = 0
        self._max_drain = 0
        self._load()

    def _load(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from unitree_go.msg import LowCmd as RosLowCmd
            from unitree_go.msg import LowState as RosLowState
        except Exception as exc:
            raise RuntimeError(
                "ROS 2 后端需要 rclpy 与 unitree_go/msg。\n"
                "  1) 装 ROS 2 Lyrical (Ubuntu 26.04 是它的 Tier-1 平台)\n"
                "  2) 编译 unitree_ros2 拿到 unitree_go 消息包\n"
                "  3) source setup.bash 之后再跑\n"
                "原始错误: " + repr(exc)
            ) from exc

        self._rclpy = rclpy
        self._RosLowCmd = RosLowCmd
        self._RosLowState = RosLowState
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node(self.node_name)
        if self.role == "sim":
            self.pub = self.node.create_publisher(RosLowState, self.state_topic, 10)
            self.sub = self.node.create_subscription(RosLowCmd, self.cmd_topic, self._on_cmd, 10)
        else:
            self.pub = self.node.create_publisher(RosLowCmd, self.cmd_topic, 10)
            self.sub = self.node.create_subscription(RosLowState, self.state_topic, self._on_state, 10)

    def _on_state(self, m) -> None:
        st = LowState()
        st.tick = int(getattr(m, "tick", 0))
        st.sim_time = float(st.tick) * 0.002
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
                mode=_scalar(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                ddq=float(getattr(src, "ddq", 0.0)), tau_est=float(getattr(src, "tau_est", 0.0)),
                temperature=_scalar(getattr(src, "temperature", 0)), lost=_scalar(getattr(src, "lost", 0)),
            )
        st.foot_force = [int(x) for x in getattr(m, "foot_force", [0, 0, 0, 0])]
        self._latest = st
        self._n_state_rx += 1

    def _on_cmd(self, m) -> None:
        cmd = LowCmd()
        cmd.mode_pr = int(getattr(m, "mode_pr", 0))
        cmd.mode_machine = int(getattr(m, "mode_machine", 0))
        for i in range(NUM_MOTOR):
            src = m.motor_cmd[i]
            cmd.motor_cmd[i] = MotorCmd(
                mode=_scalar(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                reserve=_scalar(getattr(src, "reserve", 0)),
            )
        self._latest_cmd = cmd
        self._n_cmd_rx += 1

    def _to_ros_cmd(self, cmd: LowCmd):
        m = self._RosLowCmd()
        # unitree_go/msg/LowCmd 里没有 mode_pr / mode_machine（那是 unitree_hg 才有的），
        # 所以要 hasattr 兜一下，别把后端写死成某一种 IDL。
        if hasattr(m, "mode_pr"):
            m.mode_pr = cmd.mode_pr
        if hasattr(m, "mode_machine"):
            m.mode_machine = cmd.mode_machine
        m.motor_cmd = [self._ros_motor_cmd(c) for c in cmd.motor_cmd]
        return m

    def _ros_motor_cmd(self, c):
        MotorCmdMsg = self._RosLowCmd().motor_cmd[0].__class__
        mc = MotorCmdMsg()
        mc.mode, mc.q, mc.dq = c.mode, c.q, c.dq
        mc.tau, mc.kp, mc.kd = c.tau, c.kp, c.kd
        if hasattr(mc, "reserve"):
            try:
                mc.reserve = [c.reserve, 0, 0]      # uint32[3]
            except (TypeError, ValueError):
                pass
        return mc

    def _to_ros_state(self, st: LowState):
        m = self._RosLowState()
        m.tick = int(st.tick)
        m.imu_state.quaternion = [float(x) for x in st.imu_state.quaternion]
        m.imu_state.gyroscope = [float(x) for x in st.imu_state.gyroscope]
        m.imu_state.accelerometer = [float(x) for x in st.imu_state.accelerometer]
        m.imu_state.rpy = [float(x) for x in st.imu_state.rpy]
        for i in range(NUM_MOTOR):
            src, dst = st.motor_state[i], m.motor_state[i]
            dst.mode, dst.q, dst.dq = src.mode, src.q, src.dq
            dst.ddq, dst.tau_est = src.ddq, src.tau_est
            if hasattr(dst, "temperature"):
                dst.temperature = src.temperature
            if hasattr(dst, "lost"):
                dst.lost = src.lost
        m.foot_force = [int(x) for x in st.foot_force]
        return m

    def publish_state(self, state: LowState) -> None:
        if self.role == "sim":
            self.pub.publish(self._to_ros_state(state))

    def publish_cmd(self, cmd: LowCmd) -> None:
        if self.role == "controller":
            self.pub.publish(self._to_ros_cmd(cmd))

    def _spin_to_latest(self, counter: str) -> int:
        """把订阅队列**抽干**，只留最新一条。

        为什么必须抽干：rclpy.spin_once 每次只处理**一个** ready 回调。500 Hz 的
        对端 + 500 Hz 的轮询一旦有一次落后，队列就开始堆积，于是每一拍都返回"队列里
        最旧的那条"——延迟无界增长（实测丢一半消息、延迟被 QoS 深度顶在 ~10 条 = 20 ms，
        正好等于一个控制周期）。对 50 Hz 策略回路来说这是致命的：控制器在拿 20 ms 前的
        状态算指令。抽干之后延迟回到 ~0。
        """
        n = int(getattr(self, counter))
        drained = 0
        for _ in range(64):                     # 上限保护：单拍最多抽 64 条
            self._rclpy.spin_once(self.node, timeout_sec=0.0)
            new_n = int(getattr(self, counter))
            if new_n == n:
                break
            drained += new_n - n
            n = new_n
        if drained:
            self._n_drained += drained
            self._max_drain = max(self._max_drain, drained)
        return drained

    def recv_cmd(self) -> Optional[LowCmd]:
        self._spin_to_latest("_n_cmd_rx")
        cmd, self._latest_cmd = self._latest_cmd, None
        return cmd

    def recv_state(self) -> Optional[LowState]:
        self._spin_to_latest("_n_state_rx")
        st, self._latest = self._latest, None
        return st

    def close(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def stats(self) -> dict:
        return {"transport": self.name, "domain_id": self.domain_id,
                "cmd_topic": self.cmd_topic, "state_topic": self.state_topic,
                "rx_cmd": self._n_cmd_rx, "rx_state": self._n_state_rx,
                "drained_extra": self._n_drained, "drained_extra_max": self._max_drain}
