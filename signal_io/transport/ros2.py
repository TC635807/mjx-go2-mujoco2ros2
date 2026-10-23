"""ROS 2 后端：rclpy + unitree_ros2 的 unitree_go/msg。

话题名（重要）: ROS 2 层用 **/lowcmd、/lowstate**，与 unitree_ros2 官方例程
（go2_stand_example.cpp 里的 `#define TOPIC_LOWCMD "/lowcmd"`）一致。
rmw_cyclonedds_cpp 会把 ROS 2 话题映射成 DDS 话题 "rt/" + 名字，也就是机器人固件
真正监听的 rt/lowcmd、rt/lowstate。所以 **不要**在 ROS 2 层写 "rt/lowcmd"
（那会变成 DDS 的 rt/rt/lowcmd，真机收不到）。

和 DDS 后端的关系: unitree_ros2 本身就是 CycloneDDS 上的 ROS 2 封装
（rmw_cyclonedds_cpp），所以仿真和真机之间只差 ROS_DOMAIN_ID 和网卡。

装好 ROS 2 之后:
    source /opt/ros/lyrical/setup.bash
    export ROS_DOMAIN_ID=1                 # 仿真；真机默认是 0
    python sim_env/run_sim.py --target sim_ros --duration 20 --viewer
"""

from __future__ import annotations

from typing import Optional

from ..go2_crc import lowcmd_crc
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
                 cmd_topic: str = "/lowcmd", state_topic: str = "/lowstate",
                 node_name: str = "go2_controller", role: str = "controller",
                 tick_dt: float = 0.002):
        self.role = role
        self.domain_id = int(domain_id)
        self.crc_check = bool(crc_check)
        # LowState.tick 每 +1 的秒数：仿真 = 2 ms/物理步，真机固件 = 1 ms
        self.tick_dt = float(tick_dt)
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
        # 真机固件校验 LowCmd.crc；这里按 unitree_go LowCmd_ 的 812 字节布局算
        self._n_bad_crc = 0
        self._crc_key = None
        self._crc_val = 0
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
        # 500 Hz 热路径：消息对象和 MotorCmd 类只创建一次，之后原地改字段。
        # （旧实现每个电机都 self._RosLowCmd() 构造一条 20 电机消息 = 21 条/拍，
        #   实测 _to_ros_cmd 0.72 ms，2 ms 预算被吃掉 1/3。）
        self._RosMotorCmd = self._RosLowCmd().motor_cmd[0].__class__
        self._ros_cmd_msg = self._RosLowCmd()
        self._ros_state_msg = self._RosLowState()
        if self.role == "sim":
            self.pub = self.node.create_publisher(RosLowState, self.state_topic, 10)
            self.sub = self.node.create_subscription(RosLowCmd, self.cmd_topic, self._on_cmd, 10)
        else:
            self.pub = self.node.create_publisher(RosLowCmd, self.cmd_topic, 10)
            self.sub = self.node.create_subscription(RosLowState, self.state_topic, self._on_state, 10)

    def create_publisher(self, msg_type, topic: str, qos=10):
        """在同一个 rclpy node 上挂发布者（例如 /go2/controller/debug）。"""
        return self.node.create_publisher(msg_type, topic, qos)

    def create_subscription(self, msg_type, topic: str, callback, qos=10):
        """在同一个 rclpy node 上挂额外订阅（例如 /perception/height_scan）。

        控制器主循环调 recv_state() 时会 spin_once，所以这里挂的回调会被正常处理，
        不需要再起一个 executor。
        """
        return self.node.create_subscription(msg_type, topic, callback, qos)

    def _on_state(self, m) -> None:
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
                mode=_scalar(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                ddq=float(getattr(src, "ddq", 0.0)), tau_est=float(getattr(src, "tau_est", 0.0)),
                temperature=_scalar(getattr(src, "temperature", 0)), lost=_scalar(getattr(src, "lost", 0)),
            )
        st.foot_force = [int(x) for x in getattr(m, "foot_force", [0, 0, 0, 0])]
        st.bit_flag = int(getattr(m, "bit_flag", 0)) & 0xFF
        self._latest = st
        self._n_state_rx += 1

    def _on_cmd(self, m) -> None:
        cmd = LowCmd()
        cmd.mode_pr = int(getattr(m, "mode_pr", 0))
        cmd.mode_machine = int(getattr(m, "mode_machine", 0))
        if hasattr(m, "head"):
            cmd.head = [_scalar(m.head[0]), _scalar(m.head[1])]
        if hasattr(m, "level_flag"):
            cmd.level_flag = _scalar(m.level_flag)
        if hasattr(m, "frame_reserve"):
            cmd.frame_reserve = _scalar(m.frame_reserve)
        if hasattr(m, "gpio"):
            cmd.gpio = _scalar(m.gpio)
        for i in range(NUM_MOTOR):
            src = m.motor_cmd[i]
            cmd.motor_cmd[i] = MotorCmd(
                mode=_scalar(getattr(src, "mode", 0)), q=float(src.q), dq=float(src.dq),
                tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                reserve=_scalar(getattr(src, "reserve", 0)),
            )
        # 真机固件会校验 CRC；仿真侧也校验一遍，让这条路径在 sim2real 之前就被测到。
        msg_crc = _scalar(getattr(m, "crc", 0)) & 0xFFFFFFFF
        cmd.crc = msg_crc
        if self.crc_check:
            if not msg_crc or msg_crc != lowcmd_crc(cmd):
                # 真机固件对错 CRC 大概率静默丢帧；仿真侧照同样语义丢弃，
                # 这样 CRC 回归在 sim_ros 闭环里就能暴露，而不是只记一个计数。
                self._n_bad_crc += 1
                self._n_cmd_rx += 1
                return
        self._latest_cmd = cmd
        self._n_cmd_rx += 1

    def _cmd_crc(self, cmd: LowCmd) -> int:
        """500 Hz 零阶保持时大多数拍指令没变，缓存 CRC，避免每拍都算 202 个字。"""
        key = (int(cmd.level_flag), int(cmd.head[0]), int(cmd.head[1]),
               int(getattr(cmd, "frame_reserve", 0)), int(cmd.gpio),
               tuple((int(c.mode), c.q, c.dq, c.tau, c.kp, c.kd, int(c.reserve))
                     for c in cmd.motor_cmd))
        if key != self._crc_key:
            self._crc_key = key
            self._crc_val = lowcmd_crc(cmd)
        return self._crc_val

    def _to_ros_cmd(self, cmd: LowCmd):
        m = self._ros_cmd_msg           # 复用同一个消息对象（publish 同步序列化，安全）
        # unitree_go/msg/LowCmd 里没有 mode_pr / mode_machine（那是 unitree_hg 才有的），
        # 所以要 hasattr 兜一下，别把后端写死成某一种 IDL。
        if hasattr(m, "mode_pr"):
            m.mode_pr = cmd.mode_pr
        if hasattr(m, "mode_machine"):
            m.mode_machine = cmd.mode_machine
        # 真机下发的 LowCmd 必须带官方固定头，否则固件不认。
        m.head[0] = int(cmd.head[0]) & 0xFF
        m.head[1] = int(cmd.head[1]) & 0xFF
        m.level_flag = int(cmd.level_flag) & 0xFF
        m.frame_reserve = int(getattr(cmd, "frame_reserve", 0)) & 0xFF
        m.gpio = int(cmd.gpio) & 0xFF
        for i in range(NUM_MOTOR):
            c, mc = cmd.motor_cmd[i], m.motor_cmd[i]
            mc.mode = int(c.mode)
            mc.q, mc.dq, mc.tau = c.q, c.dq, c.tau
            mc.kp, mc.kd = c.kp, c.kd
            res = mc.reserve                   # uint32[3]
            res[0], res[1], res[2] = int(c.reserve), 0, 0
        # 机器人固件校验 CRC；sim_ros profile 的 crc_check=true，这条路径在仿真里同样被验证。
        m.crc = int(self._cmd_crc(cmd)) if self.crc_check else 0
        return m

    def _to_ros_state(self, st: LowState):
        m = self._ros_state_msg       # 复用消息对象（500 Hz 热路径，避免每拍分配）
        m.tick = int(st.tick)
        imu = m.imu_state
        for k in range(4):
            imu.quaternion[k] = float(st.imu_state.quaternion[k])
        for arr_dst, arr_src in ((imu.gyroscope, st.imu_state.gyroscope),
                                 (imu.accelerometer, st.imu_state.accelerometer),
                                 (imu.rpy, st.imu_state.rpy)):
            for k in range(3):
                arr_dst[k] = float(arr_src[k])
        for i in range(NUM_MOTOR):
            src, dst = st.motor_state[i], m.motor_state[i]
            dst.mode, dst.q, dst.dq = src.mode, src.q, src.dq
            dst.ddq, dst.tau_est = src.ddq, src.tau_est
            if hasattr(dst, "temperature"):
                dst.temperature = src.temperature
            if hasattr(dst, "lost"):
                dst.lost = src.lost
        for k in range(4):
            m.foot_force[k] = int(st.foot_force[k])
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
                "bad_crc": self._n_bad_crc,
                "drained_extra": self._n_drained, "drained_extra_max": self._max_drain}
