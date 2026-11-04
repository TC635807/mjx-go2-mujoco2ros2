"""ROS 2 传输后端（真机 real_ros / 仿真 sim_ros 共用）。

===============================================================================
1. 这个文件在整个项目里的位置
===============================================================================

    控制器/仿真节点（业务代码，不认中间件）
                 │  publish_state() / recv_cmd() / recv_state() / publish_cmd()
                 ▼
          Transport 抽象接口（signal_io/transport/base.py）
                 │
     ┌───────────┼─────────────┬──────────────────┐
     ▼           ▼             ▼                  ▼
  loopback      udp          ros2（本文件）      dds（unitree_sdk2py）
  单进程      裸 UDP      rclpy + rmw_cyclonedds   raw DDS

本文件只做三件事：
  1. 把内部纯 Python 的 LowState/LowCmd（signal_io/msg.py）与 ROS 2 的
     unitree_go/msg/LowState、LowCmd 互相搬字段；
  2. 500 Hz 下把订阅队列"抽干"只留最新一条，避免延迟越积越大；
  3. 真机方向给 LowCmd 写固定头 + 812 字节布局的 CRC（signal_io/go2_crc.py）。

===============================================================================
2. 话题名：ROS 2 名 vs DDS 名（最容易错的一处）
===============================================================================

    本文件（ROS 2 层）            rmw_cyclonedds_cpp 自动映射        机器人固件
    /lowstate          ───────────────────────────────────────▶  rt/lowstate
    /lowcmd            ───────────────────────────────────────▶  rt/lowcmd

unitree_ros2 官方例程就是在 ROS 2 里发布 "/lowcmd"、订阅 "/lowstate"
（third_party/unitree_ros2/example/src/src/go2/go2_stand_example.cpp:11-12）。

WARN: 不要在 ROS 2 层写 "/rt/lowcmd" —— rmw 会再补一个 rt/ 前缀，变成 DDS 的
      "rt/rt/lowcmd"，真机永远收不到。raw DDS 后端（transport/dds.py）才用 rt/ 名。

===============================================================================
3. 两个角色（同一份类，方向相反）
===============================================================================

  role="sim"        （仿真数据源节点 / 未来硬件桥）
      pub  -> /lowstate  发布 LowState（500 Hz）
      sub  <- /lowcmd    接收 LowCmd，校验 CRC 后交给 SimNode

  role="controller" （策略控制器，真机与仿真同一个进程代码）
      pub  -> /lowcmd    发布 LowCmd（500 Hz 零阶保持，带 CRC）
      sub  <- /lowstate  接收 LowState，交给 ControllerNode

===============================================================================
4. 500 Hz 数据流（以真机为例）
===============================================================================

  机器人固件 ── rt/lowstate ──▶ sub 回调 _on_state() ──▶ self._latest（最新一帧）
                                                          │
  ControllerNode.run() 每 2 ms 调一次 recv_state()         │
      └─ _spin_to_latest()：循环 spin_once 直到没有新消息 ──┘
      └─ 返回"最新"一帧（队列里旧的被丢弃/覆盖）

  ControllerNode 每 2 ms 调 publish_cmd(cmd)：
      _to_ros_cmd()：内部 LowCmd -> ROS 2 LowCmd（固定头 + CRC，复用消息对象）
      pub.publish()：ROS 2 -> DDS rt/lowcmd -> 机器人固件

===============================================================================
5. 真机要点 / 常见失败模式
===============================================================================

  * CRC 错 -> 固件大概率静默丢帧（发 mode=1 但机器人的 motor_state.mode 一直是 0）。
    所以 crc_check=True 时收到坏 CRC 直接丢弃（_on_cmd），与固件语义一致。
  * 话题名写错（带 rt/）-> 能发出去但机器人没反应，且 ros2 topic list 里仍能看到
    自己发的话题，很容易误判为"通信正常"。
  * domain / 网卡不对 -> 连 /lowstate 都收不到；先跑 scripts/go2_real_check.py。
  * 500 Hz Python 发送在非 RT 机器上会抖：ControllerNode 的 pacer.overruns 要关注，
    必要时绑核 / RT 内核 / 改 C++ 发送。
  * LowState.tick 的物理含义：仿真里 tick = 物理步数（2 ms/步），真机固件里
    tick 按 1 ms 递增（官方 Basic_services 文档）。tick_dt 决定 sim_time 的换算。

相关文件：
  signal_io/msg.py                  内部消息定义（20 个电机槽位、本地 UDP 报文）
  signal_io/go2_crc.py              真机 LowCmd 812 字节布局 + CRC32
  signal_io/transport/base.py       Transport 抽象接口
  signal_io/transport/dds.py        raw DDS 后端（unitree_sdk2py）
  signal_io/config.py               io_profiles（domain/网卡/话题/tick_dt/crc_check）
  ros2_ws/launch/real.launch.py     真机节点图启动
  real_env/README.md                真机部署 checklist
"""

from __future__ import annotations

from typing import Optional

from ..go2_crc import lowcmd_crc
from ..msg import NUM_MOTOR, IMUState, LowCmd, LowState, MotorCmd, MotorState
from .base import Transport


def _scalar(v, default: int = 0) -> int:
    """把 ROS 2 定长数组字段安全地读成 int。

    unitree_go 的 MotorCmd.reserve 是 uint32[3]、LowCmd.head 是 uint8[2]，
    这些字段在 Python 里是 array-like，不能直接 int(v)；
    不同 ROS 2 版本对 array 的包装也不完全一致，所以统一走这个兼容函数。
    """
    try:
        return int(v)
    except (TypeError, ValueError):
        try:
            return int(v[0])
        except Exception:
            return default


class Ros2Transport(Transport):
    """rclpy + unitree_go/msg 的传输实现，仿真 / 真机同一份代码。

    生命周期：
        __init__() 里读环境变量（必须在 rclpy.init 之前由 config.apply_io_env 设好）
        -> rclpy.init()（如果还没初始化）-> 创建 Node -> 按 role 建 pub/sub。
        close() 只销毁 node，不调用 rclpy.shutdown()（同一进程可能还有别的节点）。

    线程模型：
        单线程。控制器主循环调用 recv_state()/recv_cmd() 时用 spin_once 处理回调，
        项目里不另起 executor。这也是为什么 create_subscription() 挂的回调
        （例如 /perception/height_scan）不需要额外线程。
    """

    name = "ros2"

    def __init__(self, domain_id: int = 1, crc_check: bool = True,
                 cmd_topic: str = "/lowcmd", state_topic: str = "/lowstate",
                 node_name: str = "go2_controller", role: str = "controller",
                 tick_dt: float = 0.002):
        """建传输对象。

        Args:
            domain_id: ROS_DOMAIN_ID。仿真建议 1，真机固件固定 0
                       （unitree_mujoco readme: "it is recommended to distinguish
                       from the real robot (default is 0 on the real robot)"）。
            crc_check: 是否启用真机 LowCmd CRC。
                       - real_ros：必须 True（固件校验）；
                       - sim_ros：也设 True，让 CRC 路径在仿真闭环里被验证；
                       - 纯仿真不关心 CRC 的场合可关，省一点 CPU。
            cmd_topic: ROS 2 话题名（不要带 rt/ 前缀，见模块 docstring §2）。
            state_topic: 同上。
            node_name: rclpy 节点名；真机建议保持唯一，避免和别的控制程序撞名。
            role: "sim"（发状态收指令）或 "controller"（发指令收状态）。
            tick_dt: LowState.tick 每 +1 对应多少秒。
                     仿真 = 0.002（tick 是物理步数），真机 = 0.001（固件按 1 ms 递增）。
                     它决定 st.sim_time，而控制器/健康层的时基都从这里来。
        """
        self.role = role
        self.domain_id = int(domain_id)
        self.crc_check = bool(crc_check)
        self.tick_dt = float(tick_dt)
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.node_name = node_name

        # 最新一帧缓存：回调只写这里，recv_* 只读并清空（"取走即消费"）。
        self._latest: Optional[LowState] = None
        self._latest_cmd: Optional[LowCmd] = None

        # 收到的消息计数：recv_* 用它判断"这一拍到底还有没有新消息"，从而把队列抽干。
        # （只看 _latest 是否非空不够：同一拍可能来了 0 条或 3 条。）
        self._n_state_rx = 0
        self._n_cmd_rx = 0

        # 抽干统计：drained_extra = 被丢弃的旧消息条数，用来观察延迟是否在累积。
        self._n_drained = 0
        self._max_drain = 0

        # 真机 CRC 统计与缓存（见 _on_cmd / _cmd_crc）。
        self._n_bad_crc = 0
        self._crc_key = None
        self._crc_val = 0

        self._load()

    # ---------------------------------------------------------------- 初始化
    def _load(self) -> None:
        """导入 rclpy/unitree_go、初始化 rclpy、创建节点与 pub/sub。

        Raises:
            RuntimeError: 没有装 ROS 2 或没有编译 unitree_go 消息包时给出可操作提示。
        """
        try:
            import rclpy
            from rclpy.node import Node
            from unitree_go.msg import LowCmd as RosLowCmd
            from unitree_go.msg import LowState as RosLowState
        except Exception as exc:
            raise RuntimeError(
                "ROS 2 后端需要 rclpy 与 unitree_go/msg。\n"
                "  1) 装 ROS 2 Lyrical (Ubuntu 26.04 是它的 Tier-1 平台)\n"
                "  2) 编译 unitree_ros2 拿到 unitree_go 消息包（见 docs/ros2-setup.md §3）\n"
                "  3) source setup.bash 之后再跑\n"
                "原始错误: " + repr(exc)
            ) from exc

        self._rclpy = rclpy
        self._RosLowCmd = RosLowCmd
        self._RosLowState = RosLowState
        # rclpy.init 必须只做一次；同一进程里 sim 和 controller 不会同时建两个
        # transport，但测试/工具可能复用进程，所以这里判断一下。
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node(self.node_name)

        # 500 Hz 热路径：消息对象和 MotorCmd 类只创建一次，之后原地改字段。
        # 旧实现每个电机都 self._RosLowCmd() 构造一条 20 电机消息 = 21 条/拍，
        # 实测 _to_ros_cmd 0.72 ms（2 ms 预算的 36%）；复用后是 0.013 ms。
        # 复用是安全的：rclpy 的 publish() 会在返回前同步序列化。
        self._RosMotorCmd = self._RosLowCmd().motor_cmd[0].__class__
        self._ros_cmd_msg = self._RosLowCmd()
        self._ros_state_msg = self._RosLowState()

        if self.role == "sim":
            # 仿真数据源：发 LowState、收 LowCmd。
            self.pub = self.node.create_publisher(RosLowState, self.state_topic, 10)
            self.sub = self.node.create_subscription(RosLowCmd, self.cmd_topic, self._on_cmd, 10)
        else:
            # 控制器：发 LowCmd、收 LowState。
            self.pub = self.node.create_publisher(RosLowCmd, self.cmd_topic, 10)
            self.sub = self.node.create_subscription(RosLowState, self.state_topic, self._on_state, 10)

    # ------------------------------------------------------- 给上层挂额外 pub/sub
    def create_publisher(self, msg_type, topic: str, qos=10):
        """在同一个 rclpy node 上挂额外发布者（例如 /go2/controller/obs）。

        复用同一个 node 的好处：控制器主循环 spin_once 时这些发布者/订阅者
        都能被正常处理，不需要再起 executor 或线程。
        """
        return self.node.create_publisher(msg_type, topic, qos)

    def create_subscription(self, msg_type, topic: str, callback, qos=10):
        """在同一个 rclpy node 上挂额外订阅（例如 /perception/height_scan）。

        控制器主循环调 recv_state() 时会 spin_once，所以这里挂的回调会被正常处理，
        不需要再起一个 executor。
        """
        return self.node.create_subscription(msg_type, topic, callback, qos)

    # ---------------------------------------------------------------- 收状态
    def _on_state(self, m) -> None:
        """ROS 2 LowState 回调：搬字段到内部 LowState，只保留最新一帧。

        字段对应关系（unitree_go/msg/LowState -> signal_io/msg.py LowState）：
            tick / imu_state(quat, gyro, acc, rpy) / motor_state[20]
            (mode,q,dq,ddq,tau_est,temperature,lost) / foot_force[4] / bit_flag

        只搬项目用到/健康层需要的字段；q_raw、bms_state、power_v 等暂不搬。
        """
        st = LowState()
        st.tick = int(getattr(m, "tick", 0))
        # 真机固件 tick 按 1 ms 递增，仿真按物理步（2 ms）递增；tick_dt 在 __init__ 里定。
        st.sim_time = float(st.tick) * self.tick_dt
        st.mode_machine = int(getattr(m, "mode_machine", 0))

        imu = m.imu_state
        st.imu_state = IMUState(
            quaternion=[float(x) for x in imu.quaternion],   # (w, x, y, z)
            gyroscope=[float(x) for x in imu.gyroscope],     # rad/s，机体系
            accelerometer=[float(x) for x in imu.accelerometer],  # m/s^2，含重力
            rpy=[float(x) for x in imu.rpy],                 # rad
        )

        # Go2 只用前 12 个电机槽位（顺序 FR,FL,RR,RL × hip,thigh,calf）；
        # 后 8 个是 unitree_go 给 20 电机机型预留的，真机也会填（通常是 0/未使能）。
        for i in range(NUM_MOTOR):
            src = m.motor_state[i]
            st.motor_state[i] = MotorState(
                mode=_scalar(getattr(src, "mode", 0)),
                q=float(src.q), dq=float(src.dq),
                ddq=float(getattr(src, "ddq", 0.0)),
                tau_est=float(getattr(src, "tau_est", 0.0)),
                temperature=_scalar(getattr(src, "temperature", 0)),
                lost=_scalar(getattr(src, "lost", 0)),
            )

        # 足端力顺序：FR, FL, RR, RL；Go2 AIR/PRO 可能没有该传感器（恒 0）。
        st.foot_force = [int(x) for x in getattr(m, "foot_force", [0, 0, 0, 0])]
        # bit_flag：bit0=操作指令超时、bit5=遥控超时、bit7=电机超时（官方 Basic_services）。
        # 健康层用它判断"我们的 lowcmd 是否真的到达了机器人"。
        st.bit_flag = int(getattr(m, "bit_flag", 0)) & 0xFF

        self._latest = st          # 覆盖旧值：消费者只要最新一帧
        self._n_state_rx += 1

    # ---------------------------------------------------------------- 收指令
    def _on_cmd(self, m) -> None:
        """ROS 2 LowCmd 回调（仅 role="sim"）：搬字段 + 校验 CRC。

        CRC 语义与真机固件保持一致：坏 CRC 直接丢弃，不交给 SimNode。
        这样 sim_ros 闭环就能提前暴露"CRC 实现/字段布局写错"的问题，
        而不是等到上真机才发现电机不响应。
        """
        cmd = LowCmd()
        cmd.mode_pr = int(getattr(m, "mode_pr", 0))
        cmd.mode_machine = int(getattr(m, "mode_machine", 0))
        # unitree_go LowCmd 有固定头；unitree_hg（G1/H1）没有。用 hasattr 兼容两种 IDL。
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
                mode=_scalar(getattr(src, "mode", 0)),
                q=float(src.q), dq=float(src.dq), tau=float(src.tau),
                kp=float(src.kp), kd=float(src.kd),
                reserve=_scalar(getattr(src, "reserve", 0)),
            )

        # TODO(real): 固件是否严格校验 crc、错 CRC 时是静默丢帧还是报 bit_flag，
        # 需要在真机上用"正确 CRC vs 故意错 CRC"对照实验确认。
        msg_crc = _scalar(getattr(m, "crc", 0)) & 0xFFFFFFFF
        cmd.crc = msg_crc
        if self.crc_check:
            if not msg_crc or msg_crc != lowcmd_crc(cmd):
                # 真机固件对错 CRC 大概率静默丢帧；仿真侧照同样语义丢弃。
                self._n_bad_crc += 1
                self._n_cmd_rx += 1
                return
        self._latest_cmd = cmd
        self._n_cmd_rx += 1

    def _cmd_crc(self, cmd: LowCmd) -> int:
        """计算真机 LowCmd.crc，并缓存（500 Hz 零阶保持时大多数拍指令不变）。

        为什么缓存：CRC 要打包 812 字节 + 扫 202 个 uint32，实测约 0.1 ms；
        50 Hz 策略更新之间指令完全一样，500 Hz 下发时 9/10 拍可以命中缓存。
        key 覆盖所有参与 CRC 的字段（head/level/frame_reserve/gpio + 20 个电机）。
        """
        key = (int(cmd.level_flag), int(cmd.head[0]), int(cmd.head[1]),
               int(getattr(cmd, "frame_reserve", 0)), int(cmd.gpio),
               tuple((int(c.mode), c.q, c.dq, c.tau, c.kp, c.kd, int(c.reserve))
                     for c in cmd.motor_cmd))
        if key != self._crc_key:
            self._crc_key = key
            self._crc_val = lowcmd_crc(cmd)
        return self._crc_val

    # ------------------------------------------------------------ 内部 -> ROS 2
    def _to_ros_cmd(self, cmd: LowCmd):
        """内部 LowCmd -> ROS 2 LowCmd 消息（原地改复用消息对象）。

        真机固件要求的固定字段（缺一个都可能被静默丢弃）：
            head = [0xFE, 0xEF]   level_flag = 0xFF   motor_cmd[i].mode = 0x01
            crc = crc32_core(812 字节布局，见 signal_io/go2_crc.py)
        """
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
            mc.mode = int(c.mode)           # 0x01 = FOC/伺服使能；0x00 = stop
            mc.q, mc.dq, mc.tau = c.q, c.dq, c.tau
            mc.kp, mc.kd = c.kp, c.kd
            res = mc.reserve                   # uint32[3]
            res[0], res[1], res[2] = int(c.reserve), 0, 0

        # 机器人固件校验 CRC；sim_ros profile 的 crc_check=true，这条路径在仿真里同样被验证。
        m.crc = int(self._cmd_crc(cmd)) if self.crc_check else 0
        return m

    def _to_ros_state(self, st: LowState):
        """内部 LowState -> ROS 2 LowState 消息（原地改复用消息对象，仅仿真数据源用）。"""
        m = self._ros_state_msg       # 复用消息对象（500 Hz 热路径，避免每拍分配）
        m.tick = int(st.tick)

        # IMU：quaternion(w,x,y,z) / gyroscope / accelerometer / rpy。
        # 用逐元素赋值而不是整段 list 赋值：rosidl 的 array 字段对 list 赋值
        # 在不同版本上行为不完全一致，逐元素是官方例程的写法。
        imu = m.imu_state
        for k in range(4):
            imu.quaternion[k] = float(st.imu_state.quaternion[k])
        for arr_dst, arr_src in ((imu.gyroscope, st.imu_state.gyroscope),
                                 (imu.accelerometer, st.imu_state.accelerometer),
                                 (imu.rpy, st.imu_state.rpy)):
            for k in range(3):
                arr_dst[k] = float(arr_src[k])

        # 20 个电机槽位；Go2 只用前 12 个。
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

    # ------------------------------------------------------------ 对外发布
    def publish_state(self, state: LowState) -> None:
        """role="sim" 时把 LowState 发到 /lowstate（控制器/观察器订阅）。"""
        if self.role == "sim":
            self.pub.publish(self._to_ros_state(state))

    def publish_cmd(self, cmd: LowCmd) -> None:
        """role="controller" 时把 LowCmd 发到 /lowcmd（真机固件/仿真数据源订阅）。"""
        if self.role == "controller":
            self.pub.publish(self._to_ros_cmd(cmd))

    # ------------------------------------------------------------ 收消息工具
    def _spin_to_latest(self, counter: str) -> int:
        """把订阅队列**抽干**，只留最新一条。

        为什么必须抽干：rclpy.spin_once 每次只处理**一个** ready 回调。500 Hz 的
        对端 + 500 Hz 的轮询一旦有一次落后，队列就开始堆积，于是每一拍都返回"队列里
        最旧的那条"——延迟无界增长（实测丢一半消息、延迟被 QoS 深度顶在 ~10 条 = 20 ms，
        正好等于一个控制周期）。对 50 Hz 策略回路来说这是致命的：控制器在拿 20 ms 前的
        状态算指令。抽干之后延迟回到 ~0。

        Args:
            counter: "_n_state_rx" 或 "_n_cmd_rx"，用消息计数判断是否还有新消息。
        Returns:
            本拍额外抽掉的旧消息条数（0 表示这一拍没有积压）。
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
        """role="sim" 取最新指令；没新消息返回 None（不阻塞）。"""
        self._spin_to_latest("_n_cmd_rx")
        cmd, self._latest_cmd = self._latest_cmd, None
        return cmd

    def recv_state(self) -> Optional[LowState]:
        """role="controller" 取最新状态；没新消息返回 None（不阻塞）。"""
        self._spin_to_latest("_n_state_rx")
        st, self._latest = self._latest, None
        return st

    # ------------------------------------------------------------ 生命周期/统计
    def close(self) -> None:
        """销毁 rclpy 节点（不 shutdown rclpy：同进程可能还有其他节点）。"""
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def stats(self) -> dict:
        """给 --json summary / 观察器用的运行时统计。

        bad_crc：role="sim" 收到的坏 CRC 指令数（正常应恒 0）；
        drained_extra / max：抽干掉的旧消息条数，持续 >0 说明消费速度跟不上 500 Hz。
        """
        return {"transport": self.name, "domain_id": self.domain_id,
                "cmd_topic": self.cmd_topic, "state_topic": self.state_topic,
                "rx_cmd": self._n_cmd_rx, "rx_state": self._n_state_rx,
                "bad_crc": self._n_bad_crc,
                "drained_extra": self._n_drained, "drained_extra_max": self._max_drain}
