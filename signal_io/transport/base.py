"""传输层抽象接口（Transport）。

===============================================================================
1. 为什么需要这一层
===============================================================================

项目的业务代码（SimNode / ControllerNode / ros2_controller）**不认任何中间件**，
只调用下面 4 个方法。这样同一份控制代码可以：

    loopback  同进程直连        单进程调试 / tests / verify
    udp       127.0.0.1 双 socket 无 ROS 2 / 无 DDS 的机器上跑完整链路
    ros2      rclpy + rmw_cyclonedds_cpp   **真机 real_ros 与仿真 sim_ros**
    dds       unitree_sdk2py（raw DDS）    官方 SDK 路径（Python≤3.13）

"仿真 / 真机只差一个 --target" 就是靠这一层实现的。

===============================================================================
2. 两个角色、四个方法
===============================================================================

  role="sim"（数据源：仿真 MuJoCo / 未来硬件桥）
      publish_state(LowState)   把机器人状态发出去
      recv_cmd() -> LowCmd|None 取最新控制指令（非阻塞）

  role="controller"（策略控制器）
      recv_state() -> LowState|None  取最新机器人状态（非阻塞）
      publish_cmd(LowCmd)            发控制指令（500 Hz 零阶保持）

  两个角色都走同一份 ControllerNode / SimNode 代码，所以真机与仿真的
  控制时序完全一致（50 Hz 策略 + 500 Hz 电机层）。

===============================================================================
3. 实现约束（写新后端时必须遵守）
===============================================================================

  * recv_* 必须**非阻塞**：返回 None 表示"这一拍没有新消息"。
    主循环是 500 Hz 硬节拍（Pacer），不能在这里 sleep / 等消息。
  * recv_* 返回"最新一帧"而不是排队最旧的一帧：延迟不能随负载增长
    （ros2 后端用 _spin_to_latest 抽干队列，见 transport/ros2.py）。
  * publish_* 要允许 500 Hz 调用；内部做零阶保持/缓存是后端自己的事。
  * 线程模型由后端决定；目前都是单线程（主循环里 spin_once）。
  * close() 只释放自己创建的资源，不关闭进程级全局状态（例如不 rclpy.shutdown）。

===============================================================================
4. 消息契约
===============================================================================

内部消息定义在 signal_io/msg.py，字段与 unitree_go IDL 一一对应：
    LowCmd  : head/level_flag/motor_cmd[20]/crc + 本项目内部用的 seq/stamp
    LowState: tick/imu_state/motor_state[20]/foot_force[4]/bit_flag
真机 LowCmd 的 812 字节布局与 CRC 在 signal_io/go2_crc.py。

相关文件：
  signal_io/transport/{local,ros2,dds}.py   四个后端实现
  signal_io/transport/__init__.py           make_transport 工厂
  signal_io/config.py                       io_profiles -> transport 参数
  signal_io/node.py                         SimNode / ControllerNode（唯一调用方）
"""

from __future__ import annotations

import abc
from typing import Optional

from ..msg import LowCmd, LowState


class Transport(abc.ABC):
    """所有传输后端的抽象基类：四个核心方法 + 生命周期/统计。

    子类按 role 实现其中两个方向；未用到的方向保持空实现即可
    （例如 ControllerNode 不会调用 publish_state）。
    """

    name = "abstract"

    def open(self) -> None:
        """可选：显式打开资源（socket / DDS participant 等）。

        多数后端在 __init__ 里就完成了初始化，所以默认空实现。
        """

    def close(self) -> None:
        """释放本后端创建的资源（socket、node 等）；默认空实现。"""

    # ---------------------------------------------------------------- sim 侧
    @abc.abstractmethod
    def publish_state(self, state: LowState) -> None:
        """[sim] 发布机器人状态（500 Hz）。

        调用频率由 SimNode 决定（每个物理步一次）；后端内部可以再做降频
        （例如 ros2_sensors 的 /go2/odom 50 Hz、点云 15 Hz 不是走这里）。
        """

    @abc.abstractmethod
    def recv_cmd(self) -> Optional[LowCmd]:
        """[sim] 取最新控制指令；没有新消息返回 None（**不得阻塞**）。"""

    # ---------------------------------------------------------- controller 侧
    @abc.abstractmethod
    def recv_state(self) -> Optional[LowState]:
        """[controller] 取最新机器人状态；没有新消息返回 None（**不得阻塞**）。"""

    @abc.abstractmethod
    def publish_cmd(self, cmd: LowCmd) -> None:
        """[controller] 发布控制指令（500 Hz 零阶保持）。

        时序：ControllerNode 每 2 ms 调一次，内容在 50 Hz 策略更新之间保持不变。
        真机后端必须在这里填好固件要求的头/CRC（见 transport/ros2.py）。
        """

    # ---------------------------------------------------------------- 统计
    def stats(self) -> dict:
        """可选：运行时统计（接收条数、丢包、CRC 错误等），写进 --json summary。"""
        return {}
