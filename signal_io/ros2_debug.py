#!/usr/bin/env python3
"""控制器的 ROS 2 调试发布：把观察器需要的内部状态发到话题。

===============================================================================
1. 为什么需要它
===============================================================================

  观察器（scripts/view_ros2.py）是一个独立的 ROS 2 节点，**不接触控制器内部对象**。
  控制器要把"策略看到了什么/输出了什么/健康等级"暴露出来，就只能通过话题：

      /go2/controller/obs       std_msgs/Float32MultiArray   243 维观测
      /go2/controller/action    std_msgs/Float32MultiArray   12 维策略原始动作
      /go2/controller/info      std_msgs/String（JSON）      健康/模式/速度来源/统计

  只有 run_controller.py 带 --publish-debug 时才发布；不接观察器时可以关掉，
  省掉 50 Hz 的序列化开销。

===============================================================================
2. 频率与内容
===============================================================================

  obs/action：每个 50 Hz 控制拍发一次（策略更新频率；500 Hz 发没意义）。
  info：10 Hz（人眼足够，避免 50 Hz JSON 序列化开销），内容来自
        controller.stats() + health_state + supervisor.mode。

  真机与仿真共用：观察器不关心数据来自 MuJoCo 还是真机。

相关文件：
  ros2_controller/run_controller.py   --publish-debug 时创建本类
  signal_io/node.py                   ControllerNode.on_control 回调入口
  scripts/view_ros2.py                订阅方
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

import numpy as np


class ControllerDebugPublisher:
    """把控制器内部状态发布成观察器能订阅的三个话题。"""

    def __init__(self, transport) -> None:
        """在控制器的 rclpy node 上挂三个发布者。

        Args:
            transport: 必须是 ros2 后端（内部用 create_publisher）；
                       其它后端没有 ROS 2 node，--publish-debug 会提前报错。
        """
        from std_msgs.msg import Float32MultiArray, String

        self.transport = transport
        self.obs_pub = transport.create_publisher(
            Float32MultiArray, "/go2/controller/obs", 10)
        self.action_pub = transport.create_publisher(
            Float32MultiArray, "/go2/controller/action", 10)
        self.info_pub = transport.create_publisher(String, "/go2/controller/info", 10)
        self._last_info_t = -1.0e18
        self.n_obs = 0
        self.n_action = 0
        self.n_info = 0

    def publish(self, state, cmd, ctrl, health_state=None, supervisor=None,
                now: float = 0.0) -> None:
        """每个 50 Hz 控制拍后由 ControllerNode.on_control 调用。

        Args:
            state: 当前 LowState（本类暂未直接使用）。
            cmd: 当前 LowCmd（本类暂未直接使用）。
            ctrl: 控制器对象；读取 last_obs / last_action / stats()。
            health_state: HealthState（提供 level / level_name）。
            supervisor: SafetySupervisor（提供 mode）。
            now: 控制器逻辑时钟（秒），用于 info 的 10 Hz 节流。
        """
        from std_msgs.msg import Float32MultiArray, String

        # 观测：策略实际看到的 243 维（含 scan/相位/历史帧，取决于策略档）。
        obs = getattr(ctrl, "last_obs", None)
        if obs is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in np.asarray(obs, dtype=np.float64).reshape(-1)]
            self.obs_pub.publish(msg)
            self.n_obs += 1

        # 动作：策略原始输出（12 维，未乘 action_scale、未加 default pose）。
        action = getattr(ctrl, "last_action", None)
        if action is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in np.asarray(action, dtype=np.float64).reshape(-1)]
            self.action_pub.publish(msg)
            self.n_action += 1

        # 10 Hz 足够人眼；避免 50 Hz 的 JSON 序列化开销
        if now - self._last_info_t >= 0.1:
            self._last_info_t = float(now)
            info: Dict[str, Any] = {}
            if hasattr(ctrl, "stats"):
                try:
                    info.update(ctrl.stats())
                except Exception:
                    pass
            info["level"] = int(getattr(health_state, "level", 0))
            info["level_name"] = str(getattr(health_state, "level_name", "L0"))
            info["mode"] = str(getattr(supervisor, "mode", "track"))
            info["timestamp"] = time.time()
            msg = String()
            msg.data = json.dumps(info, ensure_ascii=False)
            self.info_pub.publish(msg)
            self.n_info += 1

    def stats(self) -> Dict[str, int]:
        """发布条数，写进 run_controller 的 --json summary。"""
        return {"n_obs": self.n_obs, "n_action": self.n_action, "n_info": self.n_info}


__all__ = ["ControllerDebugPublisher"]
