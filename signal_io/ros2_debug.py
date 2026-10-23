#!/usr/bin/env python3
"""控制器的 ROS 2 调试发布：把观察器需要的内部状态发到话题。

观察器节点（scripts/view_ros2.py）不需要知道控制器内部类，只订阅：

    /go2/controller/obs       std_msgs/Float32MultiArray   243 维观测
    /go2/controller/action    std_msgs/Float32MultiArray   12 维策略原始动作
    /go2/controller/info      std_msgs/String（JSON）      健康/模式/速度来源/统计

这些话题只在 `run_controller.py --publish-debug` 时发布；不接观察器时可以关掉。
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

import numpy as np


class ControllerDebugPublisher:
    def __init__(self, transport) -> None:
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
        from std_msgs.msg import Float32MultiArray, String

        obs = getattr(ctrl, "last_obs", None)
        if obs is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in np.asarray(obs, dtype=np.float64).reshape(-1)]
            self.obs_pub.publish(msg)
            self.n_obs += 1

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
        return {"n_obs": self.n_obs, "n_action": self.n_action, "n_info": self.n_info}


__all__ = ["ControllerDebugPublisher"]
