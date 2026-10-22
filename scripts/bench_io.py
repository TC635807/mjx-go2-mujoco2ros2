#!/usr/bin/env python3
"""量一下"Python 到底慢不慢"：把 500 Hz 这一拍的预算拆开看。

    source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
    export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages ROS_DOMAIN_ID=1
    .venv/bin/python scripts/bench_io.py

每一拍（2 ms = 2000 us）里要干的事：
    物理一步 + 打包 LowState + DDS/ROS2 发出去 + 收 LowCmd + 解包 + 施加力矩
"""

from __future__ import annotations

import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import mujoco  # noqa: E402

from signal_io.config import load_config, model_xml  # noqa: E402
from signal_io.msg import LowCmd, LowState  # noqa: E402
from sim_env.robot import Go2Sim, make_stand_cmd  # noqa: E402

BUDGET_US = 2000.0
rows = []


def report(name: str, samples, unit: str = "us"):
    xs = sorted(samples)
    med = statistics.median(xs)
    p95 = xs[int(len(xs) * 0.95) - 1] if len(xs) > 1 else xs[0]
    mx = xs[-1]
    rows.append((name, med, p95, mx))
    print("  %-34s 中位 %8.1f %s  p95 %8.1f %s  最大 %8.1f %s   占 2ms 预算 %5.1f%%"
          % (name, med, unit, p95, unit, mx, unit, 100.0 * med / BUDGET_US))


def main() -> int:
    cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "go2_sim.json"))
    sim = Go2Sim(model_xml(cfg), sim_dt=cfg["sim_dt"])
    cmd = make_stand_cmd(sim.stand_pose, 60.0, 5.0)
    for _ in range(500):                      # 先站稳
        sim.write_cmd(cmd); sim.step(1)

    print("== 纯 Python / MuJoCo 侧 ==")
    N = 5000
    t = []
    for _ in range(N):
        t0 = time.perf_counter(); mujoco.mj_step(sim.model, sim.data, 1); t.append((time.perf_counter() - t0) * 1e6)
    report("mujoco mj_step（物理一步）", t)

    t = []
    for _ in range(2000):
        t0 = time.perf_counter(); st = sim.read_state(); t.append((time.perf_counter() - t0) * 1e6)
    report("read_state()（组 LowState）", t)

    t = []
    for _ in range(2000):
        t0 = time.perf_counter(); sim.write_cmd(cmd); t.append((time.perf_counter() - t0) * 1e6)
    report("write_cmd()（12 关节 PD + 限幅）", t)

    st = sim.read_state()
    t = []
    for _ in range(5000):
        t0 = time.perf_counter(); raw = st.pack(); t.append((time.perf_counter() - t0) * 1e6)
    report("LowState.pack() + CRC32", t)
    t = []
    for _ in range(5000):
        t0 = time.perf_counter(); LowState.unpack(raw); t.append((time.perf_counter() - t0) * 1e6)
    report("LowState.unpack() + CRC 校验", t)
    t = []
    c2 = cmd.pack()
    for _ in range(5000):
        t0 = time.perf_counter(); LowCmd.unpack(c2); t.append((time.perf_counter() - t0) * 1e6)
    report("LowCmd.unpack()", t)

    print("\n== ROS 2 (rclpy + CycloneDDS) ==")
    try:
        import rclpy
        from rclpy.node import Node
        from unitree_go.msg import LowCmd as RosLowCmd
        from unitree_go.msg import LowState as RosLowState
        from signal_io.transport.ros2 import Ros2Transport
    except Exception as exc:
        print("  跳过（没装 ROS 2 或 unitree_go）:", exc)
        _summary()
        return 0

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("bench_io")
    pub_state = node.create_publisher(RosLowState, "/bench_state", 10)
    pub_cmd = node.create_publisher(RosLowCmd, "/bench_cmd", 10)
    recv = {"n": 0, "lat": []}
    clock = {"t": 0.0}

    def on_cmd(m):
        recv["n"] += 1
        recv["lat"].append((time.perf_counter() - clock["t"]) * 1e6)

    node.create_subscription(RosLowCmd, "/bench_cmd", on_cmd, 10)
    tr = Ros2Transport.__new__(Ros2Transport)     # 只借它的转换函数
    tr._RosLowCmd, tr._RosLowState = RosLowCmd, RosLowState
    tr.role = "controller"
    ros_state = tr._to_ros_state(st)
    ros_cmd = tr._to_ros_cmd(cmd)

    for _ in range(200):                          # 等发现
        pub_state.publish(ros_state); rclpy.spin_once(node, timeout_sec=0.002)
    recv["lat"].clear(); recv["n"] = 0

    N = 1500
    t = []
    for _ in range(N):
        t0 = time.perf_counter(); pub_state.publish(ros_state); t.append((time.perf_counter() - t0) * 1e6)
    report("ROS2 publish(LowState)（本端）", t)

    t = []
    for _ in range(N):
        t0 = time.perf_counter(); rclpy.spin_once(node, timeout_sec=0.0); t.append((time.perf_counter() - t0) * 1e6)
    report("ROS2 spin_once（空转一圈）", t)

    # 端到端：发 LowCmd -> 回调收到（同进程，共用同一个时钟）
    lat = []
    for _ in range(500):
        clock["t"] = time.perf_counter()
        pub_cmd.publish(ros_cmd)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.0)
            if recv["n"] > len(lat):
                break
        if len(recv["lat"]) > len(lat):
            lat.append(recv["lat"][-1])
    if lat:
        report("ROS2 端到端（发->回调）", lat)
    tr._rclpy = rclpy
    node.destroy_node()

    print("\n== 结论 ==")
    _summary()
    return 0


def _summary() -> None:
    total = 0.0
    for name, med, _p95, _mx in rows:
        if "物理一步" in name or "组 LowState" in name or "12 关节" in name or "LowState.pack" in name:
            total += med
    print("  一个 500 Hz 控制拍需要做的（物理+打包+发状态+施加力矩）合计约 %.1f us = %.1f%% 预算"
          % (total, 100.0 * total / BUDGET_US))
    print("  注意：这是单线程串行跑。真机侧的 500 Hz 电机层如果也放 Python，"
          "还要加上 OS 调度抖动（本机 WSL2 实测 p98 抖动 2~5 ms）。")


if __name__ == "__main__":
    raise SystemExit(main())
