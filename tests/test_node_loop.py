#!/usr/bin/env python3
"""tests/test_node_loop.py —— ControllerNode 控制循环 / 健康时基 / 退出收尾回归。

背景（真机首测前修掉的问题，见 docs/go2-sim2real-references.md 与 real_env/README.md）：
  1. 没收到 LowState 时 duration 必须生效（旧实现 continue 跳过 k+=1，进程永不退出）；
  2. LowState 断流后 health 必须按墙钟升级 L1/L2/L3，且控制器不能再假装 L0；
  3. 退出时真机要能发阻尼收尾（kp=0, kd=8）；
  4. 真机 LowState 的 lost / temperature / bit_flag 要参与健康判定。

直接运行：python tests/test_node_loop.py
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ros2_controller.controller import ZeroController  # noqa: E402
from ros2_controller.health import HealthMonitor, Level  # noqa: E402
from signal_io.msg import LowCmd, LowState, MotorState  # noqa: E402
from signal_io.node import ControllerNode  # noqa: E402
from signal_io.transport.base import Transport  # noqa: E402

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


class FakeTransport(Transport):
    """可编程的假传输：先给 n_state 帧状态，然后返回 None（模拟断流）。"""

    name = "fake"

    def __init__(self, n_state: int = 0):
        self.n_state = int(n_state)
        self.i = 0
        self.cmds = []

    def publish_state(self, state) -> None:
        pass

    def recv_cmd(self):
        return None

    def recv_state(self):
        if self.i < self.n_state:
            self.i += 1
            st = LowState()
            st.tick = self.i
            st.sim_time = self.i * 0.002
            return st
        return None

    def publish_cmd(self, cmd) -> None:
        self.cmds.append(cmd)

    def stats(self):
        return {"transport": "fake"}


def load_cfg() -> dict:
    return json.load(open(os.path.join(ROOT, "config", "go2_sim.json"), encoding="utf-8"))


def main() -> int:
    cfg = load_cfg()

    print("== 1. 没有 LowState 时 duration 必须生效 ==")
    tr = FakeTransport(n_state=0)
    node = ControllerNode(tr, ZeroController(), sim_dt=0.002, control_dt=0.02, health=None)
    out = node.run(0.2, realtime=False)
    check("run(0.2) 正常返回", True)
    check("n_state=0", out["n_state"] == 0, str(out["n_state"]))
    check("n_cmd=0（未收到状态前不发指令）", out["n_cmd"] == 0, str(out["n_cmd"]))
    check("pacer.steps≈100（duration 生效）", abs(out["pacer"]["steps"] - 100) <= 2,
          str(out["pacer"]["steps"]))

    print("== 2. LowState 断流后 health 按墙钟升级 ==")
    tr = FakeTransport(n_state=10)
    health = HealthMonitor(cfg)
    node = ControllerNode(tr, ZeroController(), sim_dt=0.002, control_dt=0.02, health=health)
    out = node.run(1.0, realtime=False)
    check("收到 10 帧状态", out["n_state"] == 10, str(out["n_state"]))
    lv = out["level_hist"]
    check("断流后出现 L1", lv.get("L1", 0) > 0, str(lv))
    check("断流后出现 L2", lv.get("L2", 0) > 0, str(lv))
    check("断流后出现 L3", lv.get("L3", 0) > 0, str(lv))
    check("最终等级 L3", out["health"]["level_name"] == "L3",
          str(out["health"]["level_name"]))
    check("reason 含 state_timeout", any("state_timeout" in r for r in out["health"]["reasons"]),
          str(out["health"]["reasons"][:2]))

    print("== 3. 退出阻尼收尾 ==")
    tr = FakeTransport(n_state=5)
    node = ControllerNode(tr, ZeroController(), sim_dt=0.002, control_dt=0.02, health=None)
    node.run(0.05, realtime=False)
    n_before = len(tr.cmds)
    ok = node.emergency_stop(ticks=10)
    check("emergency_stop 返回 True", ok)
    check("多发 10 条阻尼指令", len(tr.cmds) - n_before == 10,
          str(len(tr.cmds) - n_before))
    damp = tr.cmds[-1]
    check("阻尼: kp=0", all(abs(m.kp) < 1e-12 for m in damp.motor_cmd[:12] for m in [m]))
    check("阻尼: kd=8", all(abs(m.kd - 8.0) < 1e-12 for m in damp.motor_cmd[:12] for m in [m]))
    check("阻尼: tau=0", all(abs(m.tau) < 1e-12 for m in damp.motor_cmd[:12] for m in [m]))
    check("阻尼: mode=0x01", all(m.mode == 0x01 for m in damp.motor_cmd[:12] for m in [m]))
    # 没有任何状态时不应发（避免对未初始化的机器人乱发）
    tr2 = FakeTransport(n_state=0)
    node2 = ControllerNode(tr2, ZeroController(), sim_dt=0.002, control_dt=0.02, health=None)
    check("无状态时 emergency_stop 返回 False", node2.emergency_stop() is False)

    print("== 4. 真机 LowState 健康信号（lost / 过温 / bit_flag）==")
    hm = HealthMonitor(cfg)
    st = LowState()
    h = hm.check(st, 0.0)
    check("干净状态 L0", h.level == Level.L0_NORMAL, h.level_name)

    st_lost = LowState()
    st_lost.motor_state[3] = MotorState(mode=1, lost=1)
    h = hm.check(st_lost, 0.1)
    check("motor lost -> L2", h.level == Level.L2_PROTECT, h.level_name)
    check("reason 含 motor_lost", any("motor_lost" in r for r in h.reasons), str(h.reasons[:2]))

    hm2 = HealthMonitor(cfg)
    st_hot = LowState()
    for i in range(12):
        st_hot.motor_state[i] = MotorState(mode=1, temperature=90)
    h = hm2.check(st_hot, 0.0)
    check("电机 90C -> L2", h.level == Level.L2_PROTECT, h.level_name)
    check("reason 含 motor_temp", any("motor_temp" in r for r in h.reasons), str(h.reasons[:2]))

    hm3 = HealthMonitor(cfg)
    st_warm = LowState()
    for i in range(12):
        st_warm.motor_state[i] = MotorState(mode=1, temperature=72)
    h = hm3.check(st_warm, 0.0)
    check("电机 72C -> L1", h.level == Level.L1_DEGRADED, h.level_name)

    hm4 = HealthMonitor(cfg)
    st_bf = LowState()
    st_bf.bit_flag = 0x01
    h = hm4.check(st_bf, 0.0)
    check("bit_flag bit0（指令超时）-> L2", h.level == Level.L2_PROTECT, h.level_name)
    check("reason 含 robot_cmd_timeout",
          any("robot_cmd_timeout" in r for r in h.reasons), str(h.reasons[:2]))

    print()
    if FAILS:
        print("FAILED: %d 项 -> %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("OK: ControllerNode 循环/时基/收尾/电机健康 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
