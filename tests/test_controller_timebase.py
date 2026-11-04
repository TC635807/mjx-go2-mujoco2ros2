#!/usr/bin/env python3
"""tests/test_controller_timebase.py —— 控制器时基回归（真机 prepose / kp ramp）。

背景（2026-11-04 真机首测前修掉的致命问题）：
    LowState.tick 在真机上是"上电以来的毫秒计数"，`sim_time = tick * tick_dt`
    可能是 1e5 秒量级。而 StandController 的 kp ramp（t < start_delay → kp=0，
    再 ramp_s 内升到满）和 PolicyController 的 prepose（t < prepose_s 才摆位）都
    按传进来的 t 判断 —— 用绝对时间会**直接跳过**：
      * stand：第一拍就是 kp=60/kd=5，机器人可能被"啪"一声甩到站姿；
      * policy：跳过 prepose，策略第一帧看见训练里没见过的关节偏差。
    修复方式：ControllerNode 传给控制器的 t 是"接管以来"的相对时间（clock0 归零），
    健康层/安全层仍用绝对 clock（语义是"多久没收到状态"）。

本测试用 tick 从 3600000 起的假状态（≈ 上电 1 小时的真机）复现该场景：
  1) 控制器收到的 t 必须从 ~0 开始，且随控制拍推进；
  2) StandController 的 kp 必须从 0 斜坡升到配置值（不是第一拍就满）。

直接运行：python tests/test_controller_timebase.py
"""
from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ros2_controller.controller import StandController  # noqa: E402
from signal_io.msg import LowCmd, LowState  # noqa: E402
from signal_io.node import ControllerNode  # noqa: E402
from signal_io.transport.base import Transport  # noqa: E402

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


class RealTickTransport(Transport):
    """模拟真机固件：tick 是上电毫秒计数（从 tick0 起），永远有新状态。"""

    name = "fake-real"

    # dt 取 0.002（= 控制循环周期），这样"每拍一个新状态"时逻辑时钟与循环同步，
    # 测试里的 t 才等于墙钟时长；真机实际 tick_dt 是 0.001（不影响本回归的语义）。
    def __init__(self, tick0: int = 3_600_000, dt: float = 0.002, n_max: int = 10 ** 9):
        self.tick0 = int(tick0)
        self.dt = float(dt)
        self.n_max = int(n_max)
        self.i = 0
        self.cmds = []

    def publish_state(self, state) -> None:
        pass

    def recv_cmd(self):
        return None

    def recv_state(self):
        if self.i >= self.n_max:
            return None
        st = LowState()
        st.tick = self.tick0 + self.i
        # ← 真机语义：sim_time = 上电以来的秒数（tick_dt × tick），量级极大
        st.sim_time = float(st.tick) * self.dt
        self.i += 1
        return st

    def publish_cmd(self, cmd) -> None:
        self.cmds.append(cmd)

    def stats(self):
        return {"transport": "fake-real"}


class RecordingController:
    """记录每次 update 收到的 t，并回一条 kp=1 的指令。"""

    name = "recording"

    def __init__(self):
        self.ts = []

    def update(self, state, t):        # noqa: ANN001
        self.ts.append(float(t))
        cmd = LowCmd()
        for i in range(12):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.kp, m.kd, m.tau = 0x01, 0.0, 0.0, 1.0, 0.0, 0.0
        return cmd

    def stats(self):
        return {"n_calls": len(self.ts)}


def main() -> int:
    cfg = json.load(open(os.path.join(ROOT, "config", "go2_sim.json"), encoding="utf-8"))

    print("== 1. 控制器收到的 t 必须是「接管以来」的相对时间（真机 tick 很大时）==")
    tr = RealTickTransport(tick0=3_600_000)
    rec = RecordingController()
    node = ControllerNode(tr, rec, sim_dt=0.002, control_dt=0.02, health=None)
    out = node.run(2.0, realtime=False)
    check("收到状态", out["n_state"] > 0, str(out["n_state"]))
    check("控制器被调用", len(rec.ts) > 10, str(len(rec.ts)))
    check("第一拍 t ≈ 0（绝对时间会是 3600）", abs(rec.ts[0]) < 0.05, "t0=%.4f" % rec.ts[0])
    check("t 单调递增", all(b > a for a, b in zip(rec.ts, rec.ts[1:])))
    check("末拍 t ≈ 运行时长 2.0 s", abs(rec.ts[-1] - 2.0) < 0.15, "t_end=%.3f" % rec.ts[-1])

    print("== 2. StandController 的 kp 必须斜坡上升（不能第一拍就 kp=60）==")
    tr2 = RealTickTransport(tick0=3_600_000)
    stand = StandController(cfg["robots"]["go2"]["stand_pose"],
                            kp=cfg["controller"]["kp"], kd=cfg["controller"]["kd"],
                            ramp_s=cfg["controller"]["ramp_s"], start_delay_s=0.5)
    node2 = ControllerNode(tr2, stand, sim_dt=0.002, control_dt=0.02, health=None)
    out2 = node2.run(2.5, realtime=False)
    cmds = tr2.cmds
    kp_first = cmds[0].motor_cmd[0].kp
    kp_mid = cmds[len(cmds) // 2].motor_cmd[0].kp
    kp_last = cmds[-1].motor_cmd[0].kp
    check("第一拍 kp ≈ 0（旧实现会是 %.0f）" % cfg["controller"]["kp"], kp_first < 1.0,
          "kp=%.2f" % kp_first)
    check("中途 kp 在斜坡上", 0.0 < kp_mid < cfg["controller"]["kp"], "kp=%.2f" % kp_mid)
    check("末拍 kp 到配置值", abs(kp_last - cfg["controller"]["kp"]) < 1e-6,
          "kp=%.2f" % kp_last)
    check("斜坡单调不减", kp_first <= kp_mid <= kp_last)
    check("summary 里 kp_alpha 到 1", abs(out2["controller"]["kp_alpha"] - 1.0) < 1e-6,
          str(out2["controller"]["kp_alpha"]))

    print()
    if FAILS:
        print("FAILED %d 项: %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("OK: 控制器时基/prepose/ramp 回归全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
