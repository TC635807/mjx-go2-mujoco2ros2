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

from ros2_controller.controller import BaseController, PolicyController, ZeroController  # noqa: E402
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

    def __init__(self, n_state: int = 0, t0: float = 0.0):
        self.n_state = int(n_state)
        self.t0 = float(t0)      # sim_time 起点（真机固件 tick 是开机毫秒 -> t0 可能几百秒）
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
            st.sim_time = self.t0 + self.i * 0.002
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

    # 2026-11-05 实机修正：motor_state[i].lost 是**累计**丢包计数，不能看绝对值
    # （否则历史上抖动过一次就永久 L2；实机 RL_thigh lost=5 导致一启动就 crouch）。
    # 新语义：1 秒窗口内新增 >= lost_win_max 才算"现在在丢"。
    hm_b = HealthMonitor(cfg)
    st_hist = LowState()
    for i in range(12):
        st_hist.motor_state[i] = MotorState(mode=1, lost=5)
    h = hm_b.check(st_hist, 100.0)
    check("累计 lost=5 但无新增 -> 仍 L0", h.level == Level.L0_NORMAL, h.level_name)

    st_lost = LowState()
    for i in range(12):
        st_lost.motor_state[i] = MotorState(mode=1, lost=5)
    st_lost.motor_state[3] = MotorState(mode=1, lost=25)
    h = hm_b.check(st_lost, 102.0)
    check("1 s 内新增 lost 20 -> L2", h.level == Level.L2_PROTECT, h.level_name)
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

    # 2026-11-05：bit0 会在"我们的第一帧 LowCmd 到达之前"闪一下，单帧不能判 L2
    # （实机就是这样：一启动 crouch 2 秒）。改成连续置位超过 bit_flag_grace_s 才报警。
    hm4 = HealthMonitor(cfg)
    st_bf = LowState()
    st_bf.bit_flag = 0x01
    h0 = hm4.check(st_bf, 10.0)
    check("bit_flag bit0 单帧不降级（去抖）", h0.level == Level.L0_NORMAL, h0.level_name)
    h = hm4.check(st_bf, 10.5)
    check("bit_flag bit0 持续 0.5s -> L2", h.level == Level.L2_PROTECT, h.level_name)
    check("reason 含 robot_cmd_timeout",
          any("robot_cmd_timeout" in r for r in h.reasons), str(h.reasons[:2]))

    print("== 5. 真机时基：sim_time 从几百秒开始，控制器也要从 t≈0 起算 ==")
    # 真机 LowState.tick 是固件开机毫秒 -> sim_time ≈ 开机秒数。若把绝对时钟直接喂给
    # 控制器，prepose_s/ramp_s/start_delay_s 这些"进程启动后多久"的窗口会全部失效
    # （1.5 s 预摆位被跳过、stand 的 kp 斜坡一步到位）。node.py 用本地时基 t_ctrl 修掉。

    class _Recorder(BaseController):
        name = "recorder"

        def __init__(self):
            self.ts = []

        def update(self, state, t):
            self.ts.append(float(t))
            return LowCmd().zero()

    rec = _Recorder()
    tr = FakeTransport(n_state=10_000, t0=1000.0)      # 模拟开机 1000 s 后接管
    node = ControllerNode(tr, rec, sim_dt=0.002, control_dt=0.02, health=None)
    out = node.run(0.3, realtime=False)
    check("控制器收到的第一个 t ≈ 0（不是 1000 s）", bool(rec.ts) and rec.ts[0] < 1e-6,
          "t0=%.4f" % (rec.ts[0] if rec.ts else float("nan")))
    check("控制器 t 单调递增", all(b >= a for a, b in zip(rec.ts, rec.ts[1:])),
          "n=%d last=%.3f" % (len(rec.ts), rec.ts[-1] if rec.ts else -1))
    check("0.3 s 内 t 覆盖到 ~0.3 s", bool(rec.ts) and abs(rec.ts[-1] - 0.3) < 0.05,
          "last=%.3f" % (rec.ts[-1] if rec.ts else -1))

    # 直接验 prepose：t_ctrl 从 0 起算时，前 1.5 s 必须走预摆位（q=训练 home），不是策略输出。
    class _FakeObs:
        home_q = np.full(12, 0.123)       # 与策略输出 0 区分开
        scan_n = 0

        def observe(self, state, t, base_lin_vel_world=None, height_scan=None):
            return np.zeros(4), None

        def commit(self, action):
            pass

        def reset(self):
            pass

        def scan_grid(self):
            return np.zeros((0, 2))

    pol = PolicyController(policy=lambda obs: np.zeros(12), obs_builder=_FakeObs(),
                           kp=20.0, kd=1.0, prepose_s=1.5, prepose_kp=20.0, prepose_kd=1.0)
    st = LowState()
    st.tick = 1_000_000                  # 开机 1000 s
    st.sim_time = 1000.0
    cmd = pol.update(st, 0.0)
    check("prepose: q = 训练 home", all(abs(cmd.motor_cmd[i].q - 0.123) < 1e-12 for i in range(12)))
    check("prepose: kp/kd = prepose 增益",
          all(abs(cmd.motor_cmd[i].kp - 20.0) < 1e-12 and abs(cmd.motor_cmd[i].kd - 1.0) < 1e-12
              for i in range(12)))
    cmd2 = pol.update(st, 2.0)
    check("prepose 结束（t=2 s）后交给策略", all(abs(cmd2.motor_cmd[i].q) < 1e-12 for i in range(12)))

    print()
    if FAILS:
        print("FAILED: %d 项 -> %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("OK: ControllerNode 循环/时基/收尾/电机健康 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
