#!/usr/bin/env python3
"""tests/test_failsafe.py —— T2 新增的三块"不需要 ROS 2 就能验"的安全能力。

为什么放在同一个文件：
  1) P0 指令掉线兜底内核（signal_io/failsafe.py）—— 纯状态机，离线可测；
  2) P1 安全层 q_des 限速（ros2_controller/safety.py 的 q_slew_rate）——
     真机上唯一能压住"首拍力矩"的东西；
  3) P2 真机危险开关闸门（ros2_controller/run_controller.py）——
     real_* 上拒绝 --no-health / --no-supervisor / --no-damping；
  4) P3 守护进程与入口的 CLI 行为（无 ROS 2 时必须优雅退出，不抛栈）。
  它们都是"安全审计与修复"这一项交付物，测试文件按任务编号命名，
  tests/run_tests.py failsafe 一次跑齐。

直接跑:
    .venv/bin/python tests/test_failsafe.py
不用 pytest。全部通过打印 "OK ..." 并以 0 退出，否则 SystemExit(1)。
"""

from __future__ import annotations

import os
import subprocess
import sys
from argparse import Namespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from ros2_controller.health import HealthState, Level  # noqa: E402
from ros2_controller.safety import SafetySupervisor  # noqa: E402
from signal_io.failsafe import (STATE_ARMED, STATE_EXHAUSTED, STATE_FIRING,  # noqa: E402
                                STATE_WAITING, FailsafeKernel, make_damping_cmd)
from signal_io.msg import NUM_JOINT, LowCmd  # noqa: E402

DT = 0.002
STAND = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)
GUARD = os.path.join(ROOT, "scripts", "failsafe_guard.py")
RUN_CTRL = os.path.join(ROOT, "ros2_controller", "run_controller.py")
RESULT = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULT.append((name, bool(ok)))
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    return bool(ok)


def make_cmd(q=None, kp: float = 20.0, kd: float = 1.0, tau: float = 0.0) -> LowCmd:
    cmd = LowCmd()
    qq = STAND if q is None else np.asarray(q, dtype=np.float64)
    for i in range(NUM_JOINT):
        m = cmd.motor_cmd[i]
        m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, float(qq[i]), 0.0, float(tau), float(kp), float(kd)
    return cmd


def qdes(cmd: LowCmd) -> np.ndarray:
    return np.array([cmd.motor_cmd[i].q for i in range(NUM_JOINT)], dtype=np.float64)


def make_health(level, q=None) -> HealthState:
    channels = {}
    if q is not None:
        from ros2_controller.health import Channel
        channels["state"] = Channel(name="state", value=np.asarray(q, dtype=np.float64),
                                    stamp=1.0, valid=True, sigma=0.0)
    return HealthState(level=Level(level), reasons=(), channels=channels,
                       entered_at=0.0, n_transitions=0)


# ================================================================ P0 兜底内核
def test_kernel_waiting_never_fires():
    print("\n== P0-A：没见过 /lowcmd 时永不发布（不能单独启动就把机器人按住）==")
    k = FailsafeKernel()
    pubs = 0
    states = set()
    for i in range(2000):                       # 4 s
        d = k.update(i * DT)
        pubs += int(d.publish)
        states.add(d.state)
    check("4 s 内 0 次发布", pubs == 0, "pubs=%d" % pubs)
    check("状态恒为 waiting", states == {STATE_WAITING}, str(states))
    st = k.stats()
    check("stats 记录 ever_seen_cmd=False", st["ever_seen_cmd"] is False, str(st["ever_seen_cmd"]))

    k2 = FailsafeKernel(require_ever_seen=False)   # --assume-always-armed 语义
    pubs2 = sum(int(k2.update(i * DT).publish) for i in range(300))
    check("require_ever_seen=False 时启动 250ms 后照常补发", pubs2 > 0, "pubs=%d" % pubs2)


def test_kernel_normal_stream():
    print("\n== P0-B：500 Hz 指令流正常时一次都不触发 ==")
    k = FailsafeKernel()
    pubs = 0
    for i in range(1000):                       # 2 s
        pubs += int(k.update(i * DT, saw_cmd=True).publish)
    st = k.stats()
    check("2 s 内 0 次发布", pubs == 0, "pubs=%d" % pubs)
    check("n_trips == 0 / state armed", st["n_trips"] == 0 and st["state"] == STATE_ARMED,
          "%s trips=%d" % (st["state"], st["n_trips"]))
    check("max_gap 记录为 1 拍 ≈ 2ms", abs(st["max_gap_ms"] - 2.0) < 1e-6, "%.3f ms" % st["max_gap_ms"])


def test_kernel_timeout_trigger():
    print("\n== P0-C：超时判定与补发节流（250 ms / 500 Hz）==")
    k = FailsafeKernel(timeout_ms=250.0, publish_hz=500.0)
    k.update(0.0, saw_cmd=True)
    # 249 ms 还没到 -> 不触发
    d = k.update(0.249)
    check("gap=249ms 不触发", (not d.publish) and d.state == STATE_ARMED, "%s %.1fms" % (d.state, d.gap_ms))
    # 252 ms -> 触发，且第一拍就发（不等一个周期）
    d = k.update(0.252)
    check("gap=252ms 立刻触发并发第一条", d.publish and d.state == STATE_FIRING,
          "%s %.1fms" % (d.state, d.gap_ms))
    n = 1
    for i in range(253, 1000):                  # 跑到 t=1.998 s（1.492 s * 500 ≈ 747 条）
        n += int(k.update(i * DT).publish)
    check("此后按 500 Hz 补发（≈747 条）", 700 <= n <= 790, "n=%d" % n)
    st = k.stats()
    check("n_trips == 1", st["n_trips"] == 1, str(st["n_trips"]))
    check("n_fire_ticks 与实发一致", st["n_fire_ticks"] == n, "%d vs %d" % (st["n_fire_ticks"], n))


def test_kernel_recover_stops():
    print("\n== P0-D：一看到 /lowcmd 立刻停发（不留尾巴、不反复触发）==")
    k = FailsafeKernel()
    k.update(0.0, saw_cmd=True)
    for i in range(1, 200):                     # 断流 0.4 s，进入 firing
        k.update(i * DT)
    check("已进入 firing", k.stats()["state"] == STATE_FIRING, k.stats()["state"])
    d = k.update(0.4, saw_cmd=True)
    check("恢复那一拍就不发", (not d.publish) and d.state == STATE_ARMED, "%s" % d.state)
    pubs = sum(int(k.update(0.4 + (i + 1) * DT, saw_cmd=True).publish) for i in range(200))
    check("恢复后 0.4 s 内 0 次发布", pubs == 0, "pubs=%d" % pubs)
    st = k.stats()
    check("n_recoveries == 1 / n_trips == 1", st["n_recoveries"] == 1 and st["n_trips"] == 1,
          "rec=%d trips=%d" % (st["n_recoveries"], st["n_trips"]))


def test_kernel_max_duration():
    print("\n== P0-E：单次补发有上限，耗尽后不重复补发 ==")
    k = FailsafeKernel(max_duration_s=0.5)
    k.update(0.0, saw_cmd=True)
    n = 0
    for i in range(1, 1000):                    # 断流 4 s
        d = k.update(i * DT)
        n += int(d.publish)
    st = k.stats()
    check("state == exhausted", st["state"] == STATE_EXHAUSTED, st["state"])
    check("补发条数 ≈ 0.5 s * 500 = 250", 240 <= n <= 260, "n=%d" % n)
    check("n_exhausted == 1", st["n_exhausted"] == 1, str(st["n_exhausted"]))
    check("total_fire_s ≈ 0.5 s", abs(st["total_fire_s"] - 0.5) < 0.02, "%.3f s" % st["total_fire_s"])
    d = k.update(5.0)
    check("耗尽后（仍没指令）不再发布", (not d.publish) and d.state == STATE_EXHAUSTED, d.state)

    # 控制器回来 -> 重新 arm -> 再断 -> 第二次触发
    d = k.update(5.002, saw_cmd=True)
    check("/lowcmd 回来后重新 armed", d.state == STATE_ARMED, "%s %s" % (d.state, d.reason))
    for i in range(1, 400):
        k.update(5.002 + i * DT)
    st = k.stats()
    check("第二次断流再次触发（n_trips == 2）", st["n_trips"] == 2, str(st["n_trips"]))


def test_kernel_jitter_no_flap():
    print("\n== P0-F：偶发丢帧（不超过 250 ms）不误触发 ==")
    k = FailsafeKernel()
    pubs = 0
    t = 0.0
    # 每 2 ms 一条，但每 100 拍故意丢 50 拍（100 ms 空洞）——远小于 250 ms
    for i in range(1000):
        t = i * DT
        missed = (i % 100) < 50
        pubs += int(k.update(t, saw_cmd=not missed).publish)
    st = k.stats()
    check("100 ms 空洞不触发", pubs == 0 and st["n_trips"] == 0,
          "pubs=%d trips=%d max_gap=%.1fms" % (pubs, st["n_trips"], st["max_gap_ms"]))
    check("max_gap 记录到 100 ms 级", st["max_gap_ms"] >= 95.0, "%.1f ms" % st["max_gap_ms"])


def test_kernel_api_and_cmd():
    print("\n== P0-G：参数校验 / 阻尼指令字段 ==")
    try:
        FailsafeKernel(publish_hz=0.0)
        ok = False
    except ValueError:
        ok = True
    check("publish_hz<=0 抛 ValueError（fail-fast）", ok, "ok")

    cmd = make_damping_cmd(kd=8.0)
    fields = [(cmd.motor_cmd[i].mode, cmd.motor_cmd[i].kp, cmd.motor_cmd[i].kd,
               cmd.motor_cmd[i].q, cmd.motor_cmd[i].dq, cmd.motor_cmd[i].tau)
              for i in range(NUM_JOINT)]
    want = (0x01, 0.0, 8.0, 0.0, 0.0, 0.0)
    check("12 个关节 = mode1/kp0/kd8/q0/dq0/tau0", all(f == want for f in fields), str(fields[:2]))
    check("后 8 个保留槽位不动（Go2 不用）",
          cmd.motor_cmd[12].kd == 0.0 and cmd.motor_cmd[19].kd == 0.0, "ok")
    check("阻尼指令默认 kd == 8", make_damping_cmd().motor_cmd[0].kd == 8.0, "ok")


# ================================================================ P1 q_des 限速
def test_q_slew_off_is_passthrough():
    print("\n== P1-A：q_slew_rate 默认关闭 = 逐位直通（仿真/旧行为）==")
    sup = SafetySupervisor(STAND)
    s = sup.process(make_cmd(STAND + 1.0), make_health(Level.L0_NORMAL), 0.0, DT)
    check("默认 q_slew_rate is None", sup.q_slew_rate is None, str(sup.q_slew_rate))
    check("q_des 直通（不限制 1 rad 跳变）", np.allclose(qdes(s), STAND + 1.0), str(qdes(s)[:3]))
    check("stats 有 q_slew_clips=0", sup.stats()["q_slew_clips"] == 0, str(sup.stats()["q_slew_clips"]))


def test_q_slew_limits_jump():
    print("\n== P1-B：real_* 默认 20 rad/s -> 单拍最多 0.04 rad ==")
    sup = SafetySupervisor(STAND, q_slew_rate=20.0)
    h0 = make_health(Level.L0_NORMAL)
    target = STAND + 1.0
    qs = []
    for k in range(30):
        s = sup.process(make_cmd(target), h0, k * DT, DT)
        qs.append(float(s.motor_cmd[0].q))
    check("第 1 拍只走到 +0.04 rad", abs(qs[0] - (STAND[0] + 0.04)) < 1e-9, "%.4f" % qs[0])
    check("第 2 拍 +0.08 rad", abs(qs[1] - (STAND[0] + 0.08)) < 1e-9, "%.4f" % qs[1])
    steps = [abs(qs[i] - qs[i - 1]) for i in range(1, len(qs))]
    check("相邻拍变化恒 <= 0.04 rad", max(steps) <= 0.04 + 1e-9, "max=%.6f" % max(steps))
    check("25 拍（50 ms）后到位 1.0 rad", abs(qs[24] - (STAND[0] + 1.0)) < 1e-9, "%.4f" % qs[24])
    st = sup.stats()
    check("q_slew_clips > 0 且 q_slew_max_step ≈ 0.96（首拍被压掉）",
          st["q_slew_clips"] > 0 and abs(st["q_slew_max_step"] - 0.96) < 1e-6, str(st["q_slew_max_step"]))

    # 反向（往下跳）同样限速
    sup2 = SafetySupervisor(STAND, q_slew_rate=20.0)
    sup2.process(make_cmd(STAND), h0, 0.0, DT)
    s = sup2.process(make_cmd(STAND - 1.0), h0, DT, DT)
    check("向下跳变同样限速到 -0.04", abs(s.motor_cmd[0].q - (STAND[0] - 0.04)) < 1e-9,
          "%.4f" % s.motor_cmd[0].q)


def test_q_slew_seeds_from_measured_q():
    print("\n== P1-B2：进入 track 时把限速历史对齐实测 q（首拍不跳台阶）==")
    MEAS = STAND + 0.2                       # 机器人真实姿态（与标称站姿不同）
    h0 = make_health(Level.L0_NORMAL, q=MEAS)
    sup = SafetySupervisor(STAND, q_slew_rate=20.0)
    s = sup.process(make_cmd(STAND + 1.0), h0, 0.0, DT)
    check("首拍 q_des = 实测 q + 0.04（不是 STAND+0.04）",
          abs(s.motor_cmd[0].q - (MEAS[0] + 0.04)) < 1e-9, "%.4f" % s.motor_cmd[0].q)
    check("只 seed 一次", sup.stats()["q_slew_seeds"] == 1, str(sup.stats()["q_slew_seeds"]))
    s = sup.process(make_cmd(STAND + 1.0), h0, DT, DT)
    check("第二拍继续按限速走（不重新 seed）",
          abs(s.motor_cmd[0].q - (MEAS[0] + 0.08)) < 1e-9, "%.4f" % s.motor_cmd[0].q)

    # 没有实测 q（health 没给 state）时退回标称站姿，且保持 pending
    sup2 = SafetySupervisor(STAND, q_slew_rate=20.0)
    s = sup2.process(make_cmd(STAND + 1.0), make_health(Level.L0_NORMAL), 0.0, DT)
    check("无 state 时退回上一拍 q_des（STAND）+0.04",
          abs(s.motor_cmd[0].q - (STAND[0] + 0.04)) < 1e-9, "%.4f" % s.motor_cmd[0].q)
    check("无 state 时不记 seed", sup2.stats()["q_slew_seeds"] == 0, "ok")

    # 从 crouch 回到 track：重新对齐实测 q（否则会从蹲姿续着走）
    sup3 = SafetySupervisor(STAND, q_slew_rate=20.0, crouch_s=0.1)
    h1 = make_health(Level.L1_DEGRADED)
    q_last = None
    for k in range(200):
        q_last = qdes(sup3.process(make_cmd(STAND), h1, k * DT, DT))
    check("crouch 已到位（q_des = crouch_pose）",
          np.allclose(q_last, sup3.crouch_pose, atol=1e-6), "ok")
    check("crouch 期间没有 seed 计数", sup3.stats()["q_slew_seeds"] == 0,
          str(sup3.stats()["q_slew_seeds"]))
    h0b = make_health(Level.L0_NORMAL, q=STAND)
    s = sup3.process(make_cmd(STAND + 1.0), h0b, 200 * DT, DT)
    check("回到 track 从实测 q 重新起算", abs(s.motor_cmd[0].q - (STAND[0] + 0.04)) < 1e-9,
          "%.4f" % s.motor_cmd[0].q)
    check("回到 track 时 reseed 一次", sup3.stats()["q_slew_seeds"] == 1,
          str(sup3.stats()["q_slew_seeds"]))


def test_q_slew_small_change_and_fields():
    print("\n== P1-C：小幅变化不受影响；只改 q，不动 kp/kd/tau ==")
    sup = SafetySupervisor(STAND, q_slew_rate=20.0)
    h0 = make_health(Level.L0_NORMAL)
    s = sup.process(make_cmd(STAND + 0.01, kp=17.0, kd=2.5, tau=1.0), h0, 0.0, DT)
    check("0.01 rad 变化 < 0.04 步长 -> 直通", np.allclose(qdes(s), STAND + 0.01), str(qdes(s)[:3]))
    check("没发生限幅（clips == 0）", sup.stats()["q_slew_clips"] == 0, str(sup.stats()["q_slew_clips"]))
    check("kp/kd 直通", s.motor_cmd[0].kp == 17.0 and s.motor_cmd[0].kd == 2.5, "ok")
    check("tau 仍按 200 N*m/s 斜坡到 0.4（限速不影响 tau 通路）",
          np.allclose([s.motor_cmd[i].tau for i in range(NUM_JOINT)], 0.4, atol=1e-12),
          str(s.motor_cmd[0].tau))


def test_q_slew_other_modes_untouched():
    print("\n== P1-D：crouch / damping / estop 不受 q_slew 影响 ==")
    h1 = make_health(Level.L1_DEGRADED)
    sup = SafetySupervisor(STAND, q_slew_rate=20.0, crouch_s=2.0)
    prev = None
    max_step = 0.0
    s = None
    for k in range(1200):                       # L1 全程 crouch
        s = sup.process(make_cmd(STAND + 2.0), h1, k * DT, DT)
        q = qdes(s)
        if prev is not None:
            max_step = max(max_step, float(np.max(np.abs(q - prev))))
        prev = q
    check("crouch 相邻拍仍 <= q_ramp_rate*dt（2.0*0.35/2*0.002 = 7e-4）",
          max_step <= 2.0 * 0.35 / 2.0 * DT + 1e-9, "max_step=%.2e" % max_step)
    check("2 s 后蹲到位（crouch_pose）", np.allclose(qdes(s), sup.crouch_pose, atol=1e-6), "ok")

    h2 = make_health(Level.L2_PROTECT, q=STAND + 0.5)
    sup2 = SafetySupervisor(STAND, q_slew_rate=20.0)
    sup2.l2_policy = "damp"
    s2 = sup2.process(make_cmd(STAND + 2.0), h2, 0.0, DT)
    check("damping: q_des = 当前 q（不被 q_slew 拖住）",
          np.allclose(qdes(s2), STAND + 0.5), str(qdes(s2)[:3]))

    h3 = make_health(Level.L3_ESTOP)
    s3 = sup2.process(make_cmd(STAND + 2.0), h3, DT, DT)
    check("estop: 20 槽位全 0", all((s3.motor_cmd[i].mode, s3.motor_cmd[i].q,
                                    s3.motor_cmd[i].kp, s3.motor_cmd[i].kd,
                                    s3.motor_cmd[i].tau) == (0, 0.0, 0.0, 0.0, 0.0)
                                   for i in range(20)), "ok")


def test_estop_policy():
    print("\n== P1-F：L3 急停两档（默认 zero 逐位不变，damp = 纯关节阻尼）==")
    h3 = make_health(Level.L3_ESTOP)
    cmd = make_cmd(STAND + 1.0, kp=60.0, kd=5.0, tau=10.0)

    # 1) 默认 "zero"：20 槽全 0（回归）
    sup0 = SafetySupervisor(STAND)
    s0 = sup0.process(cmd, h3, 0.0, DT)
    check("默认 estop_policy == zero", sup0.estop_policy == "zero", sup0.estop_policy)
    check("zero: 20 槽位全 0（含 mode）",
          all((s0.motor_cmd[i].mode, s0.motor_cmd[i].q, s0.motor_cmd[i].dq,
               s0.motor_cmd[i].tau, s0.motor_cmd[i].kp, s0.motor_cmd[i].kd)
              == (0, 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(20)), "ok")

    # 2) "damp"：12 个关节 mode=1 / kp=0 / kd=damp_kd / q=0
    sup1 = SafetySupervisor(STAND, estop_policy="damp")
    s1 = sup1.process(cmd, h3, 0.0, DT)
    joints = [(s1.motor_cmd[i].mode, s1.motor_cmd[i].kp, s1.motor_cmd[i].kd,
               s1.motor_cmd[i].q, s1.motor_cmd[i].dq, s1.motor_cmd[i].tau)
              for i in range(NUM_JOINT)]
    check("damp: 12 关节 mode=0x01/kp=0/kd=8/q=dq=tau=0",
          all(j == (0x01, 0.0, 8.0, 0.0, 0.0, 0.0) for j in joints), str(joints[0]))
    check("damp: 后 8 个保留槽位也置为 mode=1（不留半接管状态）",
          all(s1.motor_cmd[i].mode == 0x01 and s1.motor_cmd[i].kd == 8.0 for i in range(12, 20)),
          "ok")
    check("damp: stats 记录 estop_policy/estop_kd",
          sup1.stats()["estop_policy"] == "damp" and sup1.stats()["estop_kd"] == 8.0,
          str(sup1.stats()["estop_kd"]))

    # 3) damp_kd 显式给出时优先（与 L2 damping 共用一个旋钮）
    sup2 = SafetySupervisor(STAND, estop_policy="damp", damp_kd=5.0)
    s2 = sup2.process(cmd, h3, 0.0, DT)
    check("damp: 显式 damp_kd=5 -> estop_kd=5", s2.motor_cmd[0].kd == 5.0,
          "kd=%.1f" % s2.motor_cmd[0].kd)

    # 4) 非法值 fail-fast
    try:
        SafetySupervisor(STAND, estop_policy="soft")
        ok = False
    except ValueError:
        ok = True
    check("非法 estop_policy 抛 ValueError", ok, "ok")

    # 5) damp 档下 kd 只是"阻尼"，不会产生位置保持力（kp=0）
    check("damp: kp=0（纯阻尼，不会把机器人顶成站姿）",
          all(s1.motor_cmd[i].kp == 0.0 for i in range(NUM_JOINT)), "ok")


def test_resolve_estop_policy():
    print("\n== P2-E：estop_policy 取值优先级（config > real_* damp > 仿真 zero）==")
    from ros2_controller.run_controller import resolve_estop_policy
    v, src = resolve_estop_policy({}, "real_ros")
    check("real_ros 未配置 -> damp", v == "damp", "%s (%s)" % (v, src))
    v, _ = resolve_estop_policy({}, "real_sdk")
    check("real_sdk 同样 damp", v == "damp", v)
    v, _ = resolve_estop_policy({}, "sim_udp")
    check("sim_udp 未配置 -> zero（仿真逐位不变）", v == "zero", v)
    v, _ = resolve_estop_policy({}, "sim_ros")
    check("sim_ros 未配置 -> zero", v == "zero", v)
    v, _ = resolve_estop_policy({"estop_policy": "zero"}, "real_ros")
    check("config 显式 zero -> 真机也卸力", v == "zero", v)
    v, _ = resolve_estop_policy({"estop_policy": "DAMP"}, "sim_ros")
    check("config 显式 DAMP（大小写不敏感）-> damp", v == "damp", v)
    try:
        resolve_estop_policy({"estop_policy": "soft"}, "real_ros")
        ok = False
    except SystemExit:
        ok = True
    check("config 写了非法值 -> SystemExit（fail-fast）", ok, "ok")


def test_damp_kd_optional():
    print("\n== P1-E：L2 damping 的 kd 可单独配（默认不变）==")
    h2 = make_health(Level.L2_PROTECT, q=STAND + 0.1)
    sup = SafetySupervisor(STAND)
    sup.l2_policy = "damp"
    s = sup.process(make_cmd(STAND + 1.0), h2, 0.0, DT)
    check("默认 damp_kd == mode_kd（1.0，行为不变）",
          sup.damp_kd == 1.0 and s.motor_cmd[0].kd == 1.0, "kd=%.1f" % s.motor_cmd[0].kd)
    sup2 = SafetySupervisor(STAND, damp_kd=8.0)
    sup2.l2_policy = "damp"
    s2 = sup2.process(make_cmd(STAND + 1.0), h2, 0.0, DT)
    check("显式 damp_kd=8 -> damping 用 kd=8（与退出阻尼同强度）",
          s2.motor_cmd[0].kd == 8.0 and s2.motor_cmd[0].kp == 0.0, "kd=%.1f" % s2.motor_cmd[0].kd)
    check("crouch 仍用 mode_kd 不受影响",
          SafetySupervisor(STAND, damp_kd=8.0, mode_kd=1.0).mode_kd == 1.0, "ok")


# ================================================================ P2 危险开关闸门
def test_unsafe_flag_gate():
    print("\n== P2-A：real_* 上拒绝 --no-health/--no-supervisor/--no-damping ==")
    from ros2_controller.run_controller import unsafe_flag_refusals

    def args(**kw):
        base = dict(no_health=False, no_supervisor=False, no_damping=False, force_unsafe=False)
        base.update(kw)
        return Namespace(**base)

    r = unsafe_flag_refusals(args(no_health=True), "real_ros")
    check("real_ros + --no-health -> 拒绝", len(r) == 1 and r[0][1] == "--no-health", str(r))
    r = unsafe_flag_refusals(args(no_health=True, no_supervisor=True, no_damping=True), "real_ros")
    check("real_ros + 三个危险开关 -> 全部拒绝", len(r) == 3, str([x[1] for x in r]))
    check("real_sdk 同样拒绝", len(unsafe_flag_refusals(args(no_health=True), "real_sdk")) == 1, "ok")
    r = unsafe_flag_refusals(args(no_health=True, no_supervisor=True, no_damping=True,
                                 force_unsafe=True), "real_ros")
    check("--force-unsafe 显式放开", r == [], str(r))
    r = unsafe_flag_refusals(args(no_health=True, no_supervisor=True, no_damping=True), "sim_udp")
    check("仿真 target 不拦（排障刚需）", r == [], str(r))
    r = unsafe_flag_refusals(args(), "real_ros")
    check("真机不带危险开关 -> 放行", r == [], str(r))


def test_q_slew_resolution():
    print("\n== P2-B：q_slew_rate 取值优先级（config > real_* 默认 > 仿真关闭）==")
    from ros2_controller.run_controller import resolve_q_slew_rate
    v, src = resolve_q_slew_rate({}, "real_ros")
    check("real_ros 未配置 -> 20 rad/s", v == 20.0, "%r (%s)" % (v, src))
    v, _ = resolve_q_slew_rate({}, "real_sdk")
    check("real_sdk 同样 20 rad/s", v == 20.0, repr(v))
    v, _ = resolve_q_slew_rate({}, "sim_ros")
    check("sim_ros 未配置 -> 关闭", v is None, repr(v))
    v, _ = resolve_q_slew_rate({}, "sim_udp")
    check("sim_udp 未配置 -> 关闭", v is None, repr(v))
    v, _ = resolve_q_slew_rate({"q_slew_rate": 12.5}, "real_ros")
    check("config 显式 12.5 -> 用 12.5", v == 12.5, repr(v))
    v, _ = resolve_q_slew_rate({"q_slew_rate": 0}, "real_ros")
    check("config 显式 0 -> 关闭（真机上也知道自己在干什么）", v is None, repr(v))
    v, _ = resolve_q_slew_rate({"q_slew_rate": None}, "real_ros")
    check("config 显式 null -> 关闭", v is None, repr(v))


def test_cmd_vel_clip():
    print("\n== P2-D：运行期 cmd_vel 三轴限幅（安全护栏）==")
    from ros2_controller.run_controller import CMD_VEL_LIMITS, clip_cmd_vel
    check("正常 0.2 直通", clip_cmd_vel(0.2, 0.0, 0.0) == (0.2, 0.0, 0.0), str(clip_cmd_vel(0.2, 0, 0)))
    check("vx 2.0 -> 0.8（上界）", clip_cmd_vel(2.0, 0.0, 0.0)[0] == 0.8, str(clip_cmd_vel(2.0, 0, 0)))
    check("vx -1.0 -> -0.2（下界，不倒着冲）", clip_cmd_vel(-1.0, 0.0, 0.0)[0] == -0.2, "ok")
    check("vy 1.0 -> 0.3", clip_cmd_vel(0.0, 1.0, 0.0)[1] == 0.3, "ok")
    check("wz 5.0 -> 0.8", clip_cmd_vel(0.0, 0.0, 5.0)[2] == 0.8, "ok")
    check("限幅表与实际一致", CMD_VEL_LIMITS == ((-0.2, 0.8), (-0.3, 0.3), (-0.8, 0.8)),
          str(CMD_VEL_LIMITS))
    # zero/stand 控制器没有 set_command -> 入口不会包 LiveCommandController
    from ros2_controller.controller import StandController, ZeroController
    check("stand/zero 无 set_command（只有 policy 支持运行期改速度）",
          not callable(getattr(StandController(STAND), "set_command", None))
          and not callable(getattr(ZeroController(), "set_command", None)), "ok")


def test_run_controller_cli_refuses():
    print("\n== P2-C：入口真的会退出（子进程，无 ROS 2 也能到闸门）==")
    p = subprocess.run([sys.executable, RUN_CTRL, "--target", "real_ros",
                        "--no-health", "--duration", "0.1"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    out = (p.stdout or "") + (p.stderr or "")
    check("退出码 == 2（拒绝，不是崩）", p.returncode == 2, "rc=%s" % p.returncode)
    check("打印拒绝原因 + 指明 --force-unsafe",
          "拒绝在真机" in out and "--force-unsafe" in out, out.strip().splitlines()[-1][:100] if out.strip() else "")
    check("列出了被拒绝的具体开关", "--no-health" in out, "ok")


# ================================================================ P3 守护进程 CLI
def test_guard_selftest():
    print("\n== P3-A：failsafe_guard.py --selftest（不需要 ROS 2）==")
    p = subprocess.run([sys.executable, GUARD, "--selftest"], cwd=ROOT,
                       capture_output=True, text=True, timeout=180)
    out = (p.stdout or "") + (p.stderr or "")
    check("退出码 == 0", p.returncode == 0, "rc=%s" % p.returncode)
    check("7 项全过", "7 项通过, 0 项失败" in out, out.strip().splitlines()[-1][:100])
    check("打印了关键结论（断流触发 / 恢复即停）",
          "断流 >250ms 触发补发" in out and "/lowcmd 恢复立即停发" in out, "ok")


def test_guard_no_ros_graceful():
    print("\n== P3-B：无 ROS 2 时 --target real_ros 优雅退出（不抛栈）==")
    try:
        import rclpy  # noqa: F401
        have_rclpy = True
    except Exception:
        have_rclpy = False
    if have_rclpy:
        check("本机有 rclpy：跳过（避免测试真的连 DDS）", True, "SKIP")
        return
    p = subprocess.run([sys.executable, GUARD, "--target", "real_ros", "--duration", "0.1"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    err = p.stderr or ""
    check("退出码 == 2（优雅失败，不是崩）", p.returncode == 2, "rc=%s" % p.returncode)
    check("打印了原因（缺 rclpy / unitree_go）", "本机不可用" in err, "ok")
    check("没有 Python traceback", "Traceback" not in err, "ok")
    check("给了替代做法（--selftest / 装 ROS 2）", "--selftest" in err, "ok")


def main() -> int:
    print("test_failsafe.py —— 兜底内核 + q_des 限速 + 危险开关闸门（dt=%.3f）" % DT)
    test_kernel_waiting_never_fires()
    test_kernel_normal_stream()
    test_kernel_timeout_trigger()
    test_kernel_recover_stops()
    test_kernel_max_duration()
    test_kernel_jitter_no_flap()
    test_kernel_api_and_cmd()
    test_q_slew_off_is_passthrough()
    test_q_slew_limits_jump()
    test_q_slew_seeds_from_measured_q()
    test_q_slew_small_change_and_fields()
    test_q_slew_other_modes_untouched()
    test_estop_policy()
    test_resolve_estop_policy()
    test_damp_kd_optional()
    test_unsafe_flag_gate()
    test_q_slew_resolution()
    test_cmd_vel_clip()
    test_run_controller_cli_refuses()
    test_guard_selftest()
    test_guard_no_ros_graceful()
    n_fail = sum(1 for _n, ok in RESULT if not ok)
    print("\n===== test_failsafe 汇总 =====")
    print("  %d/%d 通过" % (len(RESULT) - n_fail, len(RESULT)))
    if n_fail:
        for n, ok in RESULT:
            if not ok:
                print("  FAIL: %s" % n)
        raise SystemExit(1)
    print("OK test_failsafe.py 全部 %d 项通过" % len(RESULT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
