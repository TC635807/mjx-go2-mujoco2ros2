#!/usr/bin/env python3
"""tests/test_safety.py —— SafetySupervisor 分级模式 + 旧 SafetyLimiter 回归（契约 §5）。

直接跑:
    .venv/bin/python tests/test_safety.py
不用 pytest。全部通过打印 "OK ..." 并以 0 退出，否则 SystemExit(1)。
"""

from __future__ import annotations

import inspect
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from ros2_controller.health import Channel, HealthState, Level  # noqa: E402
from signal_io.msg import NUM_JOINT, NUM_MOTOR, LowCmd  # noqa: E402
from ros2_controller.safety import SafetyLimiter, SafetySupervisor  # noqa: E402

DT = 0.002
STAND = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)
RESULT = []

# v1 实测签名（2026-10-17 基线，见交付报告的 sha256 证据）
V1_INIT_SIG = "(self, torque_limits=None, ramp_rate: 'float' = 200.0, watchdog_ms: 'float' = 60.0, enabled: 'bool' = True)"
V1_PROCESS_SIG = ("(self, cmd: 'Optional[LowCmd]', now: 'Optional[float]' = None, dt: 'float' = 0.002)"
                  " -> 'Tuple[LowCmd, bool]'")
# v1 文件里 SafetyLimiter 类块（class 行 -> 文件末尾）的 sha256，改前实测
LIMITER_V1_SHA256 = "52ce82b975cef3157aa46ef71ff7b785c91e8515a5050631140a2fef09de6bf4"


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


def make_health(level, q=None) -> HealthState:
    channels = {}
    if q is not None:
        channels["state"] = Channel(name="state", value=np.asarray(q, dtype=np.float64),
                                    stamp=1.0, valid=True, sigma=0.0)
    return HealthState(level=Level(level), reasons=(), channels=channels,
                       entered_at=0.0, n_transitions=0)


def qdes(cmd: LowCmd) -> np.ndarray:
    return np.array([cmd.motor_cmd[i].q for i in range(NUM_JOINT)], dtype=np.float64)


def taus(cmd: LowCmd) -> np.ndarray:
    return np.array([cmd.motor_cmd[i].tau for i in range(NUM_JOINT)], dtype=np.float64)


def all_motor_zero(cmd: LowCmd, n: int = NUM_MOTOR) -> bool:
    for i in range(n):
        m = cmd.motor_cmd[i]
        if (m.mode, m.q, m.dq, m.tau, m.kp, m.kd) != (0, 0.0, 0.0, 0.0, 0.0, 0.0):
            return False
    return cmd.mode_pr == 0 and cmd.mode_machine == 0


# ---------------------------------------------------------------- 旧 API 回归
def test_old_signatures():
    print("\n== 回归 A：SafetyLimiter 签名逐字不变 ==")
    init_sig = str(inspect.signature(SafetyLimiter.__init__))
    proc_sig = str(inspect.signature(SafetyLimiter.process))
    check("__init__ 签名 == v1", init_sig == V1_INIT_SIG, init_sig)
    check("process 签名 == v1", proc_sig == V1_PROCESS_SIG, proc_sig)
    p = inspect.signature(SafetyLimiter.process).parameters
    check("process 位置参数为 (self, cmd)", list(p)[:2] == ["self", "cmd"], str(list(p)))
    check("now 默认 None / dt 默认 0.002", p["now"].default is None and p["dt"].default == 0.002,
          "now=%r dt=%r" % (p["now"].default, p["dt"].default))


def test_limiter_source_unchanged():
    print("\n== 回归 0：SafetyLimiter 源码块逐位不变（sha256）==")
    import hashlib
    src_path = os.path.join(ROOT, "ros2_controller", "safety.py")
    with open(src_path, encoding="utf-8") as f:
        lines = f.read().splitlines(keepends=True)
    i = next(k for k, l in enumerate(lines) if l.startswith("class SafetyLimiter"))
    j = next(k for k, l in enumerate(lines) if l.startswith("# ") and "v2 分级监督器" in l)
    block = "".join(lines[i:j]).rstrip("\n") + "\n"   # 去掉两段之间新增的空行
    h = hashlib.sha256(block.encode()).hexdigest()
    check("class SafetyLimiter 源码 sha256 == v1 基线", h == LIMITER_V1_SHA256, h)


def test_old_ramp_and_limits():
    print("\n== 回归 B：斜率限幅 / 力矩硬限幅 / 字段直通 ==")
    lim = SafetyLimiter(ramp_rate=200.0, watchdog_ms=60.0, enabled=True)
    lim.reset()
    cmd = make_cmd(q=STAND + 0.3, kp=17.0, kd=2.5, tau=100.0)
    s1, to1 = lim.process(cmd, now=0.0, dt=0.002)
    check("第一拍 tau = ramp_rate*dt = 0.4", np.allclose(taus(s1), 0.4, atol=1e-12),
          str(taus(s1)[:3]))
    s2, _ = lim.process(cmd, now=0.002, dt=0.002)
    check("第二拍 tau = 0.8", np.allclose(taus(s2), 0.8, atol=1e-12), str(taus(s2)[:3]))
    check("watchdog 未触发返回 False", to1 is False, str(to1))
    check("q/dq/kp/kd/mode 直通", np.allclose(qdes(s1), STAND + 0.3)
          and s1.motor_cmd[0].kp == 17.0 and s1.motor_cmd[0].kd == 2.5
          and s1.motor_cmd[0].mode == 0x01, "q0=%.4f" % s1.motor_cmd[0].q)

    lim2 = SafetyLimiter(np.full(NUM_JOINT, 1.0), ramp_rate=1e6)
    lim2.reset()
    sp, _ = lim2.process(make_cmd(tau=100.0), now=0.0, dt=0.002)
    sn, _ = lim2.process(make_cmd(tau=-100.0), now=0.1, dt=0.002)
    check("力矩上限 +1.0", np.allclose(taus(sp), 1.0, atol=1e-12), str(taus(sp)[:3]))
    check("力矩下限 -1.0", np.allclose(taus(sn), -1.0, atol=1e-12), str(taus(sn)[:3]))

    lim3 = SafetyLimiter(ramp_rate=200.0)
    lim3.reset()
    for k in range(3):
        lim3.process(make_cmd(tau=1.0), now=k * DT, dt=DT)   # 0.4 -> 0.8 -> 1.0
    check("斜坡顶到 1.0", np.allclose(lim3.last_ok, 1.0, atol=1e-12), str(lim3.last_ok[:3]))
    s4, _ = lim3.process(make_cmd(tau=0.0), now=3 * DT, dt=DT)
    check("回撤一拍 = 1.0-0.4 = 0.6", np.allclose(taus(s4), 0.6, atol=1e-12), str(taus(s4)[:3]))
    # v1 的 n_clipped 是**逐关节**累加（12 关节 x 1 拍回撤）
    check("ramp_clips 逐关节计数 == 12", lim3.n_clipped == NUM_JOINT, str(lim3.n_clipped))
    check("stats() 字段不变", set(lim3.stats()) == {"watchdog_trips", "ramp_clips"}, str(lim3.stats()))

    lim4 = SafetyLimiter(np.full(NUM_JOINT, 0.5), ramp_rate=200.0, enabled=False)
    lim4.reset()
    s5, _ = lim4.process(make_cmd(tau=100.0), now=0.0, dt=DT)
    check("enabled=False 时 tau 直通不夹", np.allclose(taus(s5), 100.0, atol=1e-12), str(taus(s5)[:3]))


def test_old_watchdog():
    print("\n== 回归 C：看门狗（含超时返回 True）==")
    lim = SafetyLimiter(watchdog_ms=60.0)
    lim.reset()
    t0 = lim.last_cmd_time
    s, to = lim.process(None, now=t0 + 0.01, dt=DT)
    check("窗口内 -> hold, 返回 False", to is False and np.allclose(taus(s), 0.0), str(to))
    lim.process(make_cmd(tau=5.0), now=t0 + 0.02, dt=DT)      # last_ok=0.4
    s, to = lim.process(None, now=t0 + 0.03, dt=DT)
    check("hold 保持上一拍 tau=0.4", to is False and np.allclose(taus(s), 0.4, atol=1e-12),
          "to=%s tau=%.3f" % (to, s.motor_cmd[0].tau))
    s, to = lim.process(None, now=t0 + 0.2, dt=DT)
    check("超时 -> 返回 True（v1 行为）", to is True, str(to))
    check("超时输出为阻尼全 0", np.allclose(taus(s), 0.0) and s.motor_cmd[0].kp == 0.0
          and s.motor_cmd[0].kd == 0.0 and s.motor_cmd[0].q == 0.0, "ok")
    check("watchdog_trips == 1 且 last_ok 清零",
          lim.n_watchdog == 1 and np.allclose(lim.last_ok, 0.0), str(lim.n_watchdog))
    lim.process(make_cmd(tau=1.0), now=t0 + 100.0, dt=DT)
    s, to = lim.process(None, now=t0 + 100.01, dt=DT)
    check("收到新指令后计时重置", to is False, str(to))


# ---------------------------------------------------------------- 监督器
def test_supervisor_track():
    print("\n== 监督器 A：L0 track 直通 + 限幅 ==")
    sup = SafetySupervisor(STAND, ramp_rate=200.0)
    check("默认模式 track / level L0", sup.mode == "track" and sup.level == Level.L0_NORMAL,
          "%s %s" % (sup.mode, sup.level))
    h0 = make_health(Level.L0_NORMAL)
    cmd = make_cmd(q=STAND + 0.1, kp=20.0, kd=1.0, tau=100.0)
    s = sup.process(cmd, h0, 0.0, DT)
    check("mode == track", sup.mode == "track", sup.mode)
    check("q_des 直通", np.allclose(qdes(s), STAND + 0.1, atol=1e-12), "ok")
    check("kp/kd 直通", s.motor_cmd[0].kp == 20.0 and s.motor_cmd[0].kd == 1.0, "ok")
    check("tau 被斜率限幅到 0.4", np.allclose(taus(s), 0.4, atol=1e-12), str(taus(s)[:3]))

    sup2 = SafetySupervisor(STAND, limits=np.full(NUM_JOINT, 1.0), ramp_rate=1e6)
    for k in range(20):
        s = sup2.process(make_cmd(tau=100.0), h0, k * DT, DT)
    check("力矩硬限幅生效 max|tau| <= 1.0", float(np.max(np.abs(taus(s)))) <= 1.0,
          "max=%.4f" % float(np.max(np.abs(taus(s)))))

    sup3 = SafetySupervisor(STAND)
    sup3.limiter.last_cmd_time = time.monotonic() - 1.0
    s = sup3.process(None, h0, 0.0, DT)
    check("track 下看门狗仍然生效（无指令 1s 后卸力）",
          sup3.stats()["watchdog_trips"] == 1 and np.allclose(taus(s), 0.0),
          str(sup3.stats()["watchdog_trips"]))


def test_supervisor_crouch():
    print("\n== 监督器 B：L1 -> crouch 平滑过渡 ==")
    sup = SafetySupervisor(STAND, mode_kp=20.0, mode_kd=1.0, crouch_depth=0.35, crouch_s=2.0)
    h0 = make_health(Level.L0_NORMAL)
    h1 = make_health(Level.L1_DEGRADED)
    track_cmd = make_cmd(q=STAND + 0.2, kp=60.0, kd=5.0)
    s = None
    for k in range(10):                                  # 先在 track 把 q_des 顶到 STAND+0.2
        s = sup.process(track_cmd, h0, k * DT, DT)
    q_before = qdes(s)
    check("track 下 q_des == 指令", np.allclose(q_before, STAND + 0.2, atol=1e-12), "ok")

    prev = None
    max_step = 0.0
    q_last = None
    first_crouch = None
    for k in range(10, 10 + 1100):
        s = sup.process(track_cmd, h1, k * DT, DT)
        q = qdes(s)
        if k == 10:
            first_crouch = q
        if prev is not None:
            max_step = max(max_step, float(np.max(np.abs(q - prev))))
        prev = q
        q_last = q

    check("crouch_from = 上一拍 q_des（切换无跳变）",
          first_crouch is not None and np.allclose(first_crouch, q_before, atol=1e-12),
          "max|diff|=%.2e" % (float(np.max(np.abs(first_crouch - q_before))) if first_crouch is not None else -1))
    check("相邻两拍 q_des 变化 < 1e-3 rad", max_step < 1e-3, "max step=%.3e" % max_step)
    check("2 s 后到达下蹲姿态（knee 收拢 0.35 rad）",
          np.allclose(q_last, sup.crouch_pose, atol=1e-6), "ok")
    check("crouch 用 mode_kp/mode_kd", s.motor_cmd[0].kp == 20.0 and s.motor_cmd[0].kd == 1.0,
          "kp=%.1f kd=%.1f" % (s.motor_cmd[0].kp, s.motor_cmd[0].kd))
    check("crouch 时 tau == 0", np.allclose(taus(s), 0.0), str(taus(s)[:3]))
    check("mode == crouch / level L1", sup.mode == "crouch" and sup.level == Level.L1_DEGRADED,
          "%s %s" % (sup.mode, sup.level))
    st = sup.stats()
    check("stats 记录 crouch 停留时间 ≈ 2.2 s", abs(st["mode_dwell_s"]["crouch"] - 2.2) < 1e-6,
          "%.4f" % st["mode_dwell_s"]["crouch"])
    check("crouch_reached = True", st["crouch_reached"] is True, "ok")
    print("    crouch_pose = %s" % st["crouch_pose"])


def test_supervisor_l2_policies():
    print("\n== 监督器 C：L2 = 阻尼 + 缓慢下蹲（默认）/ damp / crouch ==")
    h2 = make_health(Level.L2_PROTECT, q=STAND + 0.05)
    track_cmd = make_cmd(q=STAND + 0.2, kp=60.0, kd=5.0)

    sup = SafetySupervisor(STAND, crouch_s=1.0, mode_kd=1.0)
    s = sup.process(track_cmd, h2, 0.0, DT)
    check("L2 进入默认先 crouch（缓慢下蹲）", sup.mode == "crouch", sup.mode)
    for k in range(1, 500):                              # 跑到 t=0.998
        s = sup.process(track_cmd, h2, k * DT, DT)
    check("蹲满 crouch_s 之前仍是 crouch", sup.mode == "crouch", sup.mode)
    s = sup.process(track_cmd, h2, 500 * DT, DT)         # t=1.0
    check("蹲满后切 damping", sup.mode == "damping", sup.mode)
    check("damping: kp=0, kd=mode_kd", s.motor_cmd[0].kp == 0.0 and s.motor_cmd[0].kd == 1.0,
          "kp=%.1f kd=%.1f" % (s.motor_cmd[0].kp, s.motor_cmd[0].kd))
    check("damping: q_des = 当前 q", np.allclose(qdes(s), STAND + 0.05, atol=1e-12),
          "q0=%.4f" % s.motor_cmd[0].q)
    check("damping: tau == 0", np.allclose(taus(s), 0.0), str(taus(s)[:3]))
    # 回归：切到 damping 后不能再翻回 crouch（否则 L2 会一直抖）
    n_sw = sup.stats()["n_switches"]
    for k in range(501, 601):
        s = sup.process(track_cmd, h2, k * DT, DT)
    check("damping 后保持 damping（不来回抖）",
          sup.mode == "damping" and sup.stats()["n_switches"] == n_sw,
          "%s n_switches %d->%d" % (sup.mode, n_sw, sup.stats()["n_switches"]))
    check("damping 停留时间在增长", sup.stats()["mode_dwell_s"]["damping"] > 0.19,
          "%.4f" % sup.stats()["mode_dwell_s"]["damping"])

    sup_damp = SafetySupervisor(STAND)
    sup_damp.l2_policy = "damp"
    s = sup_damp.process(track_cmd, h2, 0.0, DT)
    check('l2_policy="damp" 时立刻纯阻尼', sup_damp.mode == "damping" and s.motor_cmd[0].kp == 0.0,
          sup_damp.mode)

    sup_crouch = SafetySupervisor(STAND, crouch_s=0.1)
    sup_crouch.l2_policy = "crouch"
    for k in range(200):
        s = sup_crouch.process(track_cmd, h2, k * DT, DT)
    check('l2_policy="crouch" 时一直保持软 PD 下蹲',
          sup_crouch.mode == "crouch" and s.motor_cmd[0].kp == 20.0, sup_crouch.mode)

    # 无 state 通道时的 damping 退化：q_des 保持上一拍
    sup_no = SafetySupervisor(STAND)
    sup_no.l2_policy = "damp"
    h_no = make_health(Level.L2_PROTECT)
    s = sup_no.process(track_cmd, h_no, 0.0, DT)
    check("无 state 时 damping 退回上一拍 q_des 并计数",
          np.allclose(qdes(s), STAND, atol=1e-12) and sup_no.stats()["n_damping_no_state"] == 1,
          "q0=%.4f n=%d" % (s.motor_cmd[0].q, sup_no.stats()["n_damping_no_state"]))


def test_supervisor_estop():
    print("\n== 监督器 D：L3 -> estop 全 0 ==")
    sup = SafetySupervisor(STAND)
    h3 = make_health(Level.L3_ESTOP)
    s = sup.process(make_cmd(tau=100.0, kp=60.0), h3, 0.0, DT)
    check("mode == estop", sup.mode == "estop", sup.mode)
    check("20 个电机槽位全 0（含 mode）", all_motor_zero(s), "ok")
    s = sup.process(None, h3, DT, DT)
    check("cmd=None 时仍全 0", all_motor_zero(s), "ok")
    check("level 属性 = L3", sup.level == Level.L3_ESTOP, sup.level_name if hasattr(sup, "level_name") else str(sup.level))


def test_supervisor_switches_and_dwell():
    print("\n== 监督器 E：切换次数 / 每级停留时间 / reset ==")
    sup = SafetySupervisor(STAND, crouch_s=2.0)
    cmd = make_cmd(q=STAND, kp=60.0, kd=5.0)
    phases = [(Level.L0_NORMAL, 250), (Level.L1_DEGRADED, 250),
              (Level.L2_PROTECT, 250), (Level.L3_ESTOP, 250)]
    k = 0
    for lvl, n in phases:
        hs = make_health(lvl, q=STAND)
        for _ in range(n):
            sup.process(cmd, hs, k * DT, DT)
            k += 1
    st = sup.stats()
    print("    mode_dwell=%s" % st["mode_dwell_s"])
    print("    level_dwell=%s" % st["level_dwell_s"])
    check("最终 mode estop / level L3",
          st["mode"] == "estop" and st["level_name"] == "L3", "%s %s" % (st["mode"], st["level_name"]))
    check("切换次数 == 2（track->crouch, crouch->estop；L2 0.5s 未满 2s 不切）",
          st["n_switches"] == 2, str(st["n_switches"]))
    check("track dwell = 0.5 s", abs(st["mode_dwell_s"]["track"] - 0.5) < 1e-9,
          "%.6f" % st["mode_dwell_s"]["track"])
    check("crouch dwell = 1.0 s", abs(st["mode_dwell_s"]["crouch"] - 1.0) < 1e-9,
          "%.6f" % st["mode_dwell_s"]["crouch"])
    check("estop dwell = 0.5 s", abs(st["mode_dwell_s"]["estop"] - 0.5) < 1e-9,
          "%.6f" % st["mode_dwell_s"]["estop"])
    check("dwell 总和 = 2.0 s", abs(st["dwell_total_s"] - 2.0) < 1e-9, "%.6f" % st["dwell_total_s"])
    check("每级停留 0.5 s", all(abs(st["level_dwell_s"]["L%d" % i] - 0.5) < 1e-9 for i in range(4)),
          str(st["level_dwell_s"]))
    check("n_process == 1000", st["n_process"] == 1000, str(st["n_process"]))

    sup.reset()
    st2 = sup.stats()
    check("reset() 后回 track / 计数清零",
          st2["mode"] == "track" and st2["n_switches"] == 0 and st2["dwell_total_s"] == 0.0,
          "%s %s" % (st2["mode"], st2["n_switches"]))


def test_supervisor_disabled():
    print("\n== 监督器 F：enabled=False 完全旁路 ==")
    sup = SafetySupervisor(STAND, enabled=False)
    s = sup.process(make_cmd(tau=100.0), make_health(Level.L3_ESTOP), 0.0, DT)
    check("关掉后不卸力：tau 直通 100", np.allclose(taus(s), 100.0, atol=1e-12), str(taus(s)[:3]))
    check("模式保持 track", sup.mode == "track", sup.mode)


def main() -> int:
    print("test_safety.py —— SafetyLimiter 回归 + SafetySupervisor（dt=%.3f）" % DT)
    test_old_signatures()
    test_limiter_source_unchanged()
    test_old_ramp_and_limits()
    test_old_watchdog()
    test_supervisor_track()
    test_supervisor_crouch()
    test_supervisor_l2_policies()
    test_supervisor_estop()
    test_supervisor_switches_and_dwell()
    test_supervisor_disabled()
    n_fail = sum(1 for _n, ok in RESULT if not ok)
    print("\n===== test_safety 汇总 =====")
    print("  %d/%d 通过" % (len(RESULT) - n_fail, len(RESULT)))
    if n_fail:
        for n, ok in RESULT:
            if not ok:
                print("  FAIL: %s" % n)
        raise SystemExit(1)
    print("OK test_safety.py 全部 %d 项通过" % len(RESULT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
