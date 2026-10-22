#!/usr/bin/env python3
"""tests/test_health.py —— HealthMonitor 分级/迟滞/锁存验收（契约 §4）。

直接跑:
    .venv/bin/python tests/test_health.py
不用 pytest。全部通过打印 "OK ..." 并以 0 退出，否则 SystemExit(1)。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from ros2_controller.health import Channel, HealthMonitor, Level  # noqa: E402
from signal_io.msg import NUM_JOINT, LowState, MotorState  # noqa: E402

DT = 0.002
RESULT = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULT.append((name, bool(ok)))
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    return bool(ok)


def make_state(rpy=(0.0, 0.0, 0.0), q=None) -> LowState:
    st = LowState()
    st.imu_state.rpy = list(rpy)
    qq = np.zeros(NUM_JOINT) if q is None else np.asarray(q, dtype=np.float64)
    for i in range(NUM_JOINT):
        st.motor_state[i] = MotorState(q=float(qq[i]))
    return st


def cmd_vel(valid: bool, now: float) -> Channel:
    return Channel(name="cmd_vel", value=np.array([0.5, 0.0, 0.0]), stamp=now,
                   valid=valid, sigma=0.0)


def drive(mon: HealthMonitor, n_steps: int, plan, dt: float = DT):
    """plan(k) -> (state | None, extra | None)；返回 (迁移事件, 最后一次 HealthState)。"""
    events = []
    prev = None
    last = None
    for k in range(n_steps + 1):
        t = k * dt
        state, extra = plan(k)
        hs = mon.check(state, t, extra=extra)
        if prev is None or hs.level != prev:
            events.append({"t": t, "level": hs.level, "reasons": tuple(hs.reasons),
                           "entered_at": hs.entered_at})
        prev = hs.level
        last = hs
    return events, last


def reasons_contain(reasons, needle: str) -> bool:
    return any(needle in r for r in reasons)


# ---------------------------------------------------------------- 用例
def test_defaults_and_cfg():
    print("\n== 默认阈值 / cfg 覆盖 ==")
    mon = HealthMonitor()
    check("默认 estop=200ms", abs(mon.estop_s - 0.2) < 1e-12, "%.4f s" % mon.estop_s)
    check("默认 protect=60ms", abs(mon.protect_s - 0.06) < 1e-12, "%.4f s" % mon.protect_s)
    check("默认 recover=1.0s", abs(mon.recover_s - 1.0) < 1e-12, "%.4f s" % mon.recover_s)
    check("默认 tilt=60deg", abs(mon.max_tilt_deg - 60.0) < 1e-12, "%.1f deg" % mon.max_tilt_deg)

    mon2 = HealthMonitor({"health": {"protect_ms": 100.0, "recover_s": 0.25}})
    check("cfg 从 health 子段覆盖", abs(mon2.protect_s - 0.1) < 1e-12
          and abs(mon2.recover_s - 0.25) < 1e-12,
          "protect=%.3f recover=%.3f" % (mon2.protect_s, mon2.recover_s))
    mon3 = HealthMonitor({"estop_ms": 500.0})
    check("cfg 直接就是 health 段", abs(mon3.estop_s - 0.5) < 1e-12, "%.3f s" % mon3.estop_s)
    check("cfg 未覆盖项保持默认", abs(mon3.protect_s - 0.06) < 1e-12, "%.3f s" % mon3.protect_s)


def test_l0_baseline():
    print("\n== L0：全部通道健康 ==")
    mon = HealthMonitor()
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        return make_state(q=np.zeros(NUM_JOINT)), {"cmd_vel": cmd_vel(True, t)}

    events, last = drive(mon, 50, plan)
    check("全程 L0（只有 1 次 '迁移'=初始状态）", len(events) == 1 and events[0]["level"] == Level.L0_NORMAL,
          "events=%s" % [(round(e["t"], 3), int(e["level"])) for e in events])
    check("level_name == 'L0'", last.level_name == "L0", last.level_name)
    check("reasons 为空", last.reasons == (), str(last.reasons))
    st = mon.stats()
    check("n_transitions == 0", st["n_transitions"] == 0, str(st["n_transitions"]))
    check("state 通道 valid", st["state_valid"] is True, str(st["state_valid"]))
    check("channels 含 state/cmd_vel", st["channels"] == ["cmd_vel", "state"], str(st["channels"]))
    check("n_checks 正确", st["n_checks"] == 51, str(st["n_checks"]))


def test_full_chain():
    print("\n== L0 -> L1 -> L2 -> L3 -> reset -> L0 全链路 ==")
    mon = HealthMonitor()
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        state = make_state(q=np.zeros(NUM_JOINT)) if k < 150 else None   # 0.298 s 后不再来 state
        extra = {"cmd_vel": cmd_vel(k < 50, t)}                          # 0.1 s 起指令丢失
        return state, extra

    events, last = drive(mon, 599, plan)
    got = [(e["t"], e["level"]) for e in events]
    print("    实际迁移: %s" % [(round(t, 4), "L%d" % int(l)) for t, l in got])
    check("共 4 次迁移", len(events) == 4, str(len(events)))
    check("t=0 起于 L0", got[0][1] == Level.L0_NORMAL and got[0][0] == 0.0, str(got[0]))
    check("t≈0.100 升 L1", got[1][1] == Level.L1_DEGRADED and abs(got[1][0] - 0.100) <= 2 * DT,
          "t=%.4f" % got[1][0])
    check("L1 原因 = cmd_vel 缺失",
          reasons_contain(events[1]["reasons"], "cmd_vel:invalid"), str(events[1]["reasons"]))
    check("t≈0.360 升 L2（state 看门狗 60ms）",
          got[2][1] == Level.L2_PROTECT and abs(got[2][0] - 0.360) <= 2 * DT, "t=%.4f" % got[2][0])
    check("L2 原因 = state_watchdog",
          reasons_contain(events[2]["reasons"], "state_watchdog"), str(events[2]["reasons"]))
    check("t≈0.500 升 L3（state 完全没来 200ms）",
          got[3][1] == Level.L3_ESTOP and abs(got[3][0] - 0.500) <= 2 * DT, "t=%.4f" % got[3][0])
    check("L3 原因 = state_timeout",
          reasons_contain(events[3]["reasons"], "state_timeout"), str(events[3]["reasons"]))
    check("entered_at == 迁移时刻（升级立即）",
          all(abs(e["entered_at"] - e["t"]) < 1e-12 for e in events), "ok")
    check("n_transitions == 3", mon.stats()["n_transitions"] == 3, str(mon.stats()["n_transitions"]))

    # k=300 起一切恢复正常，但 L3 锁存
    rec = mon.check(make_state(), 300 * DT, extra={"cmd_vel": cmd_vel(True, 300 * DT)})
    check("恢复输入后仍锁在 L3", rec.level == Level.L3_ESTOP, rec.level_name)
    check("锁存原因可见", reasons_contain(rec.reasons, "estop_latched"), str(rec.reasons))

    for k in range(301, 600):
        t = k * DT
        hs = mon.check(make_state(), t, extra={"cmd_vel": cmd_vel(True, t)})
    check("锁存期内一直 L3", hs.level == Level.L3_ESTOP, hs.level_name)

    mon.reset(now=2.0)
    hs = mon.check(make_state(), 2.0 + DT, extra={"cmd_vel": cmd_vel(True, 2.0 + DT)})
    check("reset() 后回到 L0", hs.level == Level.L0_NORMAL and hs.reasons == (),
          "%s %s" % (hs.level_name, hs.reasons))
    st = mon.stats()
    check("reset() 清掉统计", st["n_transitions"] == 0 and st["l3_latched"] is False, str(st["n_transitions"]))


def test_hysteresis_jitter_no_flap():
    print("\n== 迟滞 A：阈值附近抖动不引起反复跳变 ==")
    mon = HealthMonitor()
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        bad = (k // 25) % 2 == 1        # 0.05 s 坏 / 0.05 s 好，周期 0.1 s << recover_s=1 s
        return make_state(), {"cmd_vel": cmd_vel(not bad, t)}

    events, last = drive(mon, 1499, plan)
    print("    实际迁移: %s" % [(round(e["t"], 4), "L%d" % int(e["level"])) for e in events])
    check("只发生 1 次升级（L0->L1）", len(events) == 2
          and events[1]["level"] == Level.L1_DEGRADED, str(len(events)))
    check("稳定停在 L1（未被好帧拉回）", last.level == Level.L1_DEGRADED, last.level_name)
    check("n_transitions == 1", mon.stats()["n_transitions"] == 1, str(mon.stats()["n_transitions"]))


def test_hysteresis_recover_s():
    print("\n== 迟滞 B：降级必须连续满足 recover_s ==")
    mon = HealthMonitor()
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        valid = k >= 50                 # 0.1 s 起指令恢复
        return make_state(), {"cmd_vel": cmd_vel(valid, t)}

    events, last = drive(mon, 600, plan)
    print("    实际迁移: %s" % [(round(e["t"], 4), "L%d" % int(e["level"])) for e in events])
    check("t=0 直接进 L1", events[0]["level"] == Level.L1_DEGRADED and events[0]["t"] == 0.0,
          str(events[0]))
    check("t≈1.100 才回 L0（0.1+1.0）",
          len(events) == 2 and events[1]["level"] == Level.L0_NORMAL
          and abs(events[1]["t"] - 1.100) <= 2 * DT, str([(round(e["t"], 4), int(e["level"])) for e in events]))

    # 逐点确认：恢复后 0.9 s（t=1.0）还没回落
    mon2 = HealthMonitor()
    mon2.reset(0.0)
    drive(mon2, 449, plan)              # 跑到 t=0.898
    hs = mon2.check(make_state(), 500 * DT, extra={"cmd_vel": cmd_vel(True, 500 * DT)})  # t=1.0
    check("recover_s 未满时仍在 L1", hs.level == Level.L1_DEGRADED, hs.level_name)
    hs = mon2.check(make_state(), 550 * DT, extra={"cmd_vel": cmd_vel(True, 550 * DT)})  # t=1.1
    check("recover_s 满时回 L0", hs.level == Level.L0_NORMAL, hs.level_name)
    check("n_transitions == 2", mon2.stats()["n_transitions"] == 2, str(mon2.stats()["n_transitions"]))


def test_l3_triggers_and_latch():
    print("\n== L3：姿态超限 / 关节 NaN / reset 解锁 ==")
    mon = HealthMonitor()
    mon.reset(0.0)
    hs = mon.check(make_state(rpy=(0.0, math.radians(61.0), 0.0)), DT)
    check("pitch 61deg -> L3", hs.level == Level.L3_ESTOP, hs.level_name)
    check("原因含 tilt", reasons_contain(hs.reasons, "tilt="), str(hs.reasons))
    check("l3_latched", mon.stats()["l3_latched"] is True, str(mon.stats()["l3_latched"]))
    mon.reset(now=1.0)
    hs = mon.check(make_state(), 1.0 + DT)
    check("reset() 解锁 -> L0", hs.level == Level.L0_NORMAL and mon.latched is False, hs.level_name)

    mon.reset(now=0.0)
    st = make_state()
    st.motor_state[5].q = float("nan")
    hs = mon.check(st, DT)
    check("关节角 NaN -> L3", hs.level == Level.L3_ESTOP, hs.level_name)
    check("原因含 joint_nan", reasons_contain(hs.reasons, "joint_nan"), str(hs.reasons))

    # LowState 完全没来
    mon.reset(now=0.0)
    hs0 = mon.check(None, 0.0)
    check("没来但在 estop 窗口内仍 L0", hs0.level == Level.L0_NORMAL, hs0.level_name)
    hs1 = mon.check(None, 0.25)
    check("250ms 没来 -> L3", hs1.level == Level.L3_ESTOP and reasons_contain(hs1.reasons, "state_timeout"),
          "%s %s" % (hs1.level_name, hs1.reasons))


def test_l2_imu_and_sigma():
    print("\n== L2：IMU 无效 / NaN / sigma 超限 ==")
    mon = HealthMonitor()
    mon.reset(0.0)
    hs = mon.check(make_state(rpy=(0.0, float("nan"), 0.0)), DT)
    check("IMU NaN -> L2", hs.level == Level.L2_PROTECT and reasons_contain(hs.reasons, "imu_nan"),
          "%s %s" % (hs.level_name, hs.reasons))

    mon.reset(0.0)
    st = make_state()
    st.imu_state.rpy = [0.0]
    hs = mon.check(st, DT)
    check("IMU 维度不足 -> L2", hs.level == Level.L2_PROTECT and reasons_contain(hs.reasons, "imu_invalid"),
          "%s %s" % (hs.level_name, hs.reasons))

    mon.reset(0.0)
    ch = Channel(name="leg_odom", value=np.zeros(3), stamp=DT, valid=True, sigma=0.6)
    hs = mon.check(make_state(), DT, extra={"leg_odom": ch})
    check("sigma 0.6 > sigma_max 0.5 -> L2",
          hs.level == Level.L2_PROTECT and reasons_contain(hs.reasons, "leg_odom:sigma"),
          "%s %s" % (hs.level_name, hs.reasons))
    check("check() 填了 age_s", abs(ch.age_s) < 1e-12, "%.6f" % ch.age_s)

    mon.reset(0.0)
    ch = Channel(name="leg_odom", value=np.zeros(3), stamp=DT, valid=True, sigma=0.3)
    hs = mon.check(make_state(), DT, extra={"leg_odom": ch})
    check("sigma 0.3 -> L1 sigma_warn",
          hs.level == Level.L1_DEGRADED and reasons_contain(hs.reasons, "sigma_warn"),
          "%s %s" % (hs.level_name, hs.reasons))

    mon.reset(0.0)
    ch = Channel(name="leg_odom", value=np.zeros(3), stamp=DT, valid=True, sigma=0.0)
    hs = mon.check(make_state(), DT, extra={"leg_odom": ch})
    check("sigma 0 -> L0", hs.level == Level.L0_NORMAL, hs.level_name)

    # 通道超时：必选通道 100ms 没更新 -> L2；可选通道同样情况只 L1
    mon.reset(0.0)
    ch = Channel(name="leg_odom", value=np.zeros(3), stamp=1.0, valid=True, sigma=0.0)
    hs = mon.check(make_state(), 1.0 + 0.1, extra={"leg_odom": ch})
    check("必选通道 100ms 未更新 -> L2",
          hs.level == Level.L2_PROTECT and reasons_contain(hs.reasons, "leg_odom:stale"),
          "%s %s" % (hs.level_name, hs.reasons))
    mon.reset(0.0)
    ch = Channel(name="height_scan", value=np.zeros(35), stamp=1.0, valid=True, sigma=0.0)
    hs = mon.check(make_state(), 1.0 + 0.1, extra={"height_scan": ch})
    check("可选通道 100ms 未更新 -> L1",
          hs.level == Level.L1_DEGRADED and reasons_contain(hs.reasons, "height_scan:stale"),
          "%s %s" % (hs.level_name, hs.reasons))

    # 必选通道 invalid -> L2；可选通道 invalid -> L1
    mon.reset(0.0)
    ch = Channel(name="leg_odom", value=None, stamp=0.0, valid=False)
    hs = mon.check(make_state(), DT, extra={"leg_odom": ch})
    check("必选通道 invalid -> L2", hs.level == Level.L2_PROTECT, hs.level_name)
    mon.reset(0.0)
    ch = Channel(name="cmd_vel", value=None, stamp=0.0, valid=False)
    hs = mon.check(make_state(), DT, extra={"cmd_vel": ch})
    check("可选通道 invalid -> L1", hs.level == Level.L1_DEGRADED, hs.level_name)


def test_l1_confidence():
    print("\n== L1：估计速度置信度 ==")
    mon = HealthMonitor()
    mon.reset(0.0)
    ch = Channel(name="linvel_conf", value=np.array([0.1]), stamp=DT, valid=True, sigma=0.0)
    hs = mon.check(make_state(), DT, extra={"linvel_conf": ch})
    check("confidence 0.1 < 0.3 -> L1",
          hs.level == Level.L1_DEGRADED and reasons_contain(hs.reasons, "low_conf"),
          "%s %s" % (hs.level_name, hs.reasons))
    mon.reset(0.0)
    ch = Channel(name="linvel_conf", value=np.array([0.9]), stamp=DT, valid=True, sigma=0.0)
    hs = mon.check(make_state(), DT, extra={"linvel_conf": ch})
    check("confidence 0.9 -> L0", hs.level == Level.L0_NORMAL, hs.level_name)


def test_cfg_override_behaviour():
    print("\n== cfg 覆盖真的改变行为 ==")
    mon = HealthMonitor({"max_tilt_deg": 10.0, "recover_s": 0.1})
    mon.reset(0.0)
    hs = mon.check(make_state(rpy=(0.0, math.radians(20.0), 0.0)), DT)
    check("max_tilt_deg=10 时 20deg 就 L3", hs.level == Level.L3_ESTOP, hs.level_name)
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        return make_state(), {"cmd_vel": cmd_vel(k >= 5, t)}

    drive(mon, 20, plan)
    hs = mon.check(make_state(), 21 * DT, extra={"cmd_vel": cmd_vel(True, 21 * DT)})
    # 恢复从 t=0.01 起算，recover_s=0.1 -> t=0.11 回落；t=0.042 时还不该回落
    check("recover_s=0.1 时 0.032s 未回落", hs.level == Level.L1_DEGRADED, hs.level_name)
    drive(mon, 80, plan)
    check("recover_s=0.1 时最终回 L0", mon.level == Level.L0_NORMAL, mon.stats()["level_name"])


def test_stats_dwell():
    print("\n== stats(): 停留时间 / 通道清单 ==")
    mon = HealthMonitor()
    mon.reset(0.0)

    def plan(k):
        t = k * DT
        return make_state(), {"cmd_vel": cmd_vel(True, t)}

    drive(mon, 500, plan)
    st = mon.stats()
    check("dwell L0 ≈ 1.0 s", abs(st["dwell_s"]["L0"] - 1.0) < 1e-9, "%.6f" % st["dwell_s"]["L0"])
    check("dwell_total ≈ 1.0 s", abs(st["dwell_total_s"] - 1.0) < 1e-9, "%.6f" % st["dwell_total_s"])
    check("n_checks == 501", st["n_checks"] == 501, str(st["n_checks"]))
    check("ever_got_state", st["ever_got_state"] is True, str(st["ever_got_state"]))
    check("optional_channels 已记录", st["optional_channels"] == ["cmd_vel", "height_scan"],
          str(st["optional_channels"]))


def main() -> int:
    print("test_health.py —— HealthMonitor（%d 关节，dt=%.3fs）" % (NUM_JOINT, DT))
    test_defaults_and_cfg()
    test_l0_baseline()
    test_full_chain()
    test_hysteresis_jitter_no_flap()
    test_hysteresis_recover_s()
    test_l3_triggers_and_latch()
    test_l2_imu_and_sigma()
    test_l1_confidence()
    test_cfg_override_behaviour()
    test_stats_dwell()
    n_fail = sum(1 for _n, ok in RESULT if not ok)
    print("\n===== test_health 汇总 =====")
    print("  %d/%d 通过" % (len(RESULT) - n_fail, len(RESULT)))
    if n_fail:
        for n, ok in RESULT:
            if not ok:
                print("  FAIL: %s" % n)
        raise SystemExit(1)
    print("OK test_health.py 全部 %d 项通过" % len(RESULT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
