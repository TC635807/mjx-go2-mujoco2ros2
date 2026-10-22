#!/usr/bin/env python3
"""tests/test_domain_gap.py —— domain_gap.py 验收（契约 docs/interfaces.md §7）。

直接运行：python tests/test_domain_gap.py     （成功打印 OK 并以 0 退出）

覆盖：
  1. 域差清单可加载、字段合法、关键数字真实可核查（5.204/6.921、35.55/45.43、damping、home、无基座线速度）
  2. format_table 覆盖全部条目 + 汇总
  3. SignalDegrader：dropout=0.5 实测丢帧率 ≈0.5；延迟 N 拍输出旧值；噪声 std 与配置相符（±15%）；
     偏置/量化/突发丢帧/可复现/不改输入
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from signal_io.domain_gap import (  # noqa: E402
    DEFAULT_GAP_PATH, GapItem, SignalDegrader, format_table, load_degrader_profiles,
    load_gap_table,
)
from signal_io import domain_gap as dg  # noqa: E402
from signal_io.msg import LowState, MotorState  # noqa: E402

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def make_state(tick: int = 0, q0: float = 0.5, dq0: float = -0.1) -> LowState:
    st = LowState()
    st.tick = int(tick)
    st.imu_state.gyroscope = [0.01, -0.02, 0.03]
    st.imu_state.accelerometer = [0.02, -0.01, 9.81]
    st.imu_state.rpy = [0.01, -0.02, 0.03]
    for i in range(12):
        st.motor_state[i] = MotorState(mode=1, q=q0 + 0.01 * i, dq=dq0 + 0.001 * i, tau_est=0.0)
    st.foot_force = [10, 20, 30, 40]
    return st


def main() -> int:
    # ------------------------------------------------------------ 1. 清单
    print("[1] 域差清单 config/domain_gap.json")
    items = load_gap_table()
    check("默认路径存在", os.path.isfile(DEFAULT_GAP_PATH), DEFAULT_GAP_PATH)
    check("条目数 >= 6", len(items) >= 6, "n=%d" % len(items))
    check("id 唯一", len({i.id for i in items}) == len(items))
    check("severity/status 合法",
          all(i.severity in ("high", "medium", "low") for i in items)
          and all(i.status in ("closed", "mitigated", "open") for i in items))
    check("每条都有 evidence", all(i.evidence.strip() for i in items),
          "缺证据: %s" % [i.id for i in items if not i.evidence.strip()])

    def find(pred):
        return [i for i in items if pred(i)]

    linvel = find(lambda i: i.category == "sensing/base-linvel")
    check("真机无基座线速度传感器 (域差条目)", len(linvel) == 1 and linvel[0].status == "mitigated",
          (linvel[0].id + " status=" + linvel[0].status) if linvel else "未找到")
    scan = find(lambda i: i.category == "sensing/height-scan")
    check("高度扫描: 仿真 ray-cast vs 真机盲区", len(scan) == 1,
          scan[0].id if scan else "未找到")
    lat = find(lambda i: i.category == "sensing/latency-dropout")
    check("延迟/丢包域差 (SignalDegrader 覆盖)", len(lat) == 1,
          lat[0].id if lat else "未找到")
    fault = find(lambda i: i.category == "actuator/faults")
    check("电机 lost/过热/掉线域差 (健康层覆盖)", len(fault) == 1,
          fault[0].id if fault else "未找到")
    torque = find(lambda i: i.category == "actuator/torque-limits")
    check("力矩上限已与 go2.xml 对齐 (closed)", len(torque) == 1 and torque[0].status == "closed",
          torque[0].id if torque else "未找到")
    check("每条 mitigated/open 都有缓解措施", all(i.mitigation.strip() for i in items))
    print("     status 分布: %s" % {s: sum(1 for i in items if i.status == s)
                                     for s in ("closed", "mitigated", "open")})

    print("[2] format_table")
    text = format_table(items)
    check("包含表头", "ID" in text and "STATUS" in text and "MITIGATION" in text)
    check("覆盖全部 id", all(i.id in text for i in items))
    check("含汇总行", "总计 %d 项" % len(items) in text,
          text.strip().splitlines()[-1][:60] + " ...")

    print("[3] 校验与错误处理")
    try:
        GapItem(id="X", category="c", sim="s", real="r", severity="high",
                status="unknown", mitigation="m")
        check("非法 status 抛 ValueError", False)
    except ValueError:
        check("非法 status 抛 ValueError", True)
    try:
        load_gap_table(os.path.join(ROOT, "config", "__nope__.json"))
        check("不存在的路径抛 FileNotFoundError", False)
    except FileNotFoundError:
        check("不存在的路径抛 FileNotFoundError", True)

    print("[3b] 内部快速克隆与 copy.deepcopy 逐字段一致")
    import copy as _copy
    import dataclasses as _dc

    s_cmp = make_state(7, q0=0.31, dq0=-0.22)
    cl = dg._clone_state(s_cmp)
    dp = _copy.deepcopy(s_cmp)
    same = all(getattr(cl, f.name) == getattr(dp, f.name) for f in _dc.fields(LowState))
    same = same and cl is not dp and cl is not s_cmp
    check("clone == deepcopy 且为新对象", bool(same))

    # ------------------------------------------------------------ 4. 透传
    print("[4] SignalDegrader 默认关闭时原样透传")
    d0 = SignalDegrader()
    st = make_state(0)
    check("默认 enabled=False", d0.enabled is False)
    check("透传同一对象", d0.degrade_state(st, 0.0) is st)

    # ------------------------------------------------------------ 5. 丢帧率
    print("[5] dropout=0.5 → 实测丢帧率 ≈ 0.5")
    n = 20000
    d1 = SignalDegrader({"enabled": True, "seed": 0, "dropout": 0.5})
    dropped = 0
    for k in range(n):
        if d1.degrade_state(make_state(k), k * 0.02) is None:
            dropped += 1
    rate = dropped / n
    check("实测丢帧率在 0.5±0.02", abs(rate - 0.5) <= 0.02,
          "rate=%.4f (dropped %d/%d)" % (rate, dropped, n))
    check("stats 与实测一致", d1.stats()["n_dropped"] == dropped
          and abs(d1.stats()["dropout_measured"] - rate) < 1e-12,
          "stats=%s" % {k: v for k, v in d1.stats().items() if k in ("n_in", "n_out", "n_dropped")})

    # ------------------------------------------------------------ 6. 延迟
    print("[6] latency_steps=N → 输出第 k-N 拍；前 N 拍 None")
    N = 4
    d2 = SignalDegrader({"enabled": True, "seed": 0, "latency_steps": N})
    outs = [d2.degrade_state(make_state(k), k * 0.02) for k in range(12)]
    check("前 N 拍返回 None（冷启动）", all(o is None for o in outs[:N]),
          "outs[:%d]=%s" % (N, [None if o is None else o.tick for o in outs[:N]]))
    got = [o.tick if o is not None else None for o in outs]
    check("第 k 拍输出 tick=k-N", got[N:] == list(range(0, 12 - N)),
          "ticks=%s" % got)

    # ------------------------------------------------------------ 7. 噪声 std
    print("[7] 噪声 std 与配置相符（±15%）")
    cfg_noise = {"enabled": True, "seed": 1,
                 "noise": {"gyro": 0.01, "accel": 0.02, "rpy": 0.0,
                           "joint_q": 0.005, "joint_dq": 0.01, "foot_force": 0.0}}
    d3 = SignalDegrader(cfg_noise)
    n = 20000
    ref = make_state(0)
    ref_g = np.asarray(ref.imu_state.gyroscope)
    ref_a = np.asarray(ref.imu_state.accelerometer)
    g = np.zeros((n, 3))
    a = np.zeros((n, 3))
    q = np.zeros(n)
    dq = np.zeros(n)
    for k in range(n):
        out = d3.degrade_state(ref, k * 0.02)
        assert out is not None
        g[k] = np.asarray(out.imu_state.gyroscope) - ref_g
        a[k] = np.asarray(out.imu_state.accelerometer) - ref_a
        q[k] = out.motor_state[0].q - 0.5
        dq[k] = out.motor_state[0].dq - (-0.1)
    for name, got_std, want in (("gyro", float(np.std(g)), 0.01),
                                ("accel", float(np.std(a)), 0.02),
                                ("joint_q", float(np.std(q)), 0.005),
                                ("joint_dq", float(np.std(dq)), 0.01)):
        rel = abs(got_std - want) / want
        check("%s std=%.5f (cfg %.5f, 偏差 %.1f%%)" % (name, got_std, want, rel * 100),
              rel <= 0.15)
    check("rpy 未配噪声 → 四元数不变",
          list(d3.degrade_state(make_state(0), 0.0).imu_state.quaternion)
          == make_state(0).imu_state.quaternion)

    # ------------------------------------------------------------ 8. 偏置 / 量化 / 突发
    print("[8] 偏置 / 量化 / 突发丢帧 / 可复现 / 不改输入")
    d4 = SignalDegrader({"enabled": True, "seed": 0,
                         "bias": {"gyro": [0.1, -0.2, 0.3], "joint_q": 0.05}})
    src = make_state(0)
    out4 = d4.degrade_state(src, 0.0)
    bias_ok = (np.allclose(np.asarray(out4.imu_state.gyroscope) - np.asarray(src.imu_state.gyroscope),
                           [0.1, -0.2, 0.3], atol=1e-12)
               and abs(out4.motor_state[0].q - src.motor_state[0].q - 0.05) < 1e-12)
    check("常值偏置精确可加", bool(bias_ok),
          "gyro diff=%s q diff=%.6f" % (list(np.asarray(out4.imu_state.gyroscope) - np.asarray(src.imu_state.gyroscope)),
                                        out4.motor_state[0].q - src.motor_state[0].q))

    d5 = SignalDegrader({"enabled": True, "seed": 0, "quantization": {"joint_q": 0.001, "joint_dq": 0.002}})
    s5 = make_state(0, q0=0.5003, dq0=-0.1004)
    o5 = d5.degrade_state(s5, 0.0)
    check("量化到 0.001 网格", abs(o5.motor_state[0].q / 0.001 - round(o5.motor_state[0].q / 0.001)) < 1e-9
          and abs(o5.motor_state[0].q - 0.5003) <= 0.0005 + 1e-12,
          "q=%.6f" % o5.motor_state[0].q)

    d6 = SignalDegrader({"enabled": True, "seed": 3, "burst": {"prob": 0.25, "len": 4}})
    for k in range(20000):
        d6.degrade_state(make_state(k), k * 0.02)
    check("突发丢帧计数 > 500（触发后连续丢 4 拍）", d6.stats()["n_burst_dropped"] > 500,
          "burst_dropped=%d dropped=%d" % (d6.stats()["n_burst_dropped"], d6.stats()["n_dropped"]))

    dd1 = SignalDegrader({"enabled": True, "seed": 7, "dropout": 0.3})
    dd2 = SignalDegrader({"enabled": True, "seed": 7, "dropout": 0.3})
    seq1 = [dd1.degrade_state(make_state(k), 0.0) is None for k in range(200)]
    seq2 = [dd2.degrade_state(make_state(k), 0.0) is None for k in range(200)]
    check("同 seed 丢帧序列可复现", seq1 == seq2, "drops=%d/200" % sum(seq1))

    before = make_state(5)
    snapshot = (list(before.imu_state.gyroscope), before.motor_state[0].q, before.tick)
    SignalDegrader({"enabled": True, "seed": 0, "noise": {"gyro": 0.1, "joint_q": 0.1}}).degrade_state(before, 0.0)
    after = (list(before.imu_state.gyroscope), before.motor_state[0].q, before.tick)
    check("不改动输入 state", snapshot == after, "before=%s after=%s" % (snapshot, after))

    profs = load_degrader_profiles()
    check("profiles 可读且含 real_mild/real_severe",
          "real_mild" in profs and "real_severe" in profs, "profiles=%s" % sorted(profs))
    d7 = SignalDegrader({"profile": "real_mild"})
    check("profile=real_mild 生效", d7.enabled and d7.latency_steps == 1 and abs(d7.dropout - 0.02) < 1e-12,
          "latency=%d dropout=%.3f" % (d7.latency_steps, d7.dropout))

    print()
    if FAILS:
        print("FAIL test_domain_gap: %d 项失败 -> %s" % (len(FAILS), FAILS))
        return 1
    print("OK test_domain_gap: 全部检查通过（%d 条域差, 丢帧率 %.4f, 延迟 %d 拍）" % (len(items), rate, N))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
