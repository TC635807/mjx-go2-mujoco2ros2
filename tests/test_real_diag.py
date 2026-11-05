#!/usr/bin/env python3
"""tests/test_real_diag.py —— 实机传感器自检诊断内核的纯 Python 验收。

覆盖（对应 shared task T1 的交付物 1）：
  1. 全好样本：真机/仿真两档都不许出现 WARN/FAIL（可选通道允许 SKIP）；
  2. 逐项坏样本：每个判定项各造一个坏样本，必须命中预期 status；
  3. 边界：频率 200/400 Hz、温度 70/85 ℃、关节越界 0.1 rad、CRC 条数边界；
  4. 鲁棒性：NaN/Inf、空数据、字段长度不足、1 条样本、时间戳不增；
  5. 序列化：to_dict()/dumps() 能过 json.dumps(allow_nan=False)；
  6. --require 语义：分组展开、未知 token、必需项 SKIP 降级成 WARN；
  7. 纯 Python：模块不 import rclpy（本机没装 ROS 2 也能跑）。

直接运行：.venv/bin/python tests/test_real_diag.py
"""

from __future__ import annotations

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from signal_io.real_diag import (  # noqa: E402
    ACCEL_G, FAIL, HOME_Q, JOINT_HI, JOINT_LO, LIDAR_FRAME_REAL, LIDAR_FRAME_SIM,
    PASS, SKIP, STATE_HZ_OK, STATE_HZ_WARN, TAU_LIMIT, WARN,
    Check, LidarSample, LowCmdSamples, LowStateSample, Report, Samples, analyze,
    dumps, render_text, synthetic_samples,
)

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def status_of(rep, name: str) -> str:
    c = rep.get(name)
    return c.status if c else "(missing)"


def expect(rep, name: str, want: str, label: str = "") -> None:
    got = status_of(rep, name)
    check("%s -> %s" % (label or name, want), got == want, "got=%s" % got)


def expect_clean(rep, label: str) -> None:
    bad = [(c.name, c.status) for c in rep.checks if c.status in (WARN, FAIL)]
    check(label + "：无 WARN/FAIL", not bad, str(bad))


# ---------------------------------------------------------------- 造样本
def good_sample(t: float = 0.0, tick: int = 0, **kw) -> LowStateSample:
    """一条"站着、已使能、IMU 正常、四脚受力"的好样本（可覆盖任意字段）。

    默认 gyro 给一点微小噪声（恒 0 会被判 WARN，因为可能是消息没填）。
    """
    base = dict(
        t=t,
        tick=tick,
        mode=[0x01] * 12,
        q=list(HOME_Q),
        dq=[0.0] * 12,
        ddq=[0.0] * 12,
        tau_est=[0.0] * 12,
        temperature=[35] * 12,
        lost=[0] * 12,
        foot_force=[50.0, 48.0, 52.0, 49.0],
        rpy=[0.0, 0.0, 0.0],
        quat=[1.0, 0.0, 0.0, 0.0],
        gyro=[0.001, -0.001, 0.0005],
        accel=[0.0, 0.0, ACCEL_G],
        bit_flag=0,
        power_v=28.8,
        power_a=3.0,
    )
    base.update(kw)
    return LowStateSample(**base)


def good_samples(n: int = 50, hz: float = 500.0, tick_step: int = 1, **kw) -> list:
    return [good_sample(t=i / hz, tick=i * tick_step, **kw) for i in range(n)]


def good_lidar(n: int = 10, hz: float = 11.0, frame: str = LIDAR_FRAME_REAL) -> list:
    return [LidarSample(t=i / hz, stamp=i / hz, frame_id=frame, n_points=3000,
                        p1=[-4.0, -4.0, -0.35], p50=[1.5, 0.2, -0.05],
                        p99=[5.2, 4.8, 0.9]) for i in range(n)]


def good_sim_samples(n: int = 50, hz: float = 500.0) -> Samples:
    """仿真档好样本：tick 2ms/拍、温度不填(0)、无电池、有 odom。"""
    s = Samples(target="sim_ros", duration_s=n / hz, tick_step=2,
                lowstate=good_samples(n, hz, tick_step=2, temperature=[0] * 12,
                                      power_v=None, power_a=None),
                lidar=good_lidar(11, 15.0, frame=LIDAR_FRAME_SIM),
                topics={"odom": 250, "sportmodestate": 250, "pose": 0, "lidar_state": 0})
    return s


def with_lowstate(samples: list, target: str = "real_ros", **kw) -> Samples:
    n = len(samples)
    return Samples(target=target, duration_s=n / 500.0, lowstate=list(samples),
                   lidar=good_lidar(), topics={}, **kw)


# ================================================================ 用例
def t_good_samples() -> None:
    print("== 1. 全好样本 ==")
    rep = analyze(synthetic_samples(target="real_ros"))
    expect_clean(rep, "真机合成样本")
    check("真机：结论 PASS", rep.worst() == PASS, rep.worst())
    check("真机：PASS 条数 >= 18", rep.counts()[PASS] >= 18, str(rep.counts()))
    check("真机：必需项 verdict=PASS",
          rep.verdict(["state", "imu", "motors", "lidar"]) == PASS,
          rep.verdict(["state", "imu", "motors", "lidar"]))

    rep2 = analyze(good_sim_samples())
    expect_clean(rep2, "仿真合成样本")
    check("仿真：温度恒 0 -> SKIP", status_of(rep2, "motors.temperature") == SKIP,
          status_of(rep2, "motors.temperature"))
    check("仿真：odom 有数据 -> PASS", status_of(rep2, "odom") == PASS,
          status_of(rep2, "odom"))
    check("仿真：点云 frame_id=world -> PASS",
          status_of(rep2, "lidar.frame_id") == PASS, status_of(rep2, "lidar.frame_id"))

    rep3 = analyze(with_lowstate(good_samples(12)))
    expect_clean(rep3, "手工好样本")
    for name in ("lowstate.rate", "lowstate.tick", "motors.mode", "motors.q",
                 "motors.dq", "motors.tau", "imu.rpy", "imu.quat", "imu.accel",
                 "imu.gyro", "foot_force", "bit_flag", "power"):
        expect(rep3, name, PASS)


def t_rate() -> None:
    print("== 2. /lowstate 频率阈值 ==")
    expect(analyze(with_lowstate([])), "lowstate.rate", FAIL, "0 条 -> FAIL")
    expect(analyze(with_lowstate(good_samples(1))), "lowstate.rate", SKIP, "1 条 -> SKIP")
    expect(analyze(with_lowstate(good_samples(20, hz=500.0))), "lowstate.rate", PASS,
           "500 Hz -> PASS")
    expect(analyze(with_lowstate(good_samples(20, hz=STATE_HZ_OK))), "lowstate.rate", PASS,
           "400 Hz -> PASS(边界)")
    expect(analyze(with_lowstate(good_samples(20, hz=300.0))), "lowstate.rate", WARN,
           "300 Hz -> WARN")
    expect(analyze(with_lowstate(good_samples(20, hz=STATE_HZ_WARN))), "lowstate.rate", WARN,
           "200 Hz -> WARN(边界)")
    expect(analyze(with_lowstate(good_samples(20, hz=100.0))), "lowstate.rate", FAIL,
           "100 Hz -> FAIL")
    # 平均频率够（>200）但中间断流 150 ms -> WARN
    broken = good_samples(200)
    for i in range(100, 200):
        broken[i].t += 0.15
    expect(analyze(with_lowstate(broken)), "lowstate.rate", WARN, "有 150ms 断流 -> WARN")
    # 时间戳全相同 -> 不能崩，且不给 PASS
    same = good_samples(10)
    for s in same:
        s.t = 1.0
    rep = analyze(with_lowstate(same))
    check("时间戳全相同：不崩且非 PASS",
          status_of(rep, "lowstate.rate") in (WARN, FAIL), status_of(rep, "lowstate.rate"))


def t_tick() -> None:
    print("== 3. tick 单调 / Δtick ==")
    expect(analyze(with_lowstate(good_samples(20, tick_step=1))), "lowstate.tick", PASS,
           "真机 Δ=1（消息计数器语义）-> PASS")
    expect(analyze(with_lowstate(good_samples(20, tick_step=2), target="sim_ros")),
           "lowstate.tick", PASS, "仿真 Δ=2（配置口径）-> PASS")
    # 两种语义都自洽：500 Hz 下若 tick 单位是 1 ms（真机）/2 ms（仿真），
    # 每拍增量就是 2 / 1 —— 不能让健康的机器人误报 WARN。
    rep_alt_real = analyze(with_lowstate(good_samples(20, tick_step=2)))
    expect(rep_alt_real, "lowstate.tick", PASS,
           "真机 Δ=2（tick 单位 1ms×500Hz 语义）-> PASS")
    check("tick 自洽说明写清了语义",
          "tick 单位" in rep_alt_real.get("lowstate.tick").detail,
          rep_alt_real.get("lowstate.tick").detail)
    expect(analyze(with_lowstate(good_samples(20, tick_step=1), target="sim_ros")),
           "lowstate.tick", PASS, "仿真 Δ=1（tick 单位 2ms×500Hz 语义）-> PASS")
    expect(analyze(with_lowstate(good_samples(20, tick_step=3))), "lowstate.tick", WARN,
           "真机 Δ=3 -> WARN（丢拍）")
    expect(analyze(with_lowstate(good_samples(20, tick_step=4))), "lowstate.tick", WARN,
           "真机 Δ=4 -> WARN（丢拍）")
    # 100 Hz 状态下 tick 单位 1 ms -> 每拍应为 10；Δ=2 自洽不了 -> WARN
    expect(analyze(with_lowstate(good_samples(20, hz=100.0, tick_step=2), tick_step=1)),
           "lowstate.tick", WARN, "100 Hz + Δ=2 不自洽 -> WARN")
    rev = good_samples(20)
    for i, s in enumerate(rev):
        s.tick = 100 - i
    expect(analyze(with_lowstate(rev)), "lowstate.tick", FAIL, "tick 递减 -> FAIL")
    dup = good_samples(20, tick_step=0)
    expect(analyze(with_lowstate(dup)), "lowstate.tick", FAIL, "tick 不变 -> FAIL")


def t_motors() -> None:
    print("== 4. 电机 mode / lost / temperature ==")
    off = good_samples(10, mode=[0] * 12)
    expect(analyze(with_lowstate(off)), "motors.mode", FAIL, "12 个 mode=0 -> FAIL")
    part = good_samples(10)
    part[0].mode = [0x01] * 11 + [0x00]
    rep = analyze(with_lowstate(part))
    check("部分未使能 -> WARN（末拍仍 0x01 时容错）",
          status_of(rep, "motors.mode") in (PASS, WARN), status_of(rep, "motors.mode"))
    mixed = good_samples(10)
    for s in mixed:
        s.mode = [0] * 6 + [0x01] * 6
    expect(analyze(with_lowstate(mixed)), "motors.mode", WARN, "半数未使能 -> WARN")
    short = good_samples(10, mode=[0x01] * 8)
    expect(analyze(with_lowstate(short)), "motors.mode", FAIL, "槽位不足 12 -> FAIL")

    lost1 = good_samples(10)
    lost1[3].lost[2] = 1
    expect(analyze(with_lowstate(lost1)), "motors.lost", WARN, "少量 lost -> WARN")
    lost_many = good_samples(10)
    for s in lost_many:
        s.lost[0] = 3
    expect(analyze(with_lowstate(lost_many)), "motors.lost", FAIL, "持续 lost -> FAIL")

    expect(analyze(with_lowstate(good_samples(10, temperature=[69] * 12))),
           "motors.temperature", PASS, "69 ℃ -> PASS")
    expect(analyze(with_lowstate(good_samples(10, temperature=[70] * 12))),
           "motors.temperature", WARN, "70 ℃ -> WARN(边界)")
    expect(analyze(with_lowstate(good_samples(10, temperature=[85] * 12))),
           "motors.temperature", FAIL, "85 ℃ -> FAIL(边界)")
    expect(analyze(with_lowstate(good_samples(10, temperature=[0] * 12))),
           "motors.temperature", SKIP, "恒 0 -> SKIP")


def t_joint_checks() -> None:
    print("== 5. 关节角 / 速度 / 力矩 ==")
    expect(analyze(with_lowstate(good_samples(10))), "motors.q", PASS, "home 站姿 -> PASS")
    # 越界 0.05 rad（calf 上限 -0.83776）
    small = good_samples(10)
    small[0].q = list(HOME_Q)
    small[0].q[2] = JOINT_HI[2] + 0.05
    expect(analyze(with_lowstate(small)), "motors.q", WARN, "越界 0.05 rad -> WARN")
    big = good_samples(10)
    big[0].q = list(HOME_Q)
    big[0].q[2] = JOINT_HI[2] + 0.5
    expect(analyze(with_lowstate(big)), "motors.q", FAIL, "越界 0.5 rad -> FAIL")
    nan_q = good_samples(10)
    nan_q[0].q = list(HOME_Q)
    nan_q[0].q[5] = float("nan")
    expect(analyze(with_lowstate(nan_q)), "motors.q", FAIL, "q 有 NaN -> FAIL")

    fast = good_samples(10)
    fast[0].dq = [0.0] * 12
    fast[0].dq[0] = 50.0
    expect(analyze(with_lowstate(fast)), "motors.dq", WARN, "|dq|=50 -> WARN")
    too_fast = good_samples(10)
    too_fast[0].dq = [0.0] * 12
    too_fast[0].dq[0] = 150.0
    expect(analyze(with_lowstate(too_fast)), "motors.dq", FAIL, "|dq|=150 -> FAIL")
    inf_dq = good_samples(10)
    inf_dq[0].dq = [0.0] * 12
    inf_dq[0].dq[0] = float("inf")
    expect(analyze(with_lowstate(inf_dq)), "motors.dq", FAIL, "dq=inf -> FAIL")

    hip_over = good_samples(10)
    hip_over[0].tau_est = [0.0] * 12
    hip_over[0].tau_est[0] = float(TAU_LIMIT[0]) * 1.2
    expect(analyze(with_lowstate(hip_over)), "motors.tau", WARN, "hip 力矩超 20% -> WARN")
    hip_way = good_samples(10)
    hip_way[0].tau_est = [0.0] * 12
    hip_way[0].tau_est[0] = float(TAU_LIMIT[0]) * 2.5
    expect(analyze(with_lowstate(hip_way)), "motors.tau", FAIL, "hip 力矩超 2 倍 -> FAIL")
    calf_ok = good_samples(10)
    calf_ok[0].tau_est = [0.0] * 12
    calf_ok[0].tau_est[2] = 40.0        # calf 额定 45.43，40 合法
    expect(analyze(with_lowstate(calf_ok)), "motors.tau", PASS, "calf 40 N·m -> PASS")
    nan_tau = good_samples(10)
    nan_tau[0].tau_est = [0.0] * 12
    nan_tau[0].tau_est[0] = float("nan")
    expect(analyze(with_lowstate(nan_tau)), "motors.tau", FAIL, "tau=NaN -> FAIL")


def t_imu() -> None:
    print("== 6. IMU ==")
    expect(analyze(with_lowstate(good_samples(10))), "imu.accel", PASS, "accel=9.81 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, accel=[0.0, 0.0, ACCEL_G + 2.0]))),
           "imu.accel", WARN, "accel 偏 2 -> WARN")
    expect(analyze(with_lowstate(good_samples(10, accel=[0.0, 0.0, 3.0]))),
           "imu.accel", FAIL, "accel 偏 6.8 -> FAIL")
    expect(analyze(with_lowstate(good_samples(10, accel=[0.0, 0.0, 0.0]))),
           "imu.accel", FAIL, "accel 恒 0 -> FAIL")
    nan_acc = good_samples(10, accel=[0.0, 0.0, float("nan")])
    expect(analyze(with_lowstate(nan_acc)), "imu.accel", FAIL, "accel=NaN -> FAIL")

    expect(analyze(with_lowstate(good_samples(10))), "imu.quat", PASS, "四元数模长 1 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, quat=[0.9, 0.0, 0.0, 0.0]))),
           "imu.quat", WARN, "模长 0.9 -> WARN")
    expect(analyze(with_lowstate(good_samples(10, quat=[0.0, 0.0, 0.0, 0.0]))),
           "imu.quat", FAIL, "四元数全 0 -> FAIL")

    expect(analyze(with_lowstate(good_samples(10))), "imu.rpy", PASS, "rpy 有限 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, rpy=[100.0, 0.0, 0.0]))),
           "imu.rpy", WARN, "rpy=100（疑似角度单位）-> WARN")
    nan_rpy = good_samples(10, rpy=[0.0, float("nan"), 0.0])
    expect(analyze(with_lowstate(nan_rpy)), "imu.rpy", FAIL, "rpy=NaN -> FAIL")

    expect(analyze(with_lowstate(good_samples(10, gyro=[0.0, 0.0, 0.0]))),
           "imu.gyro", WARN, "gyro 恒 0 -> WARN")
    tiny = good_samples(10)
    for i, s in enumerate(tiny):
        s.gyro = [0.002 * math.sin(i), 0.001, -0.001]
    expect(analyze(with_lowstate(tiny)), "imu.gyro", PASS, "gyro 静止噪声 -> PASS")
    noisy = good_samples(10)
    for i, s in enumerate(noisy):
        s.gyro = [2.0 * math.sin(i), 0.0, 0.0]
    expect(analyze(with_lowstate(noisy)), "imu.gyro", WARN, "gyro 大幅波动 -> WARN")


def t_foot_bit_flag() -> None:
    print("== 7. 足端力 / bit_flag ==")
    expect(analyze(with_lowstate(good_samples(10))), "foot_force", PASS, "四路受力 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, foot_force=[0, 0, 0, 0]))),
           "foot_force", WARN, "四路全 0 -> WARN")
    expect(analyze(with_lowstate(good_samples(10, foot_force=[-1, 5, 5, 5]))),
           "foot_force", WARN, "出现负值 -> WARN")
    expect(analyze(with_lowstate(good_samples(10, foot_force=[10, 10]))),
           "foot_force", FAIL, "只有 2 路 -> FAIL")

    expect(analyze(with_lowstate(good_samples(10))), "bit_flag", PASS, "bit_flag=0 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, bit_flag=0x01))), "bit_flag", WARN,
           "bit0 指令超时 -> WARN（只读检测时预期）")
    expect(analyze(with_lowstate(good_samples(10, bit_flag=0x80))), "bit_flag", WARN,
           "bit7 电机超时 -> WARN")
    expect(analyze(with_lowstate(good_samples(10, bit_flag=0x20))), "bit_flag", WARN,
           "未定义位 -> WARN")


def t_lowcmd() -> None:
    print("== 8. /lowcmd ==")
    base = with_lowstate(good_samples(10))
    expect(analyze(base), "lowcmd.publisher", PASS, "没有进程在发 -> PASS")
    expect(analyze(base), "lowcmd.crc", SKIP, "没有 LowCmd -> SKIP")
    expect(analyze(base, check_lowcmd=False), "lowcmd.publisher", SKIP,
           "--no-cmd-check -> SKIP")
    expect(analyze(base, check_lowcmd=False), "lowcmd.crc", SKIP,
           "--no-cmd-check -> SKIP")

    bad_crc = with_lowstate(good_samples(10),
                            lowcmd=LowCmdSamples(n=10, crc_zero=0, crc_bad=3))
    expect(analyze(bad_crc), "lowcmd.crc", FAIL, "3 条 CRC 失败 -> FAIL")
    expect(analyze(bad_crc), "lowcmd.publisher", WARN, "有进程在发 -> WARN")
    zero_crc = with_lowstate(good_samples(10),
                             lowcmd=LowCmdSamples(n=10, crc_zero=10))
    expect(analyze(zero_crc), "lowcmd.crc", FAIL, "全部 crc=0 -> FAIL")
    part_zero = with_lowstate(good_samples(10),
                              lowcmd=LowCmdSamples(n=10, crc_zero=2))
    expect(analyze(part_zero), "lowcmd.crc", WARN, "部分 crc=0 -> WARN")
    ok_cmd = with_lowstate(good_samples(10),
                           lowcmd=LowCmdSamples(n=10, crc_zero=0, crc_bad=0,
                                                last_kp=[0.0] * 12))
    expect(analyze(ok_cmd), "lowcmd.crc", PASS, "CRC 全合法 -> PASS")
    many_pub = with_lowstate(good_samples(10),
                             lowcmd=LowCmdSamples(n=4, publishers=2))
    expect(analyze(many_pub), "lowcmd.publisher", WARN, "2 个发布者 -> WARN")
    idle_pub = with_lowstate(good_samples(10),
                             lowcmd=LowCmdSamples(n=0, publishers=1))
    rep_idle = analyze(idle_pub)
    expect(rep_idle, "lowcmd.publisher", WARN, "有发布者但没消息 -> WARN")
    check("空闲发布者详情说明没收到消息",
          "没收到消息" in rep_idle.get("lowcmd.publisher").detail,
          rep_idle.get("lowcmd.publisher").detail)


def t_lidar() -> None:
    print("== 9. L1 点云 ==")
    no_lidar = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                       topics={})
    expect(analyze(no_lidar), "lidar.rate", WARN, "没有点云 -> WARN")
    expect(analyze(no_lidar), "lidar.frame_id", SKIP, "没有点云 -> SKIP")
    expect(analyze(no_lidar), "lidar.npoints", SKIP, "没有点云 -> SKIP")

    one = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                  lidar=good_lidar(1), topics={})
    expect(analyze(one), "lidar.rate", SKIP, "只有 1 帧 -> SKIP")

    good = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                   lidar=good_lidar(11, 11.0), topics={})
    expect(analyze(good), "lidar.rate", PASS, "11 Hz -> PASS")
    expect(analyze(good), "lidar.npoints", PASS, "3000 点 -> PASS")
    expect(analyze(good), "lidar.xyz", PASS, "xyz 分位正常 -> PASS")

    slow = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                   lidar=good_lidar(4, 4.0), topics={})
    expect(analyze(slow), "lidar.rate", WARN, "4 Hz -> WARN")

    wrong_frame = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                          lidar=good_lidar(11, 11.0, frame="world"), topics={})
    expect(analyze(wrong_frame), "lidar.frame_id", WARN, "真机用 world 系 -> WARN")

    empty_pts = [LidarSample(t=i / 11.0, stamp=i / 11.0, frame_id=LIDAR_FRAME_REAL,
                             n_points=0, p50=[0.0, 0.0, 0.0],
                             p1=[0.0, 0.0, 0.0], p99=[0.0, 0.0, 0.0])
                 for i in range(11)]
    empty = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                    lidar=empty_pts, topics={})
    expect(analyze(empty), "lidar.npoints", FAIL, "全空帧 -> FAIL")
    expect(analyze(empty), "lidar.xyz", WARN, "分位全重合 -> WARN")

    nan_xyz = good_lidar(11)
    nan_xyz[0].p50 = [float("nan"), 0.0, 0.0]
    broken = Samples(target="real_ros", duration_s=1.0, lowstate=good_samples(10),
                     lidar=nan_xyz, topics={})
    expect(analyze(broken), "lidar.xyz", FAIL, "xyz 分位 NaN -> FAIL")


def t_topics_power() -> None:
    print("== 10. odom/pose/sportmodestate/lidar_state + 电池 ==")
    real = with_lowstate(good_samples(10))
    expect(analyze(real), "pose", SKIP, "真机没有 pose -> SKIP")
    expect(analyze(real), "odom", SKIP, "真机没有 odom -> SKIP")
    expect(analyze(real), "sportmodestate", SKIP, "真机没有 sportmodestate -> SKIP")
    expect(analyze(real), "lidar_state", SKIP, "真机没有 lidar_state -> SKIP")

    with_topics = with_lowstate(good_samples(10))
    with_topics.topics = {"pose": 100, "odom": 100, "sportmodestate": 100,
                          "lidar_state": 100}
    rep = analyze(with_topics)
    for name in ("pose", "odom", "sportmodestate", "lidar_state"):
        expect(rep, name, PASS, name + " 有数据")

    sim_no_odom = with_lowstate(good_samples(10, tick_step=2), target="sim_ros")
    expect(analyze(sim_no_odom), "odom", WARN, "仿真没有 odom -> WARN")

    expect(analyze(with_lowstate(good_samples(10))), "power", PASS, "28.8 V -> PASS")
    expect(analyze(with_lowstate(good_samples(10, power_v=33.6))), "power", PASS,
           "33.6 V 满电 -> PASS")
    expect(analyze(with_lowstate(good_samples(10, power_v=22.5))), "power", WARN,
           "22.5 V -> WARN")
    expect(analyze(with_lowstate(good_samples(10, power_v=20.0))), "power", FAIL,
           "20 V -> FAIL")
    expect(analyze(with_lowstate(good_samples(10, power_v=36.0))), "power", WARN,
           "36 V（充电中）-> WARN")
    expect(analyze(with_lowstate(good_samples(10, power_v=None))), "power", SKIP,
           "没有 power_v -> SKIP")


def t_robustness() -> None:
    print("== 11. 鲁棒性：空/NaN/字段缺失 ==")
    rep = analyze(Samples())
    check("空 Samples 不崩", isinstance(rep, Report))
    check("空 Samples：lowstate.rate=FAIL", status_of(rep, "lowstate.rate") == FAIL,
          status_of(rep, "lowstate.rate"))
    others_fail = [c.name for c in rep.checks
                   if c.status == FAIL and c.name != "lowstate.rate"]
    check("空 Samples：除 lowstate.rate 外没有 FAIL", not others_fail, str(others_fail))

    nan_samples = good_samples(5)
    nan_samples[0].mode = [float("nan")] * 12
    nan_samples[0].q = [float("nan")] * 12
    nan_samples[0].dq = [float("nan")] * 12
    nan_samples[0].tau_est = [float("nan")] * 12
    nan_samples[0].foot_force = [float("nan")] * 4
    nan_samples[0].gyro = [float("nan")] * 3
    nan_samples[0].quat = [float("nan")] * 4
    nan_samples[0].rpy = [float("nan")] * 3
    rep = analyze(with_lowstate(nan_samples))
    check("全 NaN 样本不崩", isinstance(rep, Report))
    for name in ("motors.q", "motors.dq", "motors.tau", "imu.quat", "imu.rpy",
                 "imu.gyro"):
        expect(rep, name, FAIL, "NaN -> FAIL")

    short = good_samples(5)
    short[0].q = [0.0] * 3
    short[0].mode = [1] * 3
    rep = analyze(with_lowstate(short))
    check("字段长度不足：motors.q FAIL", status_of(rep, "motors.q") == FAIL,
          status_of(rep, "motors.q"))
    check("字段长度不足：motors.mode FAIL", status_of(rep, "motors.mode") == FAIL,
          status_of(rep, "motors.mode"))

    weird = with_lowstate(good_samples(5))
    weird.topics = {"pose": None, "odom": "3"}
    rep = analyze(weird)
    check("topics 值异常不崩", isinstance(rep, Report))

    # dict 输入 / 未知字段
    raw = {"target": "real_ros", "duration_s": 0.02,
           "lowstate": [{"t": i / 500.0, "tick": i} for i in range(10)],
           "lidar": [{"t": 0.0, "frame_id": "utlidar_lidar", "n_points": 2000}],
           "topics": {"odom": 1}, "unknown_key": 123}
    rep = analyze(raw)
    check("dict 输入可用", isinstance(rep, Report) and rep.counts()[FAIL] == 0,
          str(rep.counts()))
    check("dict 输入：默认值可用 -> rate PASS",
          status_of(rep, "lowstate.rate") == PASS, status_of(rep, "lowstate.rate"))


def t_serialization() -> None:
    print("== 12. 序列化 ==")
    nan_sample = good_samples(5)
    nan_sample[0].q = [float("nan")] * 12
    rep = analyze(with_lowstate(nan_sample))
    d = rep.to_dict()
    try:
        txt = json.dumps(d, ensure_ascii=False, allow_nan=False)
        ok = True
    except (TypeError, ValueError) as exc:  # noqa: BLE001
        ok = False
        txt = str(exc)
    check("to_dict 可 json.dumps(allow_nan=False)", ok, txt if not ok else "")
    check("to_dict 含 summary/worst/meta/checks",
          all(k in d for k in ("summary", "worst", "meta", "checks")))
    check("summary 条数之和 == len(checks)",
          sum(d["summary"].values()) == len(d["checks"]),
          "%s vs %d" % (d["summary"], len(d["checks"])))
    try:
        dumps(rep)
        ok2 = True
    except (TypeError, ValueError) as exc:  # noqa: BLE001
        ok2 = False
        txt = str(exc)
    check("dumps() 可用", ok2, txt if not ok2 else "")
    # numpy 标量/数组也要能进 JSON
    rep2 = Report(checks=[Check(PASS, "x", "numpy", np.float64(1.5))],
                  meta={"arr": np.arange(3), "nan": float("nan")})
    try:
        s = json.dumps(rep2.to_dict(), allow_nan=False)
        ok3 = True
    except (TypeError, ValueError) as exc:  # noqa: BLE001
        ok3 = False
        s = str(exc)
    check("numpy/NaN 值可序列化", ok3, s if not ok3 else "")


def t_require_and_render() -> None:
    print("== 13. --require 语义 + 文本渲染 ==")
    rep = analyze(synthetic_samples(target="real_ros"))
    matched, unknown = rep.select_required(["state", "imu", "motors", "lidar"])
    check("required 展开：4 组 -> 16 项", len(matched) == 16, str(len(matched)))
    check("required 无未知 token", unknown == [], str(unknown))
    check("required verdict=PASS", rep.verdict(["state", "imu", "motors", "lidar"]) == PASS)
    _, unknown2 = rep.select_required(["state", "no_such_item"])
    check("未知 required token 被报出来", unknown2 == ["no_such_item"], str(unknown2))

    bad = with_lowstate(good_samples(10, temperature=[90] * 12))
    rep_bad = analyze(bad)
    check("必需项含 FAIL -> motors verdict=FAIL",
          rep_bad.verdict(["motors"]) == FAIL, rep_bad.verdict(["motors"]))
    sim_rep = analyze(good_sim_samples())
    check("必需项里只有 SKIP -> 降级 WARN",
          sim_rep.verdict(["motors"]) == WARN, sim_rep.verdict(["motors"]))
    check("精确名 required 可用",
          rep.verdict(["lowstate.tick"]) == PASS, rep.verdict(["lowstate.tick"]))

    txt = render_text(rep)
    check("render 标题存在", "Go2 传感器只读自检报告" in txt)
    check("render 有结论段", "结论:" in txt)
    check("render 有上机前还差什么", "上机前还差什么" in txt)
    check("render 有统计行", "统计: PASS" in txt)
    txt2 = render_text(analyze(Samples()))
    check("空报告也能 render", "结论:" in txt2)

    # Check 状态校验
    try:
        Check("BOGUS", "x")
        ok = False
    except ValueError:
        ok = True
    check("Check 非法 status 抛 ValueError", ok)


def t_pure_python() -> None:
    print("== 14. 纯 Python（不依赖 rclpy/mujoco） ==")
    import re

    src = open(os.path.join(ROOT, "signal_io", "real_diag.py"), encoding="utf-8").read()
    # 只查真正的 import 语句（docstring 里出现"不 import rclpy"这种说明不算）
    check("real_diag.py 没有 import rclpy 语句",
          re.search(r"(?m)^s*(?:import|from)s+rclpy", src) is None)
    check("real_diag.py 没有 import mujoco 语句",
          re.search(r"(?m)^s*(?:import|from)s+mujoco", src) is None)
    check("rclpy 没有被本测试导入", "rclpy" not in sys.modules)
    check("mujoco 没有被本测试导入", "mujoco" not in sys.modules)


def t_collector_shell() -> None:
    """采集壳的"纯数据"部分：不 import rclpy，用假消息验证搬运与退出码。"""
    print("== 15. 采集壳 scripts/real_sensor_check.py（不 import rclpy）==")
    import importlib.util
    import tempfile
    from types import SimpleNamespace

    path = os.path.join(ROOT, "scripts", "real_sensor_check.py")
    spec = importlib.util.spec_from_file_location("real_sensor_check_ut", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # --- 只读保证：源码里不能出现发布 API
    try:
        mod._assert_read_only_source()
        ok = True
        err = ""
    except AssertionError as exc:  # noqa: BLE001
        ok = False
        err = str(exc)
    check("采集壳源码自检通过（无发布 API）", ok, err)
    src = open(path, encoding="utf-8").read()
    joined = "".join(mod._FORBIDDEN_PUB_TOKENS)
    check("采集壳源码不含发布 API 字面量", joined not in src)
    # 采集壳允许在函数内惰性 import rclpy（--selftest 必须能在没有 ROS 2 的机器上跑），
    # 只禁止模块顶层 import。
    check("采集壳没有顶层 import rclpy",
          __import__("re").search(r"(?m)^(?:import|from)\s+rclpy\b", src) is None)

    # --- 参数与退出码
    a = mod.parse_args(["--selftest"])
    check("--selftest 解析", a.selftest is True)
    a2 = mod.parse_args([])
    check("默认 target=real_ros", a2.target == "real_ros", a2.target)
    mod._resolve_topics(a2)
    check("真机默认 state 话题", a2.state_topic == "/lowstate", a2.state_topic)
    check("真机默认 cloud 话题", a2.cloud_topic == "/utlidar/cloud", a2.cloud_topic)
    a3 = mod.parse_args(["--target", "sim_ros"])
    mod._resolve_topics(a3)
    check("仿真默认 cloud 话题", a3.cloud_topic == "/go2/lidar/points", a3.cloud_topic)
    check("--require 解析去空去重",
          mod._parse_require("state, imu,,motors;lidar,state")
          == ["state", "imu", "motors", "lidar"],
          str(mod._parse_require("state, imu,,motors;lidar,state")))
    check("退出码 PASS -> 0", mod._exit_code(PASS, False) == 0)
    check("退出码 FAIL -> 1", mod._exit_code(FAIL, False) == 1)
    check("退出码 WARN -> 2", mod._exit_code(WARN, False) == 2)
    check("退出码 WARN+strict -> 1", mod._exit_code(WARN, True) == 1)

    # --- 假消息：LowState / LowCmd / 点云
    def fake_lowstate(tick: int, mode: int = 0x01, temp: int = 35) -> SimpleNamespace:
        imu = SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0],
                              gyroscope=[0.001, -0.001, 0.0],
                              accelerometer=[0.0, 0.0, ACCEL_G],
                              rpy=[0.0, 0.0, 0.0])
        motors = [SimpleNamespace(mode=mode, q=float(HOME_Q[i % 12]), dq=0.0, ddq=0.0,
                                  tau_est=0.0, temperature=temp, lost=0)
                  for i in range(20)]        # unitree_go 有 20 个槽位，Go2 用前 12
        return SimpleNamespace(imu_state=imu, motor_state=motors,
                               foot_force=[50, 48, 52, 49], tick=tick,
                               bit_flag=0, power_v=28.8, power_a=3.0)

    def fake_cmd(crc: int) -> SimpleNamespace:
        motors = [SimpleNamespace(mode=1, q=0.0, dq=0.0, tau=0.0, kp=20.0, kd=1.0,
                                  reserve=[0, 0, 0]) for _ in range(20)]
        return SimpleNamespace(head=[0xFE, 0xEF], level_flag=0xFF, frame_reserve=0,
                               gpio=0, motor_cmd=motors, crc=crc)

    col = mod._Collector(target="real_ros", duration_s=1.0, tick_step=1, lidar_hz=11.0)
    for i in range(5):
        col.on_state(fake_lowstate(tick=i))
    col.on_pose(SimpleNamespace())
    col.on_odom(SimpleNamespace())
    col.on_sport(SimpleNamespace())
    col.on_lidar_state(SimpleNamespace())
    cloud = SimpleNamespace(width=1000, height=1,
                            header=SimpleNamespace(frame_id="utlidar_lidar",
                                                   stamp=SimpleNamespace(sec=1, nanosec=0)))
    col.on_cloud(cloud)   # 没有 sensor_msgs_py -> 走降级分支（记录 note，不崩）
    check("on_state 搬运 5 拍", len(col.lowstate) == 5, str(len(col.lowstate)))
    check("on_state 只取前 12 个电机",
          len(col.lowstate[0].q) == 12, str(len(col.lowstate[0].q)))
    check("on_cloud 记录点数/frame_id",
          col.lidar[0].n_points == 1000 and col.lidar[0].frame_id == "utlidar_lidar")
    check("点云分位缺失时降级成 None", col.lidar[0].p50 is None)
    check("topics 计数正确",
          col.topics == {"pose": 1, "odom": 1, "sportmodestate": 1, "lidar_state": 1},
          str(col.topics))

    # --- LowCmd：合法 CRC / crc=0 / 坏 CRC
    from signal_io.go2_crc import lowcmd_crc
    from signal_io.msg import LowCmd as PyLowCmd, MotorCmd

    def py_cmd() -> PyLowCmd:
        c = PyLowCmd(head=[0xFE, 0xEF], level_flag=0xFF)
        for i in range(20):
            c.motor_cmd[i] = MotorCmd(mode=1, q=0.0, dq=0.0, tau=0.0, kp=20.0, kd=1.0)
        return c

    good_crc = lowcmd_crc(py_cmd())
    col.on_cmd(fake_cmd(good_crc))
    col.on_cmd(fake_cmd(0))
    col.on_cmd(fake_cmd((good_crc ^ 0xFFFF) & 0xFFFFFFFF))
    check("LowCmd 条数=3", col.lowcmd.n == 3, str(col.lowcmd.n))
    check("crc=0 计数=1", col.lowcmd.crc_zero == 1, str(col.lowcmd.crc_zero))
    check("坏 CRC 计数=1", col.lowcmd.crc_bad == 1, str(col.lowcmd.crc_bad))
    check("末条 kp 记录", col.lowcmd.last_kp[:2] == [20.0, 20.0], str(col.lowcmd.last_kp[:2]))

    # --- 汇总 + 判定 + 必需项表
    col.t_start, col.t_end = 0.0, 1.0
    col.lowcmd.publishers = 2
    samples = col.to_samples()
    check("to_samples 时长", abs(samples.duration_s - 1.0) < 1e-9, str(samples.duration_s))
    rep = analyze(samples, check_lowcmd=True)
    check("采集结果可判定（不崩）", isinstance(rep, Report))
    check("有别的进程发 LowCmd -> publisher WARN",
          rep.get("lowcmd.publisher").status == WARN,
          rep.get("lowcmd.publisher").status)
    check("坏 CRC -> lowcmd.crc FAIL",
          rep.get("lowcmd.crc").status == FAIL, rep.get("lowcmd.crc").status)
    table, per = mod._required_table(rep, ["state", "imu", "motors", "lidar"])
    check("必需项表覆盖 4 组", set(per) == {"state", "imu", "motors", "lidar"}, str(per))
    check("必需项表文本含分组名", "state" in table and "lidar" in table)

    # --- JSON 落盘
    tmpdir = tempfile.mkdtemp(prefix="real_sensor_check_")
    out = os.path.join(tmpdir, "sub", "report.json")
    try:
        mod._write_json(out, {"mode": "unit", "report": rep.to_dict()})
        with open(out, encoding="utf-8") as fh:
            payload = json.load(fh)
        ok2 = payload["mode"] == "unit" and "checks" in payload["report"]
        err2 = ""
    except (OSError, ValueError) as exc:  # noqa: BLE001
        ok2 = False
        err2 = str(exc)
    check("_write_json 自动建目录并写合法 JSON", ok2, err2)

    # --- 网络预检（best-effort，不 sudo）：只要求返回结构正确、不抛
    pre = mod.precheck_network(iface_hint="eth0")
    check("precheck_network 返回结构正确",
          isinstance(pre, dict) and "warnings" in pre and "interfaces" in pre)
    check("precheck_network 不抛且 ping 结果是 bool/None",
          pre.get("ping_ok") in (True, False, None))


def main() -> int:
    t_good_samples()
    t_rate()
    t_tick()
    t_motors()
    t_joint_checks()
    t_imu()
    t_foot_bit_flag()
    t_lowcmd()
    t_lidar()
    t_topics_power()
    t_robustness()
    t_serialization()
    t_require_and_render()
    t_pure_python()
    t_collector_shell()
    print()
    if FAILS:
        print("===== test_real_diag: %d 项失败 =====" % len(FAILS))
        for f in FAILS:
            print("  - " + f)
        return 1
    print("===== test_real_diag: 全部通过 =====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
