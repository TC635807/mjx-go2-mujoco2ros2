#!/usr/bin/env python3
"""scripts/probe_mode_switch.py —— 只读探针：抓"切模式/切避障瞬间"到底发生了什么。

为什么需要它（2026-11-05 实机）：按压 L2+R2、关闭避障模式时，狗会四肢抽搐。
两种可能完全不同的病因，必须分开：
  A. 服务/模式问题：切模式时步态规划器交接，目标位置跳变 -> **四条腿同时**异常
  B. 硬件问题：某条腿的电机/编码器/减速器故障 -> **只有那一条腿**异常
判据就是"哪些关节的 dq/tau 同时跳"。

用法（全程只订阅，绝不发布）：

    bash scripts/run_with_ros2.sh scripts/probe_mode_switch.py \
        --target real_ros --iface enp0s31f6 --duration 25 --json logs/probe.json

跑起来之后（看到 "开始采集" 提示）再去按遥控器 / 关避障。
结束后会打印：事件表（mode/gait/error_code/bit_flag 每次变化）+ 抽搐窗口（哪几个关节、峰值）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

MOTOR_NAMES = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
               "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]
LEG = ["FR", "FR", "FR", "FL", "FL", "FL", "RR", "RR", "RR", "RL", "RL", "RL"]


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="切模式瞬间的只读探针（不发任何 LowCmd）")
    ap.add_argument("--target", default="real_ros", choices=["real_ros"])
    ap.add_argument("--iface", default=None, help="接狗的有线网卡（如 enp0s31f6）")
    ap.add_argument("--duration", type=float, default=25.0, help="采集时长（秒）")
    ap.add_argument("--dq-thresh", type=float, default=3.0, help="|dq| 超过多少 rad/s 算抽动")
    ap.add_argument("--json", default=None, help="把原始记录写到这个 JSON")
    return ap.parse_args(argv)


def _apply_iface(iface: Optional[str]) -> None:
    if not iface:
        return
    os.environ["CYCLONEDDS_URI"] = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="%s" priority="default" multicast="default" />'
        '</Interfaces></General></Domain></CycloneDDS>' % iface)


def _assert_read_only(node) -> None:
    """只读保证：本节点不允许有任何（非隐式）发布者。"""
    implicit = ("/rosout", "/parameter_events")
    pubs = []
    for p in getattr(node, "publishers", []):
        topic = str(getattr(p, "topic_name", "") or "")
        if topic and topic not in implicit and not topic.startswith("/rosout"):
            pubs.append(topic)
    assert not pubs, "探针必须只读，但发现了发布者：%s" % pubs


class Recorder:
    def __init__(self) -> None:
        self.t0 = None
        self.rows: List[Dict] = []
        self.sport: List[Dict] = []
        self.cmd_pubs: List[Dict] = []

    def on_state(self, msg, t: float) -> None:
        if self.t0 is None:
            self.t0 = t
        dq = [float(m.dq) for m in msg.motor_state[:12]]
        tau = [float(m.tau_est) for m in msg.motor_state[:12]]
        q = [float(m.q) for m in msg.motor_state[:12]]
        self.rows.append({
            "t": round(t - self.t0, 4),
            "tick": int(msg.tick),
            "bit_flag": int(msg.bit_flag),
            "power_v": float(msg.power_v),
            "dq": dq,
            "tau": tau,
            "q": q,
        })

    def on_sport(self, msg, t: float) -> None:
        if self.t0 is None:
            self.t0 = t
        self.sport.append({
            "t": round(t - self.t0, 4),
            "mode": int(msg.mode),
            "gait": int(msg.gait_type),
            "err": int(msg.error_code),
            # 量化到 1 mm：body_height 是 float32，原始值每拍都在最后一位跳，
            # 不量化会把事件表刷满噪声
            "body_h": round(float(msg.body_height), 3),
            "yaw": float(msg.yaw_speed),
        })


def _changes(seq: List[Dict], keys: List[str]) -> List[Dict]:
    """序列里 keys 任一项发生变化 -> 记一条事件。"""
    out: List[Dict] = []
    prev = None
    for row in seq:
        cur = {k: row.get(k) for k in keys}
        if prev is not None and cur != prev:
            changed = {k: [prev.get(k), cur.get(k)] for k in keys if prev.get(k) != cur.get(k)}
            out.append({"t": row["t"], "changed": changed})
        prev = cur
    return out


def _twitch_windows(rows: List[Dict], thresh: float) -> List[Dict]:
    """找 |dq|>thresh 的连续窗口，统计涉及哪些关节、峰值多少。"""
    if not rows:
        return []
    dq = np.asarray([r["dq"] for r in rows], dtype=np.float64)
    ts = np.asarray([r["t"] for r in rows], dtype=np.float64)
    hot = np.abs(dq) > thresh
    any_hot = hot.any(axis=1)
    windows: List[Dict] = []
    i = 0
    n = len(rows)
    while i < n:
        if not any_hot[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and any_hot[j + 1]:
            j += 1
        seg = slice(i, j + 1)
        motors = [MOTOR_NAMES[k] for k in range(12) if hot[seg, k].any()]
        legs = sorted({LEG[k] for k in range(12) if hot[seg, k].any()})
        peak = float(np.max(np.abs(dq[seg])))
        windows.append({
            "t_start": round(float(ts[i]), 3),
            "t_end": round(float(ts[j]), 3),
            "duration_ms": int(round((float(ts[j]) - float(ts[i])) * 1000)),
            "legs": legs,
            "motors": motors,
            "peak_dq": round(peak, 2),
            "peak_tau": round(float(np.max(np.abs(np.asarray([r["tau"] for r in rows[i:j + 1]])))), 2),
        })
        i = j + 1
    return windows


def _summarize(rec: Recorder, dur: float, thresh: float) -> Dict:
    n = len(rec.rows)
    hz = n / dur if dur > 0 else 0.0
    dq = np.asarray([r["dq"] for r in rec.rows], dtype=np.float64) if n else np.zeros((0, 12))
    rep: Dict = {
        "duration_s": round(dur, 2),
        "lowstate_n": n,
        "lowstate_hz": round(hz, 1),
        "sport_n": len(rec.sport),
        "max_abs_dq": round(float(np.max(np.abs(dq))), 3) if n else 0.0,
        "per_motor_max_dq": ({MOTOR_NAMES[k]: round(float(np.max(np.abs(dq[:, k]))), 3)
                              for k in range(12)} if n else {}),
        "body_h_range": ([round(min(s["body_h"] for s in rec.sport), 3),
                          round(max(s["body_h"] for s in rec.sport), 3)]
                         if rec.sport else None),
        "sport_changes": _changes(rec.sport, ["mode", "gait", "err", "body_h"]),
        "bit_flag_changes": _changes(rec.rows, ["bit_flag"]),
        "cmd_pub_changes": _changes(rec.cmd_pubs, ["n"]),
        "twitch_windows": _twitch_windows(rec.rows, thresh),
    }
    return rep


def render(rep: Dict, thresh: float) -> str:
    out: List[str] = []
    out.append("=" * 72)
    out.append(" 切模式/切避障瞬间 只读探针报告")
    out.append("=" * 72)
    out.append("采集 %.2f s  /lowstate %d 条 (%.1f Hz)  /sportmodestate %d 条"
               % (rep["duration_s"], rep["lowstate_n"], rep["lowstate_hz"], rep["sport_n"]))
    out.append("全关节峰值 |dq| = %.2f rad/s（超过 %.1f rad/s 记为抽动）"
               % (rep["max_abs_dq"], thresh))
    out.append("")
    if rep.get("body_h_range"):
        out.append("body_height 范围 = %s m（站立约 0.32，趴下约 0.07）" % rep["body_h_range"])
    out.append("-- 模式/状态事件 --")
    if not rep["sport_changes"]:
        out.append("   /sportmodestate 的 mode/gait/error_code 在观测期【没有变化】")
    for ev in rep["sport_changes"]:
        part = "  ".join("%s %s->%s" % (k, v[0], v[1]) for k, v in ev["changed"].items())
        out.append("   t=%6.2fs  %s" % (ev["t"], part))
    if rep["bit_flag_changes"]:
        for ev in rep["bit_flag_changes"]:
            out.append("   t=%6.2fs  bit_flag %s->%s"
                       % (ev["t"], ev["changed"]["bit_flag"][0], ev["changed"]["bit_flag"][1]))
    if rep["cmd_pub_changes"]:
        for ev in rep["cmd_pub_changes"]:
            out.append("   t=%6.2fs  /lowcmd 发布者 %s->%s"
                       % (ev["t"], ev["changed"]["n"][0], ev["changed"]["n"][1]))
    out.append("")
    out.append("-- 抽动窗口 --")
    if not rep["twitch_windows"]:
        out.append("   没有 |dq| 超过 %.1f rad/s 的片段（这次没抽）" % thresh)
    for w in rep["twitch_windows"]:
        out.append("   t=%6.2f~%6.2fs (%d ms)  腿=%s  峰值|dq|=%.2f rad/s  峰值|tau|=%.1f N·m"
                   % (w["t_start"], w["t_end"], w["duration_ms"], ",".join(w["legs"]),
                      w["peak_dq"], w["peak_tau"]))
        out.append("        关节: %s" % ", ".join(w["motors"]))
    out.append("")
    out.append("-- 判定 --")
    wins = rep["twitch_windows"]
    if not wins:
        out.append("   本次观测没抓到抽动。要复现，请在「开始采集」之后再切模式。")
    else:
        all4 = any(len(w["legs"]) >= 3 for w in wins)
        one = all(len(w["legs"]) == 1 for w in wins)
        if all4:
            out.append("   多条腿同时抽 -> 更像【服务/模式交接】问题（规划器切换、error_code），")
            out.append("   不像单条腿硬件故障。查 /sportmodestate.error_code 与 mode 的变化时刻。")
        elif one:
            out.append("   只有一条腿抽 -> 更像【该腿硬件】问题（电机/编码器/减速器/线束），")
            out.append("   记下是哪条腿，检查它的线束与关节旷量。")
        else:
            out.append("   混合：部分腿抽。先按「服务/模式」方向查，再看是否总涉及同一条腿。")
    out.append("")
    out.append("注意：本探针全程只订阅，任何时刻都没有发出 LowCmd。")
    return "\n".join(out)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    _apply_iface(args.iface)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowCmd, LowState, SportModeState

    rclpy.init(args=None)
    node = Node("go2_probe_mode_switch")
    _assert_read_only(node)
    rec = Recorder()
    node.create_subscription(LowState, "/lowstate",
                             lambda m: rec.on_state(m, node.get_clock().now().nanoseconds * 1e-9),
                             10)
    node.create_subscription(SportModeState, "/sportmodestate",
                             lambda m: rec.on_sport(m, node.get_clock().now().nanoseconds * 1e-9),
                             qos_profile_sensor_data)
    # /lowcmd 只看发布者数量（不订阅内容，避免多一条 500 Hz 流抢 CPU）
    node.create_subscription(LowCmd, "/lowcmd", lambda m: None, 10)
    _assert_read_only(node)

    print("[probe] 只读探针就绪（不发布任何 LowCmd）")
    print("[probe] 现在开始按遥控器 / 关闭避障，持续时间 %.1f s ..." % args.duration, flush=True)
    import time as _t
    t0 = _t.monotonic()
    next_poll = t0
    while _t.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = _t.monotonic()
        if now >= next_poll:
            next_poll = now + 0.25
            try:
                n = len(node.get_publishers_info_by_topic("/lowcmd") or [])
            except Exception:  # noqa: BLE001
                n = -1
            rec.cmd_pubs.append({"t": round(now - t0, 2), "n": int(n)})
    dur = _t.monotonic() - t0
    rep = _summarize(rec, dur, args.dq_thresh)
    print()
    print(render(rep, args.dq_thresh))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".", exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"summary": rep, "lowstate": rec.rows, "sport": rec.sport},
                      fh, ensure_ascii=False, allow_nan=False)
        print("[json] 已写入 %s" % args.json)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
