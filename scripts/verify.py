#!/usr/bin/env python3
"""验收脚本：把这条链路该证明的东西一次全跑出来。

    python scripts/verify.py            # 全量（含双进程 UDP）
    python scripts/verify.py --quick    # 只跑单进程 loopback

验收项:
  1. 消息层 selftest（CRC 与逐位参考实现一致、524/684 字节往返）
  2. 单进程 loopback: PD 站立 6 s, 机器人不倒
  3. 双进程 UDP: sim 与 controller 分别是独立进程, 500 Hz 通信
  4. 实时性: RTF / 周期抖动 / 看门狗
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from signal_io import msg as msgmod  # noqa: E402
from signal_io.config import load_config, model_xml  # noqa: E402
from ros2_controller.controller import StandController  # noqa: E402
from signal_io.node import ControllerNode, SimNode  # noqa: E402
from sim_env.robot import Go2Sim  # noqa: E402
from signal_io.transport.local import LoopbackTransport  # noqa: E402

PASS, FAIL = "\033[32mPASS\033[0m", "\033[31mFAIL\033[0m"
results = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print("  [%s] %s %s" % (PASS if ok else FAIL, name, detail))
    return ok


def test_msg() -> None:
    print("\n== 1. 消息层 ==")
    msgmod._selftest()
    check("LowCmd/LowState 打包与 CRC", True, "(524 B / 684 B, 与逐位实现逐位一致)")


def test_imu_at_rest(cfg) -> None:
    print("\n== 2. IMU 合理性（站定时加速度应该 ≈ +9.81 m/s^2 朝上）==")
    from sim_env.robot import make_stand_cmd

    sim = Go2Sim(model_xml(cfg), sim_dt=cfg["sim_dt"])
    cmd = make_stand_cmd(sim.stand_pose, kp=60.0, kd=5.0)
    for _ in range(300):          # 先让它站稳：第一拍接触求解器会有瞬态，不能拿第 0 拍下结论
        sim.write_cmd(cmd)
        sim.step(1)
    st = sim.read_state()
    acc = np.array(st.imu_state.accelerometer)
    rpy = np.array(st.imu_state.rpy)
    check("加速度模长 ≈ g", abs(np.linalg.norm(acc) - 9.81) < 0.5,
          "|a| = %.3f, a = [%.3f, %.3f, %.3f]" % (np.linalg.norm(acc), acc[0], acc[1], acc[2]))
    check("陀螺仪 ≈ 0", float(np.linalg.norm(np.array(st.imu_state.gyroscope))) < 0.1,
          "|w| = %.4f rad/s" % float(np.linalg.norm(np.array(st.imu_state.gyroscope))))
    check("初始姿态水平", np.all(np.abs(rpy) < 0.05), "rpy = %s" % np.round(rpy, 4))
    check("足端接触力可读", len(st.foot_force) == 4, "foot_force = %s" % st.foot_force)


def test_loopback(cfg, duration: float = 6.0) -> dict:
    print("\n== 3. 单进程 loopback：PD 站立 %.0f s ==" % duration)
    tr = LoopbackTransport()
    sim_node = SimNode(model_xml(cfg), tr, sim_dt=cfg["sim_dt"], watchdog_ms=cfg["watchdog_ms"])
    ctrl = StandController(sim_node.sim.stand_pose, kp=cfg["controller"]["kp"],
                           kd=cfg["controller"]["kd"], ramp_s=cfg["controller"]["ramp_s"])
    ctrl_node = ControllerNode(tr, ctrl, sim_dt=cfg["sim_dt"], control_dt=cfg["control_dt"])

    # 单进程时两边交替跑：一步物理 <-> 一步控制，避免队列堆积
    n = int(duration / sim_node.dt)
    log_h = []
    t0 = time.perf_counter()
    for k in range(n):
        st = sim_node.tick()
        ctrl_node.tr.recv_state()
        ctrl_node._state = st
        if k % ctrl_node.decim == 0:
            ctrl_node._cmd = ctrl.update(st, sim_node.sim.time)
            ctrl_node._k += 1
        ctrl_node.tr.publish_cmd(ctrl_node._cmd)
        if k % 25 == 0:
            log_h.append((sim_node.sim.time, sim_node.sim.base_height, *sim_node.sim.orientation_rpy()))
    wall = time.perf_counter() - t0
    arr = np.array(log_h)
    tail = arr[arr[:, 0] > duration * 0.6]
    h = tail[:, 1]
    tilt = np.degrees(np.abs(tail[:, 2:4])).max()
    check("机器人没倒（末段倾斜 < 25°）", tilt < 25.0, "max tilt = %.1f°" % tilt)
    check("躯干高度合理（0.20~0.40 m）", 0.20 < float(h.mean()) < 0.40,
          "mean z = %.3f m" % float(h.mean()))
    check("不限速吞吐（物理+消息）≥ 500 Hz", n / wall >= 500.0,
          "%.0f Hz（= RTF %.2f, %d steps in %.2f s）" % (n / wall, n * sim_node.dt / wall, n, wall))
    check("没有触发看门狗", sim_node.n_cmd_timeout == 0, "n_cmd_timeout = %d" % sim_node.n_cmd_timeout)
    return {"tilt_deg": float(tilt), "height": float(h.mean()), "rtf": n * sim_node.dt / wall}


def test_two_process(cfg, duration: float = 6.0, target: str = "sim_udp") -> dict:
    print("\n== 4. 双进程（IO profile = %s）：sim 与 controller 各自独立进程 ==" % target)
    logs = os.path.join(ROOT, "logs")
    os.makedirs(logs, exist_ok=True)
    csv_path = os.path.join(logs, "verify_udp.csv")
    sim_json = os.path.join(logs, "verify_sim.json")
    ctrl_json = os.path.join(logs, "verify_ctrl.json")
    py = sys.executable
    common = ["--config", os.path.join(ROOT, "config", "go2_sim.json"), "--duration", str(duration),
              "--target", target]
    p_sim = subprocess.Popen([py, os.path.join(HERE, "..", "sim_env", "run_sim.py")] + common +
                             ["--log", csv_path, "--json", sim_json],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.8)                     # 等 sim 把 socket bind 好
    p_ctrl = subprocess.Popen([py, os.path.join(HERE, "..", "ros2_controller", "run_controller.py")] + common +
                              ["--json", ctrl_json],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out_sim, _ = p_sim.communicate(timeout=duration + 60)
    out_ctrl, _ = p_ctrl.communicate(timeout=duration + 60)
    ok_proc = (p_sim.returncode == 0 and p_ctrl.returncode == 0)
    check("两个进程都正常退出", ok_proc, "rc = %s / %s" % (p_sim.returncode, p_ctrl.returncode))
    if not ok_proc:
        print(out_sim[-1500:]); print(out_ctrl[-1500:])
        return {}

    with open(sim_json, encoding="utf-8") as f:
        sj = json.load(f)
    with open(ctrl_json, encoding="utf-8") as f:
        cj = json.load(f)
    t, z, roll, pitch = [], [], [], []
    with open(csv_path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); z.append(float(row["z"]))
            roll.append(float(row["roll"])); pitch.append(float(row["pitch"]))
    t = np.array(t); z = np.array(z); tilt = np.degrees(np.maximum(np.abs(roll), np.abs(pitch)))
    sel = t > duration * 0.6
    check("sim 收到指令", sj["n_cmd"] > 100, "n_cmd = %d" % sj["n_cmd"])
    check("controller 收到状态", cj["n_state"] > 100, "n_state = %d" % cj["n_state"])
    check("没有触发看门狗", sj["n_cmd_timeout"] == 0, "trips = %d" % sj["n_cmd_timeout"])
    # 限速跑时 RTF 只能在 1.0 附近摆（sleep 粒度 ~1 ms，不可能持续 > 1），
    # 所以判据给 2% 容差；真正的余量看不限速吞吐那一项。
    check("RTF ≈ 1（限速跟得上墙钟）", sj["pacer"]["rtf"] >= 0.98,
          "RTF = %.3f, p95 lag = %.2f ms" % (sj["pacer"]["rtf"], sj["pacer"]["lag_p95_ms"]))
    check("机器人站住（末段倾斜 < 25°）", float(tilt[sel].max()) < 25.0,
          "max tilt = %.1f°, mean z = %.3f m" % (float(tilt[sel].max()), float(z[sel].mean())))
    tr_stats = sj["transport"]
    check("通信无坏包", tr_stats.get("bad_crc", 0) == 0,
          "io=%s sent=%s recv=%s bad_crc=%s" % (
              tr_stats.get("transport", "?"), tr_stats.get("sent", "-"),
              tr_stats.get("recv", "-"), tr_stats.get("bad_crc", 0)))
    return {"rtf": sj["pacer"]["rtf"], "p95_ms": sj["pacer"]["lag_p95_ms"],
            "tilt": float(tilt[sel].max()), "z": float(z[sel].mean())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--target", default="sim_udp", choices=["sim_udp", "sim_ros", "sim_sdk"],
                    help="双进程测试用哪个 IO profile（sim_ros 需要 ROS 2 + unitree_go）")
    args = ap.parse_args()
    cfg = load_config(os.path.join(ROOT, "config", "go2_sim.json"), target=args.target)
    print("mujoco %s | 模型 %s" % (mujoco.__version__, model_xml(cfg)))
    test_msg()
    test_imu_at_rest(cfg)
    lb = test_loopback(cfg, args.duration)
    two = {}
    if not args.quick:
        two = test_two_process(cfg, args.duration, args.target)
    print("\n===== 汇总 =====")
    n_ok = sum(1 for _n, ok, _d in results if ok)
    for n, ok, d in results:
        print("  %s  %s %s" % ("OK  " if ok else "FAIL", n, d))
    print("  %d/%d 通过" % (n_ok, len(results)))
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
