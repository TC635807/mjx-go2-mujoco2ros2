#!/usr/bin/env python3
"""指令掉线守护进程：控制器死了/卡了，"另一个进程"替它发阻尼。

===============================================================================
1. 它是什么、为什么必须有
===============================================================================

  run_controller.py 里的三层保护（health / supervisor / 退出阻尼）全都活在**同一个
  进程**里。进程被 kill -9、段错误、OOM、DDS 线程卡死、PC 合盖休眠时，这些保护一个
  都不会执行，而机器人固件对"lowcmd 断流"的行为官方没有公开（可能是保持最后指令）。

  本脚本是**独立进程**，只干一件事：
      订阅 /lowcmd；超过 --timeout-ms（默认 250 ms）没收到任何 LowCmd
      -> 自己按 --publish-hz（默认 500 Hz）发 kp=0 / kd=8 的阻尼指令；
      一看到 /lowcmd 恢复 -> **立刻**停发；单次最多补发 --max-duration-s（默认 5 s）。

  决策逻辑全部在 signal_io/failsafe.py（纯内核，可离线单测），本文件只做 ROS 2 的
  收发、打印和节流。

===============================================================================
2. 安全默认：默认**不真的发**（先验证，再上机）
===============================================================================

  不加 --enable-publish 时，一切照常判定，但只打印 "WOULD PUBLISH"，一个字节都不发。
  推荐顺序（不要跳步）：

      1) 本机/离线验内核（不需要 ROS 2）：
             .venv/bin/python scripts/failsafe_guard.py --selftest
      2) sim_ros 闭环里验"控制器停了会被兜住"（domain 1，打不到真机）：
             scripts/run_with_ros2.sh scripts/failsafe_guard.py --target sim_ros --duration 10
         （先只 WOULD PUBLISH，确认它只在控制器停的时候才触发）
      3) sim_ros 上真发（仍是 domain 1，安全）：
             scripts/run_with_ros2.sh scripts/failsafe_guard.py --target sim_ros --enable-publish
      4) 真机、机器人吊起、遥控器 L2+R2 进 debug mode：
             scripts/run_with_ros2.sh scripts/failsafe_guard.py --target real_ros --duration 30
         先不加 --enable-publish，确认 30 s 内一次都没触发（说明控制器指令流是连续的），
         再考虑下一次实验时加 --enable-publish。

===============================================================================
3. 和控制器抢 /lowcmd：怎么确认现在有几个发布者
===============================================================================

  本脚本自己也是 /lowcmd 的发布者，所以必须能一眼看出"现在到底有几个在发"：

      ros2 topic info /lowcmd                          # 看 Publisher count
      ros2 topic info -v /lowcmd | grep -A3 "Publisher count"
      ros2 node list | grep -i -E "go2|unitree"        # 谁在发
      # 只读探针（不发任何指令，能同时看到 /lowcmd 的实际频率与 CRC）：
      scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

  Publisher count >= 2 且你在跑控制实验 -> 立刻停掉其中一个。两个 500 Hz 发布者
  抢同一条 lowcmd，机器人看到的是两条指令的交错，行为不可预测。

===============================================================================
4. 已知限制（写清楚，别当成银弹）
===============================================================================

  * **未在真机验证过**（本仓整体也还没上过真机）。先在 sim_ros 跑通再加
    --enable-publish。
  * 它救不了：guard 自己崩、PC 断电、网线全断（DDS 发不出去）。这些只能靠机器人
    固件的超时行为，未知。
  * 它不判姿态/电池/温度，只看 /lowcmd 是否还在动（那些在控制器进程内）。
  * 控制器恢复后 DDS 里可能残留 1~2 帧阻尼（kp=0，不会把机器人顶飞，但会有一拍
    软一下）。

相关文件：
  signal_io/failsafe.py           决策内核（纯函数，单测见 tests/test_failsafe.py）
  ros2_controller/run_controller.py  正常控制器（真机退出时会发 100 ms 阻尼）
  ros2_controller/health.py       /lowstate 断流分级（进程内）
  signal_io/msg.py                LowCmd / make_damping_cmd
  docs/safety-audit.md            安全审计（失效矩阵 + 上机 5 条）
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from signal_io.config import apply_io_env, io_summary, load_config  # noqa: E402
from signal_io.failsafe import (STATE_ARMED, STATE_EXHAUSTED, STATE_FIRING,  # noqa: E402
                                STATE_WAITING, FailsafeKernel, make_damping_cmd)
from signal_io.transport import make_transport  # noqa: E402


# ---------------------------------------------------------------- 离线自检
def _selftest() -> int:
    """纯内核自检：不需要 ROS 2，也不碰任何话题（--selftest 用）。"""
    DT = 0.002
    fails = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            fails.append(name)

    # 1) 从没见过 /lowcmd -> 永不发布（不能单独启动就把机器人按住）
    k = FailsafeKernel()
    pub = sum(k.update(i * DT).publish for i in range(1000))
    check("未见 /lowcmd 时不发布", pub == 0 and k.stats()["state"] == STATE_WAITING,
          "pub=%d state=%s" % (pub, k.stats()["state"]))

    # 2) 正常 500 Hz 指令流 -> 不发布
    k = FailsafeKernel()
    pub = sum(k.update(i * DT, saw_cmd=True).publish for i in range(1000))
    check("指令连续 2 s 不触发", pub == 0 and k.stats()["n_trips"] == 0, "pub=%d" % pub)

    # 3) 断流 250 ms 后触发，并按 500 Hz 补发
    k = FailsafeKernel()
    k.update(0.0, saw_cmd=True)
    first_at, n_fire = None, 0
    for i in range(1, 500):
        d = k.update(i * DT)
        if d.publish:
            n_fire += 1
            if first_at is None:
                first_at = i * DT
    check("断流 >250ms 触发补发", first_at is not None and first_at >= 0.25,
          "first=%.3fs n=%d" % (first_at or -1, n_fire))
    check("补发频率 ≈ 500 Hz", 300 <= n_fire <= 450, "n_fire=%d/1s" % n_fire)

    # 4) 一看到 /lowcmd 立刻停发
    k = FailsafeKernel()
    k.update(0.0, saw_cmd=True)
    for i in range(1, 200):
        k.update(i * DT)
    d = k.update(0.4, saw_cmd=True)
    check("/lowcmd 恢复立即停发", not d.publish and d.state == STATE_ARMED
          and k.stats()["n_recoveries"] == 1, "%s %s" % (d.state, d.reason))

    # 5) 单次补发有上限
    k = FailsafeKernel(max_duration_s=0.5)
    k.update(0.0, saw_cmd=True)
    states = []
    for i in range(1, 1000):
        d = k.update(i * DT)
        states.append(d.state)
    check("超过 max_duration_s 停止补发", states[-1] == STATE_EXHAUSTED
          and k.stats()["n_exhausted"] == 1, states[-1])

    # 6) 耗尽后不重复触发，直到 /lowcmd 回来重新 arm
    d = k.update(3.0)
    ok1 = (not d.publish) and d.state == STATE_EXHAUSTED
    d = k.update(3.002, saw_cmd=True)
    ok2 = d.state == STATE_ARMED
    check("耗尽后需 /lowcmd 重新 arm", ok1 and ok2, "%s -> %s" % (d.state, ok2))

    # 7) 阻尼指令字段
    cmd = make_damping_cmd(kd=8.0)
    ok = all((cmd.motor_cmd[i].kp == 0.0 and cmd.motor_cmd[i].kd == 8.0
              and cmd.motor_cmd[i].q == 0.0 and cmd.motor_cmd[i].tau == 0.0
              and cmd.motor_cmd[i].mode == 0x01) for i in range(12))
    check("阻尼指令 = kp0/kd8/q0/tau0/mode1", ok, "ok")

    print("\n===== failsafe selftest: %d 项通过, %d 项失败 ====="
          % (7 - len(fails), len(fails)))
    if fails:
        print("  FAIL:", fails)
        return 1
    print("OK failsafe_guard.py --selftest（未连任何话题）")
    return 0


def _parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="指令掉线守护：/lowcmd 停了就替控制器发 kp=0/kd=8 阻尼（默认只打印不发）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default="real_ros", help="IO profile（real_ros / sim_ros / ...）")
    ap.add_argument("--timeout-ms", type=float, default=250.0,
                    help="多久没收到 /lowcmd 就判定控制器掉了（默认 250 ms）")
    ap.add_argument("--publish-hz", type=float, default=500.0,
                    help="补发频率（默认 500 Hz，与固件低层一致）")
    ap.add_argument("--max-duration-s", type=float, default=5.0,
                    help="单次最多补发多久（默认 5 s；<=0 不限制，不建议）")
    ap.add_argument("--kd", type=float, default=8.0, help="阻尼指令的 kd（默认 8）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="守护进程运行时长（秒）；0 = 一直跑到 Ctrl-C")
    ap.add_argument("--enable-publish", action="store_true",
                    help="【默认关闭】真的向 /lowcmd 发阻尼指令；不加只打印 WOULD PUBLISH")
    ap.add_argument("--assume-always-armed", action="store_true",
                    help="从启动第一秒就开始计时（控制器从没起来也会补发；默认不这样）")
    ap.add_argument("--selftest", action="store_true", help="只跑离线内核自检（不需要 ROS 2）")
    ap.add_argument("--json", default=None, help="把 summary 落盘到该路径")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    if args.selftest:
        return _selftest()

    # ------------------------------------------------------------ 1) 配置 / 环境
    cfg = load_config(args.config, target=args.target)
    env = apply_io_env(cfg)          # 必须在 make_transport（rclpy.init）之前
    cmd_topic = str(cfg["transport"].get("cmd_topic", "/lowcmd"))
    mode = "ENABLE-PUBLISH（会真的发 /lowcmd）" if args.enable_publish else "DRY-RUN（只打印）"

    print("[failsafe] target   =", io_summary(cfg))
    if env:
        print("[failsafe] env      =", env)
    print("[failsafe] 话题     = %s（订阅 + 发布同一个话题）" % cmd_topic)
    print("[failsafe] 模式     =", mode)
    print("[failsafe] 判定     = 超过 %.0f ms 没收到 LowCmd -> 按 %.0f Hz 发 kp=0/kd=%.1f 阻尼，"
          "最多 %.1f s，看到 LowCmd 立刻停" % (args.timeout_ms, args.publish_hz, args.kd,
                                              args.max_duration_s))
    print("[failsafe] 先确认只有一个发布者：ros2 topic info %s（Publisher count）" % cmd_topic)

    # ------------------------------------------------------------ 2) 传输（缺 ROS 2 要优雅退出）
    try:
        tr = make_transport(cfg["transport"], role="controller")
    except (RuntimeError, ImportError, OSError, ValueError) as exc:
        print("[failsafe] ✗ 这个 target 需要 ROS 2 / DDS 后端，本机不可用：", file=sys.stderr)
        for line in str(exc).splitlines():
            print("[failsafe]   " + line, file=sys.stderr)
        print("[failsafe]   现在能做的：", file=sys.stderr)
        print("[failsafe]     1) --selftest 只验决策内核（不需要 ROS 2）；", file=sys.stderr)
        print("[failsafe]     2) 装好 ROS 2 + unitree_go 后 source 再跑（见 docs/ros2-setup.md）；",
              file=sys.stderr)
        print("[failsafe]     3) 用 --target sim_ros/sim_udp 先在仿真里验证（domain 不同，打不到真机）。",
              file=sys.stderr)
        return 2

    try:
        import rclpy
        from unitree_go.msg import LowCmd as RosLowCmd
    except Exception as exc:                       # noqa: BLE001 - 缺包要给出可操作提示
        print("[failsafe] ✗ 缺 rclpy / unitree_go.msg：%r" % (exc,), file=sys.stderr)
        print("[failsafe]   用 scripts/run_with_ros2.sh 包一层再跑（它会 source ROS 2 环境）。",
              file=sys.stderr)
        try:
            tr.close()
        except Exception:
            pass
        return 2

    # ------------------------------------------------------------ 3) 订阅 /lowcmd
    holder = {"n": 0}

    def _on_cmd(_msg) -> None:      # noqa: ANN001 - 只要"有没有来"，内容交给控制器校验
        holder["n"] += 1

    tr.create_subscription(RosLowCmd, cmd_topic, _on_cmd, 10)

    # ------------------------------------------------------------ 4) 主循环
    kernel = FailsafeKernel(timeout_ms=args.timeout_ms, publish_hz=args.publish_hz,
                            max_duration_s=args.max_duration_s,
                            require_ever_seen=not args.assume_always_armed)
    damp = make_damping_cmd(kd=args.kd)
    dt = 1.0 / max(1.0, args.publish_hz)
    n_steps = None if args.duration <= 0 else int(round(args.duration * args.publish_hz))
    t0 = time.monotonic()
    next_t = 0.0
    n_sent = 0
    n_would = 0
    n_err = 0
    n_spin_err = 0
    last_state = None
    last_wo = -1.0
    warned_exhausted = False
    k = 0

    def drain() -> None:
        """把队列抽干（一个 spin_once 只处理一条，别让 /lowstate 把 /lowcmd 挤掉）。"""
        nonlocal n_spin_err
        for _ in range(32):
            before = (holder["n"], int(tr.stats().get("rx_state", 0)))
            try:
                rclpy.spin_once(tr.node, timeout_sec=0.0)
            except Exception as exc:               # noqa: BLE001 - 单帧异常不能让守护进程死
                n_spin_err += 1
                if n_spin_err <= 3:
                    print("[failsafe] ⚠ spin_once 异常（继续跑）:", repr(exc)[:160])
                return
            if (holder["n"], int(tr.stats().get("rx_state", 0))) == before:
                return

    try:
        while n_steps is None or k < n_steps:
            now = time.monotonic() - t0
            n_before = holder["n"]
            drain()
            saw = holder["n"] > n_before
            dec = kernel.update(now, saw_cmd=saw)

            if dec.state != last_state:
                print("[failsafe] 状态 %s -> %s（%s）" % (last_state or "-", dec.state, dec.reason))
                last_state = dec.state

            if dec.publish:
                if args.enable_publish:
                    try:
                        tr.publish_cmd(damp)
                        n_sent += 1
                    except Exception as exc:        # noqa: BLE001
                        n_err += 1
                        if n_err <= 3 or n_err % 500 == 0:
                            print("[failsafe] ⚠ publish_cmd 异常（已计数 %d）:" % n_err,
                                  repr(exc)[:160])
                else:
                    n_would += 1
                    if now - last_wo >= 0.5:
                        last_wo = now
                        print("[failsafe] WOULD PUBLISH（dry-run，未发）gap=%.0fms fired=%.2fs"
                              % (dec.gap_ms, dec.fired_s))

            # 耗尽只喊一次（否则 EXHAUSTED 期间每拍都刷屏）
            if dec.state == STATE_EXHAUSTED and not warned_exhausted:
                warned_exhausted = True
                print("[failsafe] ⚠ 单次补发到上限（%.1f s）已停止：控制器仍未恢复，"
                      "请人工检查（这是故障，不是正常状态）" % args.max_duration_s)

            next_t += dt
            lag = (time.monotonic() - t0) - next_t
            if lag < 0:
                time.sleep(-lag)
            k += 1
    except KeyboardInterrupt:
        print("[failsafe] Ctrl-C：停止守护")
    finally:
        st = kernel.stats()
        st.update({"target": cfg["target"], "cmd_topic": cmd_topic,
                   "enable_publish": bool(args.enable_publish),
                   "n_published": n_sent, "n_would_publish": n_would,
                   "n_publish_err": n_err, "n_spin_err": n_spin_err,
                   "wall_s": round(time.monotonic() - t0, 3)})
        print("[failsafe] summary =", json.dumps(st, ensure_ascii=False))
        if args.json:
            os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump(st, f, ensure_ascii=False, indent=2)
        try:
            tr.close()
        except Exception:
            pass
        if st["state"] == STATE_FIRING:
            print("[failsafe] ⚠ 退出时仍在 FIRING：控制器没恢复，机器人最后收到的是阻尼指令")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
