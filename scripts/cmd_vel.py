#!/usr/bin/env python3
"""运行期速度指令发布器（真机首测用：不重启控制器就能改速度）。

===============================================================================
1. 为什么需要它
===============================================================================

  ros2_controller/run_controller.py 的 --cmd 只在启动时生效。真机上重启控制器
  = 机器人先卸力落地、再预摆位起立，首次 sim2real 时这一步最容易出事。
  所以控制器支持订阅一个 cmd_vel 话题（默认 /go2/cmd_vel，geometry_msgs/Twist），
  用本脚本在另一个终端里改速度：

      # 终端 A：策略站住（0 速）
      scripts/run_with_ros2.sh ros2_controller/run_controller.py \
          --target real_ros --controller policy --cmd 0 0 0 --duration 120

      # 终端 B：落地、确认站住后，再给前进速度
      scripts/run_with_ros2.sh scripts/cmd_vel.py 0.2 0 0 --duration 20
      # 想停：Ctrl-C 后控制器会在 --cmd-timeout(默认 3 s) 后自动回零速（原地踏步）

===============================================================================
2. 用法
===============================================================================

  python3 scripts/cmd_vel.py 0.2 0 0                 # 以 10 Hz 持续发 20 s
  python3 scripts/cmd_vel.py 0.2 0 0 --duration 30   # 指定时长
  python3 scripts/cmd_vel.py 0 0 0 --once            # 只发一条（瞬时置零）
  python3 scripts/cmd_vel.py --keys                  # 交互：w/s 前后、a/d 转向、空格归零、q 退出

  在真机容器里：
      UNITREE_NIC=<网卡> ROS_DOMAIN_ID=0 docker compose run --rm ros2 \
          bash -lc "python3 scripts/cmd_vel.py 0.2 0 0 --duration 20"

===============================================================================
3. 安全
===============================================================================

  * 本脚本只发速度指令，不碰安全层；真正的限幅/看门狗/急停在控制器里
    （ros2_controller/safety.py + health.py）。
  * 但这里也做一次硬夹紧（防手滑打错数字）：
        vx ∈ [-0.2, 0.8] m/s   vy ∈ [-0.3, 0.3] m/s   wz ∈ [-0.8, 0.8] rad/s
    —— 训练里的命令分布就是这个范围，超出它策略本来也没见过。
  * --keys 模式按 Ctrl-C 退出；控制器侧 --cmd-timeout 默认 3 s，话题一静默就回零速。

相关文件：
  ros2_controller/run_controller.py      --cmd-topic / --cmd-timeout / 订阅接线
  ros2_controller/controller.py          LiveCommandController（转发 + 失联回零）
  ros2_controller/obs.py                 WalkObsBuilder.set_command（obs 9:12）
  signal_io/config.py                    --target 决定 domain / 网卡
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from signal_io.config import apply_io_env, load_config, io_summary  # noqa: E402

# 训练命令分布的上界（超出去策略没见过，没意义还有风险）
CLIP_LO = np.array([-0.2, -0.3, -0.8])
CLIP_HI = np.array([0.8, 0.3, 0.8])


def clip_cmd(v) -> np.ndarray:
    """把 (vx,vy,wz) 夹进训练命令范围；非有限值直接报错（不静默改成 0）。"""
    a = np.asarray(v, dtype=np.float64).reshape(-1)
    if a.size != 3:
        raise ValueError("速度指令必须是 3 个数 (vx vy wz)，得到 %d 个" % a.size)
    if not bool(np.all(np.isfinite(a))):
        raise ValueError("速度指令含 NaN/Inf: %r" % (a.tolist(),))
    return np.clip(a, CLIP_LO, CLIP_HI)


def _print_cmd(c: np.ndarray) -> None:
    print("  -> vx=%.2f m/s  vy=%.2f m/s  wz=%.2f rad/s" % (c[0], c[1], c[2]))


def main() -> int:
    ap = argparse.ArgumentParser(description="运行期速度指令发布器（geometry_msgs/Twist）")
    ap.add_argument("cmd", nargs="*", type=float, default=[0.0, 0.0, 0.0],
                    help="vx vy wz（缺省 0 0 0）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"))
    ap.add_argument("--target", default="real_ros", choices=["real_ros", "real_sdk", "sim_ros"],
                    help="IO profile（决定 domain / 网卡；默认真机）")
    ap.add_argument("--topic", default="/go2/cmd_vel", help="话题名（别写 rt/ 前缀）")
    ap.add_argument("--rate", type=float, default=10.0, help="发布频率 Hz（默认 10）")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="持续时长 秒（0 = 直到 Ctrl-C）")
    ap.add_argument("--once", action="store_true", help="只发一条就退出")
    ap.add_argument("--keys", action="store_true",
                    help="交互模式：w/s 前后 ±0.05、a/d 转向 ±0.2、空格归零、q 退出")
    args = ap.parse_args()

    cmd = clip_cmd(args.cmd or [0.0, 0.0, 0.0])
    if tuple(np.asarray(args.cmd or [0, 0, 0], dtype=np.float64)) != tuple(cmd):
        print("[cmd_vel] ⚠ 指令被夹到训练范围:", np.round(cmd, 3).tolist())

    cfg = load_config(args.config, target=args.target)
    env = apply_io_env(cfg)        # 必须在 rclpy.init 之前
    print("[cmd_vel] io =", io_summary(cfg))
    if env:
        print("[cmd_vel] env =", env)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("go2_cmd_vel")
    pub = node.create_publisher(Twist, args.topic, 10)

    def publish(c: np.ndarray) -> None:
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = float(c[0]), float(c[1]), 0.0
        m.angular.x, m.angular.y, m.angular.z = 0.0, 0.0, float(c[2])
        pub.publish(m)

    print("[cmd_vel] topic=%s rate=%.1f Hz" % (args.topic, args.rate))

    if args.once:
        publish(cmd)
        time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()
        print("[cmd_vel] 已发 1 条")
        return 0

    period = 1.0 / max(args.rate, 0.5)
    t_end = None if args.duration <= 0 else time.monotonic() + args.duration
    try:
        if args.keys:
            print("[cmd_vel] 交互模式：w/s 前后、a/d 转向、空格归零、q 退出（Ctrl-C 也可以）")
            _print_cmd(cmd)
            last = time.monotonic()
            while True:
                try:
                    line = sys.stdin.readline()
                except KeyboardInterrupt:
                    break
                if not line:
                    break
                k = line.strip()[:1].lower()
                if k == "q":
                    break
                if k == "w":
                    cmd[0] += 0.05
                elif k == "s":
                    cmd[0] -= 0.05
                elif k == "a":
                    cmd[2] += 0.2
                elif k == "d":
                    cmd[2] -= 0.2
                elif k in (" ", ""):
                    cmd[:] = 0.0
                try:
                    cmd = clip_cmd(cmd)
                except ValueError as exc:
                    print("[cmd_vel] 忽略非法指令:", exc)
                _print_cmd(cmd)
                publish(cmd)
                last = time.monotonic()
            # 退出前归零
            publish(np.zeros(3))
            print("[cmd_vel] 已发零速，退出")
            return 0

        n = 0
        while t_end is None or time.monotonic() < t_end:
            publish(cmd)
            n += 1
            time.sleep(period)
        print("[cmd_vel] 结束：共发 %d 条 vx=%.2f vy=%.2f wz=%.2f"
              % (n, cmd[0], cmd[1], cmd[2]))
        return 0
    except KeyboardInterrupt:
        publish(np.zeros(3))
        print("\n[cmd_vel] Ctrl-C：已补发一条零速")
        return 0
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
