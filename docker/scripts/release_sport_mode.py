#!/usr/bin/env python3
"""官方式“释放 / 恢复运控服务（sport mode）”——用 unitree_sdk2_python 的 MotionSwitcher。

为什么需要它（对齐官方要求）：
  Quick_start 明确要求：跑底层控制（rt/lowcmd）前必须先关闭主运控服务（MCF），
  否则多个运控同时下发会导致机器人失控。官方 go2_stand_example 在程序里用
  MotionSwitcherClient 做这件事；本仓真机流程原先靠遥控器 L2+R2 手动进入
  debug mode，这个脚本把程序化释放补齐（容器里 python 3.10 + SDK 可用）。

官方服务名（Quick_start）：
  mcf            Go2 主运动控制服务（软件版本 > 1.1.6）
  sport_mode     Go2 主运动控制服务（软件版本 < 1.1.6）
  wheeled_sport  Go2-W 主运动控制服务

安全默认：不加 --yes 只做只读检查（CheckMode），不会动机器人。
用法：
  # 只读查看当前运控服务
  python3 release_sport_mode.py --iface eth1
  # 释放（机器人必须已吊起 / 处于安全状态）
  python3 release_sport_mode.py --iface eth1 --yes
  # 退出时恢复原来的运控服务
  python3 release_sport_mode.py --iface eth1 --yes --restore mcf
"""

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0", help="连接机器人的有线网卡名")
    ap.add_argument("--domain", type=int, default=0, help="真机固件固定 domain 0")
    ap.add_argument("--yes", action="store_true", help="真的释放运控（默认只读检查）")
    ap.add_argument("--restore", default=None,
                    help="释放成功后 SelectMode 恢复的服务名（如 mcf / sport_mode）")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    ChannelFactoryInitialize(args.domain, args.iface)
    msc = MotionSwitcherClient()
    msc.SetTimeout(args.timeout)
    msc.Init()

    status, result = msc.CheckMode()
    name = (result or {}).get("name", "")
    print("[sport] CheckMode: status=%s name=%r form=%r" % (status, name, (result or {}).get("form")))
    if not name:
        print("[sport] 运控服务已经是关闭状态：可以直接发 rt/lowcmd")
        return 0

    if not args.yes:
        print("[sport] 只读模式：未做任何修改。确认机器人已吊起/安全后加 --yes 释放")
        return 0

    print("[sport] WARN: 即将 ReleaseMode（关闭运控 %r）。机器人会进入阻尼/失力状态！" % name)
    for i in range(10):
        status, _ = msc.ReleaseMode()
        time.sleep(0.3)
        status, result = msc.CheckMode()
        cur = (result or {}).get("name", "")
        if not cur:
            print("[sport] ReleaseMode 成功：运控已关闭（第 %d 次尝试）" % (i + 1))
            break
        print("[sport] 仍在 %r，重试 %d/10" % (cur, i + 1))
    else:
        print("[sport] FAIL: 10 次尝试仍未释放；检查 App/遥控器是否占用运控")
        return 1

    if args.restore:
        print("[sport] SelectMode(%r) 恢复运控 ..." % args.restore)
        status, _ = msc.SelectMode(args.restore)
        time.sleep(0.5)
        status, result = msc.CheckMode()
        print("[sport] 恢复后 CheckMode: name=%r" % ((result or {}).get("name", "")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
