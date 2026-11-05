#!/usr/bin/env python3
"""scripts/go2_service.py —— 用 DDS 直接开关 Go2 机载服务（= 软件版"进/出调试模式"）。

背景（2026-11-05 实机排查）：Go2 不需要 SSH，它把机载服务都暴露在 DDS 上：
  /api/robot_state/request  ->  API id 1001=ServiceSwitch、1003=ServiceList
官方 unitree_sdk2py 自带 RobotStateClient，等价于遥控器 L2+R2：

    ServiceSwitch("sport_mode", False)   # 停掉运动服务 = 进入"调试模式"
    ServiceSwitch("sport_mode", True)    # 恢复运动服务

⚠️ 安全前提（血泪教训，别跳过）：
  停 sport_mode 的那一刻，机器人内部那个 500 Hz 的"阻尼指令源"就没了。
  **如果此时没有任何外部 LowCmd 源在发，狗会四肢疯狂抽搐**（2026-11-05 实测）。
  所以正确顺序永远是：
     ① 狗【吊起、四足离地】
     ② 先把控制器跑起来（它开始以 500 Hz 发零力矩）
     ③ 再 --stop sport_mode
     ④ 一有异常立刻 --start sport_mode

用法：
    # 只读：列出所有服务及状态（status: 1=运行, 0=停止；protect=1 表示不允许开关）
    python3 scripts/go2_service.py --list

    # 危险动作（必须显式 --yes）
    python3 scripts/go2_service.py --stop sport_mode --yes
    python3 scripts/go2_service.py --start sport_mode --yes
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Go2 机载服务查看/开关（软件版调试模式）")
    ap.add_argument("--iface", default="enp0s31f6", help="接狗的有线网卡")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain（真机固定 0）")
    ap.add_argument("--list", action="store_true", help="列出服务（只读）")
    ap.add_argument("--check-mode", action="store_true",
                    help="只读：查当前运控模式（mcf/sport_mode/空=已释放）")
    ap.add_argument("--release-mode", action="store_true",
                    help="释放当前运控模式（官方 go2_stand_example 的做法）。释放后机器人只受外部 LowCmd 控制")
    ap.add_argument("--select-mode", default=None, metavar="NAME",
                    help="切回某个运控模式，通常 --select-mode mcf")
    ap.add_argument("--stop", default=None, metavar="NAME", help="停掉某个服务，如 sport_mode")
    ap.add_argument("--start", default=None, metavar="NAME", help="启动某个服务，如 sport_mode")
    ap.add_argument("--yes", action="store_true",
                    help="确认执行开关动作（没有它只会打印将要做什么，不真的执行）")
    return ap.parse_args(argv)


def _client(iface: str, domain: int):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    ChannelFactoryInitialize(domain, iface)
    c = RobotStateClient()
    c.SetTimeout(5.0)
    c.Init()
    return c


def _motion_client(iface: str, domain: int):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    ChannelFactoryInitialize(domain, iface)
    c = MotionSwitcherClient()
    c.SetTimeout(6.0)
    c.Init()
    return c


def _check_mode(c) -> int:
    code, data = c.CheckMode()
    if code != 0:
        print("[ERROR] CheckMode 失败，code=%s" % code)
        return 1
    name = (data or {}).get("name", "")
    form = (data or {}).get("form", "")
    print("当前运控模式： name=%r form=%r" % (name, form))
    if not name:
        print("  -> 没有运控服务在跑（已释放，低层控制可以独占）")
    elif str(name) in ("mcf", "sport_mode"):
        print("  -> %s 正在跑。它会和外部 LowCmd 抢指令，必须先释放。" % name)
    return 0


def _release_mode(c, yes: bool) -> int:
    code, data = c.CheckMode()
    cur = (data or {}).get("name", "") if code == 0 else "?"
    print("当前运控模式 = %r，准备【释放】" % cur)
    print()
    print("!! ReleaseMode 会让运控不再保持姿态：站着的狗会直接瘫下去。")
    print("   1) 狗必须【吊起、四足离地】或已趴在地上")
    print("   2) 外部控制器已经以 500 Hz 在发 LowCmd（否则是无人管状态）")
    print("   3) 遥控器在手，可随时 --select-mode mcf 恢复")
    if not yes:
        print()
        print("[dry-run] 未加 --yes，不执行。")
        return 2
    code, _ = c.ReleaseMode()
    if code != 0:
        print("[ERROR] ReleaseMode 失败，code=%s" % code)
        return 1
    time.sleep(0.5)
    code, data = c.CheckMode()
    name = (data or {}).get("name", "") if code == 0 else "?"
    print("[OK] 释放后 CheckMode -> name=%r（空 = 已释放）" % name)
    if name:
        print("     [!] 还有 %r 在跑：说明它不是主运控，或又被拉起了" % name)
    return 0


def _select_mode(c, name: str, yes: bool) -> int:
    print("准备切回运控模式 %r（恢复官方运动控制）" % name)
    if not yes:
        print("[dry-run] 未加 --yes，不执行。")
        return 2
    code, _ = c.SelectMode(name)
    if code != 0:
        print("[ERROR] SelectMode 失败，code=%s" % code)
        return 1
    time.sleep(0.5)
    code, data = c.CheckMode()
    print("[OK] CheckMode -> %r" % ((data or {}).get("name", "?"),))
    return 0


def _print_list(c) -> int:
    code, lst = c.ServiceList()
    if code != 0 or lst is None:
        print("[ERROR] ServiceList 失败，code=%s（狗没上电/网线/网卡？）" % code)
        return 1
    run = [s for s in lst if int(s.status) == 1]
    prot = [s for s in lst if int(s.protect) == 1]
    print("服务数 %d，其中运行中 %d、受保护 %d" % (len(lst), len(run), len(prot)))
    print()
    print("%-28s %8s %8s" % ("service", "status", "protect"))
    print("-" * 48)
    for s in sorted(lst, key=lambda x: str(x.name)):
        flag = "RUN " if int(s.status) == 1 else "stop"
        print("%-28s %8s %8s  %s" % (str(s.name), s.status, s.protect, flag))
    print()
    print("受保护（不能开关）：%s" % ", ".join(str(s.name) for s in prot))
    print("运行中：%s" % ", ".join(str(s.name) for s in run))
    return 0


def _switch(c, name: str, on: bool, yes: bool) -> int:
    action = "启动" if on else "停掉"
    code, lst = c.ServiceList()
    if code != 0 or lst is None:
        print("[ERROR] ServiceList 失败，code=%s" % code)
        return 1
    hit = [s for s in lst if str(s.name) == name]
    if not hit:
        print("[ERROR] 没有名为 %r 的服务。可用名字：" % name)
        for s in sorted(lst, key=lambda x: str(x.name)):
            print("   ", s.name)
        return 1
    svc = hit[0]
    if int(svc.protect) == 1:
        print("[ERROR] 服务 %s 受保护（protect=1），不允许开关。" % name)
        return 1
    cur = int(svc.status) == 1
    print("服务 %-20s 现状: %s  目标: %s" % (name, "运行中" if cur else "已停止", action))
    if cur == bool(on):
        print("已经是目标状态，不需要动作。")
        return 0
    if not on:
        print()
        print("!! 停掉运动服务 = 机器人失去内部指令源。必须：")
        print("   ① 狗已【吊起、四足离地】")
        print("   ② 外部控制器已经在以 500 Hz 发 LowCmd")
        print("   ③ 遥控器在手（可随时断电/接管）")
    if not yes:
        print()
        print("[dry-run] 未加 --yes，不执行。确认无误后重跑并加 --yes。")
        return 2
    code = c.ServiceSwitch(name, bool(on))
    if code != 0:
        print("[ERROR] ServiceSwitch 失败，code=%s（5201=切换失败, 5202=服务受保护）" % code)
        return 1
    code, lst2 = c.ServiceList()
    if code == 0 and lst2:
        for s in lst2:
            if str(s.name) == name:
                print("[OK] %s 现在 status=%s（1=运行, 0=停止）" % (name, s.status))
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    want_motion = bool(args.check_mode or args.release_mode or args.select_mode)
    want_service = bool(args.list or args.stop or args.start)
    if not want_motion:
        want_service = True      # 什么都没指定时默认打服务表
    rc = 0
    try:
        if want_service:
            c = _client(args.iface, args.domain)
        else:
            c = None
        m = _motion_client(args.iface, args.domain) if want_motion else None
    except Exception as exc:  # noqa: BLE001
        print("[ERROR] 连不上 Go2 的 DDS（iface=%s domain=%d）：%s" % (args.iface, args.domain, exc))
        return 1
    if args.check_mode:
        rc = max(rc, _check_mode(m))
    if args.release_mode:
        rc = max(rc, _release_mode(m, args.yes))
    if args.select_mode:
        rc = max(rc, _select_mode(m, args.select_mode, args.yes))
    if args.list or (not want_motion and not args.stop and not args.start):
        rc = max(rc, _print_list(c))
    if args.stop:
        rc = max(rc, _switch(c, args.stop, False, args.yes))
    if args.start:
        rc = max(rc, _switch(c, args.start, True, args.yes))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
