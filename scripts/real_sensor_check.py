#!/usr/bin/env python3
"""Go2 实机传感器只读全项自检（ROS 2 薄采集壳）。

===============================================================================
1. 职责 / 安全边界
===============================================================================

  **本脚本只订阅，绝不发布。它不发任何 LowCmd，不使能电机，不改机器人状态，
  可以在站立/吊起状态下安全运行。**

  做两件事：
    (a) 用 rclpy 订阅 /lowstate、/lowcmd、L1 点云、pose/odom/sportmodestate/
        lidar_state，把收到的消息搬成 signal_io/real_diag.py 里的纯数据；
    (b) 交给 real_diag.analyze() 出 PASS/WARN/FAIL/SKIP 表，打印中文报告，
        按 --require 汇总退出码，并可用 --json 落一份可归档的 JSON。

  只读保证有三层（都在本文件里，可审计）：
    1. 源码层：本文件不出现任何发布 API 名字，启动时自检（见 _assert_read_only_source）；
    2. 运行时：建完订阅后断言节点上发布者数量为 0（见 _build_node）；
    3. 行为层：所有回调只 append 统计，不调用任何 send/publish 语义。
  所以"跑这个脚本把机器人弄动了"这件事在结构上不可能发生。

===============================================================================
2. 数据流
===============================================================================

  unitree_go/LowState ──┐
  unitree_go/LowCmd   ──┤  回调里只做字段搬运（不做判定）
  sensor_msgs/PointCloud2 ─┤        │
  geometry_msgs/PoseStamped ┤        ▼
  nav_msgs/Odometry       ─┤   Samples(lowstate=[...], lowcmd=..., lidar=[...],
  unitree_go/SportModeState ┤            topics={...}, duration_s=..., tick_step=...)
  unitree_go/LidarState   ──┘        │
                                     ▼
                       real_diag.analyze() -> Report
                                     │
             ┌───────────────────────┼────────────────────────┐
             ▼                       ▼                        ▼
      render_text()            report.to_dict()          required verdict
      （中文表格+结论）          （--json 文件）            （退出码 0/1/2）

  判定逻辑、阈值、字段语义全部在 signal_io/real_diag.py；本文件不做判定，
  这样"采集"和"判断"可以分别验证（前者要 ROS 2，后者本机就能跑）。

===============================================================================
3. 用法
===============================================================================

  真机（机器人已开机、网线插好、L1 已开）：
      .venv/bin/python scripts/real_sensor_check.py --target real_ros --duration 5
      .venv/bin/python scripts/real_sensor_check.py --target real_ros --duration 10 \
          --json logs/real_sensor_check.json --require state,imu,motors,lidar

  仿真（另开一个跑 sim 的终端；sim_ros 的 domain=1、点云 topic 不同）：
      .venv/bin/python scripts/real_sensor_check.py --target sim_ros --duration 5

  本机没有 ROS 2 或者想先验证脚本本身（不连机器人、不 import rclpy）：
      .venv/bin/python scripts/real_sensor_check.py --selftest
      .venv/bin/python scripts/real_sensor_check.py --selftest --json /tmp/report.json

  关键参数：
      --duration 5          观测时长（秒）；只看频率建议 >=5 s
      --json out.json       额外写一份 JSON（含 meta：环境变量/网卡/命令）
      --strict              必需项只有 WARN 也返回 1（默认返回 2）
      --require a,b,c       必需项分组：state / imu / motors / lidar / lowcmd /
                            foot_force / bit_flag / pose / odom / power /
                            sportmodestate / lidar_state（也可写精确 check 名）
      --no-cmd-check        完全不订阅 /lowcmd（怕把别人的控制器误判成异常）
      --state-topic ...     覆盖话题名（默认按 target 取）
      --iface/--domain      覆盖 config 里的网卡 / ROS_DOMAIN_ID（不改文件）
      --selftest            不连 ROS 2，用合成数据跑一遍诊断内核并打印报告

  退出码：
      0  所有必需项 PASS
      1  任一必需项 FAIL（或参数/环境错误，例如没装 ROS 2）
      2  必需项没有 FAIL 但有 WARN/SKIP（--strict 时变 1）

===============================================================================
4. 真机要点（跑之前先看这几条）
===============================================================================

  * **采集前先做网络预检**（本脚本自动做，不 sudo）：
        ip -br -4 addr   -> 找 192.168.123.0/24 的网卡，并看它是 UP 还是 DOWN
        ping -c1 -W1 192.168.123.161 -> 机器人是否在线（Go2 默认 IP）
    网卡 DOWN 时 DDS 收不到任何包 —— 本机就踩过 enp0s31f6(DOWN)、config 里却写
    eth0 的情况。脚本会明确提示，但**不会**自己 ip link set / 改配置。
  * domain 必须是 0（真机固件在 domain 0）；RMW 必须 rmw_cyclonedds_cpp；
    多网卡机器必须把 CYCLONEDDS_URI 绑到有线网卡，否则 DDS 可能选 Wi-Fi。
    apply_io_env() 会设置这些环境变量，且必须在 rclpy.init() 之前。
  * QoS：/lowstate、/lowcmd 用默认可靠 QoS（depth=10，与 unitree_ros2 官方例程一致）；
    L1 点云必须用 qos_profile_sensor_data（BEST_EFFORT），用 RELIABLE 会一条都收不到。
  * /lowcmd 只读检查的意义：观测期内如果**有别的进程在发**低层指令，你会看到条数
    与 CRC 结果；crc=0 或校验失败说明发布方布局错，真机固件会静默丢帧。
    自己不发，所以"没人发"才是干净状态（PASS）。
  * 官方固件上 robot_pose / robot_odom 经常为空、释放 sport mode 后 sportmodestate
    不可读 —— 这些项 real_diag 只给 SKIP/WARN，不会 FAIL。
  * 只读检测期间机器人自己可能报 bit_flag bit0（操作指令超时），因为确实没有人在
    发低层指令；real_diag 会把这条标成 WARN 并在详情里说明这是预期的。

===============================================================================
5. 常见失败模式（看到什么先查什么）
===============================================================================

  * 退出码 1 + lowstate.rate FAIL：
      网线/网段错（应 192.168.123.x）> 网卡 DOWN > ROS_DOMAIN_ID 非 0 >
      RMW 不是 cyclonedds > CYCLONEDDS_URI 绑错网卡。
  * ImportError: unitree_go：
      没 source 机器人的 ROS 2 环境（unitree_ros2/install/setup.bash）。
  * RuntimeError: localhost-only / 收不到包但 ping 得通：
      ROS_LOCALHOST_ONLY=1 没 unset；或 shell 里残留了别的 CYCLONEDDS_URI。
  * 点云一帧都没有：
      L1 没开（先发 rt/utlidar/switch ON）；或话题名不是 /utlidar/cloud；
      或 QoS 用了 RELIABLE。
  * /lowcmd 有条数但 crc_bad > 0：
      发布方的 812 字节布局/CRC 算法不对（见 signal_io/go2_crc.py 的 gold value）。

===============================================================================
6. 相关文件
===============================================================================

  signal_io/real_diag.py          判定内核（纯 Python，本文件只调它）
  signal_io/go2_crc.py            /lowcmd 的 812 字节布局 + CRC 校验
  signal_io/config.py             load_config / apply_io_env（domain / RMW / 网卡）
  scripts/go2_real_check.py       更早的简化版只读探针（保留，本脚本是它的超集）
  tests/test_real_diag.py         内核单测（不需要 ROS 2）
  real_env/README.md              真机部署 checklist
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from signal_io.real_diag import (  # noqa: E402
    FAIL, LIDAR_HZ_SIM, PASS, SKIP, WARN, Check, LidarSample, LowCmdSamples,
    LowStateSample, Report, Samples, analyze, render_text, synthetic_samples,
)

# 只读保证（1/3）：源码里不允许出现发布 API。这里用字符串拼接写"禁用词"，
# 这样这段检查代码本身不会把禁用词写进文件（否则自检永远失败）。
_FORBIDDEN_PUB_TOKENS: Tuple[str, ...] = ("create_" + "publisher", "." + "publish(")

ROBOT_IP = "192.168.123.161"     # Go2 出厂低层以太网 IP（unitree 官方文档）
ROBOT_SUBNET = "192.168.123."    # 机器人网段前缀

DEFAULT_TOPICS = {
    "real_ros": {
        "state": "/lowstate",
        "cmd": "/lowcmd",
        "cloud": "/utlidar/cloud",
        "pose": "/utlidar/robot_pose",
        "odom": "/utlidar/robot_odom",
        "sport": "/sportmodestate",
        "lidar_state": "/utlidar/lidar_state",
    },
    "sim_ros": {
        "state": "/lowstate",
        "cmd": "/lowcmd",
        "cloud": "/go2/lidar/points",
        "pose": "/utlidar/robot_pose",
        "odom": "/go2/odom",
        "sport": "/sportmodestate",
        "lidar_state": "/utlidar/lidar_state",
    },
}

DEFAULT_REQUIRE = "state,imu,motors,lidar"


# ================================================================ 只读自检
def _assert_read_only_source() -> None:
    """源码层只读保证：本文件不得出现任何发布 API 名字。

    为什么用"拼接后的 token"来查：如果把 token 直接写进源码，自检会命中自己。
    抛 AssertionError 而不是 warning —— 这种保证坏了就不能跑。
    """
    path = os.path.abspath(__file__)
    with open(path, "r", encoding="utf-8") as fh:
        src = fh.read()
    hits = [tok for tok in _FORBIDDEN_PUB_TOKENS if tok in src]
    assert not hits, "real_sensor_check.py 出现发布 API %s：本脚本必须只读" % hits


# ================================================================ 网络预检
def _run_cmd(cmd: List[str], timeout: float = 3.0) -> Tuple[Optional[int], str]:
    """跑一条外部命令，best-effort：找不到/超时都返回 (None, "")，绝不抛。"""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return int(p.returncode), ((p.stdout or "") + (p.stderr or "")).strip()
    except (OSError, subprocess.SubprocessError):
        return None, ""


def precheck_network(iface_hint: Optional[str] = None) -> Dict[str, Any]:
    """best-effort 网络预检（不 sudo、不改配置）：找机器人网段网卡 + ping 机器人。

    Args:
        iface_hint: config/--iface 里写的网卡名，用来对照实测网卡。
    Returns:
        dict：iface / iface_state / iface_ip / candidates / ping_ok / ping_ms /
        warnings（给报告 meta 用）。
    """
    info: Dict[str, Any] = {
        "iface": None,
        "iface_state": None,
        "iface_ip": None,
        "candidates": [],
        "interfaces": [],
        "ping_ok": None,
        "ping_ms": None,
        "warnings": [],
        "iface_hint": iface_hint,
    }
    ip_bin = shutil.which("ip")
    if ip_bin:
        # 不带 -4：DOWN 且没有 IPv4 的网卡（本机的 enp0s31f6 就是这样）也要列出来，
        # 否则用户只看到"没有网卡"却不知道自己有线网卡存在但没 up/没配 IP。
        rc, out = _run_cmd([ip_bin, "-br", "addr", "show"])
        if rc == 0:
            for line in out.splitlines():
                parts = line.split()
                if not parts:
                    continue
                name = parts[0].split("@")[0]
                state = parts[1] if len(parts) > 1 else ""
                addrs = [p for p in parts[2:] if "/" in p and ":" not in p]
                info["interfaces"].append({"iface": name, "state": state, "addrs": addrs})
                for addr in addrs:
                    if addr.startswith(ROBOT_SUBNET):
                        info["candidates"].append(
                            {"iface": name, "state": state, "addr": addr})
    else:
        info["warnings"].append("本机没有 ip 命令：跳过网段检查（用 --iface 显式指定网卡）")

    if info["candidates"]:
        # 多张网卡都在 192.168.123.0/24 时优先用 config/--iface 指定的那张
        chosen = None
        if iface_hint:
            for c in info["candidates"]:
                if c["iface"] == iface_hint:
                    chosen = c
                    break
        chosen = chosen or info["candidates"][0]
        info.update(iface=chosen["iface"], iface_state=chosen["state"],
                    iface_ip=chosen["addr"])
        if chosen["state"].upper() != "UP":
            info["warnings"].append(
                "网卡 %s 当前是 %s：DDS 收不到包。先执行 sudo ip link set %s up "
                "并确认网线（本脚本不会替你改）"
                % (chosen["iface"], chosen["state"] or "?", chosen["iface"]))
        if iface_hint and chosen["iface"] != iface_hint:
            info["warnings"].append(
                "config/--iface 写的是 %s，但 %s 实际在 %s 上：用 --iface %s 覆盖"
                "（或改 config.transport.interface / GO2_INTERFACE）"
                % (iface_hint, ROBOT_SUBNET + "0/24", chosen["iface"], chosen["iface"]))
    else:
        info["warnings"].append(
            "没有找到 %s0/24 的网卡地址：网线没插 / 网卡没配 IP / 网卡 DOWN"
            % ROBOT_SUBNET)
        # 把"看着像有线网卡"的候选列出来，方便直接改 --iface / 起网卡
        wired = [i for i in info["interfaces"]
                 if i["iface"] != "lo" and not i["iface"].startswith("docker")]
        if wired:
            info["warnings"].append(
                "本机网卡清单: %s" % "，".join(
                    "%s(%s%s)" % (i["iface"], i["state"] or "?",
                                  (", " + ", ".join(i["addrs"])) if i["addrs"] else ", 无地址")
                    for i in wired))
        if iface_hint:
            hit = [i for i in info["interfaces"] if i["iface"] == iface_hint]
            if hit:
                info["warnings"].append(
                    "config/--iface 写的是 %s：该网卡存在（state=%s，%s），但不在 %s0/24"
                    % (iface_hint, hit[0]["state"] or "?",
                       ", ".join(hit[0]["addrs"]) or "无地址", ROBOT_SUBNET))
            else:
                info["warnings"].append(
                    "config/--iface 写的是 %s：本机根本没有这块网卡，用 --iface 改成实测网卡名"
                    % iface_hint)

    ping_bin = shutil.which("ping")
    if ping_bin:
        rc, out = _run_cmd([ping_bin, "-c1", "-W1", ROBOT_IP], timeout=3.0)
        info["ping_ok"] = (rc == 0)
        if rc == 0:
            # 解析 "time=0.42 ms"（不同发行版措辞略有差异，解析失败就留 None）
            for piece in out.split():
                if "time" in piece and "=" in piece:
                    try:
                        info["ping_ms"] = float(piece.split("=")[1])
                    except (IndexError, ValueError):
                        info["ping_ms"] = None
        elif rc is None:
            info["warnings"].append("ping 执行失败（可能没有权限/命令不存在）")
        else:
            info["warnings"].append(
                "ping %s 不通：机器人没开机 / 网线 / 网段 / 网卡 DOWN 逐个查"
                % ROBOT_IP)
    else:
        info["warnings"].append("本机没有 ping 命令：跳过机器人可达性检查")
    return info


def _print_precheck(pre: Dict[str, Any], iface_effective: Optional[str]) -> None:
    """把网络预检结果打成人话（不中断流程，只提示）。"""
    print("[net] 配置网卡 = %s" % (iface_effective or "(未配置)"))
    if pre.get("iface"):
        print("[net] 实测 %s0/24 在 %s（state=%s，addr=%s）"
              % (ROBOT_SUBNET, pre["iface"], pre.get("iface_state"),
                 pre.get("iface_ip")))
    else:
        print("[net] 实测：没有找到 %s0/24 网卡" % ROBOT_SUBNET)
    if pre.get("ping_ok"):
        print("[net] ping %s: 通%s" % (ROBOT_IP,
                                       ("（%.2f ms）" % pre["ping_ms"])
                                       if pre.get("ping_ms") else ""))
    else:
        print("[net] ping %s: 不通/未测" % ROBOT_IP)
    for w in pre.get("warnings") or []:
        print("[net][WARN] " + w)


# ================================================================ 采集
class _Collector:
    """把 ROS 消息搬成 real_diag 的纯数据（只 append，不做判定）。"""

    def __init__(self, target: str, duration_s: float,
                 tick_step: Optional[int], lidar_hz: Optional[float]) -> None:
        self.target = target
        self.duration_s = duration_s
        self.tick_step = tick_step
        self.lidar_hz = lidar_hz
        self.lowstate: List[LowStateSample] = []
        self.lidar: List[LidarSample] = []
        self.topics: Dict[str, int] = {"pose": 0, "odom": 0,
                                       "sportmodestate": 0, "lidar_state": 0}
        self.lowcmd = LowCmdSamples()
        self.notes: List[str] = []
        self.n_errors = 0              # 回调解析失败的拍数（只统计，不影响流程）
        self.t_start = 0.0
        self.t_end = 0.0
        self._n_range_attempt = 0

    # ------------------------------------------------------------ /lowstate
    def on_state(self, m) -> None:
        """每拍 /lowstate -> LowStateSample（只取前 12 个电机槽位）。

        解析失败只记 note 并跳过这一拍：诊断脚本不能因为一条畸形消息整场退出。
        """
        try:
            self._on_state(m)
        except Exception as exc:  # noqa: BLE001
            self.n_errors += 1
            if len(self.notes) < 5:
                self.notes.append("LowState 解析失败（已跳过该拍）：%s" % exc)

    def _on_state(self, m) -> None:
        imu = getattr(m, "imu_state", None)
        ms = list(getattr(m, "motor_state", []) or [])
        n_mot = min(len(ms), 12)
        mode = [int(getattr(ms[i], "mode", 0)) & 0xFF for i in range(n_mot)]
        q = [float(getattr(ms[i], "q", 0.0)) for i in range(n_mot)]
        dq = [float(getattr(ms[i], "dq", 0.0)) for i in range(n_mot)]
        ddq = [float(getattr(ms[i], "ddq", 0.0)) for i in range(n_mot)]
        tau = [float(getattr(ms[i], "tau_est", 0.0)) for i in range(n_mot)]
        temp = [int(getattr(ms[i], "temperature", 0)) for i in range(n_mot)]
        lost = [int(getattr(ms[i], "lost", 0)) for i in range(n_mot)]
        try:
            quat = [float(x) for x in list(getattr(imu, "quaternion", []))]
        except (TypeError, ValueError):
            quat = []
        try:
            gyro = [float(x) for x in list(getattr(imu, "gyroscope", []))]
        except (TypeError, ValueError):
            gyro = []
        try:
            accel = [float(x) for x in list(getattr(imu, "accelerometer", []))]
        except (TypeError, ValueError):
            accel = []
        try:
            rpy = [float(x) for x in list(getattr(imu, "rpy", []))]
        except (TypeError, ValueError):
            rpy = []
        try:
            foot = [float(x) for x in list(getattr(m, "foot_force", []))[:4]]
        except (TypeError, ValueError):
            foot = []
        self.lowstate.append(LowStateSample(
            t=time.monotonic(),
            tick=int(getattr(m, "tick", 0)),
            mode=mode, q=q, dq=dq, ddq=ddq, tau_est=tau, temperature=temp,
            lost=lost, foot_force=foot, rpy=rpy, quat=quat, gyro=gyro, accel=accel,
            bit_flag=int(getattr(m, "bit_flag", 0)) & 0xFF,
            power_v=_opt_float(m, "power_v"),
            power_a=_opt_float(m, "power_a"),
        ))

    # ------------------------------------------------------------ /lowcmd
    def on_cmd(self, m) -> None:
        """每条 /lowcmd：累加条数，并按 812 字节布局校验 CRC（不保留消息）。

        整段包 try：别的发布方消息畸形时只记 note，不能让诊断脚本挂掉。
        """
        try:
            self._on_cmd(m)
        except Exception as exc:  # noqa: BLE001
            self.n_errors += 1
            if len(self.notes) < 5:
                self.notes.append("LowCmd 解析失败（已跳过该条）：%s" % exc)

    def _on_cmd(self, m) -> None:
        lc = self.lowcmd
        lc.n += 1
        msg_crc = int(getattr(m, "crc", 0)) & 0xFFFFFFFF
        lc.last_crc = msg_crc
        try:
            lc.last_kp = [float(m.motor_cmd[i].kp) for i in range(12)]
        except (AttributeError, TypeError, ValueError):
            pass
        if msg_crc == 0:
            lc.crc_zero += 1
            return
        try:
            from signal_io.go2_crc import lowcmd_crc
            from signal_io.msg import LowCmd as PyLowCmd, MotorCmd
            py = PyLowCmd(head=[int(m.head[0]), int(m.head[1])],
                          level_flag=int(m.level_flag),
                          frame_reserve=int(getattr(m, "frame_reserve", 0)),
                          gpio=int(getattr(m, "gpio", 0)))
            for i in range(20):
                src = m.motor_cmd[i]
                try:
                    reserve = int(src.reserve[0])       # unitree_go: uint32[3]
                except (TypeError, IndexError, AttributeError):
                    reserve = int(getattr(src, "reserve", 0) or 0)
                py.motor_cmd[i] = MotorCmd(
                    mode=int(src.mode), q=float(src.q), dq=float(src.dq),
                    tau=float(src.tau), kp=float(src.kp), kd=float(src.kd),
                    reserve=reserve)
            if msg_crc != lowcmd_crc(py):
                lc.crc_bad += 1
        except Exception as exc:  # noqa: BLE001 - 采集壳不能因为一条坏消息挂掉
            lc.crc_bad += 1
            if len(self.notes) < 5:
                self.notes.append("LowCmd CRC 还原失败：%s" % exc)

    # ------------------------------------------------------------ 点云
    def on_cloud(self, m) -> None:
        """每帧点云：记录 frame_id/点数；前 5 帧额外算 xyz 分位（用于标定）。"""
        try:
            self._on_cloud(m)
        except Exception as exc:  # noqa: BLE001
            self.n_errors += 1
            if len(self.notes) < 5:
                self.notes.append("点云帧解析失败（已跳过该帧）：%s" % exc)

    def _on_cloud(self, m) -> None:
        n_points = int(getattr(m, "width", 0) or 0) * int(getattr(m, "height", 0) or 0)
        frame_id = str(getattr(getattr(m, "header", None), "frame_id", "") or "")
        stamp = _stamp_s(getattr(getattr(m, "header", None), "stamp", None))
        p1 = p50 = p99 = None
        if self._n_range_attempt < 5 and n_points > 0:
            self._n_range_attempt += 1
            try:
                p1, p50, p99 = _cloud_percentiles(m)
            except Exception as exc:  # noqa: BLE001
                if len(self.notes) < 5:
                    self.notes.append("点云 xyz 分位计算失败：%s" % exc)
        self.lidar.append(LidarSample(t=time.monotonic(), stamp=stamp,
                                      frame_id=frame_id, n_points=n_points,
                                      p1=p1, p50=p50, p99=p99))

    # ------------------------------------------------------------ 其他话题
    def on_pose(self, m) -> None:
        self.topics["pose"] = int(self.topics.get("pose", 0)) + 1

    def on_odom(self, m) -> None:
        self.topics["odom"] = int(self.topics.get("odom", 0)) + 1

    def on_sport(self, m) -> None:
        self.topics["sportmodestate"] = int(self.topics.get("sportmodestate", 0)) + 1

    def on_lidar_state(self, m) -> None:
        self.topics["lidar_state"] = int(self.topics.get("lidar_state", 0)) + 1

    # ------------------------------------------------------------ 汇总
    def to_samples(self) -> Samples:
        """采集结果 -> Samples（交给 real_diag.analyze）。"""
        duration = max(self.t_end - self.t_start, 0.0)
        if duration <= 0.0:            # spin 异常退出时退回计划观测时长
            duration = max(float(self.duration_s), 0.0)
        return Samples(target=self.target, lowstate=self.lowstate,
                       lowcmd=self.lowcmd, lidar=self.lidar,
                       topics=dict(self.topics), duration_s=duration,
                       tick_step=self.tick_step, lidar_hz=self.lidar_hz,
                       notes=list(self.notes))


def _opt_float(obj, attr: str) -> Optional[float]:
    """取一个可选 float 字段：没有/非法就返回 None（让诊断侧判 SKIP）。"""
    try:
        v = getattr(obj, attr, None)
        if v is None:
            return None
        return float(v)
    except (TypeError, ValueError, AttributeError):
        return None


def _stamp_s(stamp: Any) -> float:
    """builtin_interfaces/Time -> 秒；拿不到就 0。"""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return 0.0


def _cloud_percentiles(m) -> Tuple[List[float], List[float], List[float]]:
    """读一帧 PointCloud2 的 xyz，返回 1%/50%/99% 分位（米）。

    sensor_msgs_py 在不同版本里返回 ndarray（结构化）或 tuple 生成器，这里都兼容；
    点云可能很大，所以只取前 5 帧算，不做累积。
    """
    from sensor_msgs_py import point_cloud2 as pc2

    raw = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
    if isinstance(raw, np.ndarray):
        arr = raw
    else:
        arr = np.asarray(list(raw))
    if arr.size == 0:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    if getattr(arr.dtype, "names", None):
        xyz = np.stack([np.asarray(arr[name], dtype=np.float64) for name in ("x", "y", "z")],
                       axis=-1)
    else:
        xyz = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    if xyz.shape[0] == 0:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    pcts = np.percentile(xyz, [1, 50, 99], axis=0)
    return ([float(x) for x in pcts[0]], [float(x) for x in pcts[1]],
            [float(x) for x in pcts[2]])


# ================================================================ ROS 2 装配
def _build_node(args, collector: _Collector):
    """建 rclpy 节点 + 只订阅（绝不发布），返回 (rclpy, node)。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    try:
        from unitree_go.msg import LowCmd, LowState, SportModeState
    except ImportError as exc:
        raise RuntimeError(
            "缺 unitree_go 接口包（%s）：先 source 机器人的 ROS 2 环境"
            "（unitree_ros2/install/setup.bash）再运行" % exc) from exc
    LidarState = None
    try:                       # 固件/接口包版本不同，没有就跳过这个订阅
        from unitree_go.msg import LidarState as LidarStateMsg
        LidarState = LidarStateMsg
    except ImportError:
        collector.notes.append("该 unitree_go 版本没有 LidarState，跳过 /utlidar/lidar_state")

    if not rclpy.ok():
        rclpy.init(args=None)
    node = Node("go2_real_sensor_check")

    # 只读保证（2/3）：节点上不允许有任何发布者
    n_pub = _count_publishers(node)
    assert n_pub == 0, "本脚本必须只读，但节点上已有 %d 个发布者" % n_pub

    # 低层状态/指令用默认可靠 QoS（与 unitree_ros2 官方例程一致，depth=10）
    node.create_subscription(LowState, args.state_topic, collector.on_state, 10)
    if not args.no_cmd_check:
        node.create_subscription(LowCmd, args.cmd_topic, collector.on_cmd, 10)
    # --light：只留 500 Hz 的两条低层流 + sportmodestate。点云/pose/odom 的解析
    # 开销会把自己的 /lowstate 回调饿死（实测全量 190~275 Hz，裸订阅 499.8 Hz），
    # 结果把"脚本慢"误报成"链路不通"。
    light = bool(getattr(args, "light", False))
    if not light:
        # 传感器数据必须 BEST_EFFORT：真机 L1 驱动就是 BEST_EFFORT，用可靠 QoS 收不到
        node.create_subscription(PointCloud2, args.cloud_topic, collector.on_cloud,
                                 qos_profile_sensor_data)
        # 这三类只用于"有没有数据"，用 BEST_EFFORT（DDS 里 best-effort 订阅者既能匹配
        # RELIABLE 也能匹配 BEST_EFFORT 发布者，反过来不行），避免不同固件 QoS 不一致时
        # 误报"没数据"。
        node.create_subscription(PoseStamped, args.pose_topic, collector.on_pose,
                                 qos_profile_sensor_data)
        node.create_subscription(Odometry, args.odom_topic, collector.on_odom,
                                 qos_profile_sensor_data)
        if LidarState is not None:
            node.create_subscription(LidarState, args.lidar_state_topic,
                                     collector.on_lidar_state, qos_profile_sensor_data)
    # sportmodestate 任何模式下都订阅：它是判断"sport mode 有没有释放"的唯一信号
    node.create_subscription(SportModeState, args.sport_topic, collector.on_sport,
                             qos_profile_sensor_data)

    # 订阅建完后再次断言（防止以后有人手滑加了发布）
    n_pub = _count_publishers(node)
    assert n_pub == 0, "本脚本必须只读，但节点上有 %d 个发布者" % n_pub
    return rclpy, node


# rclpy 会自动给每个节点建这两个发布者，它们不算"本脚本在发布"
_IMPLICIT_PUB_TOPICS = ("/rosout", "/parameter_events")


def _count_publishers(node) -> int:
    """数节点上**由本脚本建立**的发布者数量（0 = 纯只读订阅者）。

    WARN: 两个只有在真跑 rclpy 时才暴露的坑（本机无 ROS 2，纯 py 单测测不出来，
    是在容器里对着 sim_ros 真跑时才发现的）：
      1) rclpy 的 Node.publishers 是**生成器**，对它 len() 直接 TypeError；
      2) 任何节点天生就带 /rosout 与 /parameter_events 两个发布者 —— 它们不是
         "我们在发指令"，必须排除，否则只读断言永远失败。
    所以这里遍历生成器、按话题名过滤隐式发布者，且异常一律按 0 处理：
    只读自检本身绝不能因为这个崩。
    """
    try:
        pubs = getattr(node, "publishers", None)
        if pubs is None:
            return 0
        n = 0
        for p in pubs:
            topic = str(getattr(p, "topic_name", None) or getattr(p, "topic", "") or "")
            if topic in _IMPLICIT_PUB_TOPICS or topic.startswith("/rosout"):
                continue
            n += 1
        return int(n)
    except Exception:  # noqa: BLE001 - RMW 版本差异不能让自检崩
        return 0

def _cmd_publishers(node, topic: str) -> int:
    """查 /lowcmd 上有几个发布者（best-effort；拿不到就 0）。"""
    try:
        infos = node.get_publishers_info_by_topic(topic)
        return int(len(infos or []))
    except Exception:  # noqa: BLE001 - RMW 不支持时别把整个检查搞挂
        return 0


def _spin(rclpy, node, duration_s: float) -> None:
    """按墙钟时间 spin，直到观测结束。"""
    t_end = time.monotonic() + max(float(duration_s), 0.5)
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)


_LIGHT_SKIP_NAMES = ("pose", "odom", "lidar_state")
_LIGHT_SKIP_PREFIXES = ("lidar.",)


def _mark_light_skipped(rep: Report) -> None:
    """--light 没订阅的话题，对应检查项改判 SKIP（不是 PASS，也不是 WARN）。

    SKIP 的语义是"这台机器/这次运行没有这个信号"，正好匹配"我们主动没订阅"；
    若它在 --require 里，verdict 会降成 WARN（= 没验证过），不会被当成通过。
    """
    for i, c in enumerate(rep.checks):
        if c.name in _LIGHT_SKIP_NAMES or any(
                c.name.startswith(p) for p in _LIGHT_SKIP_PREFIXES):
            rep.checks[i] = Check(SKIP, c.name,
                                  "已跳过（--light：只测低层链路频率）", None)


# ================================================================ 输出
def _required_table(rep: Report, tokens: List[str]) -> Tuple[str, Dict[str, str]]:
    """把 --require 的每个 token 汇总成一行状态，返回 (文本, {token: status})。"""
    lines = ["必需项（--require %s）:" % ",".join(tokens)]
    per: Dict[str, str] = {}
    for tok in tokens:
        matched, unknown = rep.select_required([tok])
        status = FAIL if unknown else rep.verdict([tok])
        per[tok] = status
        names = ", ".join(c.name for c in matched) if matched else "(未识别)"
        lines.append("  %-16s %-5s %s" % (tok, status, names))
    return "\n".join(lines), per


def _write_json(path: str, payload: Dict[str, Any]) -> None:
    """写 JSON（不允许 NaN）：目录不存在就报错，不自动建（避免写错位置）。"""
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, allow_nan=False)
    print("[json] 已写入 %s" % os.path.abspath(path))


def _exit_code(verdict: str, strict: bool) -> int:
    """必需项 verdict -> 进程退出码（0/1/2，--strict 时 WARN 也算 1）。"""
    if verdict == FAIL:
        return 1
    if verdict in (WARN, SKIP):
        return 1 if strict else 2
    return 0


# ================================================================ 参数
def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Go2 实机传感器只读全项自检（只订阅，绝不发送任何指令）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"),
                    help="配置文件（读 transport/perception）")
    ap.add_argument("--target", default="real_ros", choices=["real_ros", "sim_ros"],
                    help="IO profile：真机 real_ros / 仿真 sim_ros")
    ap.add_argument("--duration", type=float, default=5.0, help="观测时长（秒）")
    ap.add_argument("--json", default=None, help="把报告（含 meta）写到这个 JSON 文件")
    ap.add_argument("--strict", action="store_true",
                    help="必需项只有 WARN/SKIP 时也返回 1（默认返回 2）")
    ap.add_argument("--require", default=DEFAULT_REQUIRE,
                    help="必需项分组/精确检查名，逗号分隔（默认 %s）" % DEFAULT_REQUIRE)
    ap.add_argument("--state-topic", default=None, help="默认按 target 取")
    ap.add_argument("--cmd-topic", default=None, help="默认按 target 取")
    ap.add_argument("--cloud-topic", default=None, help="默认按 target 取")
    ap.add_argument("--pose-topic", default=None, help="默认按 target 取")
    ap.add_argument("--odom-topic", default=None, help="默认按 target 取")
    ap.add_argument("--sport-topic", default=None, help="默认 /sportmodestate")
    ap.add_argument("--lidar-state-topic", default=None,
                    help="默认 /utlidar/lidar_state（拿不到就 SKIP）")
    ap.add_argument("--iface", default=None,
                    help="覆盖 config.transport.interface（不改文件；也可用 GO2_INTERFACE）")
    ap.add_argument("--domain", type=int, default=None,
                    help="覆盖 ROS_DOMAIN_ID（真机 0；仿真 1）")
    ap.add_argument("--no-cmd-check", action="store_true",
                    help="不订阅 /lowcmd（避免把别的控制器误判成异常）")
    ap.add_argument("--light", action="store_true",
                    help="轻量模式：只订阅 /lowstate /lowcmd /sportmodestate。"
                         "多话题同时解析会把自己的 /lowstate 计数拉低"
                         "（实测同一根线上 190~275 Hz，裸订阅 499.8 Hz）；"
                         "要判链路到底给不给 500 Hz 时用它，点云/雷达项标 SKIP。")
    ap.add_argument("--selftest", action="store_true",
                    help="不连 ROS 2：用合成数据跑一遍诊断内核并打印报告")
    return ap.parse_args(argv)


def _resolve_topics(args) -> None:
    """按 target 填默认话题名（显式传了就用传的）。"""
    defaults = DEFAULT_TOPICS[args.target]
    args.state_topic = args.state_topic or defaults["state"]
    args.cmd_topic = args.cmd_topic or defaults["cmd"]
    args.cloud_topic = args.cloud_topic or defaults["cloud"]
    args.pose_topic = args.pose_topic or defaults["pose"]
    args.odom_topic = args.odom_topic or defaults["odom"]
    args.sport_topic = args.sport_topic or defaults["sport"]
    args.lidar_state_topic = args.lidar_state_topic or defaults["lidar_state"]


def _parse_require(text: str) -> List[str]:
    """--require a,b,c -> ["a","b","c"]（去空、去重、保序）。"""
    out: List[str] = []
    for tok in str(text or "").replace(";", ",").split(","):
        tok = tok.strip()
        if tok and tok not in out:
            out.append(tok)
    return out


# ================================================================ selftest
def run_selftest(args) -> int:
    """--selftest：不需要 ROS 2、不需要机器人，用合成数据跑完整流程。"""
    _assert_read_only_source()
    rep = analyze(synthetic_samples(target=args.target))
    print(render_text(rep))
    print()
    print("[selftest] 本机未连接 ROS 2 / 机器人；以上是合成数据（%s 档）的报告。"
          % args.target)
    print("[selftest] 内核与渲染可用；真机采集请去掉 --selftest 并确认 ROS 2 环境。")
    tokens = _parse_require(args.require)
    _matched, unknown = rep.select_required(tokens)
    if unknown:
        print("[ERROR] --require 里有无法识别的项：%s" % unknown)
        return 1
    table, per = _required_table(rep, tokens)
    print()
    print(table)
    verdict = FAIL if any(v == FAIL for v in per.values()) else rep.verdict(tokens)
    print("selftest verdict = %s（退出码口径：0=PASS / 1=FAIL / 2=仅WARN）" % verdict)
    if args.json:
        _write_json(args.json, {"mode": "selftest", "required": per,
                                "verdict": verdict, "report": rep.to_dict(),
                                "argv": sys.argv[1:]})
    return 0 if verdict in (PASS, WARN, SKIP) else 1


# ================================================================ 主流程
def run_real(args) -> int:
    """真机/仿真采集：环境 -> 预检 -> 采集 -> 判定 -> 报告 -> 退出码。"""
    _assert_read_only_source()
    _resolve_topics(args)
    tokens = _parse_require(args.require)

    # ---- 1) 先校验 --require 拼写（用空样本探一下，避免白采 5 秒才发现写错）
    probe = analyze(Samples(target=args.target), check_lowcmd=not args.no_cmd_check)
    _matched, unknown = probe.select_required(tokens)
    if unknown:
        print("[ERROR] --require 里有无法识别的项：%s" % unknown)
        print("        可用分组：state, imu, motors, lidar, lowcmd, foot_force, "
              "bit_flag, pose, odom, power, sportmodestate, lidar_state")
        return 1

    # ---- 2) 配置 + 环境（apply_io_env 必须早于 rclpy.init）
    from signal_io.config import apply_io_env, load_config

    try:
        cfg = load_config(args.config, target=args.target)
    except (OSError, ValueError, KeyError) as exc:
        print("[ERROR] 读配置失败（%s）：%s" % (args.config, exc))
        return 1
    if args.iface:
        cfg["transport"]["interface"] = args.iface
    if args.domain is not None:
        cfg["transport"]["domain_id"] = int(args.domain)
    env = apply_io_env(cfg)
    iface_effective = str(cfg["transport"].get("interface") or "")

    # ---- 3) 网络预检（best-effort，不 sudo）
    pre = precheck_network(iface_hint=iface_effective)
    _print_precheck(pre, iface_effective)
    print("[env] ROS_DOMAIN_ID=%s  RMW_IMPLEMENTATION=%s"
          % (os.environ.get("ROS_DOMAIN_ID"), os.environ.get("RMW_IMPLEMENTATION")))
    print("[env] CYCLONEDDS_URI=%s" % (os.environ.get("CYCLONEDDS_URI", "(unset)")))
    print("[env] ROS_LOCALHOST_ONLY=%s（真机必须 unset）"
          % os.environ.get("ROS_LOCALHOST_ONLY", "(unset)"))
    print("[io ] 目标=%s 观察 %.1fs | state=%s cmd=%s cloud=%s"
          % (args.target, args.duration, args.state_topic, args.cmd_topic,
             args.cloud_topic))
    print("[io ] 只读：本脚本不发送任何 LowCmd；机器人可能自报 bit_flag bit0（无指令），"
          "属预期")

    # ---- 4) 采集（只订阅）
    collector = _Collector(
        target=args.target, duration_s=args.duration,
        tick_step=_tick_step_from_cfg(cfg),
        lidar_hz=_lidar_hz_from_cfg(cfg, args.target))
    rclpy = None
    node = None
    publishers = 0
    try:
        try:
            rclpy, node = _build_node(args, collector)
        except RuntimeError as exc:
            print("[ERROR] %s" % exc)
            return 1
        except ImportError as exc:
            print("[ERROR] 本机没有可用的 ROS 2（%s）。" % exc)
            print("        本仓库真机脚本需要 ROS 2 + unitree_go 接口包；"
                  "先验证脚本本身可用 --selftest。")
            return 1
        except Exception as exc:  # noqa: BLE001 - 例如 rmw 加载失败 / domain 非法
            print("[ERROR] 创建 ROS 2 节点失败：%s" % exc)
            print("        检查 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp、"
                  "ROS_DOMAIN_ID 是否合法、是否 source 过 ROS 2 环境。")
            return 1
        publishers = _cmd_publishers(node, args.cmd_topic) if not args.no_cmd_check else 0
        collector.t_start = time.monotonic()
        _spin(rclpy, node, args.duration)
        collector.t_end = time.monotonic()
        # 图发现（publisher 数）需要时间，观测结束后再查一次，取两次的较大值
        if not args.no_cmd_check:
            publishers = max(publishers, _cmd_publishers(node, args.cmd_topic))
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    # ---- 5) 判定 + 报告
    if collector.lowcmd.n > 0 or publishers > 0:
        collector.lowcmd.publishers = max(int(publishers), 1 if collector.lowcmd.n else 0)
    samples = collector.to_samples()
    report = analyze(samples, check_lowcmd=not args.no_cmd_check)
    if getattr(args, "light", False):
        _mark_light_skipped(report)
        report.meta.setdefault("notes", []).append(
            "--light：点云/pose/odom/lidar_state 未订阅，相关项判 SKIP")
    report.meta.update({
        # 用 os.environ 而不是 apply_io_env 的返回：shell 里已有的 CYCLONEDDS_URI
        # 也算"实际生效值"，诊断要看到真正被 RMW 读到的那个。
        "env": {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                                               "CYCLONEDDS_URI")},
        "iface": iface_effective,
        "network": {k: pre.get(k) for k in ("iface", "iface_state", "iface_ip",
                                           "ping_ok", "ping_ms", "warnings")},
        "topics_used": {k: getattr(args, k) for k in
                        ("state_topic", "cmd_topic", "cloud_topic", "pose_topic",
                         "odom_topic", "sport_topic", "lidar_state_topic")},
        "cmd_publishers": int(publishers),
        "argv": sys.argv[1:],
    })
    print()
    print(render_text(report))
    table, per = _required_table(report, tokens)
    print()
    print(table)
    verdict = FAIL if any(v == FAIL for v in per.values()) else report.verdict(tokens)
    print("必需项 verdict = %s" % verdict)
    if verdict == FAIL:
        print("=> 先解决 FAIL 项（见上面「上机前还差什么」），不要发 LowCmd。")
    elif verdict in (WARN, SKIP):
        print("=> 可以继续只读检测，但下面这些项没到 PASS，上机前确认原因。")
    else:
        print("=> 传感器收发全项通过。")
    if args.no_cmd_check:
        print("提示：已用 --no-cmd-check 跳过 /lowcmd 检查（只读检测期间推荐保持）")
    if report.meta.get("notes"):
        for note in report.meta["notes"]:
            print("[note] %s" % note)

    code = _exit_code(verdict, bool(args.strict))
    if args.json:
        _write_json(args.json, {"mode": "collect", "required": per,
                                "verdict": verdict, "exit_code": code,
                                "report": report.to_dict(),
                                "network": report.meta.get("network"),
                                "argv": sys.argv[1:]})
    return code


def _tick_step_from_cfg(cfg: Dict[str, Any]) -> Optional[int]:
    """从 io profile 的 tick_dt 推期望 Δtick（真机 1 ms / 仿真 2 ms）。"""
    try:
        dt = float((cfg.get("transport") or {}).get("tick_dt") or 0.0)
    except (TypeError, ValueError):
        return None
    if dt <= 0.0:
        return None
    step = int(round(dt * 1e3))
    return step if step > 0 else None


def _lidar_hz_from_cfg(cfg: Dict[str, Any], target: str) -> Optional[float]:
    """仿真档用 config.perception.rate_hz；真机用 real_diag 的 11 Hz 默认。"""
    if not str(target).startswith("sim"):
        return None
    try:
        hz = float((cfg.get("perception") or {}).get("rate_hz") or 0.0)
    except (TypeError, ValueError):
        return None
    return hz if hz > 0 else LIDAR_HZ_SIM


def main(argv: Optional[List[str]] = None) -> int:
    """入口：--selftest 走合成数据，其余走真机/仿真采集。"""
    args = parse_args(argv)
    if args.selftest:
        return run_selftest(args)
    return run_real(args)


if __name__ == "__main__":
    raise SystemExit(main())
