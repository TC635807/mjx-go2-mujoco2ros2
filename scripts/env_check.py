#!/usr/bin/env python3
"""上机前环境体检（env_check）：一条命令回答「这台机器现在能不能打真机」。

===============================================================================
1. 职责
===============================================================================

只做**只读**检查：不 source ROS、不启动任何节点、不发任何 DDS 报文（唯一会碰
网络的是 ping 机器人 192.168.123.161，可用 --no-network 关掉）。它回答的是：

    * 本机 python / numpy / mujoco 能不能跑本仓（版本 vs requirements.txt）；
    * MuJoCo 能不能加载 sim_env/models/go2/scene.xml（模型文件齐不齐）；
    * ROS 2 这一半齐不齐：/opt/ros、ros2 CLI、rclpy、rmw_cyclonedds_cpp、
      libddsc(CycloneDDS)、unitree_go 消息包；
    * 当前 shell 的 ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI /
      ROS_LOCALHOST_ONLY 是不是真机该有的样子；
    * 目标网卡存不存在、UP 没有、在不在 192.168.123.0/24、机器人 ping 得通吗；
    * 机器本身：CPU / 核数、实时优先级上限、磁盘余量、git commit 与脏不脏、
      脚本可执行位。

它**不**替代 scripts/real_sensor_check.py / go2_real_check.py 的传感器自检：
那一步要连真机收 /lowstate，本脚本在没上电、没插网线时也必须能跑完。

===============================================================================
2. 数据流（谁调用它、退出码怎么被用）
===============================================================================

    用户手动:      .venv/bin/python scripts/env_check.py --target real_ros
    上机脚本:      scripts/real_run.sh 的 preflight 阶段
                     -> .venv/bin/python scripts/env_check.py --target real_ros \
                            --iface "$GO2_IFACE" --json logs/real_<ts>/preflight.json
                     -> 退出码非 0 时 real_run.sh 拒绝起控制器（--force 可越过）

    退出码约定（**这是 real_run.sh 的安全闸门，改动前先读这一行**）：
        0  = 全部 OK（允许有 SKIP），或只有 WARN 且没开 --strict
        1  = 至少一项 FAIL；--strict 时任何 WARN 也算 1
    因此新增检查项时，"这台机器缺了它还能不能安全上机"决定它是 FAIL 还是 WARN。

输出三种形态（可同时要）：
    * 人看的中文表（默认，stdout）；
    * 机器读的 JSON（--json PATH 落盘，--json - 直接打到 stdout）；
    * 末尾的汇总 + 每一条 FAIL/WARN 的「修法」。

===============================================================================
3. 真机要点（每项检查的期望值，现场对照这一节）
===============================================================================

    python      >= 3.10；且**必须是本仓 .venv 里的解释器**（rclpy 在系统 python，
                mujoco/numpy 在 venv —— 两者由 scripts/run_with_ros2.sh 拼起来）
    numpy       >= 1.26        （venv 里）
    mujoco      >= 3.2, < 4    （venv 里，见 requirements.txt）
    /opt/ros/*  存在，能 source setup.bash（Humble/Jazzy/Lyrical 都行）
    rclpy 可 import；rmw_cyclonedds_cpp 看 librmw_cyclonedds_cpp.so 在不在
                （它是 C++ rmw，没有 python 模块；固件侧是 CycloneDDS，rmw 选错 = 收不到 /lowstate）
    libddsc     CycloneDDS 的 C 库（rmw_cyclonedds_cpp 依赖它）
    unitree_go  Go2 的 LowState/LowCmd 消息包（colcon build 后才能 import）
    ROS_DOMAIN_ID       真机 = 0（仿真 1；串台就收不到状态）
    RMW_IMPLEMENTATION  rmw_cyclonedds_cpp
    CYCLONEDDS_URI      绑到有线网卡（否则多网卡机器可能走 Wi-Fi）
    ROS_LOCALHOST_ONLY  **必须未设置/0**（设成 1 只走回环，真机必然不通）
    网卡                UP 且 IPv4 在 192.168.123.0/24（PC 侧静态配 192.168.123.99 之类）
    机器人              192.168.123.161 能 ping 通（通不通只说明物理链路/DDS 可达性）
    CPU/核数            500 Hz 控制建议 >= 4 核
    ulimit -r           实时优先级上限（0 = 用不了 chrt；本仓默认不用 SCHED_FIFO，
                        所以只是提示，不阻断）
    磁盘                >= 5 GiB（真机日志 + 策略 npz）
    git                 commit 记录一下；有未提交改动提醒（不是错误）
    脚本可执行位        scripts/*.sh、scripts/*.py 要能直接被调用

===============================================================================
4. 常见失败模式 -> 怎么修（打印时也会出现在每条 FAIL/WARN 下面）
===============================================================================

  "没找到 /opt/ros/*"         -> 装 ROS 2（docs/ros2-setup.md），或用 docker/ 那套容器。
  "rclpy import 失败但 ROS 在" -> shell 没 source：用 scripts/run_with_ros2.sh 跑脚本，
                                 或先 source /opt/ros/<distro>/setup.bash。
  "rmw_cyclonedds_cpp 缺失"   -> sudo apt install ros-<distro>-rmw-cyclonedds-cpp。
  "libddsc 找不到"            -> 同上（rmw_cyclonedds_cpp 会带来），或 apt install cyclonedds。
  "unitree_go 不可见"          -> cd ros2_ws && colcon build --packages-select unitree_go，
                                 再 source ros2_ws/install/setup.bash（run_with_ros2.sh 会自动 source）。
  "网卡 eth0 不存在"           -> 本机是 enp0s31f6 之类：export GO2_IFACE=<名字>
                                 或 scripts/real_run.sh --iface <名字>（脚本会自动探测）。
  "网卡存在但 DOWN"            -> 插网线；sudo ip link set <iface> up。
  "不在 192.168.123.0/24"      -> sudo ip addr add 192.168.123.99/24 dev <iface>。
  "ping 192.168.123.161 不通"  -> 网线/机器人没上电/IP 冲突/另一台机器也配了同网段；
                                 注意 ping 不通不一定 DDS 不通（ICMP 可能被过滤），但先修它。
  "ROS_LOCALHOST_ONLY=1"       -> unset ROS_LOCALHOST_ONLY（真机必然收不到）。
  "git dirty"                  -> 上机前确认改动是有意的（Lead 会复核 diff）。

===============================================================================
5. 用法
===============================================================================

    .venv/bin/python scripts/env_check.py                       # 真机，全项
    .venv/bin/python scripts/env_check.py --no-network          # 只在本机看（不 ping）
    .venv/bin/python scripts/env_check.py --iface enp0s31f6     # 指定网卡
    .venv/bin/python scripts/env_check.py --target sim_ros      # 检查仿真 ROS 2 那一半
    .venv/bin/python scripts/env_check.py --json -              # 只输出 JSON（脚本/CI 用）
    .venv/bin/python scripts/env_check.py --json logs/pre.json --strict

相关文件：
  config/go2_sim.json              io_profiles（期望的 domain / interface）
  signal_io/config.py              load_config / detect_iface / parse_ip_addr
  scripts/run_with_ros2.sh         真正起控制器时的 ROS 2 环境 wrapper
  scripts/go2_real_check.py        传感器只读自检（要连真机，和本脚本互补）
  scripts/real_run.sh              上机启动器（本脚本是它的 preflight 闸门）
  docs/real-bringup.md             现场 runbook
"""

from __future__ import annotations

import argparse
import glob
import importlib.metadata
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import unicodedata
from typing import Any, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ================================================================ 状态与汇总
# 状态语义（决定退出码，见模块 docstring §2）：
#   OK   这项没问题
#   WARN 不理想但能上机（或换个方式能绕过）；--strict 时算失败
#   FAIL 现在就这样上机不安全 / 跑不起来
#   SKIP 本机/本次不适用（例如 --no-network、仿真目标不 ping 机器人）
OK, WARN, FAIL, SKIP = "OK", "WARN", "FAIL", "SKIP"

ROBOT_IP = "192.168.123.161"          # Go2 运控固件地址（real_env/README.md §2）
ROBOT_NET_PREFIX = "192.168.123."     # 与 signal_io.config.ROBOT_NET_PREFIX 保持一致
MIN_DISK_GIB = 5.0

# 检查项里会去 import 的东西，只在真正需要的检查里 import，避免"缺依赖就整体崩"
_PY_DEPS = ("numpy", "mujoco")


def item(cid: str, name: str, status: str, detail: str = "", hint: str = "") -> Dict[str, str]:
    """造一条检查结果（所有检查项统一这个结构，JSON 里也是它）。"""
    return {"id": cid, "name": name, "status": status, "detail": detail, "hint": hint}


# ================================================================ 纯函数（可单测）
def version_tuple(ver: str) -> tuple:
    """把版本串拆成可比较的整数元组：'3.13.7' -> (3, 13, 7)。

    宽松解析：遇到非数字段就截断（'3.2.0rc1' -> (3, 2, 0)、'2.5.4.dev0' 同理），
    够用来比 requirements.txt 里的下限/上限，不追求 PEP 440 全语义（不引第三方包）。
    """
    out: List[int] = []
    for part in str(ver or "").strip().lstrip("vV").split("."):
        m = re.match(r"^(\d+)", part)
        if not m:
            break
        out.append(int(m.group(1)))
    return tuple(out)


def _pad(a: tuple, b: tuple) -> tuple:
    """把两个版本元组补到同长度再比较（避免 (3,2) 与 (3,2,0) 比较结果反直觉）。"""
    n = max(len(a), len(b))
    return a + (0,) * (n - len(a)), b + (0,) * (n - len(b))


def spec_ok(spec: str, version: str) -> bool:
    """按 requirements.txt 风格的条件判断版本是否满足。

    支持逗号分隔的多条件（AND）和 >=, <=, ==, !=, >, <, ~= 七种算子（~= 按 >= 保守处理）：
        spec_ok(">=3.2,<4", "3.17.0") -> True
        spec_ok(">=3.2,<4", "4.0.1")  -> False
        spec_ok(">=1.26",  "2.5.4")   -> True
    空 spec 视为满足（requirements 里只写包名的情况）。
    """
    for clause in str(spec or "").split(","):
        clause = clause.strip()
        if not clause:
            continue
        m = re.match(r"^(>=|<=|==|!=|~=|>|<)?\s*([0-9][0-9a-zA-Z.\-+]*)$", clause)
        if not m:
            return False                     # 写错了就当不满足，让 CHECK 报出来
        op, want = m.group(1) or "==", m.group(2)
        if op == "~=":                       # ~=X.Y 语义较绕；这里只保证下限，不误报
            op = ">="
        va, vb = _pad(version_tuple(version), version_tuple(want))
        ok = {"==": va == vb, "!=": va != vb, ">=": va >= vb, "<=": va <= vb,
              ">": va > vb, "<": va < vb}[op]
        if not ok:
            return False
    return True


def parse_requirements(text: str) -> Dict[str, str]:
    """解析 requirements.txt 文本 -> {包名(小写, - 归一): 版本条件}。

    只关心 name<op>version 这种形式，注释/空行/-r/extra 忽略；
    包名统一小写并把 _ 换成 -（numpy 的两种写法等价）。
    """
    out: Dict[str, str] = {}
    for raw in (text or "").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or line.startswith("-"):
            continue
        m = re.match(r"^([A-Za-z0-9_.\-]+)\s*(\[[^\]]*\])?\s*([<>=!~][^;]*)?$", line)
        if not m:
            continue
        name = m.group(1).lower().replace("_", "-")
        # "numpy >= 1.26" 这种写法也归一成 ">=1.26"（版本号本身不含空格）
        out[name] = re.sub(r"\s+", "", m.group(3) or "")
    return out


def read_requirements(path: Optional[str] = None) -> Dict[str, str]:
    """读 requirements.txt（缺文件返回空 dict，用内置兜底版本条件）。"""
    p = path or os.path.join(ROOT, "requirements.txt")
    try:
        with open(p, "r", encoding="utf-8") as f:
            return parse_requirements(f.read())
    except OSError:
        return {}


def summarize(items: List[Dict[str, str]]) -> Dict[str, int]:
    """统计各状态条数（JSON 与人类可读汇总共用）。"""
    counts = {OK: 0, WARN: 0, FAIL: 0, SKIP: 0}
    for it in items:
        counts[it.get("status", SKIP)] = counts.get(it.get("status", SKIP), 0) + 1
    return counts


def exit_code_for(items: List[Dict[str, str]], strict: bool = False) -> int:
    """按 §2 的约定算退出码：有 FAIL -> 1；--strict 时有 WARN 也 -> 1。"""
    counts = summarize(items)
    if counts[FAIL] > 0:
        return 1
    if strict and counts[WARN] > 0:
        return 1
    return 0


def report_dict(items: List[Dict[str, str]], target: str, iface: str,
                iface_source: str, strict: bool, no_network: bool,
                extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """把结果打包成 --json 的结构（字段名是给 real_run.sh / 其它工具读的，别乱改）。"""
    counts = summarize(items)
    return {
        "tool": "env_check",
        "schema": 1,
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": platform.node(),
        "python": sys.executable,
        "target": target,
        "iface": iface,
        "iface_source": iface_source,
        "strict": bool(strict),
        "no_network": bool(no_network),
        "counts": counts,
        "exit_code": exit_code_for(items, strict),
        "items": items,
        "extra": extra or {},
    }


def disp_width(s: str) -> int:
    """中文对齐用：东亚全角字符算 2 列（终端里中英文混排才不会歪）。"""
    return sum(2 if unicodedata.east_asian_width(c) in ("W", "F") else 1 for c in s)


def pad(s: str, width: int) -> str:
    """按显示宽度右侧补空格（配合 disp_width）。"""
    return s + " " * max(0, width - disp_width(s))


def format_table(items: List[Dict[str, str]]) -> str:
    """渲染中文表：结果 | id | 检查项 | 详情。"""
    rows = [(it["status"], it["id"], it["name"], it["detail"]) for it in items]
    head = ("结果", "id", "检查项", "详情")
    w_status = max([disp_width(head[0])] + [disp_width(r[0]) for r in rows])
    w_id = max([disp_width(head[1])] + [disp_width(r[1]) for r in rows])
    w_name = max([disp_width(head[2])] + [disp_width(r[2]) for r in rows])
    lines = [pad(head[0], w_status) + "  " + pad(head[1], w_id) + "  "
             + pad(head[2], w_name) + "  " + head[3],
             "-" * (w_status + w_id + w_name + 24)]
    for st, cid, name, detail in rows:
        lines.append(pad(st, w_status) + "  " + pad(cid, w_id) + "  "
                     + pad(name, w_name) + "  " + detail)
    return "\n".join(lines)


# ================================================================ 各类检查
def check_python(args) -> List[Dict[str, str]]:
    """python 版本 + 解释器是否就是本仓 .venv（两者不匹配是现场最常见的困惑）。"""
    out = []
    ver = "%d.%d.%d" % sys.version_info[:3]
    out.append(item("py_version", "python 版本", OK if spec_ok(">=3.10", ver) else FAIL,
                    "%s (%s)" % (ver, platform.python_implementation()),
                    "" if spec_ok(">=3.10", ver) else "本仓要求 >= 3.10（requirements.txt 首行）"))

    want = os.path.join(ROOT, ".venv", "bin", "python")
    env_py = os.environ.get("GO2_PYTHON") or ""      # 容器里 = /opt/venv/bin/python
    me = os.path.realpath(sys.executable)
    if env_py and os.path.realpath(env_py) == me:
        # 容器（docker 官方栈）里的正常形态：不是 .venv，但正是 GO2_PYTHON 指的那个
        out.append(item("py_venv", "python 解释器", OK,
                        "%s（来自 GO2_PYTHON）" % sys.executable))
    elif not os.path.isfile(want):
        out.append(item("py_venv", "python 解释器", WARN,
                        "当前 %s；没找到 %s" % (sys.executable, want),
                        "本机没建 venv：python3 -m venv .venv && "
                        ".venv/bin/pip install -r requirements.txt；"
                        "容器里请用 python3（/opt/venv），不要用 .venv/bin/python，并 "
                        "export GO2_PYTHON=/opt/venv/bin/python"))
    elif me == os.path.realpath(want):
        out.append(item("py_venv", "python 解释器", OK, sys.executable))
    else:
        out.append(item("py_venv", "python 解释器", WARN,
                        "当前 %s，期望 %s" % (sys.executable, want),
                        "用 .venv/bin/python 跑本脚本；控制器一律走 scripts/run_with_ros2.sh。 容器里请用 python3（/opt/venv），不要用 .venv/bin/python（宿主 3.13 建的 venv 在容器里 import numpy 会失败）"))
    return out


def check_deps(args) -> List[Dict[str, str]]:
    """numpy / mujoco 版本 vs requirements.txt（用 importlib.metadata，不 import）。"""
    out = []
    reqs = read_requirements(args.requirements)
    fallback = {"numpy": ">=1.26", "mujoco": ">=3.2,<4"}
    for pkg in _PY_DEPS:
        need = reqs.get(pkg) or fallback[pkg]
        try:
            got = importlib.metadata.version(pkg)
        except importlib.metadata.PackageNotFoundError:
            out.append(item("dep_" + pkg, "%s 版本" % pkg, FAIL, "未安装（要求 %s）" % need,
                            ".venv/bin/pip install -r requirements.txt（不要用系统 pip）；"
                            "容器里请用 python3（/opt/venv），不要用 .venv/bin/python"))
            continue
        ok = spec_ok(need, got)
        out.append(item("dep_" + pkg, "%s 版本" % pkg, OK if ok else FAIL,
                        "%s（要求 %s）" % (got, need),
                        "" if ok else "版本不满足：.venv/bin/pip install -r requirements.txt；"
                        "容器里请用 python3（/opt/venv），不要用 .venv/bin/python"))
    return out


def check_mujoco_model(args) -> List[Dict[str, str]]:
    """MuJoCo 能不能真的加载 scene.xml（文件缺失/XML 坏/资源路径错都在这里暴露）。"""
    scene = os.path.join(ROOT, "sim_env", "models", "go2", "scene.xml")
    cfg = _load_cfg(args)
    if cfg:
        scene = os.path.join(ROOT, cfg["model_dir"], cfg["scene"])
    if not os.path.isfile(scene):
        return [item("mujoco_model", "MuJoCo 模型", FAIL, "模型文件不存在: %s" % scene,
                     "按 docs/ros2-setup.md / scripts/fetch_go2_model.sh 取回模型")]
    try:
        import mujoco
    except Exception as exc:                     # ImportError 或 .so 载入失败
        return [item("mujoco_model", "MuJoCo 模型", WARN, "mujoco 不可 import: %r" % (exc,),
                     "先修上面的 dep_mujoco；本项检查会被跳过")]
    try:
        m = mujoco.MjModel.from_xml_path(scene)
    except Exception as exc:
        return [item("mujoco_model", "MuJoCo 模型", FAIL, "加载失败: %r" % (str(exc)[:160],),
                     "检查 %s 引用的 mesh/纹理是否齐、XML 是否被改坏" % scene)]
    return [item("mujoco_model", "MuJoCo 模型", OK,
                 "scene.xml OK (nq=%d, nu=%d, %s)" % (m.nq, m.nu,
                                                      os.path.basename(scene)))]


def _ros_prefixes() -> List[str]:
    """/opt/ros 下所有带 setup.bash 的发行版前缀。"""
    out = []
    for setup in sorted(glob.glob("/opt/ros/*/setup.bash")):
        out.append(os.path.dirname(setup))
    return out


def check_ros_install(args) -> List[Dict[str, str]]:
    """/opt/ros、ros2 CLI、rclpy、rmw_cyclonedds_cpp —— ROS 2 这一半齐不齐。"""
    out = []
    prefixes = _ros_prefixes()
    if prefixes:
        out.append(item("ros2_prefix", "ROS 2 安装", OK,
                        " ".join(os.path.basename(p) for p in prefixes) + " ("
                        + ", ".join(prefixes) + ")"))
    else:
        out.append(item("ros2_prefix", "ROS 2 安装", FAIL, "没找到 /opt/ros/*/setup.bash",
                        "装 ROS 2（docs/ros2-setup.md）或用 docker/ 那套官方栈；"
                        "本仓 .venv 不含 rclpy"))

    ros2_cli = shutil.which("ros2")
    if not prefixes:
        cli_hint = "先装 ROS 2（/opt/ros 都没有），见 docs/ros2-setup.md"
    else:
        cli_hint = "source /opt/ros/<distro>/setup.bash（或直接用 "
        "scripts/run_with_ros2.sh，它替你 source）"
    out.append(item("ros2_cli", "ros2 CLI", OK if ros2_cli else (FAIL if not prefixes else WARN),
                    ros2_cli or "PATH 里没有 ros2",
                    "" if ros2_cli else cli_hint))

    # rclpy / rmw：import 失败分两种情形 —— ROS 在但没 source（WARN，wrapper 能救），
    # 或者压根没装（FAIL）。
    def _imp(mod: str, cid: str, name: str) -> Dict[str, str]:
        try:
            __import__(mod)
            return item(cid, name, OK, "import %s OK" % mod)
        except Exception as exc:
            if not prefixes:
                return item(cid, name, FAIL, "import %s 失败: %r" % (mod, str(exc)[:120]),
                            "本机没有 ROS 2（见 ros2_prefix）")
            return item(cid, name, WARN, "import %s 失败: %r" % (mod, str(exc)[:120]),
                        "shell 没 source ROS 2：用 scripts/run_with_ros2.sh 跑，或先 "
                        "source /opt/ros/<distro>/setup.bash")

    out.append(_imp("rclpy", "rclpy", "rclpy"))

    # rmw_cyclonedds_cpp 是 **C++ 的 rmw 实现**，没有 python 模块可 import
    # （之前这里 __import__('rmw_cyclonedds_cpp') 恒失败，在容器里报过假 WARN）。
    # 正确判据：库文件在不在 —— 它由 ros-<distro>-rmw-cyclonedds-cpp 提供。
    libs: List[str] = []
    for p in list(prefixes) + glob.glob("/opt/ros/*") + [os.environ.get("AMENT_PREFIX_PATH", "")]:
        if not p:
            continue
        libs += glob.glob(os.path.join(p, "lib", "librmw_cyclonedds_cpp.so*"))
    if libs:
        out.append(item("rmw_cyclonedds", "rmw_cyclonedds_cpp", OK,
                        "library OK: " + os.path.basename(sorted(libs)[0])))
    else:
        out.append(item(
            "rmw_cyclonedds", "rmw_cyclonedds_cpp", WARN if prefixes else FAIL,
            "没找到 librmw_cyclonedds_cpp.so",
            "必须用 CycloneDDS：sudo apt install ros-<distro>-rmw-cyclonedds-cpp"
            "（用 fastrtps 收不到固件的 /lowstate）"))
    return out


def check_libddsc(args) -> List[Dict[str, str]]:
    """CycloneDDS 的 C 库（rmw_cyclonedds_cpp 依赖它）；顺便报一下版本。"""
    path = None
    try:
        import ctypes.util
        path = ctypes.util.find_library("ddsc")
    except Exception:
        path = None
    if not path:
        for pat in ("/usr/lib/*/libddsc.so*", "/usr/local/lib/libddsc.so*",
                    "/opt/ros/*/lib/libddsc.so*"):
            hits = sorted(glob.glob(pat))
            if hits:
                path = hits[-1]
                break
    if not path:
        ld = None
        try:
            p = subprocess.run(["ldconfig", "-p"], capture_output=True, text=True, timeout=5)
            for line in (p.stdout or "").splitlines():
                if "libddsc.so" in line:
                    ld = line.split("=>")[-1].strip()
                    break
        except (OSError, subprocess.SubprocessError):
            pass
        path = ld
    if not path:
        return [item("libddsc", "CycloneDDS 库", WARN, "没找到 libddsc",
                     "rmw_cyclonedds_cpp 会依赖它：sudo apt install cyclonedds 或 "
                     "ros-<distro>-rmw-cyclonedds-cpp（上机前必须先装）")]
    ver = ""
    try:
        p = subprocess.run(["dpkg-query", "-W", "-f=${Version}", "cyclonedds"],
                           capture_output=True, text=True, timeout=5)
        if p.returncode == 0 and p.stdout.strip():
            ver = " (%s)" % p.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return [item("libddsc", "CycloneDDS 库", OK, str(path) + ver)]


def find_unitree_go() -> Optional[str]:
    """找一个能让 import unitree_go 生效的来源（多路探测，见模块 docstring §3）。

    顺序（越靠前越可信）：
      1) 当前 shell 的 AMENT_PREFIX_PATH / ROS_PACKAGE_PATH 里含 unitree_go 的段
      2) /opt/ros/*/share/unitree_go
      3) $HOME/unitree_ws/install/share/unitree_go（官方 unitree_ros2 工作区）
      4) 本仓 ros2_ws/install/share/unitree_go（可能是别的 python 版本构建的，容器里要跳过）
      5) ros2 pkg prefix unitree_go（最权威，但要 ros2 CLI，且可能慢）
    Returns:
        命中来源路径（字符串，用于打印）；都没有返回 None。
    """
    for var in ("AMENT_PREFIX_PATH", "ROS_PACKAGE_PATH"):
        for seg in (os.environ.get(var) or "").split(os.pathsep):
            seg = seg.strip()
            if seg and "unitree_go" in seg and os.path.isdir(seg):
                return "%s=%s" % (var, seg)
    for pat in ("/opt/ros/*/share/unitree_go",
                os.path.expanduser("~/unitree_ws/install/share/unitree_go"),
                os.path.join(ROOT, "ros2_ws", "install", "share", "unitree_go")):
        hits = [p for p in sorted(glob.glob(pat)) if os.path.isdir(p)]
        if hits:
            return hits[0]
    if shutil.which("ros2"):
        try:
            p = subprocess.run(["ros2", "pkg", "prefix", "unitree_go"],
                               capture_output=True, text=True, timeout=10)
            if p.returncode == 0 and p.stdout.strip():
                return "ros2 pkg prefix -> " + p.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    return None


def check_unitree_go(args) -> List[Dict[str, str]]:
    """unitree_go 消息包（LowState/LowCmd 的 Python 绑定）是否可见。"""
    src = find_unitree_go()
    if not src:
        return [item("unitree_go", "unitree_go 消息包", FAIL,
                     "没找到 unitree_go（ROS_PACKAGE_PATH/AMENT_PREFIX_PATH、"
                     "/opt/ros/*/share、~/unitree_ws、ros2_ws/install 都没有）",
                     "cd ros2_ws && colcon build --packages-select unitree_go，"
                     "然后 source ros2_ws/install/setup.bash；或装官方 unitree_ros2 后 "
                     "source 它的 install/setup.bash")]
    try:
        __import__("unitree_go.msg")
        return [item("unitree_go", "unitree_go 消息包", OK, "import OK <- %s" % src)]
    except Exception as exc:
        # 找得到但 import 不进来：几乎都是「python 版本/venv 不匹配」（本仓就踩过）
        return [item("unitree_go", "unitree_go 消息包", WARN,
                     "找到 %s 但 import 失败: %r" % (src, str(exc)[:120]),
                     "消息包是用别的 python 版本构建的（容器里用 GO2_SKIP_PROJECT_WS=1 "
                     "跳过本仓 ros2_ws/install）；用 scripts/run_with_ros2.sh 跑，"
                     "它按正确顺序 source")]


def check_ros_env(args, expected: Dict[str, Any]) -> List[Dict[str, str]]:
    """当前 shell 的 ROS/DDS 环境变量是不是真机该有的样子。

    注意：控制器进程里 apply_io_env() 会在 rclpy.init() 之前重设这几个变量，
    所以「shell 里不对」通常是 WARN（手工敲 ros2 topic list 时才会踩）。
    例外是 ROS_LOCALHOST_ONLY=1：apply_io_env 不会 unset 它，必须人工清掉 -> FAIL。
    """
    out = []
    real = str(args.target).startswith("real")
    want_domain = str(expected.get("domain_id", 0))
    got = os.environ.get("ROS_DOMAIN_ID")
    if got is None:
        out.append(item("env_domain_id", "ROS_DOMAIN_ID", WARN,
                        ("未设置（真机应为 %s）" % want_domain) if real else
                        ("未设置（仿真应为 %s）" % want_domain),
                        "控制器会自己设；但手工 ros2 topic list 前请 export "
                        "ROS_DOMAIN_ID=%s，否则看不到真机话题" % want_domain))
    elif got == want_domain:
        out.append(item("env_domain_id", "ROS_DOMAIN_ID", OK, got))
    else:
        out.append(item("env_domain_id", "ROS_DOMAIN_ID", WARN,
                        "%s（本 profile 应为 %s）" % (got, want_domain),
                        "仿真/真机串台：export ROS_DOMAIN_ID=%s" % want_domain))

    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw == "rmw_cyclonedds_cpp":
        out.append(item("env_rmw", "RMW_IMPLEMENTATION", OK, rmw))
    else:
        out.append(item("env_rmw", "RMW_IMPLEMENTATION", WARN,
                        rmw or "未设置",
                        "必须 rmw_cyclonedds_cpp（固件侧是 CycloneDDS）："
                        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"))

    uri = os.environ.get("CYCLONEDDS_URI") or ""
    iface = expected.get("interface") or ""
    if not uri:
        out.append(item("env_cyclonedds_uri", "CYCLONEDDS_URI", WARN,
                        "未设置（多网卡机器会随机选网卡）",
                        "控制器会按 GO2_IFACE/config 自动生成；手工调试时可 "
                        "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>"
                        '<NetworkInterface name="%s"/></Interfaces></General></Domain>'
                        "</CycloneDDS>'" % (iface or "eth0")))
    elif iface and iface in uri:
        out.append(item("env_cyclonedds_uri", "CYCLONEDDS_URI", OK, "已绑定网卡 %s" % iface))
    else:
        out.append(item("env_cyclonedds_uri", "CYCLONEDDS_URI", WARN,
                        "已设置但没提到目标网卡 %r" % iface,
                        "确认绑的是有线网卡，否则可能走 Wi-Fi 收不到 /lowstate"))

    lh = os.environ.get("ROS_LOCALHOST_ONLY")
    if lh in (None, "", "0"):
        out.append(item("env_localhost_only", "ROS_LOCALHOST_ONLY", OK, lh or "未设置"))
    else:
        out.append(item("env_localhost_only", "ROS_LOCALHOST_ONLY", FAIL,
                        "%s（只走回环）" % lh,
                        "unset ROS_LOCALHOST_ONLY —— 否则真机一定收不到 /lowstate，"
                        "且 apply_io_env 不会替你清"))
    return out


def check_iface(args, expected: Dict[str, Any], iface_source: str) -> List[Dict[str, str]]:
    """目标网卡存不存在、UP 没有、是不是 192.168.123.0/24（真机必查）。"""
    sys.path.insert(0, ROOT)
    try:
        from signal_io.config import detect_iface, list_interfaces
    except Exception as exc:
        return [item("iface_exists", "目标网卡", SKIP, "无法 import signal_io.config: %r"
                     % (str(exc)[:100],), "先解决 numpy/仓库路径问题")]
    iface = args.iface or expected.get("interface") or "eth0"
    recs = list_interfaces()
    names = [r["name"] for r in recs]
    rec = next((r for r in recs if r["name"] == iface), None)

    if not rec:
        guess = detect_iface()
        out = [item("iface_exists", "目标网卡 %s" % iface, FAIL,
                    "不存在（本机网卡: %s）" % (", ".join(names) or "无"),
                    ("自动探测建议用 %s：export GO2_IFACE=%s，或 "
                     "scripts/real_run.sh --iface %s" % (guess, guess, guess)) if guess else
                    "先 ip -br addr 看有线口叫什么，再 --iface 指定")]
        out.append(item("iface_state", "网卡状态", SKIP, "网卡不存在，跳过"))
    else:
        # lo 的 operstate 在 ip -br 里显示 UNKNOWN（内核里它是 UP 的），不能当故障；
        # 有线口才是真的 UP/DOWN。
        up = iface == "lo" or rec.get("state") == "UP"
        out = [item("iface_exists", "目标网卡 %s" % iface, OK,
                    "存在（state=%s，来源=%s）" % (rec.get("state"), iface_source))]
        if iface == "lo":
            out.append(item("iface_state", "网卡状态 lo", OK, "回环（仿真用）"))
        else:
            out.append(item("iface_state", "网卡状态 %s" % iface, OK if up else FAIL,
                            "UP" if up else
                            "%s（网线没插 / 对端没上电 / 没 ip link set up）"
                            % rec.get("state"),
                            "" if up else "插网线并 sudo ip link set %s up" % iface))

    if str(args.target).startswith("sim"):
        out.append(item("iface_subnet", "机器人网段", SKIP, "仿真目标不检查 192.168.123.0/24"))
        return out
    if args.no_network:
        out.append(item("iface_subnet", "机器人网段", SKIP, "--no-network 跳过"))
        return out
    if rec:
        addrs = ["%s/%d" % (a, p) for a, p in rec.get("ipv4") or []]
        in_net = any(a.startswith(ROBOT_NET_PREFIX) for a, _ in rec.get("ipv4") or [])
        out.append(item("iface_subnet", "机器人网段 %s" % iface, OK if in_net else WARN,
                        ("IPv4: " + (", ".join(addrs) or "无")) +
                        ("" if in_net else "（不在 192.168.123.0/24）"),
                        "" if in_net else "sudo ip addr add 192.168.123.99/24 dev %s" % iface))
    return out


def check_ping(args) -> List[Dict[str, str]]:
    """ping 机器人运控地址（唯一会发网络包的检查项）。"""
    if str(args.target).startswith("sim"):
        return [item("ping_robot", "ping 机器人", SKIP, "仿真目标不 ping")]
    if args.no_network:
        return [item("ping_robot", "ping %s" % ROBOT_IP, SKIP, "--no-network 跳过")]
    if not shutil.which("ping"):
        return [item("ping_robot", "ping 机器人", SKIP, "本机没有 ping 命令")]
    t0 = time.time()
    try:
        p = subprocess.run(["ping", "-c", "1", "-W", "2", ROBOT_IP],
                           capture_output=True, text=True, timeout=6)
        rc = p.returncode
    except (OSError, subprocess.SubprocessError) as exc:
        return [item("ping_robot", "ping 机器人", SKIP, "ping 执行失败: %r" % (str(exc)[:100],))]
    dt = (time.time() - t0) * 1e3
    if rc == 0:
        return [item("ping_robot", "ping %s" % ROBOT_IP, OK, "通（%.1f ms）" % dt)]
    return [item("ping_robot", "ping %s" % ROBOT_IP, FAIL, "不通（rc=%d, %.0f ms）" % (rc, dt),
                 "查网线/机器人是否上电/PC 是否 192.168.123.x/IP 冲突。"
                 "注意 ICMP 可能被过滤，ping 不通不代表 DDS 一定不通，但上机前应先修它")]


def check_host(args) -> List[Dict[str, str]]:
    """CPU / 核数 / 实时优先级上限 / 磁盘（都是提示性质，除磁盘太满外不阻断）。"""
    out = []
    model = platform.processor() or ""
    try:
        with open("/proc/cpuinfo", "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith("model name") and not model:
                    model = line.split(":", 1)[1].strip()
    except OSError:
        pass
    cores = os.cpu_count() or 0
    out.append(item("cpu", "CPU / 核数", OK if cores >= 4 else WARN,
                    "%s x%d" % (model or "未知", cores),
                    "" if cores >= 4 else "500 Hz 控制 + 观测构建建议 >= 4 核"))

    rt = None
    try:
        import resource
        rt = resource.getrlimit(resource.RLIMIT_RTPRIO)
    except Exception:
        rt = None
    if rt is None:
        out.append(item("rtprio", "实时优先级 ulimit -r", SKIP, "本平台不支持查询"))
    else:
        soft = rt[0]
        out.append(item("rtprio", "实时优先级 ulimit -r", OK if soft > 0 else WARN,
                        "soft=%s hard=%s%s" % (soft, rt[1],
                                               "（0 = 用不了 chrt）" if soft == 0 else ""),
                        "" if soft > 0 else "本仓默认不用 SCHED_FIFO（500 Hz 靠墙钟节拍），"
                        "一般不影响；若现场抖动明显再考虑 limits.conf 提 rtprio"))

    try:
        du = shutil.disk_usage(ROOT)
        free_gib = du.free / (1024.0 ** 3)
        out.append(item("disk", "磁盘剩余", OK if free_gib >= MIN_DISK_GIB else WARN,
                        "%.1f GiB 可用（%s）" % (free_gib, ROOT),
                        "" if free_gib >= MIN_DISK_GIB else "清 logs/ 与 build/，"
                        "真机排障靠日志，别把盘写满"))
    except OSError as exc:
        out.append(item("disk", "磁盘剩余", SKIP, "查询失败: %r" % (str(exc)[:80],)))
    return out


def check_git(args) -> List[Dict[str, str]]:
    """git commit + 是否 dirty（上机前要能对上是哪份代码）。"""
    if not shutil.which("git") or not os.path.isdir(os.path.join(ROOT, ".git")):
        return [item("git", "git 版本信息", SKIP, "不是 git 仓库或没有 git")]
    try:
        head = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, timeout=5)
        status = subprocess.run(["git", "-C", ROOT, "status", "--porcelain"],
                                capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
        return [item("git", "git 版本信息", SKIP, "git 执行失败: %r" % (str(exc)[:80],))]
    sha = (head.stdout or "").strip()
    dirty = [l for l in (status.stdout or "").splitlines() if l.strip()]
    if head.returncode != 0 or not sha:
        return [item("git", "git 版本信息", SKIP, "读不到 HEAD")]
    if dirty:
        return [item("git", "git 版本信息", WARN, "%s，%d 处未提交改动" % (sha, len(dirty)),
                     "确认这些改动是有意的（上机出问题要能对上代码）：git status --short")]
    return [item("git", "git 版本信息", OK, "%s（干净）" % sha)]


def check_exec_bits(args) -> List[Dict[str, str]]:
    """上机要直接调用的脚本有没有可执行位（现场 chmod 会很尴尬）。"""
    names = ["scripts/run_with_ros2.sh", "scripts/real_run.sh", "scripts/real_sensor_check.py",
             "scripts/go2_real_check.py", "scripts/env_check.py"]
    bad, ok, missing = [], [], []
    for rel in names:
        p = os.path.join(ROOT, rel)
        if not os.path.isfile(p):
            missing.append(rel)
            continue
        (ok if os.access(p, os.X_OK) else bad).append(rel)
    if bad:
        return [item("exec_bits", "脚本可执行位", WARN, "缺可执行位: " + ", ".join(bad),
                     "chmod +x " + " ".join(bad))]
    return [item("exec_bits", "脚本可执行位", OK,
                 "%d 个脚本可执行%s" % (len(ok), ("（未生成: %s）" % ", ".join(missing))
                                       if missing else ""))]


# ================================================================ 主流程
def _load_cfg(args) -> Optional[Dict[str, Any]]:
    """尽力加载配置拿期望值（domain/interface）；失败返回 None（用内置兜底）。"""
    path = args.config if args.config and os.path.isfile(args.config) else None
    try:
        sys.path.insert(0, ROOT)
        from signal_io.config import load_config
        return load_config(path, target=args.target)
    except Exception:
        return None


def run_checks(args) -> tuple:
    """跑全部检查，返回 (items, expected, iface, iface_source)。"""
    cfg = _load_cfg(args)
    if cfg:
        expected = dict(cfg.get("transport") or {})
        iface_source = "config"
    else:
        expected = ({"name": "ros2", "domain_id": 0, "interface": "eth0"}
                    if str(args.target).startswith("real")
                    else {"name": "ros2", "domain_id": 1, "interface": "lo"})
        iface_source = "fallback"
    if args.iface:
        expected["interface"] = args.iface
        iface_source = "--iface"
    elif not cfg:
        # 配置读不出来时，至少试试自动探测，避免把 "eth0" 当成事实报错
        try:
            from signal_io.config import detect_iface
            guess = detect_iface()
        except Exception:
            guess = None
        if guess:
            expected["interface"] = guess
            iface_source = "auto"

    items: List[Dict[str, str]] = []
    items += check_python(args)
    items += check_deps(args)
    items += check_mujoco_model(args)
    items += check_ros_install(args)
    items += check_libddsc(args)
    items += check_unitree_go(args)
    items += check_ros_env(args, expected)
    items += check_iface(args, expected, iface_source)
    items += check_ping(args)
    items += check_host(args)
    items += check_git(args)
    items += check_exec_bits(args)
    return items, expected, str(expected.get("interface") or ""), iface_source


def print_report(items: List[Dict[str, str]], rep: Dict[str, Any]) -> None:
    """人看的输出：表 + 汇总 + 每条 FAIL/WARN 的修法。"""
    print("=" * 78)
    print("Go2 上机环境体检  target=%s  iface=%s（来源 %s）  host=%s"
          % (rep["target"], rep["iface"], rep["iface_source"], rep["host"]))
    print("python=%s  %s" % (rep["python"], rep["ts"]))
    print("=" * 78)
    print(format_table(items))
    counts = rep["counts"]
    print()
    print("---- 汇总: OK %d / WARN %d / FAIL %d / SKIP %d ----"
          % (counts[OK], counts[WARN], counts[FAIL], counts[SKIP]))
    bad = [it for it in items if it["status"] in (FAIL, WARN)]
    if bad:
        print()
        print("---- 需要处理（FAIL 现在上机不安全，WARN 建议先修）----")
        for it in bad:
            print("[%s] %s (%s): %s" % (it["status"], it["name"], it["id"], it["detail"]))
            if it.get("hint"):
                print("        修: %s" % it["hint"])
    if rep["exit_code"] == 0:
        print()
        print("结论：通过。可以继续 scripts/real_run.sh 的 zero 阶段（机器人先吊起）。")
    else:
        print()
        print("结论：不通过（退出码 %d）。先按上面的「修」逐条处理；"
              "真机现场确需硬上时 real_run.sh 要显式 --force。" % rep["exit_code"])


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Go2 上机前环境体检（只读；真机/仿真通用）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例: .venv/bin/python scripts/env_check.py --target real_ros --no-network")
    ap.add_argument("--target", default="real_ros",
                    choices=["real_ros", "sim_ros", "real_sdk", "sim_sdk"],
                    help="期望的 IO profile（决定 domain/网卡/是否 ping 机器人）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "go2_sim.json"),
                    help="配置文件（读 io_profiles 的期望 domain/interface）")
    ap.add_argument("--requirements", default=os.path.join(ROOT, "requirements.txt"),
                    help="依赖版本要求（默认 requirements.txt；缺失时用内置兜底）")
    ap.add_argument("--iface", default=None, help="目标网卡名（默认取 config；上机脚本会传）")
    ap.add_argument("--no-network", action="store_true",
                    help="不发任何网络包：跳过 ping 和 192.168.123.0/24 检查")
    ap.add_argument("--json", nargs="?", const="-", default=None, metavar="PATH",
                    help="把结果写成 JSON（'--json -' 只输出 JSON；给路径时表格照常打印）")
    ap.add_argument("--strict", action="store_true", help="WARN 也算失败（退出码 1）")
    args = ap.parse_args()

    items, _expected, iface, iface_source = run_checks(args)
    rep = report_dict(items, args.target, iface, iface_source, args.strict,
                      args.no_network,
                      extra={"robot_ip": ROBOT_IP, "root": ROOT,
                             "versions": {"python": "%d.%d.%d" % sys.version_info[:3]}})

    if args.json == "-":
        print(json.dumps(rep, ensure_ascii=False, indent=2))
    else:
        print_report(items, rep)
        if args.json:
            out = os.path.abspath(args.json)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w", encoding="utf-8") as f:
                json.dump(rep, f, ensure_ascii=False, indent=2)
            print("\nJSON 已写入: %s" % out)
    return rep["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
