#!/usr/bin/env python3
"""tests/test_env_check.py —— 环境体检脚本（scripts/env_check.py）的纯逻辑验收。

为什么单独测这些函数而不是「跑一遍 env_check 看输出」：
    * env_check 在没装 ROS 2 / 没插网线的机器上必然有 FAIL，跑真实检查没法断言；
    * 真正容易写错、也最容易悄悄退化的是这几块纯逻辑：
        1. 版本比较（requirements.txt 的 "mujoco>=3.2,<4" 这类条件）；
        2. ip -br addr 的解析（DOWN 没地址、容器里 eth0@if12、IPv6 混排）；
        3. FAIL/WARN -> 退出码 的汇总约定（real_run.sh 的安全闸门就挂在这上面）；
        4. --json 的结构（real_run.sh / 其它工具按字段名读，不能乱改）；
        5. 中文表格的宽度计算（只是不崩 + 不出乱码，不比对具体排版）。

覆盖：
  1. version_tuple / spec_ok：单条件、多条件、上下限、~=、非法 spec；
  2. parse_requirements：注释 / 空行 / -r / extra / 下划线归一；
  3. parse_ip_addr + pick_iface + interface_in_subnet（fixture 文本，不碰网卡）；
  4. summarize / exit_code_for：FAIL -> 1，WARN 只在 --strict 下 -> 1；
  5. report_dict：字段齐全、counts 一致、exit_code 与约定一致；
  6. format_table / pad / disp_width：中英文混排不崩；
  7. 脚本本体：--help 可用（不跑检查、不碰网络）、--json - 输出可解析。

直接运行：.venv/bin/python tests/test_env_check.py
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def load_env_check():
    """按文件路径加载 scripts/env_check.py（scripts/ 不是包，没有 __init__.py）。"""
    path = os.path.join(ROOT, "scripts", "env_check.py")
    spec = importlib.util.spec_from_file_location("go2_env_check", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# fixture：本机真实输出的一种（enp0s31f6 是 DOWN 且没有地址；容器里会带 @if12）
IP_BR_ADDR_FIXTURE = """\
lo               UNKNOWN        127.0.0.1/8 ::1/128
enp0s31f6        DOWN
eth0@if12        UP             192.168.123.99/24 fe80::1/64
wlp0s20f3        UP             10.236.67.3/16 2001:da8::3/128
docker0          DOWN           172.17.0.1/16
"""


def main() -> int:
    ec = load_env_check()
    from signal_io.config import (ROBOT_NET_PREFIX, interface_in_subnet,  # noqa: E402
                                  parse_ip_addr, pick_iface)

    # ------------------------------------------------------------ 1. 版本比较
    print("== 1. 版本比较 ==")
    check("version_tuple('3.13.7')", ec.version_tuple("3.13.7") == (3, 13, 7),
          str(ec.version_tuple("3.13.7")))
    check("version_tuple('2.5.4.dev0') 截断", ec.version_tuple("2.5.4.dev0") == (2, 5, 4),
          str(ec.version_tuple("2.5.4.dev0")))
    check("version_tuple('3.2.0rc1') 截断", ec.version_tuple("3.2.0rc1") == (3, 2, 0))
    check("version_tuple('') 安全", ec.version_tuple("") == ())
    check("spec_ok('>=3.2,<4', '3.17.0')",
          ec.spec_ok(">=3.2,<4", "3.17.0") is True)
    check("spec_ok('>=3.2,<4', '4.0.1') 上限生效",
          ec.spec_ok(">=3.2,<4", "4.0.1") is False)
    check("spec_ok('>=3.2,<4', '3.2') 下限含等号",
          ec.spec_ok(">=3.2,<4", "3.2") is True)
    check("spec_ok('>=3.2,<4', '3.1.9') 下限生效",
          ec.spec_ok(">=3.2,<4", "3.1.9") is False)
    check("spec_ok('>=1.26', '2.5.4')", ec.spec_ok(">=1.26", "2.5.4") is True)
    check("spec_ok('>=1.26', '1.25.2')", ec.spec_ok(">=1.26", "1.25.2") is False)
    check("spec_ok('>=3.10', '3.9.18') python 版本下限",
          ec.spec_ok(">=3.10", "3.9.18") is False)
    check("spec_ok('>=3.10', '3.13.7')", ec.spec_ok(">=3.10", "3.13.7") is True)
    check("spec_ok('~=3.2', '3.9.0') 按 >= 保守处理",
          ec.spec_ok("~=3.2", "3.9.0") is True)
    check("spec_ok('', '1.0') 空条件算满足", ec.spec_ok("", "1.0") is True)
    check("spec_ok('>=x.y', '1.0') 非法条件 -> False",
          ec.spec_ok(">=x.y", "1.0") is False)

    # ------------------------------------------------------------ 2. requirements 解析
    print("== 2. requirements.txt 解析 ==")
    req_text = """\
# 注释不算
mujoco>=3.2,<4
numpy >= 1.26   # 行尾注释
-r other.txt
some_extra[pandas]>=1.0
  """
    reqs = ec.parse_requirements(req_text)
    check("mujoco 条件", reqs.get("mujoco") == ">=3.2,<4", repr(reqs.get("mujoco")))
    check("numpy 条件（去掉空格与注释）", reqs.get("numpy") == ">=1.26",
          repr(reqs.get("numpy")))
    check("-r 行被忽略", all(not k.startswith("-") for k in reqs))
    check("_ 归一成 - 并保留条件", reqs.get("some-extra") == ">=1.0",
          repr(reqs.get("some-extra")))
    check("只有 3 个有效包", len(reqs) == 3, repr(sorted(reqs)))

    # ------------------------------------------------------------ 3. ip -br addr 解析
    print("== 3. ip -br addr 解析（fixture，不碰网卡）==")
    recs = parse_ip_addr(IP_BR_ADDR_FIXTURE)
    by = {r["name"]: r for r in recs}
    check("解析出 5 个网卡", len(recs) == 5, str([r["name"] for r in recs]))
    check("容器网卡名剥掉 @if12", "eth0" in by and "eth0@if12" not in by)
    check("DOWN 且无地址的网卡也能解析",
          by["enp0s31f6"]["state"] == "DOWN" and by["enp0s31f6"]["ipv4"] == [])
    check("IPv4 地址与前缀", by["eth0"]["ipv4"] == [("192.168.123.99", 24)],
          repr(by["eth0"]["ipv4"]))
    check("IPv6 单独归类（lo）",
          ("::1", 128) in by["lo"]["ipv6"] and by["lo"]["ipv4"] == [("127.0.0.1", 8)])
    check("无线网卡双栈",
          by["wlp0s20f3"]["ipv4"] == [("10.236.67.3", 16)]
          and len(by["wlp0s20f3"]["ipv6"]) == 1)
    check("空输入安全", parse_ip_addr("") == [] and parse_ip_addr(None) == [])
    check("杂乱行不崩", parse_ip_addr("garbage\n\n  x\n") == [])

    check("interface_in_subnet(eth0) 命中机器人网段",
          interface_in_subnet(by["eth0"]) is True)
    check("interface_in_subnet(wlan) 不命中",
          interface_in_subnet(by["wlp0s20f3"]) is False)
    check("ROBOT_NET_PREFIX 与 env_check 常量一致",
          ec.ROBOT_NET_PREFIX == ROBOT_NET_PREFIX == "192.168.123.")
    check("pick_iface 优先选 192.168.123.0/24 的 UP 网卡",
          pick_iface(recs) == "eth0", str(pick_iface(recs)))
    check("pick_iface 不选 DOWN / 不选无线 / 不选 lo",
          pick_iface([{"name": "enp0s31f6", "state": "DOWN", "ipv4": [], "ipv6": []},
                      {"name": "lo", "state": "UP", "ipv4": [("127.0.0.1", 8)],
                       "ipv6": []},
                      {"name": "wlp0s20f3", "state": "UP",
                       "ipv4": [("10.0.0.2", 24)], "ipv6": []}]) is None)
    check("pick_iface 退化时选非无线 UP 网卡",
          pick_iface([{"name": "wlp0s20f3", "state": "UP",
                       "ipv4": [("10.0.0.2", 24)], "ipv6": []},
                      {"name": "enp1s0", "state": "UP",
                       "ipv4": [("10.0.1.2", 24)], "ipv6": []}]) == "enp1s0")
    check("pick_iface 空列表 -> None", pick_iface([]) is None)

    # ------------------------------------------------------------ 4. 汇总 / 退出码
    print("== 4. FAIL/WARN 汇总与退出码 ==")
    mk = ec.item
    all_ok = [mk("a", "A", ec.OK, "x"), mk("b", "B", ec.OK, "x"),
              mk("c", "C", ec.SKIP, "x")]
    with_warn = all_ok + [mk("d", "D", ec.WARN, "x")]
    with_fail = with_warn + [mk("e", "E", ec.FAIL, "x")]
    check("全 OK 退出码 0", ec.exit_code_for(all_ok) == 0)
    check("有 WARN 默认仍 0", ec.exit_code_for(with_warn) == 0)
    check("有 WARN + --strict -> 1", ec.exit_code_for(with_warn, strict=True) == 1)
    check("有 FAIL -> 1", ec.exit_code_for(with_fail) == 1)
    check("有 FAIL + --strict -> 1", ec.exit_code_for(with_fail, strict=True) == 1)
    check("SKIP 不影响退出码", ec.exit_code_for(all_ok, strict=True) == 0)
    check("counts 统计正确", ec.summarize(with_fail) ==
          {ec.OK: 2, ec.WARN: 1, ec.FAIL: 1, ec.SKIP: 1}, str(ec.summarize(with_fail)))

    # ------------------------------------------------------------ 5. JSON 结构
    print("== 5. --json 结构 ==")
    rep = ec.report_dict(with_fail, "real_ros", "eth0", "config", False, True)
    need = {"tool", "schema", "ts", "host", "python", "target", "iface", "iface_source",
            "strict", "no_network", "counts", "exit_code", "items", "extra"}
    check("字段齐全", need <= set(rep), str(sorted(need - set(rep))))
    check("tool/schema 固定", rep["tool"] == "env_check" and rep["schema"] == 1)
    check("target/iface 透传", rep["target"] == "real_ros" and rep["iface"] == "eth0")
    check("counts 与 items 一致", rep["counts"] == ec.summarize(with_fail))
    check("exit_code 与约定一致", rep["exit_code"] == ec.exit_code_for(with_fail))
    check("items 每项字段齐全",
          all({"id", "name", "status", "detail", "hint"} <= set(it) for it in rep["items"]))
    check("status 取值合法",
          all(it["status"] in (ec.OK, ec.WARN, ec.FAIL, ec.SKIP) for it in rep["items"]))
    check("可 JSON 序列化（ensure_ascii=False）",
          isinstance(json.dumps(rep, ensure_ascii=False), str))

    # ------------------------------------------------------------ 6. 中文表格
    print("== 6. 中文表格宽度 ==")
    check("全角字符算 2 列", ec.disp_width("中文") == 4 and ec.disp_width("ab") == 2)
    check("pad 按显示宽度补齐", ec.disp_width(ec.pad("中文", 8)) == 8)
    check("pad 不截断超长内容", ec.pad("很长很长很长的中文", 4) == "很长很长很长的中文")
    table = ec.format_table(with_fail)
    check("表格含表头与每行 id",
          "检查项" in table and all(it["id"] in table for it in with_fail))
    check("空列表不崩", isinstance(ec.format_table([]), str))

    # ------------------------------------------------------------ 7. 脚本本体可运行
    print("== 7. 脚本本体（不跑真实检查）==")
    script = os.path.join(ROOT, "scripts", "env_check.py")
    p = subprocess.run([sys.executable, script, "--help"], capture_output=True, text=True)
    check("--help 退出码 0", p.returncode == 0, p.stderr.strip()[-200:])
    for opt in ("--target", "--iface", "--no-network", "--json", "--strict"):
        check("--help 里有 %s" % opt, opt in (p.stdout or ""))
    check("脚本有可执行位", os.access(script, os.X_OK))

    # 用仿真目标 + --no-network 跑一遍：唯一一次真实执行，但**不发任何网络包**
    # （sim_ros 目标本来就跳过 ping 与网段检查）。只断言 JSON 契约，不断言具体 OK/FAIL。
    p = subprocess.run([sys.executable, script, "--target", "sim_ros", "--no-network",
                        "--json", "-"], capture_output=True, text=True, timeout=120)
    ok_json = False
    try:
        got = json.loads(p.stdout)
        ok_json = (got.get("tool") == "env_check"
                   and got.get("exit_code") in (0, 1)
                   and got["counts"] == ec.summarize(got["items"])
                   and p.returncode == got["exit_code"])
    except Exception as exc:                 # JSON 坏 / 字段缺
        print("    解析失败: %r" % (str(exc)[:160],))
        print("    stderr:", (p.stderr or "").strip()[-300:])
    check("--json - 输出可解析且退出码=exit_code", ok_json)
    check("items 数量合理（>= 15 项）", ok_json and len(got["items"]) >= 15,
          str(len(got.get("items", []))) if ok_json else "n/a")

    print()
    if FAILS:
        print("FAILED: %d 项 -> %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("OK: env_check 纯逻辑 + JSON 契约 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
