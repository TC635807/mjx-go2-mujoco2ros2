#!/usr/bin/env python3
"""tests/test_go2_crc.py —— Go2 真机 LowCmd 的 812 字节布局 + CRC32 验收。

背景（真机固件行为，见 unitree_ros2 / unitree_sdk2_python）：
  * Go2 的 LowCmd 必须是 812 字节的 unitree_go::msg::dds_::LowCmd_ 布局；
  * 固件校验 `crc`：crc32_core(前 808 字节按 202 个 uint32)；
  * 下发前必须设置 head=FE EF、level_flag=0xFF（官方 low-level 例程一致）。

覆盖：
  1. 布局长度 = 812，crc 覆盖 202 字；
  2. 4 组 gold value（由 Unitree 官方 crc_amd64.so 对同一布局算出）；
  3. 与官方 C 实现交叉验证（third_party 里的 .so 存在时）；
  4. 逐字节可复现：同一指令两次打包完全一致；改任意一个电机字段 CRC 必变；
  5. 头部字段（head/level_flag/gpio）参与 CRC。

直接运行：python tests/test_go2_crc.py
"""
from __future__ import annotations

import ctypes
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from signal_io.go2_crc import (  # noqa: E402
    LOWCMD_CRC_WORDS, LOWCMD_PACK_FMT, LOWCMD_SIZE, REFERENCE_CRC,
    lowcmd_crc, lowcmd_with_crc, pack_lowcmd,
)
from signal_io.msg import LowCmd, MotorCmd, crc32_core  # noqa: E402

FAILS = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def make_cmd(head=(0xFE, 0xEF), level=0xFF, motors=None) -> LowCmd:
    c = LowCmd(head=[head[0], head[1]], level_flag=level)
    for i in range(20):
        m = (motors or {}).get(i, (1, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
        c.motor_cmd[i] = MotorCmd(*m)
    return c


def official_default() -> LowCmd:
    return make_cmd(motors={i: (1, 2.146e9, 0.0, 0.0, 0.0, 0.0, 0) for i in range(20)})


def stand_cmd() -> LowCmd:
    motors = {i: (1, 0.0, 0.0, 0.0, 20.0, 1.0, 0) for i in range(12)}
    return make_cmd(motors=motors)


def main() -> int:
    print("== 1. 布局 ==")
    check("LowCmd 布局 812 字节", LOWCMD_SIZE == 812, str(LOWCMD_SIZE))
    check("CRC 覆盖 202 字 (808 字节)", LOWCMD_CRC_WORDS == 202,
          str(LOWCMD_CRC_WORDS))
    check("struct 格式长度 812", struct.calcsize(LOWCMD_PACK_FMT) == 812,
          str(struct.calcsize(LOWCMD_PACK_FMT)))
    check("打包长度 812", len(pack_lowcmd(official_default())) == 812)

    print("== 2. 官方 gold value ==")
    zero = LowCmd(head=[0, 0], level_flag=0)
    for i in range(20):
        zero.motor_cmd[i] = MotorCmd(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
    vectors = {
        "zeros": zero,
        "official_default": official_default(),
        "mode1_q0_all20": make_cmd(),
        "stand_kp20_kd1": stand_cmd(),
    }
    for name, cmd in vectors.items():
        got = lowcmd_crc(cmd)
        want = REFERENCE_CRC[name]
        check("CRC[%s] = 0x%08X" % (name, want), got == want, "got 0x%08X" % got)

    print("== 3. 与官方 C 实现交叉验证 ==")
    so = os.path.join(ROOT, "third_party", "unitree_sdk2_python", "unitree_sdk2py",
                      "utils", "lib", "crc_amd64.so")
    if not os.path.isfile(so):
        print("  [SKIP] 没有 %s（不影响其它检查）" % so)
    else:
        lib = ctypes.CDLL(os.path.abspath(so))
        lib.crc32_core.argtypes = (ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32)
        lib.crc32_core.restype = ctypes.c_uint32
        for name, cmd in vectors.items():
            buf = pack_lowcmd(cmd, crc=0)
            words = (ctypes.c_uint32 * LOWCMD_CRC_WORDS)(
                *struct.unpack("<%dI" % LOWCMD_CRC_WORDS,
                               buf[:LOWCMD_CRC_WORDS * 4]))
            c_crc = int(lib.crc32_core(words, LOWCMD_CRC_WORDS))
            check("C 库 CRC[%s] 与 Python 一致" % name, c_crc == lowcmd_crc(cmd),
                  "C=0x%08X py=0x%08X" % (c_crc, lowcmd_crc(cmd)))

    print("== 4. 确定性与敏感性 ==")
    a = stand_cmd()
    b = stand_cmd()
    check("同一指令打包逐字节一致", lowcmd_with_crc(a) == lowcmd_with_crc(b))
    check("crc 字段在最后 4 字节",
          lowcmd_with_crc(a)[-4:] == int(lowcmd_crc(a)).to_bytes(4, "little"))
    base = lowcmd_crc(stand_cmd())
    changed_q = stand_cmd()
    changed_q.motor_cmd[5].q = 0.01
    changed_tau = stand_cmd()
    changed_tau.motor_cmd[11].tau = 0.5
    changed_kp = stand_cmd()
    changed_kp.motor_cmd[0].kp = 21.0
    for name, c in (("q", changed_q), ("tau", changed_tau), ("kp", changed_kp)):
        check("改 motor_cmd.%s 后 CRC 变化" % name, lowcmd_crc(c) != base)

    print("== 5. 固定头参与 CRC ==")
    h1 = make_cmd(head=(0xFE, 0xEF), level=0xFF)
    h2 = make_cmd(head=(0x00, 0x00), level=0xFF)
    h3 = make_cmd(head=(0xFE, 0xEF), level=0xEE)
    h4 = make_cmd(head=(0xFE, 0xEF), level=0xFF)
    h4.gpio = 1
    check("head 改变 -> CRC 改变", lowcmd_crc(h1) != lowcmd_crc(h2))
    check("level_flag 改变 -> CRC 改变", lowcmd_crc(h1) != lowcmd_crc(h3))
    check("gpio 改变 -> CRC 改变", lowcmd_crc(h1) != lowcmd_crc(h4))

    print("== 6. 与 msg.py 自带 crc32_core 同源 ==")
    raw = pack_lowcmd(a, crc=0)
    check("crc32_core(pack) == lowcmd_crc",
          crc32_core(raw, LOWCMD_CRC_WORDS) == lowcmd_crc(a))

    print()
    if FAILS:
        print("FAILED: %d 项 -> %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("OK: Go2 LowCmd 布局/CRC 全部通过（%d 项）" % 30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
