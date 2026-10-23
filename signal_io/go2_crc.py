"""Go2 `unitree_go::msg::dds_::LowCmd_` 的**真机字节布局**与 CRC32。

为什么单独一个模块:
  * 真机固件校验 `LowCmd.crc`。CRC 覆盖的是 DDS IDL 结构体的**内存布局**
    （812 字节，去掉最后 4 字节 crc 后按 uint32 取 202 个字），
    不是 ROS 2 CDR 序列化后的字节。
  * unitree_sdk2py 里这份布局是硬编码的：
    `unitree_sdk2_python/unitree_sdk2py/utils/crc.py` 的
    `__packFmtLowCmd = '<4B4IH2x' + 'B3x5f3I' * 20 + '4B' + '55Bx2I'`；
    unitree_ros2 里是 `example/src/common/motor_crc.cpp` 的
    `crc32_core((uint32_t*)&raw, (sizeof(LowCmd) >> 2) - 1)`。
    两边逐字节一致，本模块照抄同一布局，因此不依赖 cyclonedds / unitree_sdk2py。

字段顺序（与 unitree_go/msg/LowCmd.msg 一致）:
    head[2] level_flag frame_reserve sn[2] version[2] bandwidth
    motor_cmd[20] bms_cmd wireless_remote[40] led[12] fan[2] gpio
    reserve crc
"""

from __future__ import annotations

import struct

from .msg import NUM_MOTOR, LowCmd, crc32_core

# 812 = sizeof(unitree_go::msg::dds_::LowCmd_)，uint32 对齐
LOWCMD_SIZE = 812
LOWCMD_CRC_WORDS = (LOWCMD_SIZE >> 2) - 1        # 202 个字 / 808 字节

# '<4B4IH2x' = head[2], level_flag, frame_reserve, sn[2], version[2], bandwidth, pad
# 'B3x5f3I' * 20 = 每个 MotorCmd_: mode, pad3, q, dq, tau, kp, kd, reserve[3]
# '4B' = bms_cmd (off + reserve[3])
# '55Bx2I' = wireless_remote[40] + led[12] + fan[2] + gpio, pad1, reserve, crc
LOWCMD_PACK_FMT = "<4B4IH2x" + "B3x5f3I" * NUM_MOTOR + "4B" + "55Bx2I"
assert 4 + 4 * 4 + 2 + 2 + NUM_MOTOR * 36 + 4 + 55 + 1 + 8 == LOWCMD_SIZE

# 官方 low-level 例程的固定头（unitree_ros2 go2_stand_example.cpp / sdk2py go2_stand_example.py）
HEAD = (0xFE, 0xEF)
LEVEL_FLAG_LOWLEVEL = 0xFF
GPIO_DEFAULT = 0


def pack_lowcmd(cmd: LowCmd, crc: int = 0) -> bytes:
    """把内部 LowCmd 打包成 812 字节的真机布局（crc 字段放在最后）。"""
    head = cmd.head if getattr(cmd, "head", None) else HEAD
    level_flag = int(getattr(cmd, "level_flag", LEVEL_FLAG_LOWLEVEL))
    gpio = int(getattr(cmd, "gpio", GPIO_DEFAULT))
    vals: list = [int(head[0]) & 0xFF, int(head[1]) & 0xFF,
                  level_flag & 0xFF, int(getattr(cmd, "frame_reserve", 0)) & 0xFF,
                  0, 0, 0, 0,          # sn[2], version[2]
                  0]                   # bandwidth
    for i in range(NUM_MOTOR):
        m = cmd.motor_cmd[i]
        vals += [int(m.mode) & 0xFF, float(m.q), float(m.dq), float(m.tau),
                 float(m.kp), float(m.kd),
                 int(getattr(m, "reserve", 0)) & 0xFFFFFFFF, 0, 0]
    vals += [0, 0, 0, 0]           # bms_cmd.off + reserve[3]
    vals += [0] * 54 + [gpio]      # wireless_remote[40] + led[12] + fan[2] + gpio
    vals += [0, int(crc) & 0xFFFFFFFF]
    return struct.pack(LOWCMD_PACK_FMT, *vals)


def lowcmd_crc(cmd: LowCmd) -> int:
    """真机 `LowCmd.crc`：对 812 字节布局除 crc 外的 202 个 uint32 做 crc32_core。"""
    buf = pack_lowcmd(cmd, crc=0)
    return crc32_core(buf, LOWCMD_CRC_WORDS)


def lowcmd_with_crc(cmd: LowCmd) -> bytes:
    buf = pack_lowcmd(cmd, crc=0)
    return buf[:-4] + int(lowcmd_crc(cmd)).to_bytes(4, "little")


def verify_lowcmd_crc(cmd: LowCmd) -> bool:
    """校验真机下发/收到的 LowCmd；CRC=0 视为未填（仿真侧允许）。"""
    if int(getattr(cmd, "crc", 0)) == 0:
        return False
    return int(cmd.crc) == lowcmd_crc(cmd)


# ---------------------------------------------------------------- 参考向量
# 由 Unitree 官方 crc_amd64.so 对同一 812 字节布局算出，用作回归测试的 gold value：
#   third_party/unitree_sdk2_python/unitree_sdk2py/utils/lib/crc_amd64.so
#   crc32_core(words, 202)
REFERENCE_CRC = {
    # 全零 812 字节（head/level/电机全 0）
    "zeros": 0x4B5A4880,
    # 官方默认: head=FE EF, level_flag=FF, 20 个电机 mode=1 / q=PosStopF
    "official_default": 0x9BEBAE54,
    # head=FE EF, level=FF, 20 个电机 mode=1 / q=0
    "mode1_q0_all20": 0x57ECF19D,
    # head=FE EF, level=FF, 12 个关节 kp=20/kd=1，其余 8 个零
    "stand_kp20_kd1": 0x8426E4C0,
}
