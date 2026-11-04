"""Go2 真机 LowCmd 的 812 字节内存布局 + CRC32（纯 Python，不依赖 SDK）。

===============================================================================
1. 为什么需要这个模块
===============================================================================

真机固件对每个 LowCmd 都校验 crc；CRC 覆盖的范围是 **unitree_go::msg::dds_::LowCmd_
这个 IDL 结构体的内存布局**（含对齐 padding），而不是 ROS 2 的 CDR 序列化字节。

两个官方实现逐字节一致，本模块照抄同一布局：

  * unitree_sdk2py（Python，权威布局）:
        third_party/unitree_sdk2_python/unitree_sdk2py/utils/crc.py:19
        __packFmtLowCmd = '<4B4IH2x' + 'B3x5f3I' * 20 + '4B' + '55Bx2I'
  * unitree_ros2（C++，ROS 2 例程）:
        third_party/unitree_ros2/example/src/src/common/motor_crc.cpp:3-39
        raw.crc = crc32_core((uint32_t*)&raw, (sizeof(LowCmd) >> 2) - 1)

为什么不能用下面这些"看起来更正常"的做法：

  * 用 ROS 2 CDR 序列化字节算 CRC:
      CDR 的 padding 与 C 结构体布局不同（实测 CDR 里 motor_cmd[0].mode 在 offset 22，
      固件布局在 offset 24），算出来一定不对。
  * 用 zlib.crc32 / binascii.crc32:
      宇树的 crc32_core 是 poly=0x04C11DB7、init=0xFFFFFFFF、
      **逐位 MSB-first、无反射、无 final xor**，与标准 CRC-32 不同。
  * 直接发 crc=0:
      固件大概率静默丢帧 —— 症状是"话题有数据、电机不响应"
      （发 mode=1 但 LowState.motor_state.mode 一直是 0）。

===============================================================================
2. 812 字节布局（与 unitree_go/msg/LowCmd.msg 字段顺序一致）
===============================================================================

  偏移   长度  字段                         说明
  -----  ----  ---------------------------  ------------------------------------
  0      2     head[2]                      固定头 0xFE 0xEF（帧校验）
  2      1     level_flag                   0xFF = LOWLEVEL（低层控制）
  3      1     frame_reserve                保留
  4      8     sn[2]                        uint32[2]，本项目恒 0
  12     8     version[2]                   uint32[2]，本项目恒 0
  20     2     bandwidth                    uint16，本项目恒 0
  22     2     (padding)                    对齐到 4 字节
  24     720   motor_cmd[20]                每个 36 字节，见下
  744    4     bms_cmd                      off + reserve[3]
  748    40    wireless_remote[40]          遥控器原始数据（本项目恒 0）
  788    12    led[12]
  800    2     fan[2]
  802    1     gpio
  803    1     (padding)
  804    4     reserve
  808    4     crc                          **CRC 本身，不参与计算**
  -----  ----  ---------------------------  ------------------------------------
  合计 812 字节；CRC 覆盖前 808 字节 = 202 个 uint32。

  MotorCmd（36 字节）:
      0  mode(uint8)     1  (padding 3)   4  q(float32)    8  dq(float32)
      12 tau(float32)   16  kp(float32)  20  kd(float32)  24  reserve(uint32[3])

  Go2 只用前 12 个电机槽位（FR,FL,RR,RL × hip,thigh,calf），后 8 个保留；
  官方例程把 20 个都填成 mode=0x01、q=PosStopF(2.146e9)、kp=kd=tau=0。

===============================================================================
3. CRC 算法
===============================================================================

  crc32_core(buf, n_words)（实现在 signal_io/msg.py）：
      crc = 0xFFFFFFFF
      对每个 little-endian uint32 word，按 MSB-first 逐位做多项式除法
      多项式 0x04C11DB7，不做最终异或
  覆盖 buf[0:808]（202 个字），结果写回 LowCmd.crc（小端 uint32）。

===============================================================================
4. 怎么验证这份实现是对的
===============================================================================

  * tests/test_go2_crc.py：
      - 4 组 gold value（由官方 crc_amd64.so 算出）；
      - 用 ctypes 直接调官方 third_party/.../lib/crc_amd64.so 交叉验证；
      - 修改任一电机字段/头字段，CRC 必须变化。
  * 仿真闭环：sim_ros 也开 crc_check，控制器每拍算 CRC、sim 侧逐条校验，
    bad_crc 必须恒 0（见 signal_io/transport/ros2.py）。
  * 真机：scripts/go2_real_check.py 订阅 /lowcmd 并逐条校验 CRC。

相关文件：
  signal_io/msg.py              crc32_core / LowCmd / MotorCmd
  signal_io/transport/ros2.py   真机 LowCmd 组装与校验
  signal_io/transport/dds.py    raw DDS 路径用 unitree_sdk2py 的 CRC 类
  tests/test_go2_crc.py         gold value + 官方 C 库交叉验证
"""

from __future__ import annotations

import struct

from .msg import NUM_MOTOR, LowCmd, crc32_core

# 812 = sizeof(unitree_go::msg::dds_::LowCmd_)，uint32 对齐
LOWCMD_SIZE = 812
LOWCMD_CRC_WORDS = (LOWCMD_SIZE >> 2) - 1        # 202 个字 / 808 字节

# struct 格式串逐段解释（与官方 unitree_sdk2py 完全一致）：
#   '<4B4IH2x'   head[2] + level_flag + frame_reserve, sn[2], version[2],
#                bandwidth(uint16), 2 字节对齐 padding
#   'B3x5f3I'*20 每个 MotorCmd_: mode(uint8) + 3 字节 padding +
#                q/dq/tau/kp/kd(5×float32) + reserve(uint32[3]) = 36 字节
#   '4B'         bms_cmd: off + reserve[3]
#   '55Bx2I'     wireless_remote[40] + led[12] + fan[2] + gpio,
#                1 字节 padding, reserve(uint32), crc(uint32)
LOWCMD_PACK_FMT = "<4B4IH2x" + "B3x5f3I" * NUM_MOTOR + "4B" + "55Bx2I"
assert 4 + 4 * 4 + 2 + 2 + NUM_MOTOR * 36 + 4 + 55 + 1 + 8 == LOWCMD_SIZE

# 官方 low-level 例程的固定头（unitree_ros2 go2_stand_example.cpp / sdk2py go2_stand_example.py）
HEAD = (0xFE, 0xEF)          # 帧头，固件用它做帧校验
LEVEL_FLAG_LOWLEVEL = 0xFF   # 低层控制（0xEE 是高层的 HIGHLEVEL）
GPIO_DEFAULT = 0


def pack_lowcmd(cmd: LowCmd, crc: int = 0) -> bytes:
    """把内部 LowCmd 打包成 812 字节的固件布局（crc 放在最后 4 字节）。

    Args:
        cmd: signal_io/msg.py 的 LowCmd（20 个电机槽位）。
        crc: 要写进末尾的 CRC 值；计算 CRC 时传 0，填消息时传 lowcmd_crc()。
    Returns:
        812 字节 little-endian 缓冲。

    注意：sn/version/bandwidth/bms/wireless_remote/led/fan/reserve 当前硬编码为 0，
    与本项目发布方一致；如果以后要校验别人的 LowCmd 而这些字段非 0，
    需要把它们也加进打包参数，否则 CRC 会算错。
    """
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
    """计算真机 LowCmd.crc（对 812 字节布局除 crc 外的 202 个 uint32）。

    这是真机能否响应控制指令的关键之一；CRC 错时固件通常静默丢帧。
    """
    buf = pack_lowcmd(cmd, crc=0)
    return crc32_core(buf, LOWCMD_CRC_WORDS)


def lowcmd_with_crc(cmd: LowCmd) -> bytes:
    """返回带 CRC 的完整 812 字节（调试/抓包对账用；正常下发走 transport）。"""
    buf = pack_lowcmd(cmd, crc=0)
    return buf[:-4] + int(lowcmd_crc(cmd)).to_bytes(4, "little")


def verify_lowcmd_crc(cmd: LowCmd) -> bool:
    """校验一条 LowCmd 的 CRC。

    CRC=0 视为"未填"（仿真侧某些桥不发 CRC），返回 False 而不是 True，
    避免把"没校验"误当成"校验通过"。
    """
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
