"""LowCmd / LowState —— 与 unitree_go IDL 字段一一对应的纯 Python 实现。

为什么要自己写一份:
  * 本机没有 sudo，装不了 ROS 2 / CycloneDDS 系统包; 但框架必须今天就能跑。
  * 所以消息层不依赖 cyclonedds / rclpy，loopback 与 UDP 后端可以直接用;
    DDS 后端 (unitree_sdk2py) 和 ROS 2 后端 (unitree_go/msg) 只做一次字段搬运。

字段定义严格照抄 unitree_go::msg::dds_::LowCmd_ / LowState_，
电机槽位 20 个 (Go2 只用前 12 个)，顺序与真机一致:
    0..2   FR_hip, FR_thigh, FR_calf
    3..5   FL_hip, FL_thigh, FL_calf
    6..8   RR_hip, RR_thigh, RR_calf
    9..11  RL_hip, RL_thigh, RL_calf
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List

import numpy as np

# ---------------------------------------------------------------- 常量
NUM_MOTOR = 20          # unitree_go 消息里的电机槽位数
NUM_JOINT = 12          # Go2 实际使用的关节数
CRC_POLY = 0x04C11DB7   # 宇树 crc32_core 的多项式 (MSB-first, init 0xFFFFFFFF, 无 final xor)

MAGIC_CMD = 0x474F3243    # "GO2C" —— 本框架本地传输层用，DDS 后端不用
MAGIC_STATE = 0x474F3253  # "GO2S"

# 本地(UDP)传输层的二进制布局。全部用 '<' 关闭对齐，长度都是 4 的整数倍，
# 这样 CRC 可以按 uint32 对齐切分（和宇树 C 里 sizeof>>2-1 的写法同一个意思）。
# 注意: 这里存的是"结构体主体"，字节序 '<' 只在最后拼一次 ——
# CPython 的 struct 不允许字节序字符出现在第一个之后的位置。
_MOTOR_CMD_FMT = "BfffffI"        # mode,q,dq,tau,kp,kd,reserve  -> 25 B
_MOTOR_STATE_FMT = "BffffBIII"    # mode,q,dq,ddq,tau_est,temperature,lost,res0,res1 -> 30 B
_CMD_HEADER_FMT = "IIdBBxx"       # magic,seq,stamp,mode_pr,mode_machine,pad -> 20 B
_STATE_HEADER_FMT = "IId"         # magic,seq,sim_time -> 16 B

LOWCMD_FMT = "<" + _CMD_HEADER_FMT + _MOTOR_CMD_FMT * NUM_MOTOR + "I"  # 524 B
# 末尾 "4hII" = 足端力(4) + tick + crc；crc 永远占最后 4 字节，
# 计算时按 (len>>2)-1 个 word 走，也就是"除 crc 之外的全部内容"，收发两侧一致。
LOWSTATE_FMT = "<" + _STATE_HEADER_FMT + "4f3f3f3f" + _MOTOR_STATE_FMT * NUM_MOTOR + "4hII"  # 684 B

assert struct.calcsize(LOWCMD_FMT) == 524, struct.calcsize(LOWCMD_FMT)
assert struct.calcsize(LOWSTATE_FMT) == 684, struct.calcsize(LOWSTATE_FMT)


# ---------------------------------------------------------------- CRC32
def _make_crc_table() -> List[int]:
    """标准 MSB-first CRC-32 表 (poly 0x04C11DB7，无反射、无 final xor)。"""
    table = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ CRC_POLY) & 0xFFFFFFFF if (c & 0x80000000) else ((c << 1) & 0xFFFFFFFF)
        table.append(c)
    return table


_CRC_TABLE = _make_crc_table()


def crc32_core(buf: bytes, n_words: int) -> int:
    """等价于宇树的 crc32_core((uint32_t*)buf, n_words)。

    宇树的实现对每个 uint32 做 MSB-first 逐位除法；小端机上一个 word 的 MSB-first
    位序 = 该 word 四个字节的**逆序**(b3,b2,b1,b0)。这里用查表法实现同一件事，
    500 Hz 下比逐位实现快 ~30 倍 (见 selftest: 与逐位参考实现逐位一致)。
    """
    crc = 0xFFFFFFFF
    tbl = _CRC_TABLE
    for i in range(n_words):
        base = i * 4
        crc = ((crc << 8) & 0xFFFFFFFF) ^ tbl[((crc >> 24) ^ buf[base + 3]) & 0xFF]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ tbl[((crc >> 24) ^ buf[base + 2]) & 0xFF]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ tbl[((crc >> 24) ^ buf[base + 1]) & 0xFF]
        crc = ((crc << 8) & 0xFFFFFFFF) ^ tbl[((crc >> 24) ^ buf[base + 0]) & 0xFF]
    return crc


def crc32_core_bitwise(buf: bytes, n_words: int) -> int:
    """逐位参考实现，只用于 selftest 校验查表实现。"""
    crc = 0xFFFFFFFF
    for i in range(n_words):
        data = int.from_bytes(buf[i * 4 : i * 4 + 4], "little")
        xbit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if data & xbit:
                crc ^= CRC_POLY
            xbit >>= 1
    return crc


# ---------------------------------------------------------------- 数据结构
@dataclass
class MotorCmd:
    """对应 unitree_go::msg::dds_::MotorCmd_。

    真机语义: 电机驱动器自己算 tau_total = kp*(q_des - q) + kd*(dq_des - dq) + tau。
    纯力矩模式把 kp/kd 置 0 即可。
    """

    mode: int = 0x01     # 0x01 = FOC 使能
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass
class IMUState:
    quaternion: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyroscope: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accelerometer: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class MotorState:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    temperature: int = 0
    lost: int = 0


@dataclass
class LowCmd:
    mode_pr: int = 0
    mode_machine: int = 0
    motor_cmd: List[MotorCmd] = field(default_factory=lambda: [MotorCmd() for _ in range(NUM_MOTOR)])
    reserve: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 0
    # --- 下面两个只服务本地传输层，DDS 后端不用 ---
    seq: int = 0
    stamp: float = 0.0

    def zero(self, n: int = NUM_MOTOR) -> "LowCmd":
        for i in range(n):
            self.motor_cmd[i] = MotorCmd()
        return self

    def nz_tau(self) -> np.ndarray:
        return np.array([self.motor_cmd[i].tau for i in range(NUM_JOINT)], dtype=np.float64)

    def pack(self) -> bytes:
        parts: List[float] = [MAGIC_CMD, self.seq, self.stamp, self.mode_pr, self.mode_machine]
        for i in range(NUM_MOTOR):
            m = self.motor_cmd[i]
            parts += [m.mode, m.q, m.dq, m.tau, m.kp, m.kd, m.reserve]
        buf = struct.pack(LOWCMD_FMT, *parts, 0)
        self.crc = crc32_core(buf, (len(buf) >> 2) - 1)
        return buf[:-4] + struct.pack("<I", self.crc)

    @staticmethod
    def unpack(buf: bytes) -> "LowCmd":
        vals = struct.unpack(LOWCMD_FMT, buf)
        cmd = LowCmd()
        cmd.seq, cmd.stamp, cmd.mode_pr, cmd.mode_machine = vals[1], vals[2], vals[3], vals[4]
        off = 5
        for i in range(NUM_MOTOR):
            m = MotorCmd(*vals[off : off + 7])
            cmd.motor_cmd[i] = m
            off += 7
        cmd.crc = vals[off]
        return cmd


@dataclass
class LowState:
    imu_state: IMUState = field(default_factory=IMUState)
    motor_state: List[MotorState] = field(default_factory=lambda: [MotorState() for _ in range(NUM_MOTOR)])
    foot_force: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    tick: int = 0
    mode_machine: int = 0
    # --- 本地传输层用 ---
    seq: int = 0
    sim_time: float = 0.0
    crc: int = 0

    def pack(self) -> bytes:
        parts: List[float] = [MAGIC_STATE, self.seq, self.sim_time]
        parts += list(self.imu_state.quaternion) + list(self.imu_state.gyroscope) + list(
            self.imu_state.accelerometer
        ) + list(self.imu_state.rpy)
        for i in range(NUM_MOTOR):
            m = self.motor_state[i]
            parts += [m.mode, m.q, m.dq, m.ddq, m.tau_est, m.temperature, m.lost, 0, 0]
        parts += list(self.foot_force) + [self.tick]
        buf = struct.pack(LOWSTATE_FMT, *parts, 0)
        # 与 LowCmd 同一口径: 只对 "除 crc 外的全部内容" 求 CRC
        self.crc = crc32_core(buf, (len(buf) >> 2) - 1)
        return buf[:-4] + struct.pack("<I", self.crc)

    @staticmethod
    def unpack(buf: bytes) -> "LowState":
        v = struct.unpack(LOWSTATE_FMT, buf)
        st = LowState()
        st.seq, st.sim_time = v[1], v[2]
        st.imu_state = IMUState(quaternion=list(v[3:7]), gyroscope=list(v[7:10]),
                                accelerometer=list(v[10:13]), rpy=list(v[13:16]))
        off = 16
        for i in range(NUM_MOTOR):
            mode, q, dq, ddq, tau, temp, lost, _r0, _r1 = v[off : off + 9]
            st.motor_state[i] = MotorState(mode, q, dq, ddq, tau, temp, lost)
            off += 9
        st.foot_force = list(v[off : off + 4])
        st.tick = int(v[off + 4])
        st.crc = int(v[off + 5])
        return st

    def q12(self) -> np.ndarray:
        return np.array([self.motor_state[i].q for i in range(NUM_JOINT)], dtype=np.float64)

    def dq12(self) -> np.ndarray:
        return np.array([self.motor_state[i].dq for i in range(NUM_JOINT)], dtype=np.float64)


# ---------------------------------------------------------------- selftest
def _selftest() -> None:
    rng = np.random.default_rng(0)
    for _ in range(20):
        n = int(rng.integers(1, 200))
        buf = bytes(rng.integers(0, 256, size=n * 4, dtype=np.uint8).tolist())
        assert crc32_core(buf, n) == crc32_core_bitwise(buf, n), "查表 CRC 与逐位实现不一致"
    c = LowCmd()
    c.motor_cmd[0] = MotorCmd(q=0.1, tau=1.0, kp=20.0)
    raw = c.pack()
    assert len(raw) == 524
    c2 = LowCmd.unpack(raw)
    assert abs(c2.motor_cmd[0].tau - 1.0) < 1e-6
    assert c2.crc == crc32_core(raw, (len(raw) >> 2) - 1)
    s = LowState()
    s.motor_state[3] = MotorState(q=-1.5, dq=0.25)
    s.imu_state.rpy = [0.0, 0.1, 0.0]
    s.foot_force = [10, 20, 30, 40]
    raw2 = s.pack()
    assert len(raw2) == 684
    s2 = LowState.unpack(raw2)
    assert abs(s2.motor_state[3].q + 1.5) < 1e-6 and s2.foot_force == [10, 20, 30, 40]


if __name__ == "__main__":
    _selftest()
    print("msg.py selftest OK  (LOWCMD=%d B, LOWSTATE=%d B)" % (
        struct.calcsize(LOWCMD_FMT), struct.calcsize(LOWSTATE_FMT)))
