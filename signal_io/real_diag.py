#!/usr/bin/env python3
"""实机传感器自检的诊断内核：采样数据 -> PASS/WARN/FAIL/SKIP 报告。

===============================================================================
1. 职责
===============================================================================

  把"从机器人/仿真采到的一批采样"判成一张可读的表：

    输入  Samples：/lowstate 逐拍快照（tick / IMU / 12 电机 / 足端力 / bit_flag /
          电池电压）、/lowcmd 统计（条数 / crc=0 条数 / CRC 校验失败条数 / 发布者数）、
          L1 点云逐帧统计（时间戳 / frame_id / 点数 / xyz 分位）、以及
          pose / odom / sportmodestate / lidar_state 的条数。
    输出  Report：checks=[Check(status, name, detail, value)...] + meta。
          render_text() 给人看（中文、对齐、带结论与"上机前还差什么"），
          to_dict()      落 JSON（可直接 json.dumps）。

  **本模块不 import rclpy、不 import mujoco、不发也不收任何 DDS 流量**：
  ROS 2 订阅全部在 scripts/real_sensor_check.py 这个薄壳里，壳只负责"把消息搬成
  这里的纯数据"。所以本模块在本机（没装 ROS 2、没有机器人）也能跑单测与 --selftest。

===============================================================================
2. 数据流
===============================================================================

  scripts/real_sensor_check.py（rclpy 只订阅，绝不发布）
        │  回调里逐拍/逐帧搬运，不在这里做任何判定
        ▼
  Samples(target=..., lowstate=[LowStateSample...], lowcmd=LowCmdSamples,
          lidar=[LidarSample...], topics={"pose": n, "odom": n, ...},
          duration_s=..., tick_step=...)
        │  analyze(samples, target=..., tick_step=..., lidar_hz=...)
        ▼
  Report(checks, meta)
        ├─ render_text() -> 中文对齐表格 + 结论段
        ├─ to_dict()     -> --json 文件（含 meta：环境变量/网卡/命令）
        └─ verdict(["state","imu","motors","lidar"]) -> PASS/WARN/FAIL（退出码依据）

===============================================================================
3. 判定项与阈值
===============================================================================

  检查项（name）         PASS / WARN / FAIL 口径
  ---------------------  --------------------------------------------------
  lowstate.rate          >=400 Hz；200~400 WARN；<200 FAIL；>600 WARN（重复/时钟异常）
  lowstate.tick          单调不减 && Δtick 中位自洽（真机 1 / 仿真 2，或按 tick 单位
                         ×状态频率推算出的 2 / 1；两种语义都算 PASS，见函数注释）
  motors.mode            12 个 mode 全 0x01；全 0 FAIL（未使能）
  motors.lost            全 0；少量 WARN；>1% 的电机-拍 FAIL
  motors.temperature     <70 WARN 阈值；>=70 WARN；>=85 FAIL；恒 0 -> SKIP
  motors.q               在关节范围内；超出 <=0.25 rad WARN（固件限位比模型宽）；更多 FAIL
  motors.dq              有限；|dq|>40 WARN；>100 FAIL
  motors.tau             |tau_est| 不超额定；超 1 倍 WARN；超 2 倍 FAIL
  imu.rpy                有限；量级 >2π 疑似单位错
  imu.quat               有限且 ||q||≈1（±0.05 PASS / ±0.2 WARN）
  imu.accel              |a|≈9.81（±1.5 PASS / ±3 WARN / 更远 FAIL；恒 0 FAIL）
  imu.gyro               静止时逐轴 std 小（<=0.3 PASS / <=1.5 WARN）；恒 0 WARN
  foot_force             4 路（FR,FL,RR,RL）能读到非 0；恒 0 WARN（AIR/PRO 无此传感器）
  bit_flag               0 -> PASS；bit0 指令超时 / bit7 电机超时 / 其他位 -> WARN
  lowcmd.publisher       观测期内无人发 -> PASS；有进程在发 -> WARN
  lowcmd.crc             全部 crc=0 或 CRC 校验失败 -> FAIL（固件静默丢帧）
  lidar.rate             真机 ~11 Hz / 仿真 15 Hz；缺失 WARN
  lidar.frame_id         期望 utlidar_lidar（真机）/ world（仿真）；不符 WARN
  lidar.npoints          中位 >=100 PASS；全空帧 FAIL
  lidar.xyz              xyz 分位有限、非退化、量级合理
  pose / odom            有数据 PASS；真机缺失 SKIP（官方固件常见）；仿真 odom 缺失 WARN
  sportmodestate         有数据 PASS；缺失 SKIP（释放 sport mode 后不可读）
  lidar_state            有数据 PASS；缺失 SKIP（部分固件不提供）
  power                  电压有值才判：<21.6 V FAIL / <23 V WARN / 无字段 SKIP

  **severity 语义**：FAIL = 上机前必须解决；WARN = 可以上但要盯着；SKIP = 本机/本固件
  不具备该信号（不是错误）。所以"预期可能没有"的项（odom/pose/sportmodestate/
  lidar_state/电池电压/足端力）永远只给 SKIP 或 WARN，不会 FAIL。

===============================================================================
4. 真机要点（阈值从哪来）
===============================================================================

  * 500 Hz / tick 语义：unitree_ros2 官方低层例程就是 500 Hz；真机固件 tick 按 1 ms
    递增，仿真按 2 ms 物理步递增（config io_profiles.*.tick_dt）。因此 500 Hz 下
    "每拍 +1"（消息计数器）与"每拍 +2/+1"（tick 单位 × 频率）两种说法都自洽 ——
    lowstate.tick 两条都接受，只把丢拍（3~4 倍）和冻结当成问题。
  * 电机 mode=0x01：FOC 伺服使能；mode=0 表示固件没有接受低层控制
    （常见原因：没 ReleaseMode / 没进 debug mode / LowCmd CRC 错被丢帧）。
  * 温度 70/85 ℃：与 ros2_controller/health.py 的 temp_warn_c / temp_max_c 一致。
  * 关节范围与力矩上限取自 sim_env/models/go2/go2.xml（官方 URDF 同值）：
      hip   ±1.0472 rad，thigh 前腿 [-1.5708, 3.4907] / 后腿 [-0.5236, 4.5379]，
      calf  [-2.7227, -0.83776]；力矩 hip/thigh 23.7 N·m，calf 45.43 N·m。
  * foot_force 顺序固定 FR,FL,RR,RL；Go2 AIR/PRO 可能没有该传感器（恒 0）。
  * bit_flag：bit0=操作指令超时（本项目只读检测期间**没人在发 LowCmd，所以 bit0
    置位是预期的**）、bit5=遥控超时、bit7=电机超时。
  * 点云 frame_id 真机是 utlidar_lidar（雷达系）；仿真 /go2/lidar/points 是 world 系。
    两者外参/坐标系不同，不能混用（见 signal_io/perception_node.py §3）。
  * 电池：Go2 标称 28.8 V，满电约 33.6 V，低电保护约 21.6 V。

===============================================================================
5. 常见失败模式（这张表想提前抓到的）
===============================================================================

  * 收不到 /lowstate（rate FAIL）：网线没插 / 网段不对（应为 192.168.123.x）/
    网卡 DOWN / ROS_DOMAIN_ID 不是 0 / RMW 不是 rmw_cyclonedds_cpp /
    CYCLONEDDS_URI 绑错网卡（多网卡机器最容易踩，会选到 Wi-Fi）。
  * 有 /lowstate 但 motors.mode 全 0：低层没接管（没 ReleaseMode / CRC 错 / 固件不认）。
  * 有 /lowcmd 但 crc 全 0 或校验失败：发布方布局/算法不对，固件静默丢帧，
    症状是"话题有数据、电机不动"。
  * 点云缺失：L1 没开（先发 rt/utlidar/switch ON）或话题名不同。
  * q 超范围：关节顺序映射错（FR,FL,RR,RL × hip,thigh,calf 不能重排）或量程单位错。
  * IMU 恒 0：消息没填 / 传感器冻结；accel 模长远偏离 9.81：轴向/单位错。

===============================================================================
6. 相关文件
===============================================================================

  scripts/real_sensor_check.py    ROS 2 采集壳（只订阅；--selftest 在本机跑本模块）
  tests/test_real_diag.py         本模块的纯 Python 单测（好样本/坏样本/边界/NaN）
  signal_io/msg.py                LowCmd/LowState 字段语义与 20 槽位约定
  signal_io/go2_crc.py            真机 812 字节布局 + CRC（采集壳校验 /lowcmd 用）
  ros2_controller/health.py       温度/bit_flag 阈值来源（本模块与之对齐）
  real_env/README.md              真机部署 checklist（上机前还差什么看这里）
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

# ================================================================ 常量
NUM_JOINT = 12
# 关节顺序与训练/策略/真机完全一致，禁止重排
MOTOR_NAMES: Tuple[str, ...] = (
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
)
FOOT_ORDER: Tuple[str, ...] = ("FR", "FL", "RR", "RL")
# 标称站姿（config/go2_sim.json robots.go2.home_q，训练侧 keyframe 同值）
HOME_Q = np.array(
    [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5],
    dtype=np.float64,
)

# 关节范围（go2.xml）：[lower, upper]，按 leg*3 + joint 排
_JOINT_RANGE = np.array(
    [
        [-1.0472, 1.0472], [-1.5708, 3.4907], [-2.7227, -0.83776],   # FR
        [-1.0472, 1.0472], [-1.5708, 3.4907], [-2.7227, -0.83776],   # FL
        [-1.0472, 1.0472], [-0.5236, 4.5379], [-2.7227, -0.83776],   # RR
        [-1.0472, 1.0472], [-0.5236, 4.5379], [-2.7227, -0.83776],   # RL
    ],
    dtype=np.float64,
)
JOINT_LO = _JOINT_RANGE[:, 0]
JOINT_HI = _JOINT_RANGE[:, 1]
# 力矩上限：hip/thigh 23.7 N·m，calf 45.43 N·m（go2.xml motor ctrlrange）
TAU_LIMIT = np.array([23.7, 23.7, 45.43] * 4, dtype=np.float64)

# --- 阈值（集中放这里，测试直接引常量，避免"测试和实现对不上"）
STATE_HZ_EXPECTED = 500.0
STATE_HZ_OK = 400.0
STATE_HZ_WARN = 200.0
STATE_HZ_HIGH = 600.0
STATE_GAP_WARN_S = 0.10       # 单次断流超过 100 ms -> 即使平均频率够也 WARN

TEMP_WARN_C = 70
TEMP_MAX_C = 85

# 超出「MuJoCo 模型关节范围」多少 rad 以内只 WARN。
# 真机固件软限位与 MJCF 模型限位不是同一张表：实测 Go2 趴着折腿时 FR 小腿
# q=-2.824 rad，模型下限 -2.7227 -> 天然越界 0.10 rad（=5.8°，撞机械限位）。
# 因此容差取 0.25 rad：小于它只当「模型/固件限位差异」，大于它才怀疑顺序/单位错。
Q_RANGE_TOL_WARN = 0.25
DQ_WARN = 40.0                # 关节峰值转速量级（rad/s）
DQ_FAIL = 100.0
TAU_RATIO_WARN = 1.0
TAU_RATIO_FAIL = 2.0

ACCEL_G = 9.81
ACCEL_TOL_OK = 1.5
ACCEL_TOL_WARN = 3.0
QUAT_TOL_OK = 0.05
QUAT_TOL_WARN = 0.20

GYRO_STD_OK = 0.3             # 静止时逐轴 std 上限（rad/s）
GYRO_STD_WARN = 1.5

FOOT_MIN_OK = 1.0

LIDAR_HZ_REAL = 11.0          # L1 周向扫描约 11 Hz
LIDAR_HZ_SIM = 15.0           # config perception.rate_hz
LIDAR_FRAME_REAL = "utlidar_lidar"
LIDAR_FRAME_SIM = "world"
LIDAR_POINTS_MIN = 100.0
LIDAR_P99_MAX_M = 100.0

POWER_LOW_FAIL = 21.6
POWER_LOW_WARN = 23.0
POWER_FULL_WARN = 34.5

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"
STATUSES: Tuple[str, ...] = (PASS, WARN, FAIL, SKIP)
_STATUS_RANK = {PASS: 0, SKIP: 1, WARN: 2, FAIL: 3}

# --require 的别名 -> 检查项分组（名字前缀）。"state" 指整个 /lowstate 家族。
REQUIRE_GROUPS: Dict[str, Tuple[str, ...]] = {
    "state": ("lowstate",),
    "lowstate": ("lowstate",),
    "imu": ("imu",),
    "motors": ("motors",),
    "motor": ("motors",),
    "foot_force": ("foot_force",),
    "bit_flag": ("bit_flag",),
    "lowcmd": ("lowcmd",),
    "lidar": ("lidar",),
    "cloud": ("lidar",),
    "pose": ("pose",),
    "odom": ("odom",),
    "sportmodestate": ("sportmodestate",),
    "lidar_state": ("lidar_state",),
    "power": ("power",),
}


# ================================================================ 数据结构
@dataclass
class Check:
    """一条判定结果。

    Attributes:
        status: PASS / WARN / FAIL / SKIP 之一（构造时校验，写错直接抛）。
        name:   稳定的检查项 ID，如 "lowstate.rate" / "motors.temperature"。
                也是 --require 匹配的依据（见 REQUIRE_GROUPS）。
        detail: 给人看的中文结论（含实测值、阈值、下一步动作）。
        value:  机器可读的标量/列表（进 JSON 用），不参与排版。
    """

    status: str
    name: str
    detail: str = ""
    value: Any = None

    def __post_init__(self) -> None:
        if self.status not in _STATUS_RANK:
            raise ValueError("Check.status 必须是 %s 之一，收到 %r"
                             % ("/".join(STATUSES), self.status))
        if not self.name:
            raise ValueError("Check.name 不能为空")

    @property
    def is_pass(self) -> bool:
        return self.status == PASS

    def to_dict(self) -> Dict[str, Any]:
        """JSON 可序列化的一条（value 里的 NaN/Inf 会变成 None）。"""
        return {"status": self.status, "name": self.name,
                "detail": str(self.detail), "value": _jsonable(self.value)}


@dataclass
class Report:
    """一次自检的全部结论。

    Attributes:
        checks: 按输出顺序排列的 Check 列表（同一类检查固定位置，便于对照）。
        meta:   环境/规模信息（target、观测时长、频率、话题条数、命令行、网卡…）。
    """

    checks: List[Check] = field(default_factory=list)
    meta: Dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------ 汇总
    def counts(self) -> Dict[str, int]:
        """各状态条数（JSON 里叫 summary）。"""
        out = {s: 0 for s in STATUSES}
        for c in self.checks:
            out[c.status] = out.get(c.status, 0) + 1
        return out

    def worst(self) -> str:
        """最坏状态：有 FAIL 就是 FAIL，其次 WARN，其次 PASS；全是 SKIP 才是 SKIP。

        注意 SKIP 不算"坏"——它表示本机/本固件没有这个信号（见模块文档 §3）。
        """
        statuses = {c.status for c in self.checks}
        if FAIL in statuses:
            return FAIL
        if WARN in statuses:
            return WARN
        if PASS in statuses:
            return PASS
        return SKIP

    def get(self, name: str) -> Optional[Check]:
        """按检查项 ID 取一条（单测/调用方方便）。"""
        for c in self.checks:
            if c.name == name:
                return c
        return None

    def names(self) -> List[str]:
        return [c.name for c in self.checks]

    # ------------------------------------------------------------ 必需项
    def select_required(self, tokens: Sequence[str]) -> Tuple[List[Check], List[str]]:
        """把 --require 的 token 展开成具体 Check。

        token 可以是 REQUIRE_GROUPS 里的组名（state/imu/motors/lidar/lowcmd/...），
        也可以是精确的检查项 ID（如 "lowstate.tick"、"lowcmd.crc"）。

        Returns:
            (命中的 Check 列表（去重、保持报告顺序）, 无法识别的 token 列表)
        """
        matched_names: List[str] = []
        unknown: List[str] = []
        groups = [str(t).strip() for t in tokens if str(t).strip()]
        for tok in groups:
            group = REQUIRE_GROUPS.get(tok, (tok,))
            hits = [c for c in self.checks
                    if c.name in group
                    or any(c.name.startswith(g + ".") for g in group)]
            if not hits:
                unknown.append(tok)
                continue
            for c in hits:
                if c.name not in matched_names:
                    matched_names.append(c.name)
        matched = [c for c in self.checks if c.name in matched_names]
        return matched, unknown

    def verdict(self, tokens: Sequence[str]) -> str:
        """按 --require 的 token 汇总：任 FAIL -> FAIL；任 WARN/SKIP -> WARN。

        SKIP 参与时算 WARN：必需项"没采到/没判成"等于没验证过，不能当成通过。
        """
        matched, _unknown = self.select_required(tokens)
        worst = PASS
        for c in matched:
            rank = _STATUS_RANK[c.status]
            if rank > _STATUS_RANK[worst]:
                worst = c.status if c.status != SKIP else WARN
        return worst

    # ------------------------------------------------------------ 序列化
    def to_dict(self) -> Dict[str, Any]:
        """整份报告转 dict；保证 json.dumps(..., allow_nan=False) 可用。"""
        return {
            "summary": self.counts(),
            "worst": self.worst(),
            "meta": _jsonable(self.meta),
            "checks": [c.to_dict() for c in self.checks],
        }


# ---------------------------------------------------------------- 采样数据
@dataclass
class LowStateSample:
    """一拍 /lowstate 的纯数据快照（字段与 unitree_go/LowState 语义一致）。

    默认值就是"一台站着、已使能、IMU 正常、四脚受力"的好样本，
    单测里改一个字段就能构造坏样本（见 tests/test_real_diag.py）。
    """

    t: float = 0.0                     # 本机单调时钟（秒），只用来估频率
    tick: int = 0                      # 固件 tick（真机 1 ms/拍；仿真 2 ms/拍）
    mode: Sequence[int] = field(default_factory=lambda: [0x01] * NUM_JOINT)
    q: Sequence[float] = field(default_factory=lambda: list(HOME_Q))
    dq: Sequence[float] = field(default_factory=lambda: [0.0] * NUM_JOINT)
    ddq: Sequence[float] = field(default_factory=lambda: [0.0] * NUM_JOINT)
    tau_est: Sequence[float] = field(default_factory=lambda: [0.0] * NUM_JOINT)
    temperature: Sequence[int] = field(default_factory=lambda: [35] * NUM_JOINT)
    lost: Sequence[int] = field(default_factory=lambda: [0] * NUM_JOINT)
    foot_force: Sequence[float] = field(default_factory=lambda: [50.0] * 4)
    rpy: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quat: Sequence[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyro: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, ACCEL_G])
    bit_flag: int = 0
    power_v: Optional[float] = None    # 消息里没有就是 None -> 电池项 SKIP
    power_a: Optional[float] = None


@dataclass
class LowCmdSamples:
    """观测期内的 /lowcmd 统计（由采集壳逐条累加，不保留消息本体）。

    Attributes:
        n:          收到的 LowCmd 条数。
        crc_zero:   crc 字段为 0 的条数（= 发布方没填 CRC，固件大概率丢弃）。
        crc_bad:    非 0 但按 812 字节布局校验失败的条数（布局/算法错）。
        publishers: 观测到的发布者个数（rclpy graph 查得到就填，查不到填 0）。
        last_crc:   最后一条的 crc（调试用）。
        last_kp:    最后一条前 12 个电机的 kp（判断是位置控制还是纯力矩）。
    """

    n: int = 0
    crc_zero: int = 0
    crc_bad: int = 0
    publishers: int = 0
    last_crc: Optional[int] = None
    last_kp: List[float] = field(default_factory=list)


@dataclass
class LidarSample:
    """一帧 L1 点云的统计量（不保存整块点云，避免内存爆掉）。

    Attributes:
        t:        本机单调时钟（秒）。
        stamp:    header.stamp（秒），用于看时间戳是否单调/是否用仿真时钟。
        frame_id: header.frame_id（真机应为 utlidar_lidar）。
        n_points: width*height。
        p1/p50/p99: xyz 三轴的 1%/50%/99% 分位（米）；采集壳只对前几帧算。
    """

    t: float = 0.0
    stamp: float = 0.0
    frame_id: str = ""
    n_points: int = 0
    p1: Optional[Sequence[float]] = None
    p50: Optional[Sequence[float]] = None
    p99: Optional[Sequence[float]] = None


@dataclass
class Samples:
    """采集壳交给诊断内核的全部输入（纯数据，可 JSON 落盘后离线复算）。

    Attributes:
        target:      IO profile（real_ros / sim_ros）。
        lowstate:    逐拍快照。
        lowcmd:      /lowcmd 统计。
        lidar:       逐帧点云统计。
        topics:      其他话题 -> 条数（pose / odom / sportmodestate / lidar_state）。
        duration_s:  实际观测时长（秒）；时间戳不可用时用它兜底算频率。
        tick_step:   期望的 Δtick（真机 1 / 仿真 2）；None 时按 target 推断。
        lidar_hz:    期望点云频率；None 时按 target 推断（真机 11 / 仿真 15）。
        notes:       采集壳想额外说明的话（例如"已关闭 lowcmd 检查"）。
    """

    target: str = "real_ros"
    lowstate: List[LowStateSample] = field(default_factory=list)
    lowcmd: LowCmdSamples = field(default_factory=LowCmdSamples)
    lidar: List[LidarSample] = field(default_factory=list)
    topics: Dict[str, int] = field(default_factory=dict)
    duration_s: float = 0.0
    tick_step: Optional[int] = None
    lidar_hz: Optional[float] = None
    notes: List[str] = field(default_factory=list)

    # ------------------------------------------------------------ 宽容构造
    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Samples":
        """从 dict 构造（支持缺字段/多余字段；采样项可以是 dict 或 dataclass）。"""
        kw: Dict[str, Any] = {}
        for f in fields(cls):
            if f.name in data:
                kw[f.name] = data[f.name]
        if "lowstate" in kw:
            kw["lowstate"] = [_as_sample(x, LowStateSample) for x in (kw["lowstate"] or [])]
        if "lidar" in kw:
            kw["lidar"] = [_as_sample(x, LidarSample) for x in (kw["lidar"] or [])]
        if "lowcmd" in kw:
            lc = kw["lowcmd"] or {}
            if isinstance(lc, Mapping):
                kw["lowcmd"] = LowCmdSamples(**{k: v for k, v in lc.items()
                                                if k in _fields_of(LowCmdSamples)})
            elif not isinstance(lc, LowCmdSamples):
                raise TypeError("Samples.lowcmd 必须是 LowCmdSamples 或 dict")
        if "topics" in kw:
            kw["topics"] = {str(k): int(v or 0) for k, v in dict(kw["topics"] or {}).items()}
        if "notes" in kw:
            kw["notes"] = [str(x) for x in (kw["notes"] or [])]
        return cls(**kw)


# ================================================================ 小工具
def _fields_of(dc_type) -> set:
    return {f.name for f in fields(dc_type)}


def _as_sample(obj: Any, dc_type):
    """dict -> dataclass（只取该 dataclass 认识的字段），其他原样返回。"""
    if isinstance(obj, dc_type):
        return obj
    if isinstance(obj, Mapping):
        known = _fields_of(dc_type)
        return dc_type(**{k: v for k, v in obj.items() if k in known})
    if is_dataclass(obj) and type(obj).__name__ == dc_type.__name__:
        return obj
    raise TypeError("%s 需要 %s 或 dict，收到 %r" % (dc_type.__name__, dc_type.__name__, type(obj)))


def _jsonable(obj: Any) -> Any:
    """把 numpy 标量/数组、NaN/Inf 变成 json.dumps 能接受的普通值。

    NaN/Inf -> None：JSON 没有 NaN，写出去会让下游解析器（jq/浏览器）报错。
    """
    if isinstance(obj, Mapping):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, np.ndarray):
        return [_jsonable(v) for v in obj.tolist()]
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return f if math.isfinite(f) else None
    if obj is None or isinstance(obj, str):
        return obj
    return str(obj)


def _as_float_matrix(samples: Sequence[Any], attr: str, width: int) -> Tuple[np.ndarray, int]:
    """把每个样本的某字段搬成 (n, width) 的 float 矩阵。

    Returns:
        (矩阵, 字段长度不足 width 的样本数)。不足的槽位填 NaN，
        调用方据此区分"字段缺失"（FAIL）与"值非法"（看 NaN 判定）。
    """
    rows: List[np.ndarray] = []
    n_short = 0
    for smp in samples:
        raw = getattr(smp, attr, None)
        try:
            arr = np.asarray(raw if raw is not None else [], dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            arr = np.zeros(0, dtype=np.float64)
        if arr.size < width:
            n_short += 1
            arr = np.pad(arr, (0, width - arr.size), constant_values=np.nan)
        rows.append(arr[:width])
    if not rows:
        return np.zeros((0, width), dtype=np.float64), n_short
    return np.vstack(rows), n_short


def _finite_stats(arr: np.ndarray) -> Tuple[np.ndarray, int]:
    """返回有限值子数组与 NaN/Inf 个数（reshape(-1) 后统计）。"""
    flat = np.asarray(arr, dtype=np.float64).reshape(-1)
    ok = np.isfinite(flat)
    return flat[ok], int(flat.size - int(ok.sum()))


def _fmt(x: float, nd: int = 2) -> str:
    """数字格式化：非有限值直接写 nan/inf，避免格式化异常。"""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return str(x)
    if not math.isfinite(v):
        return "nan"
    return ("%." + str(nd) + "f") % v


def _target_is_sim(target: str) -> bool:
    return str(target or "").startswith("sim")


def _expected_tick_step(samples: Samples) -> int:
    if samples.tick_step:
        return int(samples.tick_step)
    return 2 if _target_is_sim(samples.target) else 1


def _expected_lidar_hz(samples: Samples) -> float:
    if samples.lidar_hz:
        return float(samples.lidar_hz)
    return LIDAR_HZ_SIM if _target_is_sim(samples.target) else LIDAR_HZ_REAL


def _expected_lidar_frame(samples: Samples) -> str:
    return LIDAR_FRAME_SIM if _target_is_sim(samples.target) else LIDAR_FRAME_REAL


def _rate_of(ts: Sequence[float], duration_s: float = 0.0) -> Tuple[float, float]:
    """由本机时间戳估频率。

    Returns:
        (频率 Hz, 最大相邻间隔 s)。
        时间戳不足 2 条时用 duration_s 兜底；都没有则返回 (0, 0)。
    """
    t = np.asarray(list(ts), dtype=np.float64).reshape(-1)
    t = t[np.isfinite(t)]
    if t.size >= 2:
        span = float(t[-1] - t[0])
        gap = float(np.max(np.diff(t))) if t.size > 1 else 0.0
        if span > 1e-9:
            return (t.size - 1) / span, gap
        return 0.0, gap
    if t.size == 1 and duration_s > 0:
        return 1.0 / duration_s, 0.0
    return 0.0, 0.0


# ================================================================ 逐项判定
def _check_lowstate_rate(smp: Samples) -> List[Check]:
    """/lowstate 频率：≈500 Hz。"""
    ts = [s.t for s in smp.lowstate]
    n = len(ts)
    if n == 0:
        return [Check(FAIL, "lowstate.rate",
                      "整个观测期没收到 /lowstate：先查网线/网段(192.168.123.x)/网卡是否 UP/"
                      "ROS_DOMAIN_ID=0/RMW=rmw_cyclonedds_cpp/CYCLONEDDS_URI 是否绑到有线网卡",
                      0.0)]
    if n == 1:
        return [Check(SKIP, "lowstate.rate",
                      "只收到 1 条 /lowstate，无法估频率（延长 --duration 再跑）", 1)]
    hz, gap = _rate_of(ts, smp.duration_s)
    span = float(ts[-1] - ts[0])
    detail = "%s Hz（%d 条 / %.2f s）" % (_fmt(hz, 1), n, span)
    if hz < STATE_HZ_WARN:
        return [Check(FAIL, "lowstate.rate",
                      detail + "：<%.0f Hz，链路基本收不到状态（查网卡/domain/网卡绑定）"
                      % STATE_HZ_WARN, round(hz, 1))]
    if hz < STATE_HZ_OK:
        return [Check(WARN, "lowstate.rate",
                      detail + "：<%.0f Hz（期望 ≈%.0f Hz），有丢包或调度跟不上"
                      % (STATE_HZ_OK, STATE_HZ_EXPECTED), round(hz, 1))]
    if gap > STATE_GAP_WARN_S:
        return [Check(WARN, "lowstate.rate",
                      detail + "：平均频率够，但出现 %.0f ms 的断流（DDS 抖动/网络丢包）"
                      % (gap * 1e3), round(hz, 1))]
    if hz > STATE_HZ_HIGH:
        return [Check(WARN, "lowstate.rate",
                      detail + "：>%.0f Hz，疑似重复投递或时钟异常" % STATE_HZ_HIGH,
                      round(hz, 1))]
    return [Check(PASS, "lowstate.rate", detail + "（期望 ≈%.0f Hz）" % STATE_HZ_EXPECTED,
                  round(hz, 1))]


def _check_lowstate_tick(smp: Samples) -> List[Check]:
    """tick 单调 + Δtick 中位与"tick 单位/状态频率"自洽（期望：真机 1 / 仿真 2）。

    这里要容忍一个**真实存在的歧义**：unitree_go LowState.tick 的含义在社区里有两
    种理解——(a) 每收到一条 LowState 就 +1（消息计数器）；(b) tick 单位是 1 ms（真机
    固件）/ 2 ms（仿真物理步），那 500 Hz 下每拍应 +2。本项目 config 的 tick_dt
    （真机 0.001 / 仿真 0.002）支持 (b)，而任务口径写的是 (a)。

    所以判定接受两种自洽值，避免把"健康但语义不同"的机器人误报成 WARN：
        expected = 配置期望（真机 1 / 仿真 2）
        alt      = round(状态周期 / tick 单位)，即若 tick 单位是 ms 时应有的增量
                   （真机 500 Hz × 1 ms/tick -> 2；仿真 500 Hz × 2 ms/tick -> 1）
    真正的故障（tick 冻结/回退、Δtick 是期望的 3~4 倍 = 丢拍）仍然会被抓出来。
    """
    ticks = np.asarray([int(getattr(s, "tick", 0)) for s in smp.lowstate], dtype=np.int64)
    if ticks.size < 2:
        return [Check(SKIP, "lowstate.tick", "样本不足 2 条，无法判断 tick 递增", ticks.size)]
    d = np.diff(ticks)
    n_nonmono = int(np.count_nonzero(d <= 0))
    pos = d[d > 0]
    med = int(np.median(pos)) if pos.size else 0
    expected = _expected_tick_step(smp)
    hz, _ = _rate_of([s.t for s in smp.lowstate], smp.duration_s)
    # tick 单位（ms）= 配置的 tick_dt。状态周期 = 1000/hz。
    tick_unit_ms = float(expected)
    alt = 0
    if hz > 1e-9 and tick_unit_ms > 0:
        alt = max(1, int(round((1e3 / hz) / tick_unit_ms)))
    accepted = {expected}
    if alt > 0:
        accepted.add(alt)
    span_txt = "tick %d→%d，Δ中位=%d（期望 %d%s）" % (
        int(ticks[0]), int(ticks[-1]), med, expected,
        ("，按 tick 单位推算 %d" % alt) if alt and alt != expected else "")
    frac_nonmono = n_nonmono / float(d.size)
    if frac_nonmono > 0.01:
        return [Check(FAIL, "lowstate.tick",
                      span_txt + "：%d/%d 拍 tick 不增（时间戳/tick 错乱，状态流不可信）"
                      % (n_nonmono, d.size), [int(ticks[0]), int(ticks[-1]), med])]
    if med <= 0:
        return [Check(FAIL, "lowstate.tick", span_txt + "：tick 不递增（固件没更新 tick）",
                      [int(ticks[0]), int(ticks[-1]), med])]
    if med in accepted:
        if med != expected:
            span_txt += "（按 tick 单位推算自洽：%s）" % (
                "仿真 2 ms/步" if _target_is_sim(smp.target) else "真机 1 ms/tick")
        if n_nonmono:
            span_txt += "（有 %d 拍重复 tick，可忽略）" % n_nonmono
        return [Check(PASS, "lowstate.tick", span_txt,
                      [int(ticks[0]), int(ticks[-1]), med])]
    if med > expected * 2:
        return [Check(WARN, "lowstate.tick",
                      span_txt + "：Δtick 明显偏大，疑似丢拍（DDS 丢包/调度跟不上）",
                      [int(ticks[0]), int(ticks[-1]), med])]
    return [Check(WARN, "lowstate.tick",
                  span_txt + "：与 tick 单位/状态频率不自洽（连拍、丢拍或 target 选错）",
                  [int(ticks[0]), int(ticks[-1]), med])]


def _check_motors_mode(smp: Samples) -> List[Check]:
    """12 个电机 mode 应全 0x01（FOC 使能）。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.mode", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "mode", NUM_JOINT)
    if n_short:
        return [Check(FAIL, "motors.mode",
                      "有 %d 拍消息的 motor_state 槽位不足 12 个（字段缺失，IDL/桥接不对）"
                      % n_short, None)]
    modes = np.nan_to_num(mat, nan=-1.0).astype(np.int64)
    all_on = np.all(modes == 0x01, axis=1)
    n_all_on = int(np.count_nonzero(all_on))
    last = modes[-1].tolist()
    if n_all_on == 0 and np.all(modes == 0):
        return [Check(FAIL, "motors.mode",
                      "12 个电机 mode 全为 0：低层没接管（没 ReleaseMode/没进 debug mode/"
                      "LowCmd CRC 错被丢帧），此时发指令电机不会动", last)]
    if n_all_on == modes.shape[0]:
        return [Check(PASS, "motors.mode", "12 个电机 mode 全 0x01（FOC 使能）", last)]
    frac = n_all_on / float(modes.shape[0])
    if frac >= 0.99:
        return [Check(PASS, "motors.mode",
                      "12 个电机 mode 基本全 0x01（偶发 %d 拍未使能）"
                      % (modes.shape[0] - n_all_on), last)]
    if np.all(modes == 0):
        return [Check(FAIL, "motors.mode", "12 个电机 mode 全为 0（未使能）", last)]
    return [Check(WARN, "motors.mode",
                  "只有 %.0f%% 的拍 12 个 mode 全 0x01，末拍=%s（部分电机未使能/被覆盖）"
                  % (frac * 100.0, last), last)]


def _check_motors_lost(smp: Samples) -> List[Check]:
    """电机通信丢失计数（lost）应全 0。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.lost", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "lost", NUM_JOINT)
    if n_short:
        return [Check(WARN, "motors.lost",
                      "有 %d 拍 motor_state 槽位不足 12 个，lost 无法完整判" % n_short, None)]
    lost = np.nan_to_num(mat, nan=0.0)
    total = int(lost.size)
    n_bad = int(np.count_nonzero(lost))
    if n_bad == 0:
        return [Check(PASS, "motors.lost", "12 个电机 lost 全 0（无通信丢失）", 0)]
    frac = n_bad / float(max(total, 1))
    idx = sorted({int(i) for i in np.nonzero(np.any(lost != 0, axis=0))[0]})
    worst = int(np.max(lost))
    txt = "lost 非 0：%d/%d 个电机-拍，涉及 %s，最大 %d" % (
        n_bad, total, [MOTOR_NAMES[i] for i in idx], worst)
    if frac > 0.01:
        return [Check(FAIL, "motors.lost", txt + "（>1%，电机通信持续丢失，先查线束/固件）",
                      worst)]
    return [Check(WARN, "motors.lost", txt + "（少量丢包，可能是接插件松动/电磁干扰）", worst)]


def _check_motors_temperature(smp: Samples) -> List[Check]:
    """电机温度：>=70 ℃ WARN，>=85 ℃ FAIL（与 health.py 同阈值）。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.temperature", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "temperature", NUM_JOINT)
    if n_short:
        return [Check(WARN, "motors.temperature",
                      "有 %d 拍 motor_state 槽位不足 12 个，温度判不全" % n_short, None)]
    vals, n_bad = _finite_stats(mat)
    if vals.size == 0:
        return [Check(WARN, "motors.temperature", "温度字段全是 NaN/Inf", None)]
    tmax = float(np.max(vals))
    idx = MOTOR_NAMES[int(np.nanargmax(np.nan_to_num(mat, nan=-1e9)) % NUM_JOINT)]
    if tmax <= 0:
        return [Check(SKIP, "motors.temperature",
                      "温度字段恒 <=0（仿真侧不填、或该固件不上报），无法判断", tmax)]
    if tmax >= TEMP_MAX_C:
        return [Check(FAIL, "motors.temperature",
                      "最高 %d ℃（%s）>= %d ℃，先散热再上机" % (tmax, idx, TEMP_MAX_C), tmax)]
    if tmax >= TEMP_WARN_C:
        return [Check(WARN, "motors.temperature",
                      "最高 %d ℃（%s）>= %d ℃（健康层会降到 L1）" % (tmax, idx, TEMP_WARN_C),
                      tmax)]
    return [Check(PASS, "motors.temperature",
                  "最高 %d ℃（< %d ℃）" % (tmax, TEMP_WARN_C), tmax)]


def _check_motors_q(smp: Samples) -> List[Check]:
    """关节角 q 必须在关节范围内（顺序 FR,FL,RR,RL × hip,thigh,calf）。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.q", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "q", NUM_JOINT)
    if n_short:
        return [Check(FAIL, "motors.q",
                      "有 %d 拍关节角槽位不足 12 个（字段缺失）" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "motors.q",
                      "关节角出现 %d 个 NaN/Inf（控制器会直接把 NaN 传给电机）" % n_nonfinite,
                      n_nonfinite)]
    below = JOINT_LO[None, :] - mat
    above = mat - JOINT_HI[None, :]
    viol = np.maximum(below, above)
    worst = float(np.max(viol))
    if worst <= 0.0:
        return [Check(PASS, "motors.q", "12 个关节角都在范围内（最大越界 0）", 0.0)]
    flat = int(np.argmax(viol))
    i_t, i_j = divmod(flat, NUM_JOINT)
    txt = ("%s q=%.3f rad 超出 [%.4f, %.4f] 达 %.3f rad"
           % (MOTOR_NAMES[i_j], mat[i_t, i_j], JOINT_LO[i_j], JOINT_HI[i_j], worst))
    if worst <= Q_RANGE_TOL_WARN:
        return [Check(WARN, "motors.q",
                      txt + "（小幅越界：真机固件软限位/零位标定与 MuJoCo 模型不完全一致，"
                      "趴下折腿时膝关节顶到限位属正常）", round(worst, 3))]
    return [Check(FAIL, "motors.q",
                  txt + "（大幅越界：疑似关节顺序映射错或单位用错）", round(worst, 3))]


def _check_motors_dq(smp: Samples) -> List[Check]:
    """关节速度 dq 有限且不超量级。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.dq", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "dq", NUM_JOINT)
    if n_short:
        return [Check(FAIL, "motors.dq", "有 %d 拍关节速度槽位不足 12 个" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "motors.dq",
                      "关节速度出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    peak = float(np.max(np.abs(mat)))
    if peak > DQ_FAIL:
        return [Check(FAIL, "motors.dq",
                      "|dq| 峰值 %.1f rad/s > %.0f（严重超量级，先确认没有甩腿/失控）"
                      % (peak, DQ_FAIL), round(peak, 2))]
    if peak > DQ_WARN:
        return [Check(WARN, "motors.dq",
                      "|dq| 峰值 %.1f rad/s > %.0f（吊起/搬动时可能出现，落地静止应回落）"
                      % (peak, DQ_WARN), round(peak, 2))]
    return [Check(PASS, "motors.dq", "|dq| 峰值 %.2f rad/s（< %.0f）" % (peak, DQ_WARN),
                  round(peak, 2))]


def _check_motors_tau(smp: Samples) -> List[Check]:
    """关节力矩估计 tau_est 有限且不超硬件额定（hip/thigh 23.7，calf 45.43 N·m）。"""
    if not smp.lowstate:
        return [Check(SKIP, "motors.tau", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "tau_est", NUM_JOINT)
    if n_short:
        return [Check(WARN, "motors.tau", "有 %d 拍力矩槽位不足 12 个" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "motors.tau",
                      "tau_est 出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    ratio = np.abs(mat) / TAU_LIMIT[None, :]
    worst = float(np.max(ratio))
    flat = int(np.argmax(ratio))
    i_t, i_j = divmod(flat, NUM_JOINT)
    txt = ("%s |tau_est|=%.1f N·m，额定 %.1f N·m（%.0f%%）"
           % (MOTOR_NAMES[i_j], abs(mat[i_t, i_j]), TAU_LIMIT[i_j], worst * 100.0))
    if worst > TAU_RATIO_FAIL:
        return [Check(FAIL, "motors.tau",
                      txt + "：超额定 2 倍，先查标定/撞地/驱动器参数", round(worst, 2))]
    if worst > TAU_RATIO_WARN:
        return [Check(WARN, "motors.tau",
                      txt + "：超额定（静止站立不该超；若正撞地/抱闸可解释）", round(worst, 2))]
    return [Check(PASS, "motors.tau", txt, round(worst, 2))]


def _check_imu_rpy(smp: Samples) -> List[Check]:
    """IMU 姿态角有限、量级像弧度。"""
    if not smp.lowstate:
        return [Check(SKIP, "imu.rpy", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "rpy", 3)
    if n_short:
        return [Check(FAIL, "imu.rpy", "有 %d 拍 rpy 不足 3 维" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "imu.rpy", "rpy 出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    peak = float(np.max(np.abs(mat)))
    last = [round(float(x), 3) for x in mat[-1]]
    if peak > 2.0 * math.pi + 0.5:
        return [Check(WARN, "imu.rpy",
                      "|rpy| 峰值 %.2f > 2π，疑似单位是角度而不是弧度" % peak,
                      round(peak, 3))]
    return [Check(PASS, "imu.rpy", "rpy 有限，末帧(roll,pitch,yaw)=%s rad" % last,
                  [round(float(x), 3) for x in last])]


def _check_imu_quat(smp: Samples) -> List[Check]:
    """四元数有限且模长≈1。"""
    if not smp.lowstate:
        return [Check(SKIP, "imu.quat", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "quat", 4)
    if n_short:
        return [Check(FAIL, "imu.quat", "有 %d 拍 quaternion 不足 4 维" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "imu.quat", "四元数出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    norm = np.linalg.norm(mat, axis=1)
    err = float(np.max(np.abs(norm - 1.0)))
    if err <= QUAT_TOL_OK:
        return [Check(PASS, "imu.quat",
                      "四元数模长偏差 %.4f（|q|=%.4f）" % (err, float(np.median(norm))),
                      round(err, 4))]
    if err <= QUAT_TOL_WARN:
        return [Check(WARN, "imu.quat",
                      "四元数模长偏差 %.3f，可能归一化不严/传感器抖动" % err, round(err, 3))]
    return [Check(FAIL, "imu.quat",
                  "四元数模长偏差 %.2f（|q|=%.3f）：数值无效或字段没填" % (err, float(np.median(norm))),
                  round(err, 3))]


def _check_imu_accel(smp: Samples) -> List[Check]:
    """加速度模长应≈9.81 m/s²（含重力）。"""
    if not smp.lowstate:
        return [Check(SKIP, "imu.accel", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "accel", 3)
    if n_short:
        return [Check(FAIL, "imu.accel", "有 %d 拍 accelerometer 不足 3 维" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "imu.accel", "加速度出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    norm = np.linalg.norm(mat, axis=1)
    med = float(np.median(norm))
    err = abs(med - ACCEL_G)
    txt = "|accel| 中位 %.2f m/s²（期望 %.2f）" % (med, ACCEL_G)
    if med <= 0.1:
        return [Check(FAIL, "imu.accel", txt + "：加速度恒 0（IMU 没填/冻结）", round(med, 2))]
    if err <= ACCEL_TOL_OK:
        return [Check(PASS, "imu.accel", txt, round(med, 2))]
    if err <= ACCEL_TOL_WARN:
        return [Check(WARN, "imu.accel", txt + "：略偏离，可能正在被搬动/标定偏置",
                      round(med, 2))]
    return [Check(FAIL, "imu.accel",
                  txt + "：偏离 >%.1f，疑似轴向/单位错或 IMU 异常" % ACCEL_TOL_WARN,
                  round(med, 2))]


def _check_imu_gyro(smp: Samples) -> List[Check]:
    """角速度静止噪声（逐轴 std）；恒 0 也要报，因为可能是没填/冻结。"""
    if not smp.lowstate:
        return [Check(SKIP, "imu.gyro", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "gyro", 3)
    if n_short:
        return [Check(FAIL, "imu.gyro", "有 %d 拍 gyroscope 不足 3 维" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "imu.gyro", "角速度出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    if mat.shape[0] < 2:
        return [Check(SKIP, "imu.gyro", "样本不足 2 条，无法算噪声", None)]
    std = np.std(mat, axis=0)
    std_max = float(np.max(std))
    txt = "逐轴 std=%s rad/s" % [round(float(x), 4) for x in std]
    if np.allclose(mat, 0.0, atol=1e-12):
        return [Check(WARN, "imu.gyro",
                      txt + "：角速度恒 0（疑似消息没填/传感器冻结；真静止也该有微小噪声）",
                      round(std_max, 5))]
    if std_max <= GYRO_STD_OK:
        return [Check(PASS, "imu.gyro", txt + "（<=%.1f，静止量级）" % GYRO_STD_OK,
                      round(std_max, 4))]
    if std_max <= GYRO_STD_WARN:
        return [Check(WARN, "imu.gyro",
                      txt + "（>%.1f：观测期内机器人在动，吊起/搬动时正常）" % GYRO_STD_OK,
                      round(std_max, 4))]
    return [Check(WARN, "imu.gyro",
                  txt + "（>%.1f rad/s：明显在运动，本条只能当参考，落地静止复测）"
                  % GYRO_STD_WARN, round(std_max, 4))]


def _check_foot_force(smp: Samples) -> List[Check]:
    """足端力 4 路（FR,FL,RR,RL）是否能读到。"""
    if not smp.lowstate:
        return [Check(SKIP, "foot_force", "没有 /lowstate 样本可判", None)]
    mat, n_short = _as_float_matrix(smp.lowstate, "foot_force", 4)
    if n_short:
        return [Check(FAIL, "foot_force",
                      "有 %d 拍 foot_force 不足 4 路（字段缺失）" % n_short, None)]
    n_nonfinite = int(np.count_nonzero(~np.isfinite(mat)))
    if n_nonfinite:
        return [Check(FAIL, "foot_force", "足端力出现 %d 个 NaN/Inf" % n_nonfinite, n_nonfinite)]
    mean = np.mean(mat, axis=0)
    peak = float(np.max(np.abs(mat)))
    per = {FOOT_ORDER[i]: round(float(mean[i]), 1) for i in range(4)}
    if not np.any(mat):
        return [Check(WARN, "foot_force",
                      "4 路（FR,FL,RR,RL）全为 0：可能是 AIR/PRO 没有足端力传感器，"
                      "或机器人没落地没受力；不能据此判断" , per)]
    if float(np.min(mat)) < 0:
        return [Check(WARN, "foot_force",
                      "均值 %s，出现负值（量程/符号异常）" % per, per)]
    if peak < FOOT_MIN_OK:
        return [Check(WARN, "foot_force", "均值 %s，几乎无受力" % per, per)]
    return [Check(PASS, "foot_force",
                  "4 路均值 %s（顺序 FR,FL,RR,RL；z 向受力）" % per, per)]


def _check_bit_flag(smp: Samples) -> List[Check]:
    """LowState.bit_flag 异常位。"""
    if not smp.lowstate:
        return [Check(SKIP, "bit_flag", "没有 /lowstate 样本可判", None)]
    flags = np.asarray([int(getattr(s, "bit_flag", 0)) & 0xFF for s in smp.lowstate],
                       dtype=np.int64)
    seen = sorted({int(x) for x in flags})
    or_all = int(np.bitwise_or.reduce(flags)) if flags.size else 0
    if or_all == 0:
        return [Check(PASS, "bit_flag", "bit_flag 全 0（无异常位）", 0)]
    bits = [b for b in range(8) if or_all & (1 << b)]
    labels = []
    for b in bits:
        if b == 0:
            labels.append("bit0=操作指令超时")
        elif b == 5:
            labels.append("bit5=遥控超时")
        elif b == 7:
            labels.append("bit7=电机超时")
        else:
            labels.append("bit%d" % b)
    detail = "bit_flag 出现过 0x%02X（%s）" % (or_all, "，".join(labels))
    if or_all & 0x01:
        detail += "；只读检测期间没人在发 LowCmd，bit0 置位是预期的"
    if or_all & 0x80:
        detail += "；bit7 说明电机通信超时，先查线束"
    return [Check(WARN, "bit_flag", detail, {"or": or_all, "seen": seen})]


def _check_lowcmd_publisher(smp: Samples, check_lowcmd: bool) -> List[Check]:
    """/lowcmd 是否有别的进程在发（只读检测最怕和别人的控制器撞车）。"""
    if not check_lowcmd:
        return [Check(SKIP, "lowcmd.publisher", "已用 --no-cmd-check 关闭 /lowcmd 检查", None)]
    lc = smp.lowcmd
    pubs = int(lc.publishers or 0)
    if lc.n <= 0 and pubs <= 0:
        return [Check(PASS, "lowcmd.publisher",
                      "观测期内没有任何进程发布 /lowcmd（只读检测期间的干净状态）", 0)]
    if pubs > 1:
        return [Check(WARN, "lowcmd.publisher",
                      "有 %d 个 /lowcmd 发布者：上电前必须确认不是两个控制器抢发" % pubs,
                      pubs)]
    if lc.n <= 0:
        return [Check(WARN, "lowcmd.publisher",
                      "检测到 %d 个 /lowcmd 发布者，但观测期内没收到消息"
                      "（可能只是空闲，或发布方 QoS 与订阅不匹配）" % pubs, pubs)]
    return [Check(WARN, "lowcmd.publisher",
                  "观测期内收到 %d 条 /lowcmd（有别的进程在发）：上机前确认它是谁、"
                  "会不会和你的控制器抢控制权" % lc.n, lc.n)]


def _check_lowcmd_crc(smp: Samples, check_lowcmd: bool) -> List[Check]:
    """/lowcmd 的 CRC 是否合法（crc=0 / 812 字节布局校验失败都致命）。"""
    if not check_lowcmd:
        return [Check(SKIP, "lowcmd.crc", "已用 --no-cmd-check 关闭 /lowcmd 检查", None)]
    lc = smp.lowcmd
    if lc.n <= 0:
        return [Check(SKIP, "lowcmd.crc", "没有 LowCmd 可校验（没有进程在发）", None)]
    if lc.crc_zero >= lc.n:
        return [Check(FAIL, "lowcmd.crc",
                      "%d 条 LowCmd 的 crc 全是 0（发布方没填 CRC）：真机固件大概率静默丢帧"
                      % lc.n, {"n": lc.n, "crc_zero": lc.crc_zero})]
    if lc.crc_bad > 0:
        return [Check(FAIL, "lowcmd.crc",
                      "%d/%d 条 CRC 校验失败：发布方 812 字节布局/算法不对（固件会丢帧）"
                      % (lc.crc_bad, lc.n),
                      {"n": lc.n, "crc_bad": lc.crc_bad})]
    if lc.crc_zero > 0:
        return [Check(WARN, "lowcmd.crc",
                      "%d/%d 条 crc=0（其余合法）：部分指令会被固件丢弃"
                      % (lc.crc_zero, lc.n), {"n": lc.n, "crc_zero": lc.crc_zero})]
    detail = "%d 条 LowCmd CRC 全部合法（812 字节布局）" % lc.n
    if lc.last_kp and all(abs(float(k)) < 1e-9 for k in lc.last_kp):
        detail += "；末条 kp 全 0 = 纯力矩下发，确认策略侧力矩幅值已限幅"
    return [Check(PASS, "lowcmd.crc", detail, {"n": lc.n})]


def _check_lidar_rate(smp: Samples) -> List[Check]:
    """L1 点云频率（真机 ≈11 Hz，仿真 15 Hz）。"""
    if not smp.lidar:
        return [Check(WARN, "lidar.rate",
                      "没收到点云：真机先确认 L1 已开（发 rt/utlidar/switch ON）且话题名正确；"
                      "仿真侧确认 --publish-sensors 打开", 0.0)]
    if len(smp.lidar) < 2:
        return [Check(SKIP, "lidar.rate",
                      "只收到 1 帧点云，无法估频率（延长 --duration）", len(smp.lidar))]
    hz, _gap = _rate_of([x.t for x in smp.lidar], smp.duration_s)
    exp = _expected_lidar_hz(smp)
    detail = "%s Hz（%d 帧）" % (_fmt(hz, 1), len(smp.lidar))
    if hz <= 0:
        return [Check(WARN, "lidar.rate", detail + "：时间戳异常，无法估频率", round(hz, 1))]
    if hz < 0.5 * exp or hz > 2.0 * exp:
        return [Check(WARN, "lidar.rate",
                      detail + "：偏离期望 ≈%.0f Hz（丢帧或驱动分频不对）" % exp,
                      round(hz, 1))]
    return [Check(PASS, "lidar.rate", detail + "（期望 ≈%.0f Hz）" % exp, round(hz, 1))]


def _check_lidar_frame_id(smp: Samples) -> List[Check]:
    """点云 frame_id 应对应雷达系（真机 utlidar_lidar，仿真 world）。"""
    if not smp.lidar:
        return [Check(SKIP, "lidar.frame_id", "没有点云帧可判", None)]
    ids = sorted({str(x.frame_id or "") for x in smp.lidar})
    exp = _expected_lidar_frame(smp)
    if ids == [exp]:
        return [Check(PASS, "lidar.frame_id", "frame_id=%s（符合 %s 期望）" % (exp, smp.target), exp)]
    if len(ids) == 1 and ids[0] in (LIDAR_FRAME_REAL, LIDAR_FRAME_SIM):
        return [Check(WARN, "lidar.frame_id",
                      "frame_id=%s，与 %s 期望的 %s 不同（下游会按这个坐标系做外参，别混用）"
                      % (ids[0], smp.target, exp), ids)]
    if len(ids) > 1:
        return [Check(WARN, "lidar.frame_id", "frame_id 不唯一：%s（多源点云混流）" % ids, ids)]
    if ids == [""]:
        return [Check(WARN, "lidar.frame_id", "frame_id 为空：点云没有坐标系，下游外参无从应用", ids)]
    return [Check(WARN, "lidar.frame_id",
                  "frame_id=%s，期望 %s（坐标系/外参会错）" % (ids[0], exp), ids)]


def _check_lidar_npoints(smp: Samples) -> List[Check]:
    """每帧点数是否合理（L1 单帧几千点；空帧说明雷达没出数据）。"""
    if not smp.lidar:
        return [Check(SKIP, "lidar.npoints", "没有点云帧可判", None)]
    npts = np.asarray([int(x.n_points) for x in smp.lidar], dtype=np.int64)
    med = float(np.median(npts))
    if med >= LIDAR_POINTS_MIN:
        return [Check(PASS, "lidar.npoints",
                      "每帧点数 中位=%d 最大=%d" % (int(med), int(np.max(npts))),
                      int(med))]
    if int(np.max(npts)) <= 0:
        return [Check(FAIL, "lidar.npoints",
                      "所有点云帧 width*height=0（空帧）：雷达没出数据/字段没解析", 0)]
    return [Check(WARN, "lidar.npoints",
                  "每帧点数 中位=%d < %.0f（偏少，可能是近距离遮挡或降采样）"
                  % (int(med), LIDAR_POINTS_MIN), int(med))]


def _check_lidar_xyz(smp: Samples) -> List[Check]:
    """xyz 分位有限、非退化、量级合理（用于标定外参）。"""
    frames = [x for x in smp.lidar if x.p50 is not None]
    if not frames:
        return [Check(SKIP, "lidar.xyz", "没有点云 xyz 分位（采集壳未计算）", None)]
    p50 = np.asarray([np.asarray(x.p50, dtype=np.float64).reshape(-1)[:3] for x in frames])
    p1 = np.asarray([np.asarray(x.p1 if x.p1 is not None else x.p50, dtype=np.float64)
                     .reshape(-1)[:3] for x in frames])
    p99 = np.asarray([np.asarray(x.p99 if x.p99 is not None else x.p50, dtype=np.float64)
                      .reshape(-1)[:3] for x in frames])
    if np.any(~np.isfinite(p50)) or np.any(~np.isfinite(p1)) or np.any(~np.isfinite(p99)):
        return [Check(FAIL, "lidar.xyz", "xyz 分位出现 NaN/Inf", None)]
    med = np.median(p50, axis=0)
    span = np.max(np.abs(p99 - p1), axis=0) if p99.size else np.zeros(3)
    detail = "xyz 中位分位 p50=%s m，p1~p99 跨度=%s m" % (
        [round(float(x), 3) for x in med], [round(float(x), 3) for x in span])
    if float(np.max(np.abs(med))) > LIDAR_P99_MAX_M or float(np.max(np.abs(p99))) > LIDAR_P99_MAX_M:
        return [Check(WARN, "lidar.xyz",
                      detail + "：量级 >%.0f m，疑似单位/坐标系错" % LIDAR_P99_MAX_M,
                      [round(float(x), 3) for x in med])]
    if float(np.max(span)) <= 1e-9:
        return [Check(WARN, "lidar.xyz",
                      detail + "：所有点重合（p1==p99），点云退化", [round(float(x), 3) for x in med])]
    return [Check(PASS, "lidar.xyz", detail, [round(float(x), 3) for x in med])]


def _check_topic(smp: Samples, key: str, label: str, absent: str, missing_status: str) -> List[Check]:
    """通用话题存在性判定（pose/odom/sportmodestate/lidar_state 共用）。"""
    n = int((smp.topics or {}).get(key, 0) or 0)
    if n > 0:
        return [Check(PASS, label, "收到 %d 条" % n, n)]
    return [Check(missing_status, label, absent, 0)]


def _check_power(smp: Samples) -> List[Check]:
    """电池电压（消息里有 power_v 才判）。"""
    vals = []
    cur = []
    for s in smp.lowstate:
        v = getattr(s, "power_v", None)
        try:
            fv = float(v) if v is not None else float("nan")
        except (TypeError, ValueError):
            fv = float("nan")
        if math.isfinite(fv) and fv > 0.0:
            vals.append(fv)
        a = getattr(s, "power_a", None)
        try:
            fa = float(a) if a is not None else float("nan")
        except (TypeError, ValueError):
            fa = float("nan")
        if math.isfinite(fa):
            cur.append(fa)
    if not vals:
        return [Check(SKIP, "power",
                      "LowState 没有可用的 power_v（该固件/消息不含电池电压）", None)]
    v = float(np.median(vals))
    detail = "电压中位 %.2f V" % v
    if cur:
        detail += "，电流中位 %.2f A" % float(np.median(cur))
    if v < POWER_LOW_FAIL:
        return [Check(FAIL, "power",
                      detail + "：低于保护电压 %.1f V，先充电" % POWER_LOW_FAIL, round(v, 2))]
    if v < POWER_LOW_WARN:
        return [Check(WARN, "power",
                      detail + "：电量偏低（<%.1f V），建议先充电" % POWER_LOW_WARN, round(v, 2))]
    if v > POWER_FULL_WARN:
        return [Check(WARN, "power",
                      detail + "：高于满电电压，可能正在充电（充电时别上机）",
                      round(v, 2))]
    return [Check(PASS, "power", detail, round(v, 2))]


# ================================================================ 主入口
def analyze(samples: Any,
            target: Optional[str] = None,
            tick_step: Optional[int] = None,
            lidar_hz: Optional[float] = None,
            check_lowcmd: bool = True) -> Report:
    """对一批采样做全项判定，返回 Report。

    Args:
        samples: Samples 实例或等价的 dict（见 Samples.from_dict）。
        target:  覆盖 samples.target（real_ros / sim_ros）。
        tick_step: 覆盖期望 Δtick（真机 1 / 仿真 2）。
        lidar_hz: 覆盖期望点云频率。
        check_lowcmd: False 时 /lowcmd 两项给 SKIP（对应 --no-cmd-check）。
    Returns:
        Report（checks 顺序固定；meta 带规模与期望值，便于 --json 归档）。
    """
    if not isinstance(samples, Samples):
        if isinstance(samples, Mapping):
            samples = Samples.from_dict(samples)
        else:
            raise TypeError("analyze() 需要 Samples 或 dict，收到 %r" % type(samples))
    if target is not None:
        samples.target = str(target)
    if tick_step is not None:
        samples.tick_step = int(tick_step)
    if lidar_hz is not None:
        samples.lidar_hz = float(lidar_hz)
    # 采样项也允许是 dict
    if samples.lowstate and not isinstance(samples.lowstate[0], LowStateSample):
        samples.lowstate = [_as_sample(x, LowStateSample) for x in samples.lowstate]
    if samples.lidar and not isinstance(samples.lidar[0], LidarSample):
        samples.lidar = [_as_sample(x, LidarSample) for x in samples.lidar]

    checks: List[Check] = []
    checks += _check_lowstate_rate(samples)
    checks += _check_lowstate_tick(samples)
    checks += _check_motors_mode(samples)
    checks += _check_motors_lost(samples)
    checks += _check_motors_temperature(samples)
    checks += _check_motors_q(samples)
    checks += _check_motors_dq(samples)
    checks += _check_motors_tau(samples)
    checks += _check_imu_rpy(samples)
    checks += _check_imu_quat(samples)
    checks += _check_imu_accel(samples)
    checks += _check_imu_gyro(samples)
    checks += _check_foot_force(samples)
    checks += _check_bit_flag(samples)
    checks += _check_lowcmd_publisher(samples, check_lowcmd)
    checks += _check_lowcmd_crc(samples, check_lowcmd)
    checks += _check_lidar_rate(samples)
    checks += _check_lidar_frame_id(samples)
    checks += _check_lidar_npoints(samples)
    checks += _check_lidar_xyz(samples)
    checks += _check_topic(samples, "pose", "pose",
                           "没收到位姿（真机常见：官方固件 robot_pose 经常为空，"
                           "控制器应回退 leg_fk 而不是报错）", SKIP)
    checks += _check_topic(samples, "odom", "odom",
                           ("没收到 odom（真机常见：robot_odom 常为空，需接 LIO/EKF；"
                            "仿真应检查 --publish-sensors）"
                            if not _target_is_sim(samples.target)
                            else "仿真没收到 odom：检查 --publish-sensors 与 odom 话题名"),
                           WARN if _target_is_sim(samples.target) else SKIP)
    checks += _check_topic(samples, "sportmodestate", "sportmodestate",
                           "没收到 SportModeState（释放 sport mode 后不可读，属预期）", SKIP)
    checks += _check_topic(samples, "lidar_state", "lidar_state",
                           "没收到 LidarState（部分固件/驱动不提供，属预期）", SKIP)
    checks += _check_power(samples)

    # ---- meta：给 --json 归档和排障用，全部 JSON 可序列化
    state_hz, state_gap = _rate_of([s.t for s in samples.lowstate], samples.duration_s)
    lidar_hz_val, _ = _rate_of([x.t for x in samples.lidar], samples.duration_s)
    meta: Dict[str, Any] = {
        "target": samples.target,
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "duration_s": round(float(samples.duration_s or 0.0), 3),
        "expected_state_hz": STATE_HZ_EXPECTED,
        "expected_tick_step": _expected_tick_step(samples),
        "expected_lidar_hz": _expected_lidar_hz(samples),
        "expected_lidar_frame": _expected_lidar_frame(samples),
        "n_lowstate": len(samples.lowstate),
        "lowstate_hz": round(float(state_hz), 2),
        "lowstate_max_gap_ms": round(float(state_gap) * 1e3, 1),
        "n_lowcmd": int(samples.lowcmd.n),
        "n_lidar": len(samples.lidar),
        "lidar_hz": round(float(lidar_hz_val), 2),
        "topics": dict(samples.topics or {}),
        "check_lowcmd": bool(check_lowcmd),
    }
    if samples.notes:
        meta["notes"] = list(samples.notes)
    return Report(checks=checks, meta=meta)


# ================================================================ 文本渲染
def _display_width(s: str) -> int:
    """显示宽度：CJK 全角算 2，其余算 1（用于中文表格对齐）。"""
    import unicodedata

    w = 0
    for ch in str(s):
        if unicodedata.east_asian_width(ch) in ("W", "F"):
            w += 2
        elif unicodedata.combining(ch):
            continue
        else:
            w += 1
    return w


def _pad(s: str, width: int) -> str:
    """按显示宽度左对齐补空格。"""
    s = str(s)
    return s + " " * max(1, width - _display_width(s))


def _verdict_text(rep: Report) -> Tuple[str, str]:
    """返回 (结论标题, 建议行)。"""
    cnt = rep.counts()
    if cnt.get(FAIL, 0):
        return ("不通过 —— 有 %d 项 FAIL，先修再上机" % cnt[FAIL],
                "按下面的 FAIL 逐条处理；FAIL 项没清掉不要发 LowCmd。")
    if cnt.get(WARN, 0):
        return ("可用但有风险 —— %d 项 WARN，注意观察" % cnt[WARN],
                "WARN 项不影响只读检测，但上机时可能演化成故障，建议先确认原因。")
    return ("全部通过", "没有任何 FAIL/WARN，可以按 checklist 继续下一步。")


def render_text(rep: Report) -> str:
    """把 Report 渲染成中文对齐表格 + 结论 + "上机前还差什么"。

    Args:
        rep: analyze() 的返回值。
    Returns:
        多行字符串（调用方直接 print）。
    """
    if not isinstance(rep, Report):
        raise TypeError("render_text() 需要 Report，收到 %r" % type(rep))
    meta = rep.meta or {}
    lines: List[str] = []
    title = "Go2 传感器只读自检报告"
    lines.append("=" * (max(_display_width(title) + 16, 64)))
    lines.append("  " + title)
    lines.append("=" * (max(_display_width(title) + 16, 64)))
    head = "目标=%s  观测=%.2f s  /lowstate=%d 条 (%s Hz)  /lowcmd=%d 条  点云=%d 帧 (%s Hz)" % (
        meta.get("target", "?"), float(meta.get("duration_s", 0.0) or 0.0),
        int(meta.get("n_lowstate", 0) or 0), _fmt(meta.get("lowstate_hz", 0.0), 1),
        int(meta.get("n_lowcmd", 0) or 0), int(meta.get("n_lidar", 0) or 0),
        _fmt(meta.get("lidar_hz", 0.0), 1))
    lines.append(head)
    if meta.get("generated_at"):
        lines.append("时间=%s  期望：Δtick=%s  点云≈%s Hz  frame_id=%s" % (
            meta.get("generated_at"), meta.get("expected_tick_step"),
            _fmt(meta.get("expected_lidar_hz", 0.0), 0), meta.get("expected_lidar_frame")))
    lines.append("-" * 96)
    # 列宽按"显示宽度"算：中文状态/检查项在终端里占 2 列，用 %-Ns 会错位。
    name_w = max([24] + [_display_width(c.name) for c in rep.checks]) + 2

    def _row(idx: str, status: str, name: str, detail: str) -> str:
        return "%s %s %s %s" % (_pad(idx, 3), _pad(status, 6), _pad(name, name_w), detail)

    lines.append(_row("#", "状态", "检查项", "详情"))
    lines.append("-" * 96)
    for i, c in enumerate(rep.checks, 1):
        lines.append(_row(str(i), c.status, c.name, c.detail))
    lines.append("-" * 96)
    cnt = rep.counts()
    lines.append("统计: PASS %d  WARN %d  FAIL %d  SKIP %d" % (
        cnt.get(PASS, 0), cnt.get(WARN, 0), cnt.get(FAIL, 0), cnt.get(SKIP, 0)))
    verdict, advice = _verdict_text(rep)
    lines.append("结论: " + verdict)
    lines.append("      " + advice)
    lines.append("")
    lines.append("上机前还差什么:")
    todo = [c for c in rep.checks if c.status in (FAIL, WARN)]
    if not todo:
        lines.append("  （无：FAIL/WARN 都是 0）")
    else:
        order = {FAIL: 0, WARN: 1}
        for c in sorted(todo, key=lambda x: order[x.status]):
            lines.append("  [%s] %s: %s" % (c.status, c.name, c.detail))
    skipped = [c.name for c in rep.checks if c.status == SKIP]
    if skipped:
        lines.append("")
        lines.append("未参与判定(SKIP): %s" % ", ".join(skipped))
        lines.append("  说明: SKIP = 本机/本固件没有这个信号（不是错误）；若它在 --require 里，")
        lines.append("        退出码会降级成 WARN —— 相当于「没验证过」，不要当作通过。")
    return "\n".join(lines)


# ================================================================ 合成样本（--selftest/单测）
def synthetic_samples(target: str = "real_ros",
                      duration_s: float = 5.0,
                      hz: float = STATE_HZ_EXPECTED,
                      lidar_hz: Optional[float] = None,
                      seed: int = 0,
                      with_lidar: bool = True) -> Samples:
    """造一份"全部正常"的合成采样，用于 --selftest 与单测基线。

    真机档：tick 1 ms/拍、温度 35 ℃、足端力非 0、电池 28.8 V、无 odom/pose。
    仿真档：tick 2 ms/拍、温度 0（仿真不填）、电池 None、有 odom（50 Hz）。
    点云：真机 frame_id=utlidar_lidar，仿真 frame_id=world。

    Args:
        target: real_ros / sim_ros。
        duration_s: 合成时长（秒）。
        hz: /lowstate 频率。
        lidar_hz: 点云频率；None 按 target 取 11/15。
        seed: 随机种子（噪声可复现）。
        with_lidar: False 时造一个"点云缺失"的样本（用于演示 WARN）。
    """
    rng = np.random.default_rng(int(seed))
    sim = _target_is_sim(target)
    n = max(int(round(float(duration_s) * float(hz))), 2)
    step = 2 if sim else 1
    temp = 0 if sim else 35
    power = None if sim else 28.8
    samples = Samples(target=target, duration_s=float(duration_s),
                      tick_step=step,
                      lidar_hz=(LIDAR_HZ_SIM if sim else LIDAR_HZ_REAL))
    for i in range(n):
        q = HOME_Q + rng.normal(0.0, 0.004, NUM_JOINT)
        samples.lowstate.append(LowStateSample(
            t=i / float(hz),
            tick=i * step,
            mode=[0x01] * NUM_JOINT,
            q=q.tolist(),
            dq=rng.normal(0.0, 0.01, NUM_JOINT).tolist(),
            ddq=rng.normal(0.0, 0.05, NUM_JOINT).tolist(),
            tau_est=rng.normal(0.0, 0.4, NUM_JOINT).tolist(),
            temperature=[temp] * NUM_JOINT,
            lost=[0] * NUM_JOINT,
            foot_force=[50.0, 48.0, 52.0, 49.0],
            rpy=[rng.normal(0.0, 0.002), rng.normal(0.0, 0.002), 0.0],
            quat=[1.0, 0.0, 0.0, 0.0],
            gyro=rng.normal(0.0, 0.002, 3).tolist(),
            accel=(np.array([0.0, 0.0, ACCEL_G]) + rng.normal(0.0, 0.02, 3)).tolist(),
            bit_flag=0,
            power_v=power,
            power_a=None if sim else 3.0,
        ))
    if with_lidar:
        lhz = float(lidar_hz if lidar_hz else (LIDAR_HZ_SIM if sim else LIDAR_HZ_REAL))
        nf = max(int(round(float(duration_s) * lhz)), 2)
        for i in range(nf):
            samples.lidar.append(LidarSample(
                t=i / lhz,
                stamp=i / lhz,
                frame_id=LIDAR_FRAME_SIM if sim else LIDAR_FRAME_REAL,
                n_points=3000,
                p1=[-4.0, -4.0, -0.35],
                p50=[1.5, 0.2, -0.05],
                p99=[5.2, 4.8, 0.9],
            ))
    samples.topics = {"sportmodestate": n if sim else 0,
                      "odom": 250 if sim else 0,
                      "pose": 0,
                      "lidar_state": 0}
    samples.notes = ["合成样本（--selftest / 单测用），不代表真实数据"]
    return samples


def selftest_report(target: str = "real_ros") -> Tuple[Report, str]:
    """--selftest 用：造好样本跑一遍并给文本，方便脚本一行调用。"""
    rep = analyze(synthetic_samples(target=target))
    return rep, render_text(rep)


def dumps(rep: Report) -> str:
    """报告 -> JSON 字符串（UTF-8，中文不转义，禁止 NaN）。"""
    return json.dumps(rep.to_dict(), ensure_ascii=False, indent=2, allow_nan=False)


__all__ = [
    "Check", "Report", "LowStateSample", "LowCmdSamples", "LidarSample", "Samples",
    "analyze", "render_text", "synthetic_samples", "selftest_report", "dumps",
    "PASS", "WARN", "FAIL", "SKIP", "STATUSES", "REQUIRE_GROUPS",
    "MOTOR_NAMES", "FOOT_ORDER", "HOME_Q", "JOINT_LO", "JOINT_HI", "TAU_LIMIT",
]
