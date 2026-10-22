"""91 维行走观测构造 —— 训练 (mjx-go2-training/envs/go2_walk.py) 与部署共用同一份定义。

观测布局 (WALK_OBS_DIM = 91 = 48 + 35 + 8)，顺序/缩放严格照抄 go2_walk.py 的
_get_obs / height_scan_features / step 里的相位推进 (已逐行核对源码):

    0:3    global_linvel  * 2.0        # data.qvel[0:3]  世界系线速度 (oracle/里程计)
    3:6    global_angvel  * 0.25       # data.qvel[3:6]  free joint: 角速度在**机体系**
    6:9    projected_gravity           # site_xmat[imu].T @ [0,0,-1]，直立 ≈ [0,0,-1]
    9:12   command        * 2.0        # 训练固定 [0.5, 0, 0]
   12:24   qpos[7:] - key_ctrl[0]      # 关节角 - 训练 home (Go2 的 home, 不是 Go2 的)
   24:36   qvel[6:]                    # 关节速度，**不乘任何 scale**
   36:48   last_action (12)            # 上一步策略输出 (原始动作, 非 ctrl)
   48:83   height_scan (35)            # clip(z_base - h_ground - 0.28, ±0.35) * 5.0
   83:87   cos(phi_i) (4)
   87:91   sin(phi_i) (4)
   最后统一 clip 到 ±100

三个易错点 (已逐条核对源码, 不要照抄 go2_walk.py 文件头的旧注释):
 1. obs_scale.dofs_velocity = 0.05 在代码里**没有被使用** —— _get_obs 里
    dofs_velocity = data.qvel[6:] 原样进 obs。本项目必须同样不缩放。
 2. 高度扫描网格 np.meshgrid(gx, gy, indexing="ij").ravel() → **x 外层、y 内层**，
    index = i * ny + j。
 3. 相位是**每腿一个** phi_i = trot_i + Phi，trot = [0, pi, pi, 0] (FR,FL,RR,RL)；
    全局相位 Phi 以 2*pi*gait_freq (1.8 Hz) 推进，并归一化到 [-pi, pi)。

只依赖 numpy + 标准库 (禁止 jax/brax/rclpy)。
"""

from __future__ import annotations

import os

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from signal_io.msg import NUM_JOINT, LowState

__all__ = [
    "WALK_OBS_DIM", "LEG", "TROT_PHASE", "TRAIN_HOME_Q",
    "WalkObsConfig", "ObsMeta", "WalkObsBuilder",
    "projected_gravity", "phase_encode", "height_scan_feature",
]

# ---------------------------------------------------------------- 常量

WALK_OBS_DIM = 91

#: 腿顺序 (与 msg.MOTOR_JOINTS 的 3 关节分组一致)
LEG: Tuple[str, ...] = ("FR", "FL", "RR", "RL")

#: trot 步态每腿的固定相位偏置 (FR, FL, RR, RL) —— 对角腿同相
TROT_PHASE = np.array([0.0, np.pi, np.pi, 0.0], dtype=np.float64)

#: 训练 home (mj_model.key_ctrl[0]，Go2 的，不是 Go2 的):
#: [0.0, 0.8, -1.5] * 2 (FR,FL) + [0.0, 1.0, -1.5] * 2 (RR,RL)
TRAIN_HOME_Q = np.array(
    [0.0, 0.8, -1.5] * 2 + [0.0, 1.0, -1.5] * 2, dtype=np.float64)

_TWO_PI = 2.0 * np.pi


# ---------------------------------------------------------------- 纯函数


def projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    """机体系重力方向 R^T @ [0,0,-1]，直立时 ≈ [0,0,-1]。

    quat_wxyz = LowState.imu_state.quaternion (w,x,y,z)；与训练侧
    site_xmat[imu].T @ [0,0,-1] 等价 (imu site 无附加旋转时)。
    """
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, -1.0])
    w, x, y, z = q / n
    # R (机体系 -> 世界系, 见 quat_to_mat)。R^T @ [0,0,-1] = -R[2,:] = -第三**行**。
    # 2026-10-20 修复: 原实现写成 -[R02, R12, R22] (第三**列**) —— 纯 yaw 时两者
    # 恰好都等于 [0,0,-1], 但一旦有 roll/pitch, 重力 xy 的**符号反了**。
    # 训练侧 get_gravity = site_xmat[imu].T @ [0,0,-1] (= 本式); 对账脚本
    # /tmp/obs_parity.py 实测修复前随机姿态 max|diff|=0.627, 修复后 0。
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.array([-r20, -r21, -r22], dtype=np.float64)


def quat_to_mat(quat_wxyz: np.ndarray) -> np.ndarray:
    """(w,x,y,z) -> 3x3 旋转矩阵 (机体系 -> 世界系)。"""
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def phase_encode(phase: np.ndarray) -> np.ndarray:
    """[cos(phi_0..3), sin(phi_0..3)] —— 与训练 jp.concatenate([cos, sin]) 同序。"""
    ph = np.asarray(phase, dtype=np.float64).reshape(4)
    return np.concatenate([np.cos(ph), np.sin(ph)]).astype(np.float32)


def height_scan_feature(z_base, h_ground, base: float = 0.28,
                        clip_m: float = 0.35, scale: float = 5.0):
    """训练侧 height_scan 特征: clip(z_base - h_ground - base, ±clip_m) * scale。

    注意 (契约 §1.2 已更正): **平地上并不恒为 0** —— 平地 h_ground=0 时
    特征 = clip(z_base - 0.28, ±0.35) * 5.0，只有 z_base == 0.28 才为 0:
        z_base=0.35 -> +0.35 ; z_base=0.20 -> -0.40 ; z_base=1.0 -> 饱和 +1.75
    地面比标称高 (h_ground>0) 时特征为负 (前方更高/上坡)。

    这是给感知层/自检用的纯函数; observe() 仍接收**已算好**的 (35,) 特征。
    """
    d = (np.asarray(z_base, dtype=np.float64)
         - np.asarray(h_ground, dtype=np.float64) - float(base))
    return np.clip(d, -float(clip_m), float(clip_m)) * float(scale)


def _wrap_pi(x):
    """归一化到 [-pi, pi) —— 与训练 fmod(phi+pi, 2pi)-pi 同义。"""
    return np.mod(np.asarray(x, dtype=np.float64) + np.pi, _TWO_PI) - np.pi


# ---------------------------------------------------------------- 配置


@dataclass
class WalkObsConfig:
    linear_velocity_scale: float = 2.0
    angular_velocity_scale: float = 0.25
    obs_clip: float = 100.0
    command: Tuple[float, float, float] = (0.5, 0.0, 0.0)
    include_height_scan: bool = True
    include_gait_phase: bool = True
    gait_freq: float = 1.8
    gait_freq_scaled_by_cmd: bool = False   # False: 恒定步频
    gait_freq_min: float = 0.6
    scan_noise: float = 0.0                 # 对称均匀噪声半宽 (m)
    # height scan 几何
    scan_nx: int = 7
    scan_ny: int = 5
    scan_x_range: Tuple[float, float] = (-0.3, 0.9)
    scan_y_range: Tuple[float, float] = (-0.4, 0.4)
    scan_base: float = 0.28
    scan_clip_m: float = 0.35
    scan_scale: float = 5.0
    # v27: 观测历史帧数 (1 = 无历史)。历史内容 = 48 base + 8 phase (若有),
    # 不含高度扫描; obs 增加 (history_frames-1)*(48+8) 维, 与训练侧一致。
    history_frames: int = 1
    # 观测 0:3 的线速度用哪个坐标系。
    #   "world"  = data.qvel[0:3] (go2_walk 的 frame="global", 框架原有语义)
    #   "body"   = R^T @ v_world     (go2_walk 的 frame="body")
    # Go2 训练用的是 body (允许 vy/wz 指令后必须用机体系, 否则"边走边转"不可实现)。
    # 喂错帧在小偏航时几乎看不出来, 但一转向就会明显退化 —— 必须按训练机型取。
    linvel_frame: str = "world"


@dataclass
class ObsMeta:
    """观测是否可用 / 是否降级 —— 给健康层与日志用。"""

    valid: bool
    reasons: Tuple[str, ...]          # 空 = 全好
    linvel_source: str                # "oracle" | "estimator" | "zero"
    height_scan_source: str           # "raycast" | "external" | "zero"
    age_s: float                      # 最旧一路输入的年龄


# ---------------------------------------------------------------- 构造器


class WalkObsBuilder:
    """把 LowState + 基座线速度 (+ 高程扫描) 拼成 91 维策略观测。

    约定 (对齐训练侧语义):
      * reset() 后 last_action 清零、全局步态相位随机 U(0, 2*pi)；
      * observe() 用 t 的增量推进相位 (每次控制步调用一次即等效于训练
        phase += 2*pi*f*dt)；t 必须单调不减；
      * commit(action) 在**动作下发之后**调用，记的是策略原始输出 (未 clip)，
        下一步的 obs 才会带上它 —— 与训练 info["last_act"] = action 在
        _get_obs 之后写入同序。
    """

    def __init__(self, obs_cfg: Optional[WalkObsConfig] = None,
                 home_q: Optional[np.ndarray] = None) -> None:
        self.cfg = obs_cfg if obs_cfg is not None else WalkObsConfig()
        if home_q is None:
            home_q = TRAIN_HOME_Q
        hq = np.asarray(home_q, dtype=np.float64).reshape(-1)
        if hq.size != NUM_JOINT:
            raise ValueError("home_q 必须是 %d 维，得到 %d" % (NUM_JOINT, hq.size))
        self.home_q = hq.copy()

        # 相位/噪声共用的 RNG: 只在这里建一次, reset() 从序列里继续取 ->
        # 同一 builder 的序列可复现, 但每次 reset 的相位不同 (训练侧同义)
        self._rng = np.random.default_rng(0)

        # 高度扫描采样网格 (机体系 xy, 米)。meshgrid(indexing="ij") -> x 外层 y 内层
        c = self.cfg
        gx = np.linspace(c.scan_x_range[0], c.scan_x_range[1], c.scan_nx)
        gy = np.linspace(c.scan_y_range[0], c.scan_y_range[1], c.scan_ny)
        GX, GY = np.meshgrid(gx, gy, indexing="ij")
        self._scan_xy = np.stack([GX.ravel(), GY.ravel()], axis=-1).astype(np.float64)
        self._scan_n = int(c.scan_nx * c.scan_ny)

        self._cmd = np.asarray(c.command, dtype=np.float64).reshape(3)

        self.obs_dim = 48
        if c.include_height_scan:
            self.obs_dim += self._scan_n
        if c.include_gait_phase:
            self.obs_dim += 8
        # v27: 历史帧缓冲维度 (base48 + phase8, 不含 scan)
        self._hist_frame_dim = 48 + (8 if c.include_gait_phase else 0)
        self._hist_frames = max(1, int(c.history_frames))
        self._hist_dim = (self._hist_frames - 1) * self._hist_frame_dim
        self.obs_dim += self._hist_dim

        self.reset()

    # -------------------------------------------------------- 状态复位

    def reset(self) -> None:
        """last_action 清零，相位随机化，时间基准清空。"""
        self.last_action = np.zeros(NUM_JOINT, dtype=np.float64)
        # reset 时 Phi ~ U(0, 2pi)：与训练一致 (避免策略依赖固定起始相位)
        self._phase = float(self._rng.uniform(0.0, _TWO_PI))
        self._t_prev: Optional[float] = None
        self._hist = np.zeros(self._hist_dim, dtype=np.float64)
        self._hist_ready = False
        self._stats = {
            "n_observe": 0, "n_state_none": 0, "n_linvel_zero": 0,
            "n_scan_none": 0, "n_nonfinite": 0, "n_clipped": 0,
            "n_scan_noised": 0,
        }

    # -------------------------------------------------------- 相位

    @property
    def phase(self) -> float:
        """当前全局步态相位 Phi (rad)。每腿相位 = TROT_PHASE + Phi。"""
        return float(self._phase)

    def set_phase(self, phi: float) -> None:
        """显式设置全局相位 (测试/复现用；契约之外的附加 API，不改契约签名)。"""
        self._phase = float(_wrap_pi(phi))

    def _effective_gait_freq(self) -> float:
        c = self.cfg
        if not c.gait_freq_scaled_by_cmd:
            return float(c.gait_freq)
        spd = float(np.linalg.norm(self._cmd[:2]))
        return float(max(c.gait_freq * spd / 0.5, c.gait_freq_min))

    def _advance_phase(self, t: float) -> None:
        # 相位按**真实流逝时间**推进 (与训练 step 里的 phase += 2*pi*f*dt 同义;
        # 部署时每控制步调用一次 observe, dt 即控制周期)。t 回退时按 0 处理。
        if self._t_prev is None:
            dt = 0.0
        else:
            dt = max(float(t) - self._t_prev, 0.0)
        self._t_prev = float(t)
        if dt > 0.0:
            self._phase = float(_wrap_pi(
                self._phase + _TWO_PI * self._effective_gait_freq() * dt))

    @property
    def leg_phases(self) -> np.ndarray:
        """(4,) 每腿相位 phi_i = trot_i + Phi (FR,FL,RR,RL)。"""
        return _wrap_pi(TROT_PHASE + self._phase)

    # -------------------------------------------------------- 扫描网格

    @property
    def scan_n(self) -> int:
        """高度扫描采样点数 (7x5=35 / 17x11=187)。"""
        return int(self._scan_n)

    def scan_grid(self) -> np.ndarray:
        """(scan_n,2) 机体系采样点 (x 外层 y 内层)，供感知层 HeightMapper 复用。"""
        return self._scan_xy.copy()

    def scan_feature(self, z_base, h_ground) -> np.ndarray:
        """用本配置的 base/clip_m/scale 算 height_scan 特征 (与训练同定义)。

        平地上 h_ground=0 时**不是 0**，而是 clip(z_base-0.28, ±0.35)*5.0。
        """
        c = self.cfg
        return height_scan_feature(z_base, h_ground,
                                   base=c.scan_base, clip_m=c.scan_clip_m,
                                   scale=c.scan_scale)

    # -------------------------------------------------------- 观测

    def observe(self, state: Optional[LowState], t: float, *,
                base_lin_vel_world: Optional[np.ndarray],
                height_scan: Optional[np.ndarray] = None,
                ) -> Tuple[np.ndarray, ObsMeta]:
        """返回 (obs(float32, obs_dim), meta)。

        state is None -> valid=False 的全 0 obs (契约 §2)。
        height_scan is None -> 全 0 (平地); base_lin_vel_world is None -> 0。
        """
        c = self.cfg
        reasons = []
        self._advance_phase(t)
        self._stats["n_observe"] += 1

        # ---- 缺状态: 契约要求 valid=False 的全 0 obs
        if state is None:
            self._stats["n_state_none"] += 1
            return (np.zeros(self.obs_dim, dtype=np.float32),
                    ObsMeta(valid=False, reasons=("state_none",),
                            linvel_source="zero", height_scan_source="zero",
                            age_s=float("inf")))

        # ---- 线速度输入
        if base_lin_vel_world is None:
            lin_vel = np.zeros(3)
            linvel_source = "zero"
            reasons.append("linvel_missing")
        else:
            lin_vel = np.asarray(base_lin_vel_world, dtype=np.float64).reshape(-1)
            if lin_vel.size != 3:
                raise ValueError("base_lin_vel_world 必须是 3 维，得到 %d" % lin_vel.size)
            if not np.all(np.isfinite(lin_vel)):
                reasons.append("linvel_nonfinite")
                lin_vel = np.zeros(3)
                linvel_source = "zero"
            elif not np.any(lin_vel):
                linvel_source = "zero"
            else:
                # 契约签名无法区分 oracle/estimator (两者都是 (3,) 数组)，
                # 这里按"非零 = 外部估计/真值"记为 estimator，零记为 zero。
                linvel_source = "estimator"

        # ---- 高度扫描输入
        if not c.include_height_scan:
            scan = None
            scan_source = "zero"
        elif height_scan is None:
            scan = np.zeros(self._scan_n, dtype=np.float64)
            scan_source = "zero"
            reasons.append("height_scan_none")
            self._stats["n_scan_none"] += 1
        else:
            scan = np.asarray(height_scan, dtype=np.float64).reshape(-1)
            if scan.size != self._scan_n:
                raise ValueError(
                    "height_scan 必须是 %d 维，得到 %d" % (self._scan_n, scan.size))
            scan_source = "raycast"
            if not np.all(np.isfinite(scan)):
                reasons.append("scan_nonfinite")
                scan = np.where(np.isfinite(scan), scan, 0.0)
            if c.scan_noise > 0.0:
                # 训练侧噪声加在**地面高度**上 (±noise 米)，特征线性区等价于
                # 特征上 ±noise*scale；乘完再按米制 clip 语义截断。
                # 部署时把 scan_noise 设 0 即可完全关掉 (默认就是 0.0)。
                n = self._rng.uniform(-1.0, 1.0, size=scan.shape)
                scan = np.clip(scan + n * (c.scan_noise * c.scan_scale),
                               -c.scan_clip_m * c.scan_scale,
                               c.scan_clip_m * c.scan_scale)
                self._stats["n_scan_noised"] += 1

        # ---- 线速度坐标系: 训练是 body 帧时, 把世界系速度转到机体系
        # (go2_walk.py 的 command_config.frame="body" 走这条; go2 仍是 world)
        if c.linvel_frame == "body" and linvel_source != "zero":
            lin_vel = quat_to_mat(state.imu_state.quaternion).T @ lin_vel

        # ---- 48 维 base
        q = state.q12()
        dq = state.dq12()
        gyro = np.asarray(state.imu_state.gyroscope, dtype=np.float64).reshape(-1)
        if gyro.size != 3:
            raise ValueError("imu_state.gyroscope 必须是 3 维")

        base = np.concatenate([
            lin_vel * c.linear_velocity_scale,               # 0:3
            gyro * c.angular_velocity_scale,                 # 3:6
            projected_gravity(np.asarray(state.imu_state.quaternion, dtype=np.float64)),  # 6:9
            self._cmd * c.linear_velocity_scale,             # 9:12
            q - self.home_q,                                 # 12:24
            dq,                                              # 24:36 (不缩放!)
            self.last_action,                                # 36:48
        ])

        parts = [base]
        if scan is not None:
            parts.append(scan)
        if c.include_gait_phase:
            parts.append(phase_encode(self.leg_phases))

        # v27: 历史帧 —— 与训练侧同序: base48 + phase8 组成一帧, 最近帧在前,
        # 追加在 obs 末尾; 本步 obs 用**更新前**的缓冲, 更新在 obs 组装后。
        if self._hist_dim > 0:
            _frame = (np.concatenate([base, phase_encode(self.leg_phases)])
                      if c.include_gait_phase else base)
            if not self._hist_ready:
                # 训练侧 reset 会先算一次 obs 并把 reset 帧写进历史缓冲 ->
                # 第一步推理看到的历史是 [frame_reset, 0...]。部署没有 reset 观测,
                # 这里在第一次 observe 时等价补上 (否则策略首步看到全 0 历史, OOD)。
                _n = self._hist_frames
                _z = np.zeros(max(0, _n - 2) * self._hist_frame_dim)
                self._hist = (np.concatenate([_frame, _z]) if _n > 1
                              else np.zeros(0))
                self._hist_ready = True
            parts.append(self._hist.copy())
            _n = self._hist_frames
            if _n > 1:
                self._hist = np.concatenate(
                    [_frame, self._hist[: (_n - 2) * self._hist_frame_dim]])

        obs = np.concatenate(parts).astype(np.float64)
        if obs.size != self.obs_dim:                     # pragma: no cover
            raise AssertionError("内部布局错误: %d != %d" % (obs.size, self.obs_dim))

        n_bad = int(np.count_nonzero(~np.isfinite(obs)))
        if n_bad:
            reasons.append("nonfinite_input")
            self._stats["n_nonfinite"] += 1
            obs = np.where(np.isfinite(obs), obs, 0.0)

        pre_clip = obs.copy()
        obs = np.clip(obs, -c.obs_clip, c.obs_clip)
        self._stats["n_clipped"] += int(np.count_nonzero(obs != pre_clip))
        self._stats["last_absmax"] = float(np.max(np.abs(obs))) if obs.size else 0.0

        if linvel_source == "zero" and base_lin_vel_world is not None:
            self._stats["n_linvel_zero"] += 1

        age = 0.0
        st_sim = float(getattr(state, "sim_time", 0.0) or 0.0)
        if st_sim > 0.0:
            age = max(0.0, float(t) - st_sim)

        meta = ObsMeta(valid=(n_bad == 0), reasons=tuple(reasons),
                       linvel_source=linvel_source,
                       height_scan_source=scan_source, age_s=age)
        return obs.astype(np.float32), meta

    # -------------------------------------------------------- 动作回写

    def commit(self, action: np.ndarray) -> None:
        """记录上一拍策略输出 (原始动作, 未 clip)，供下一帧 obs 的 36:48。"""
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.size != NUM_JOINT:
            raise ValueError("action 必须是 %d 维，得到 %d" % (NUM_JOINT, a.size))
        self.last_action = a.copy()

    # -------------------------------------------------------- 统计

    def stats(self) -> dict:
        d = dict(self._stats)
        d.update({
            "dim": int(self.obs_dim),
            "phase": float(self._phase),
            "gait_freq": float(self._effective_gait_freq()),
            "last_action_absmax": float(np.max(np.abs(self.last_action))),
            # 高程扫描噪声: 是否启用 / 半宽 (米与特征两种单位) / 已加噪帧数。
            # 部署时 scan_noise=0.0 (默认), 评测时用它判断 obs 差多少。
            "scan_noise_m": float(self.cfg.scan_noise),
            "scan_noise_feature_halfwidth": float(
                self.cfg.scan_noise * self.cfg.scan_scale),
            "scan_noise_enabled": bool(self.cfg.scan_noise > 0.0),
            "scan_noised": bool(self._stats["n_scan_noised"] > 0),
        })
        return d
