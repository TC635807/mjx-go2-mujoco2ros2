"""上层控制器（50 Hz）。这里只放最基础的两个，用来验证链路。

要接自己的东西（LQR / 状态机 / 训练好的策略网络），只要实现
    update(state: LowState, t: float) -> LowCmd
就行 —— 它跟传输后端没关系，仿真和真机都能跑同一份代码。
"""

from __future__ import annotations

import abc
from typing import Callable, Optional, Sequence

import numpy as np

from signal_io.msg import NUM_JOINT, LowCmd, LowState
from sim_env.robot import TORQUE_LIMITS, make_stand_cmd


class BaseController(abc.ABC):
    name = "base"

    def reset(self) -> None:
        pass

    @abc.abstractmethod
    def update(self, state: Optional[LowState], t: float) -> LowCmd:
        ...

    def stats(self) -> dict:
        return {}


class ZeroController(BaseController):
    """纯卸力：motor_cmd 全 0。用来确认链路通了但机器人会趴下。"""

    name = "zero"

    def update(self, state: Optional[LowState], t: float) -> LowCmd:
        return LowCmd().zero()


class StandController(BaseController):
    """关节级 PD 顶在标称站姿上。

    这条指令和宇树官方 example 里的 stand_go2 是同一件事：motor_cmd[i] 的
    q/kp/kd 交给"电机"，电机自己算 kp*(q_des-q)+kd*(dq_des-dq)+tau。
    ramp_s 秒内把 kp 从 0 线性升到目标值，避免一上来就是满增益把机器人弹飞。
    """

    name = "stand"

    def __init__(self, stand_pose: np.ndarray, kp: float = 60.0, kd: float = 3.5,
                 ramp_s: float = 1.0, start_delay_s: float = 0.5):
        self.stand_pose = np.asarray(stand_pose, dtype=np.float64)
        self.kp = float(kp)
        self.kd = float(kd)
        self.ramp_s = float(ramp_s)
        self.start_delay_s = float(start_delay_s)
        self._alpha = 0.0
        self._last_cmd = make_stand_cmd(self.stand_pose, 0.0, 0.0)

    def reset(self) -> None:
        self._alpha = 0.0

    def update(self, state: Optional[LowState], t: float) -> LowCmd:
        if t < self.start_delay_s or self.ramp_s <= 0.0:
            a = 0.0
        else:
            a = min(1.0, (t - self.start_delay_s) / self.ramp_s)
        self._alpha = a
        cmd = make_stand_cmd(self.stand_pose, self.kp * a, self.kd * a)
        self._last_cmd = cmd
        return cmd

    def stats(self) -> dict:
        return {"kp_alpha": round(self._alpha, 3)}


class WbcLqrStub(BaseController):
    """占位：把你自己的 LQR / MPC / 策略网络放这里。

    obs 用 state.motor_state[0:12].q / dq 和 state.imu_state.rpy，
    输出 12 维力矩，填进 cmd.motor_cmd[i].tau（kp/kd 置 0）。
    """

    name = "lqr_stub"

    def __init__(self, stand_pose: np.ndarray, kd: float = 2.0):
        self.stand_pose = np.asarray(stand_pose, dtype=np.float64)
        self.kd = float(kd)
        self.limits = np.array(TORQUE_LIMITS, dtype=np.float64)

    def update(self, state: Optional[LowState], t: float) -> LowCmd:
        cmd = LowCmd()
        if state is None:
            return cmd
        q = state.q12()
        dq = state.dq12()
        # 例：重力补偿 + 关节阻尼。真正的控制器替换掉这一行即可。
        tau = np.clip(-self.kd * dq, -self.limits, self.limits)
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.kp, m.kd, m.tau = 0x01, 0.0, 0.0, 0.0, 0.0, float(tau[i])
        return cmd


class PolicyController(BaseController):
    """把导出的 brax PPO 策略（go2_walk）挂到 50 Hz 控制层上。

    与训练的一致性靠两件事保证，都已有逐位证据：
      - 观测：obs.WalkObsBuilder（91 维），已与训练环境 envs/go2_walk.py::_get_obs 对账，
        逐段 max|diff| = 0（height scan 那段的差异就是训练自带的 ±0.02 m actor 噪声）；
      - 动作：12 维**关节绝对位置目标**，按训练映射写进 LowCmd 的 q/kp/kd
        （对应训练侧 <position kp=20 kv=1>），并且**先按关节范围夹紧** ——
        训练是 clip(action, actuator_ctrlrange)，而 ctrlrange==关节范围（autolimits=true）。

    观测里"基座线速度"这一路真机上没有传感器，默认走 estimator.LegOdometry；
    height scan 默认走"接触腿正运动学推出的离地高度"（平地假设），
    有激光/高程图时用 height_scan_fn 换成真感知（两者共用同一个 obs 定义）。
    """

    name = "policy"

    def __init__(self, policy, obs_builder, kp: float = 20.0, kd: float = 1.0,
                 command: Sequence[float] = (0.5, 0.0, 0.0),
                 estimator=None, height_scan_fn: Optional[Callable[[LowState, float], np.ndarray]] = None,
                 kin=None, contact_force_thresh: float = 30.0, foot_radius: float = 0.023,
                 height_scan_mode: str = "leg_fk", clearance_hz: float = 3.0,
                 prepose_s: float = 0.0, prepose_kp: float = 20.0, prepose_kd: float = 1.0,
                 joint_lo: Optional[np.ndarray] = None, joint_hi: Optional[np.ndarray] = None,
                 action_clip: bool = True, ramp_s: float = 0.0, start_delay_s: float = 0.0,
                 linvel_fn=None):
        self.policy = policy
        self.obs = obs_builder
        self.kp = float(kp)
        self.kd = float(kd)
        self.command = np.asarray(command, dtype=np.float64).reshape(3)
        self.estimator = estimator
        self.height_scan_fn = height_scan_fn
        self.kin = kin
        self.contact_force_thresh = float(contact_force_thresh)
        # 足端球半径：正运动学给的是**足端球心**，离地高度还要加上这个半径，
        # 否则会系统性偏低 0.023 m（实测 Go2 站立：FK 0.2479 vs 真实 0.2696）。
        self.foot_radius = float(foot_radius)
        # height scan 的来源：
        #   "nominal" —— 直接按标称站高算（平地上 feature=0）。最稳，但地形起伏看不见。
        #   "leg_fk"  —— 接触腿正运动学估离地高度。能反映实际站高/地形，但单腿摆动时噪声大，
        #                必须低通 + 限幅，否则 obs 里这一路会抖 ±0.15（实测能把走路抖倒）。
        if height_scan_mode not in ("nominal", "leg_fk"):
            raise ValueError("height_scan_mode 只能是 nominal / leg_fk，得到 %r" % height_scan_mode)
        self.height_scan_mode = height_scan_mode
        self.clearance_hz = float(clearance_hz)
        # 预摆位：仿真侧"上电后自己顶着标称站姿，等第一条 LowCmd"，但**策略期望的初始姿态
        # 是训练 home**（Go2: 前腿 0.8/-1.5、后腿 1.0/-1.5），而 Go2 的 home 是 0.9/-1.8。
        # 直接交接 = 策略第一眼就看见一个训练里没见过的关节偏差（大腿 +0.1、膝盖 -0.3 rad），
        # 实测这个瞬态足以把走路搞崩。所以先花 prepose_s 秒把姿态摆到训练 home 再接管。
        self.prepose_s = float(prepose_s)
        self.prepose_kp = float(prepose_kp)
        self.prepose_kd = float(prepose_kd)
        self.home_q = np.asarray(obs_builder.home_q if hasattr(obs_builder, "home_q")
                                 else np.zeros(NUM_JOINT), dtype=np.float64)
        self.n_prepose = 0
        self._zc: Optional[float] = None
        self._zc_raw: Optional[float] = None
        self.joint_lo = None if joint_lo is None else np.asarray(joint_lo, dtype=np.float64)
        self.joint_hi = None if joint_hi is None else np.asarray(joint_hi, dtype=np.float64)
        self.action_clip = bool(action_clip)
        self.ramp_s = float(ramp_s)
        self.start_delay_s = float(start_delay_s)
        self.linvel_fn = linvel_fn
        self.n_calls = 0
        self.n_clamped = 0
        self.last_action: Optional[np.ndarray] = None
        self.last_obs: Optional[np.ndarray] = None
        self.last_meta = None
        self._alpha = 0.0
        self._vel = np.zeros(3)
        # v6: 扫描点数按 obs 配置走 (35 旧档 / 187 大扫描档), 不再硬编码 35
        self._scan_n = int(getattr(obs_builder, "scan_n", 35))
        self._scan = np.zeros(self._scan_n)
        self._scan_source = "zero"
        self._n_bad_scan = 0
        self._scan_missing_since: Optional[float] = None
        self._warned_scan_missing = False
        self._t_last: Optional[float] = None

    # ------------------------------------------------------------------ 生命周期
    def reset(self) -> None:
        self.obs.reset()
        if self.estimator is not None:
            self.estimator.reset()
        self.n_calls = 0
        self.n_clamped = 0
        self.n_prepose = 0
        self.last_action = None
        self._alpha = 0.0
        self._t_last = None
        self._zc = None
        self._zc_raw = None
        self._n_bad_scan = 0
        self._scan_missing_since = None
        self._warned_scan_missing = False

    # ------------------------------------------------------------------ 速度 / 离地高度
    def _linvel(self, state: LowState, t: float) -> Optional[np.ndarray]:
        if self.linvel_fn is not None:
            return np.asarray(self.linvel_fn(state, t), dtype=np.float64)
        if self.estimator is None:
            return None
        dt = 0.02 if self._t_last is None else max(1e-4, min(0.1, t - self._t_last))
        return self.estimator.update(state, dt, t)

    def leg_ground_clearance(self, state: LowState, dt: float = 0.02) -> float:
        raw = self._leg_clearance_raw(state)
        if not np.isfinite(raw):
            return float("nan")
        # 一阶低通：单腿摆动会让瞬时值跳，直接喂给策略等于给 obs 加噪声
        alpha = 1.0 - float(np.exp(-2.0 * np.pi * max(self.clearance_hz, 1e-3) * max(dt, 1e-4)))
        self._zc_raw = raw
        self._zc = raw if self._zc is None else self._zc + alpha * (raw - self._zc)
        # 物理上不可能的值（摔倒/腾空）夹回合理范围，避免把策略推到分布外
        return float(np.clip(self._zc, 0.10, 0.45))

    def _leg_clearance_raw(self, state: LowState) -> float:
        """接触腿正运动学推出的"躯干离地高度"（平地假设）。

        脚在机体系里 z 为负，-z 是躯干到**足端球心**的高度，再加 foot_radius 才是到地面。
        取接触足的**中位数**，没有接触腿时退化为最低那只脚。
        真机上这就是"不用激光也能拿到的站立高度"。
        """
        if self.kin is None:
            return float("nan")
        P = self.kin.foot_positions_body(state.q12())
        ff = np.asarray(state.foot_force, dtype=np.float64)
        contact = ff > self.contact_force_thresh
        zs = -P[:, 2] + self.foot_radius
        if contact.any():
            return float(np.median(zs[contact]))
        return float(zs.min())

    def _height_scan(self, state: LowState, t: float) -> np.ndarray:
        if self.height_scan_fn is not None:
            sc = self.height_scan_fn(state, t)
            if sc is not None:
                sc = np.asarray(sc, dtype=np.float64).reshape(-1)
                if sc.size == self._scan_n:
                    self._scan_source = "external"
                    self._scan_missing_since = None
                    self._warned_scan_missing = False
                    return sc
                # 话题里的点数与当前策略不匹配：不要静默喂错维度，降级到 leg_fk/nominal
                self._n_bad_scan += 1
            if not self._warned_scan_missing:
                # 外部 height_scan 一直没数据：真机上说明 odom/点云链路没通，
                # 此时策略吃的是"单个标量的 leg_fk"（平地近似、地形不可信），必须显式提示。
                if self._scan_missing_since is None:
                    self._scan_missing_since = float(t)
                elif float(t) - self._scan_missing_since > 2.0:
                    print("[ctrl] ⚠ 外部 height_scan 已 >2 s 无数据：回退 leg_fk（"
                          "地形上不可信；检查 perception/odom/点云，或改用 --scan leg_fk 明确降级）")
                    self._warned_scan_missing = True
        n_scan = self._scan_n
        if self.kin is None or self.height_scan_mode == "nominal":
            self._scan_source = "nominal"
            dt = 0.02 if self._t_last is None else max(1e-4, min(0.1, t - self._t_last))
            return np.zeros(n_scan)
        dt = 0.02 if self._t_last is None else max(1e-4, min(0.1, t - self._t_last))
        z = self.leg_ground_clearance(state, dt)
        if not np.isfinite(z):
            self._scan_source = "zero"
            return np.zeros(n_scan)
        self._scan_source = "leg_fk"
        # 平地假设：35 个采样点的地面高度都是 0（真地形走 perception.HeightMapper）
        n = self.obs.scan_grid().shape[0]
        return np.asarray(self.obs.scan_feature(np.full(n, z), np.zeros(n)),
                          dtype=np.float64).reshape(n)

    # ------------------------------------------------------------------ 50 Hz 一拍
    def update(self, state: Optional[LowState], t: float) -> LowCmd:
        cmd = LowCmd()
        if state is None:
            return cmd
        # ---- 预摆位阶段：只做"把关节摆到训练 home"，不跑策略
        if self.prepose_s > 0.0 and t < self.prepose_s:
            self.n_prepose += 1
            self._alpha = 1.0
            for i in range(NUM_JOINT):
                m = cmd.motor_cmd[i]
                m.mode = 0x01
                m.q = float(self.home_q[i])
                m.dq = 0.0
                m.kp = self.prepose_kp
                m.kd = self.prepose_kd
                m.tau = 0.0
            return cmd
        a = 1.0
        if self.ramp_s > 0.0:
            a = 0.0 if t < self.start_delay_s else min(1.0, (t - self.start_delay_s) / self.ramp_s)
        self._alpha = a
        vel = self._linvel(state, t)
        scan = self._height_scan(state, t)
        self._vel = np.zeros(3) if vel is None else np.asarray(vel, dtype=np.float64).reshape(3)
        self._scan = scan
        obs_vec, meta = self.obs.observe(state, t, base_lin_vel_world=vel, height_scan=scan)
        action = np.asarray(self.policy(obs_vec), dtype=np.float64).reshape(NUM_JOINT)
        self.obs.commit(action)
        applied = action
        if self.action_clip and self.joint_lo is not None and self.joint_hi is not None:
            applied = np.clip(action, self.joint_lo, self.joint_hi)
            if np.any(np.abs(applied - action) > 1e-9):
                self.n_clamped += 1
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode = 0x01
            m.q = float(applied[i])
            m.dq = 0.0
            m.kp = self.kp * a
            m.kd = self.kd * a
            m.tau = 0.0
        self.n_calls += 1
        self.last_action = action
        self.last_obs = obs_vec
        self.last_meta = meta
        self._t_last = t
        return cmd

    @property
    def last_height_scan(self) -> Optional[np.ndarray]:
        """最近一次真正喂给策略的 35 维高度特征（可视化观察器要用）。"""
        return None if self._scan is None else self._scan

    def stats(self) -> dict:
        out = {"policy": getattr(self.policy, "name", "policy"), "n_calls": self.n_calls,
               "prepose_s": self.prepose_s, "n_prepose": self.n_prepose,
               "n_action_clamped": self.n_clamped,
               "ramp_alpha": round(self._alpha, 3),
               "obs_valid": None if self.last_meta is None else bool(self.last_meta.valid),
               "linvel_source": None if self.last_meta is None else self.last_meta.linvel_source,
               "height_scan_source": self._scan_source,
               "n_bad_scan": int(self._n_bad_scan),
               "ground_clearance_raw": None if self._zc_raw is None else round(float(self._zc_raw), 4),
               "ground_clearance": None if self._zc is None else round(float(self._zc), 4),
               "base_lin_vel_world": [round(float(x), 4) for x in self._vel],
               "height_scan_mean": round(float(np.mean(self._scan)), 4)}
        if self.estimator is not None and hasattr(self.estimator, "confidence"):
            out["linvel_confidence"] = round(float(self.estimator.confidence), 3)
        if self.last_action is not None:
            out["action_abs_mean"] = round(float(np.mean(np.abs(self.last_action))), 4)
            out["action_abs_max"] = round(float(np.max(np.abs(self.last_action))), 4)
        perc = getattr(self, "perception", None)
        if perc:
            lid = perc.get("lidar")
            mp = perc.get("mapper")
            lid_s = lid.stats() if hasattr(lid, "stats") else {}
            mp_s = mp.stats() if hasattr(mp, "stats") else {}
            out["perception"] = {
                "rate_hz": perc.get("rate_hz", 0.0),
                "lidar_calls": lid_s.get("n_calls"),
                "lidar_n_rays": lid_s.get("n_rays"),
                "lidar_hit_rate": (round(float(lid_s.get("hit_rate", 0.0)), 3)
                                   if lid_s.get("hit_rate") is not None else None),
                "lidar_occluded": lid_s.get("n_occluded"),
                "lidar_multiray": lid_s.get("multiray"),
                "map_cells": mp_s.get("n_cells"),
                "map_cells_fresh": mp_s.get("n_cells_fresh"),
                "map_last_scan_observed": mp_s.get("last_scan_observed"),
                "map_last_scan_points": mp_s.get("last_scan_points"),
            }
        return out

