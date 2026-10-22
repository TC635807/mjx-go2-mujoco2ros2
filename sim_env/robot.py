"""MuJoCo 侧的机器人仿真封装：加载 Go2 模型、施加力矩、打包 LowState。

模型直接复用 unitree_mujoco 的 unitree_robots/go2（官方，执行器已经是 <motor>），
所以这里的力学语义和真机一致:
    tau_motor = kp*(q_des - q) + kd*(dq_des - dq) + tau
真机上是电机驱动器算这个式子；仿真里我们在每个 sim 步之前算一遍再写进 data.ctrl。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from signal_io.msg import NUM_JOINT, IMUState, LowCmd, LowState, MotorState

# Unitree 电机编号顺序（与真机 LowState/LowCmd 一致，也和 unitree_mujoco 的 go2.xml 执行器顺序一致）
MOTOR_JOINTS: Tuple[str, ...] = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
# 关节力矩上限 (N*m)：Go2 官方 go2.xml 里 <motor ctrlrange> 的取值
TORQUE_LIMITS: Tuple[float, ...] = (23.7, 23.7, 45.43) * 4
LEG_ORDER: Tuple[str, ...] = ("FR", "FL", "RR", "RL")
GRAVITY = 9.81


@dataclass
class RobotSpec:
    name: str = "go2"
    motor_joints: Tuple[str, ...] = MOTOR_JOINTS
    torque_limits: np.ndarray = None  # type: ignore[assignment]
    stand_pose: np.ndarray = None     # type: ignore[assignment]
    # True  = 标称站姿取自模型 XML 的 keyframe "home"（本项目原有行为）
    # False = 上游（config/*.json 的 robots.<name>.stand_pose）显式给了站姿，忽略 keyframe。
    #         见 config/go2_sim.json 的 _stand_pose_note：Go2 官方 keyframe (0,0.9,-1.8)
    #         在 kp=20/kd=1 下站不住，必须换成 unitree_rl_gym 惯例姿态。
    stand_pose_from_xml: bool = True
    # reset 时的躯干出生高度（None = 沿用模型 keyframe / qpos0 的 z）
    stand_height: Optional[float] = None

    def __post_init__(self) -> None:
        if self.torque_limits is None:
            self.torque_limits = np.array(TORQUE_LIMITS, dtype=np.float64)
        if self.stand_pose is None:
            # 官方 keyframe "home": 每条腿 0, 0.9, -1.8
            self.stand_pose = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)


def robot_spec(name: str) -> RobotSpec:
    """按机型造规格。Go2 是**训练侧**（mjx-go2-training 的策略在它上面训的），
    Go2 是**部署侧**（真机）。两者的力矩上限与标称姿态都不同：

        go2: hip/thigh ±23.7 N·m, knee ±35.55 N·m, home = (0,0.8,-1.5)×2 + (0,1.0,-1.5)×2
        go2: hip/thigh ±23.7 N·m, knee ±45.43 N·m, home = (0,0.9,-1.8)×4

    力矩上限来自各自官方 MJCF 的 <motor ctrlrange>，不是拍脑袋填的。
    """
    table = {
        "go2": {"limits": (23.7, 23.7, 35.55),
                "stand": (0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 1.0, -1.5, 0.0, 1.0, -1.5)},
        "go2": {"limits": (23.7, 23.7, 45.43),
                "stand": (0.0, 0.9, -1.8) * 4},
    }
    if name not in table:
        raise KeyError("未知机型 %r，可选: %s" % (name, ", ".join(sorted(table))))
    t = table[name]
    return RobotSpec(name=name,
                     torque_limits=np.array(t["limits"] * 4, dtype=np.float64),
                     stand_pose=np.array(t["stand"], dtype=np.float64))


def _actuator_of_joint(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise KeyError("模型里找不到关节 %r" % joint_name)
    for i in range(model.nu):
        if int(model.actuator_trnid[i, 0]) == jid:
            return i
    raise KeyError("关节 %r 上没有执行器（期望是 <motor>）" % joint_name)


class Go2Sim:
    """单环境 CPU 版 Go2 仿真。500 Hz 一步，跟 DDS/ROS2 后端无关。"""

    def __init__(self, xml_path: str, spec: Optional[RobotSpec] = None, sim_dt: Optional[float] = None,
                 keyframe: str = "home", foot_soft_contact: bool = False,
                 solver_iterations: Optional[int] = None,
                 solver_ls_iterations: Optional[int] = None):
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(xml_path)
        self.spec = spec or RobotSpec()
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        if sim_dt is not None:
            self.model.opt.timestep = float(sim_dt)
        self.dt = float(self.model.opt.timestep)

        # 关节 / 执行器索引
        self.act_id = np.array([_actuator_of_joint(self.model, j) for j in self.spec.motor_joints], dtype=int)
        self.qpos_adr = np.array([self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.spec.motor_joints], dtype=int)
        self.dof_adr = np.array([self.model.jnt_dofadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.spec.motor_joints], dtype=int)
        self.base_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if self.base_body < 0:
            self.base_body = 1
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        if self.imu_site < 0:
            raise KeyError("模型里没有 imu site")
        self.root_joint = int(self.model.body_jntadr[self.base_body])

        # 足端 geom（每条腿一个球），用于 foot_force
        self.foot_geom = np.full(4, -1, dtype=int)
        for k, leg in enumerate(LEG_ORDER):
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, leg + "_calf")
            if body < 0:
                continue
            for g in range(self.model.ngeom):
                if self.model.geom_bodyid[g] == body and self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                    self.foot_geom[k] = g
                    break

        # ---- 接触模型 / 求解器：要与**训练侧**对齐，否则策略是在另一个物理世界里跑 ----
        # mjx-go2-training 默认 foot_soft_contact=True：
        #   geom_solimp = [0.9, 0.95, 0.023, 0.5, 2.0]，geom_condim = 3
        # 且 warp 后端 solver_iterations=2 / ls=5。CPU Newton 下 iterations=2 会严重欠解，
        # 所以这里默认只对齐接触参数，求解器迭代数由调用方显式给。
        self.foot_soft_contact = bool(foot_soft_contact)
        if solver_iterations is not None:
            self.model.opt.iterations = int(solver_iterations)
        if solver_ls_iterations is not None:
            self.model.opt.ls_iterations = int(solver_ls_iterations)

        self.stand_pose = self.spec.stand_pose.copy()
        if self.foot_soft_contact:
            self._soften_feet()
        kid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe) if keyframe else -1
        if kid >= 0 and self.spec.stand_pose_from_xml:
            q = np.asarray(self.model.key_qpos[kid])[7:]
            if q.size == NUM_JOINT:
                self.stand_pose = q.copy()
        self.keyframe_id = kid
        self.stand_height = self.spec.stand_height
        self.torque_limits = np.asarray(self.spec.torque_limits, dtype=np.float64)

        self.tick = 0
        self.time = 0.0
        self.tau_applied = np.zeros(NUM_JOINT)
        self.n_contact_overflow = 0
        self.reset()

    def _soften_feet(self) -> None:
        """足端改软接触（复刻 mjx-go2-training 的 foot_soft_contact=True）。"""
        n = 0
        for g in self.foot_geom:
            if g < 0:
                continue
            self.model.geom_solimp[g] = np.array([0.9, 0.95, 0.023, 0.5, 2.0])
            self.model.geom_condim[g] = 3
            n += 1
        return n

    # ---------------------------------------------------------------- 复位
    def reset(self, height: Optional[float] = None, yaw: float = 0.0) -> None:
        mujoco.mj_resetData(self.model, self.data)
        if self.keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.keyframe_id)
        q = self.stand_pose.copy()
        self.data.qpos[7 : 7 + NUM_JOINT] = q
        # 轻微屈膝让足端接地，避免一开始就悬空/穿地
        self.data.qpos[7 : 7 + NUM_JOINT] = q
        h = height if height is not None else getattr(self, "stand_height", None)
        if h is not None:
            self.data.qpos[2] = float(h)
        if yaw:
            c, s = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
            self.data.qpos[3:7] = [c, 0.0, 0.0, s]
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.tick = 0
        self.time = 0.0

    # ---------------------------------------------------------------- 一步
    def write_cmd(self, cmd: LowCmd, clip: bool = True) -> np.ndarray:
        """把 LowCmd 变成 motor 上真正施加的力矩（含关节级 PD + 限幅）。"""
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.dof_adr]
        tau = np.empty(NUM_JOINT)
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            v = m.kp * (m.q - q[i]) + m.kd * (m.dq - dq[i]) + m.tau
            if clip:
                v = float(np.clip(v, -self.torque_limits[i], self.torque_limits[i]))
            tau[i] = v
        for i in range(NUM_JOINT):
            self.data.ctrl[self.act_id[i]] = tau[i]
        self.tau_applied = tau
        return tau

    def step(self, nstep: int = 1) -> None:
        mujoco.mj_step(self.model, self.data, nstep)
        self.tick += nstep
        self.time += self.dt * nstep
        if self.data.nefc > self.model.njmax:
            self.n_contact_overflow += 1

    # ---------------------------------------------------------------- 读状态
    def _imu(self) -> IMUState:
        quat = np.asarray(self.data.xquat[self.base_body], dtype=np.float64)   # w x y z
        # 机体系角速度：free joint 的 qvel[3:6] 就是它（已验证 objVel(world) == R @ qvel[3:6]），
        # 也正是训练侧 obs 用的 data.qvel[3:6]。
        # ⚠️ 不要用 mj_objectVelocity(..., flg_local=1)：对 BODY 它返回的是**惯性主轴系**
        # (body_iquat ≈ 绕 (x+z) 90°) 下的量，实测 qvel[3:6]=[0.3,0.4,0.5] 会变成
        # [0.504,-0.402,0.290]，与真机 IMU 的机体系读数不符。
        if int(self.model.jnt_type[self.root_joint]) == int(mujoco.mjtJoint.mjJNT_FREE):
            ang_local = np.asarray(self.data.qvel[3:6], dtype=np.float64).copy()
        else:
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                     self.base_body, vel, 1)
            ang_local = vel[0:3]
        acc6 = np.zeros(6)
        mujoco.mj_objectAcceleration(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, self.imu_site, acc6, 0)
        rot = np.asarray(self.data.site_xmat[self.imu_site], dtype=np.float64).reshape(3, 3)
        acc_body = rot.T @ (acc6[0:3] + np.array([0.0, 0.0, GRAVITY]))
        rpy = _quat_to_rpy(quat)
        return IMUState(quaternion=[float(x) for x in quat],
                        gyroscope=[float(x) for x in ang_local],
                        accelerometer=[float(x) for x in acc_body],
                        rpy=[float(x) for x in rpy])

    def _foot_forces(self) -> List[int]:
        out = [0, 0, 0, 0]
        f6 = np.zeros(6)
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            k = -1
            if g1 in self.foot_geom:
                k = int(np.where(self.foot_geom == g1)[0][0])
            elif g2 in self.foot_geom:
                k = int(np.where(self.foot_geom == g2)[0][0])
            if k < 0:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, f6)
            out[k] += int(abs(f6[0]))
        return [int(np.clip(v, -32768, 32767)) for v in out]

    def read_state(self, tick: Optional[int] = None) -> LowState:
        st = LowState()
        st.tick = int(self.tick if tick is None else tick)
        st.sim_time = float(self.time)
        st.imu_state = self._imu()
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.dof_adr]
        for i in range(NUM_JOINT):
            st.motor_state[i] = MotorState(mode=0x01, q=float(q[i]), dq=float(dq[i]),
                                           ddq=0.0, tau_est=float(self.tau_applied[i]),
                                           temperature=0, lost=0)
        st.foot_force = self._foot_forces()
        return st

    # ---------------------------------------------------------------- 便捷量
    @property
    def base_height(self) -> float:
        return float(self.data.qpos[2])

    def orientation_rpy(self) -> np.ndarray:
        return _quat_to_rpy(np.asarray(self.data.xquat[self.base_body], dtype=np.float64))

    def is_upright(self, deg: float = 30.0) -> bool:
        r, p, _ = self.orientation_rpy()
        return abs(np.degrees(r)) < deg and abs(np.degrees(p)) < deg

    def info(self) -> Dict[str, object]:
        return {
            "nq": self.model.nq, "nv": self.model.nv, "nu": self.model.nu,
            "timestep": self.dt, "njmax": int(self.model.njmax),
            "stand_pose": [round(float(x), 4) for x in self.stand_pose],
            "actuator_names": [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                               for i in range(self.model.nu)],
        }


def _quat_to_rpy(q: Sequence[float]) -> np.ndarray:
    """MuJoCo 的 (w,x,y,z) -> roll/pitch/yaw，与 unitree LowState 的 rpy 同序。"""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny, cosy)
    return np.array([roll, pitch, yaw])


def make_stand_cmd(stand_pose: np.ndarray, kp: float = 60.0, kd: float = 3.5) -> LowCmd:
    """一条"关节级 PD 顶在标称站姿"的指令，和 unitree 官方 stand 例程等价。"""
    cmd = LowCmd()
    for i in range(NUM_JOINT):
        m = cmd.motor_cmd[i]
        m.mode = 0x01
        m.q = float(stand_pose[i])
        m.dq = 0.0
        m.tau = 0.0
        m.kp = kp
        m.kd = kd
    return cmd
