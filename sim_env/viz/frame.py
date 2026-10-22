"""可视化用的统一帧数据（所有 viz 组件只消费这一个结构）。

为什么要单独抽出来：3D 叠加、曲线、仪表盘、终端、记录器是四个人并行写的，
如果各自去 sim/controller 里掏字段，接口会立刻漂移。所以这里做**唯一**的采集点。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import numpy as np

from signal_io.msg import NUM_JOINT, LowCmd, LowState
from sim_env.robot import LEG_ORDER


@dataclass
class VizFrame:
    t: float = 0.0
    wall: float = 0.0
    base_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    base_lin_vel_world: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_lin_vel_truth: Optional[np.ndarray] = None
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    joint_q: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    joint_dq: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    joint_q_des: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    joint_tau: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    foot_pos_world: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    foot_force: np.ndarray = field(default_factory=lambda: np.zeros(4))
    scan_xy_body: np.ndarray = field(default_factory=lambda: np.zeros((35, 2)))
    scan_xy_world: np.ndarray = field(default_factory=lambda: np.zeros((35, 3)))
    height_scan: np.ndarray = field(default_factory=lambda: np.zeros(35))
    # v6: 扫描网格形状 (7x5=35 旧档 / 17x11=187 大扫描档)。可视化按它 reshape。
    scan_nx: int = 7
    scan_ny: int = 5
    phase: np.ndarray = field(default_factory=lambda: np.zeros(4))
    action: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    action_applied: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINT))
    obs: np.ndarray = field(default_factory=lambda: np.zeros(91))
    level: int = 0
    level_name: str = "L0"
    mode: str = "track"
    obs_valid: bool = True
    sources: Dict[str, str] = field(default_factory=dict)
    rtf: float = 0.0
    overruns: int = 0
    n_cmd: int = 0
    n_state: int = 0
    # 额外诊断（可选，可视化组件不要依赖）
    extra: Dict[str, float] = field(default_factory=dict)


# ------------------------------------------------------------------ 地面高度来源
class FlatGround:
    """平地假设：地面高度恒为 0。"""

    name = "flat"

    def height(self, xy: np.ndarray) -> np.ndarray:
        return np.zeros(np.asarray(xy).shape[0], dtype=np.float64)


class RaycastGround:
    """从上方往下打射线量地面高度（只打环境 geom 组，避开机器人本体）。

    仿真的高度采样点用它才是**真值**；控制器那边用的是 SimLidar+HeightMapper，
    两条摆在一起看，就是在看"观测器估得准不准"。

    2026-10-22: mask 从 (1,0,0,0,0,0) 改为 (1,1,0,0,1,0) —— group1 = 视觉 hfield/地形网格,
    group4 = 碰撞 box/瓦片 (v33 视觉统一后 box 从 group0 移到 group4)。旧 mask 看不到
    box -> 障碍物处射线打回地面高度, 点被障碍物网格挡住, 表现为"扫描点碰到障碍物就消失"。
    """

    name = "raycast"

    def __init__(self, model, data, geomgroup=(1, 1, 0, 0, 1, 0), start_z: float = 2.0):
        import mujoco
        self._mj = mujoco
        self.model = model
        self.data = data
        self.geomgroup = np.asarray(geomgroup, dtype=np.uint8)
        self.start_z = float(start_z)
        self.geomid = np.zeros(1, dtype=np.int32)

    def height(self, xy: np.ndarray) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        out = np.zeros(xy.shape[0], dtype=np.float64)
        pnt = np.zeros(3)
        for i, (x, y) in enumerate(xy):
            pnt[0], pnt[1], pnt[2] = x, y, self.start_z
            dist = self._mj.mj_ray(self.model, self.data, pnt, np.array([0.0, 0.0, -1.0]),
                                   self.geomgroup, 1, -1, self.geomid)
            out[i] = (self.start_z - dist) if dist >= 0.0 else 0.0
        return out


def make_ground(model, data, mode: str = "auto"):
    """auto：静态 geom 里有非平面的东西（台阶/障碍）就用射线，否则平地。"""
    if mode == "flat":
        return FlatGround()
    if mode == "raycast":
        return RaycastGround(model, data)
    import mujoco
    # 只看**世界体(bodyid==0)**上的静态 geom：地板是 plane 就是平地，
    # 只要有台阶/盒子这类非平面，就得射线量。机器人自己的碰撞 geom 不算
    # （它们在 trunk/腿的 body 上，bodyid != 0）。
    for g in range(model.ngeom):
        if int(model.geom_bodyid[g]) != 0:
            continue
        if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        if int(model.geom_contype[g]) != 0 or int(model.geom_conaffinity[g]) != 0:
            return RaycastGround(model, data)
    return FlatGround()


# ------------------------------------------------------------------ 采集
def _quat_to_mat(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FrameCollector:
    """把 sim / controller / 健康层 / 安全层 当前状态收成一份 VizFrame。"""

    def __init__(self, sim, ground=None, scan_xy_body: Optional[np.ndarray] = None,
                 scan_nx: int = 7, scan_ny: int = 5):
        self.sim = sim
        self.ground = ground if ground is not None else FlatGround()
        self.scan_xy_body = (np.zeros((35, 2)) if scan_xy_body is None
                             else np.asarray(scan_xy_body, dtype=np.float64).copy())
        self.scan_nx = int(scan_nx)
        self.scan_ny = int(scan_ny)

    def set_scan_grid(self, xy_body: np.ndarray) -> None:
        self.scan_xy_body = np.asarray(xy_body, dtype=np.float64).copy()

    def collect(self, t: float, wall: float, cmd: Optional[LowCmd] = None,
                health=None, supervisor=None, ctrl=None, rtf: float = 0.0,
                overruns: int = 0, n_cmd: int = 0, n_state: int = 0) -> VizFrame:
        sim = self.sim
        f = VizFrame(t=float(t), wall=float(wall))
        f.base_pos = np.asarray(sim.data.qpos[0:3], dtype=np.float64).copy()
        f.base_quat = np.asarray(sim.data.qpos[3:7], dtype=np.float64).copy()
        f.rpy = np.asarray(sim.orientation_rpy(), dtype=np.float64)
        f.joint_q = np.asarray(sim.data.qpos[sim.qpos_adr], dtype=np.float64).copy()
        f.joint_dq = np.asarray(sim.data.qvel[sim.dof_adr], dtype=np.float64).copy()
        f.joint_tau = np.asarray(sim.tau_applied, dtype=np.float64).copy()
        f.base_lin_vel_truth = np.asarray(sim.data.qvel[0:3], dtype=np.float64).copy()

        # 足端（世界系）+ 接触力
        try:
            geoms = [g for g in np.asarray(sim.foot_geom).reshape(-1) if int(g) >= 0]
            for k, g in enumerate(geoms[:4]):
                f.foot_pos_world[k] = np.asarray(sim.data.geom_xpos[int(g)], dtype=np.float64)
        except Exception:
            pass
        st = getattr(sim, "_last_state", None)
        if st is not None:
            f.foot_force = np.asarray(st.foot_force, dtype=np.float64)[:4]

        # 指令目标
        if cmd is not None:
            f.joint_q_des = np.array([float(cmd.motor_cmd[i].q) for i in range(NUM_JOINT)])

        # 高度采样点：机体系 -> 世界系（只按 yaw 旋转）+ 地面高度
        yaw = float(np.arctan2(2.0 * (f.base_quat[0] * f.base_quat[3] + f.base_quat[1] * f.base_quat[2]),
                               1.0 - 2.0 * (f.base_quat[2] ** 2 + f.base_quat[3] ** 2)))
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        xy_w = np.stack([c * self.scan_xy_body[:, 0] - s * self.scan_xy_body[:, 1],
                         s * self.scan_xy_body[:, 0] + c * self.scan_xy_body[:, 1]], axis=-1)
        xy_w = xy_w + f.base_pos[None, :2]
        z_ground = np.asarray(self.ground.height(xy_w), dtype=np.float64)
        # +6mm 抬高: 避免采样点球体与地面/台阶面 z-fighting 被遮挡
        f.scan_xy_world = np.concatenate([xy_w, (z_ground + 0.006)[:, None]], axis=-1)
        f.scan_nx, f.scan_ny = self.scan_nx, self.scan_ny

        # 来自控制器 / 健康层的东西（没有就保持默认，stand 控制器也能看）
        if ctrl is not None:
            if getattr(ctrl, "last_obs", None) is not None:
                f.obs = np.asarray(ctrl.last_obs, dtype=np.float64).reshape(-1)
            if getattr(ctrl, "last_action", None) is not None:
                f.action = np.asarray(ctrl.last_action, dtype=np.float64).reshape(-1)[:NUM_JOINT]
            if cmd is not None:
                f.action_applied = f.joint_q_des.copy()
            # 策略**实际拿到的**基座线速度 (世界系)。以前这里没赋值 ->
            # 曲线图里"估计 vx"恒为 0、误差恒等于 -真值, 看起来像里程计全废。
            # oracle 模式下它应当与 base_lin_vel_truth 几乎重合。
            vel = getattr(ctrl, "_vel", None)
            if vel is not None:
                f.base_lin_vel_world = np.asarray(vel, dtype=np.float64).reshape(-1)[:3].copy()
            meta = getattr(ctrl, "last_meta", None)
            if meta is not None:
                f.obs_valid = bool(meta.valid)
                f.sources = {"linvel": str(meta.linvel_source), "scan": str(meta.height_scan_source)}
            gc = getattr(ctrl, "_zc", None)
            if gc is not None:
                f.extra["ground_clearance"] = float(gc)
            builder = getattr(ctrl, "obs", None)
            if builder is not None and hasattr(builder, "leg_phases"):
                try:
                    f.phase = np.asarray(builder.leg_phases(), dtype=np.float64).reshape(-1)[:4]
                except Exception:
                    pass
            getter = getattr(ctrl, "last_height_scan", None)
            n_scan = self.scan_nx * self.scan_ny
            if getter is not None:
                f.height_scan = np.asarray(getter, dtype=np.float64).reshape(-1)[:n_scan]
            elif f.obs.size >= 48 + n_scan:
                f.height_scan = f.obs[48:48 + n_scan].copy()
        elif f.obs.size >= 48 + self.scan_nx * self.scan_ny:
            f.height_scan = f.obs[48:48 + self.scan_nx * self.scan_ny].copy()

        # 健康层 / 安全层
        if health is not None:
            f.level = int(getattr(health, "level", 0))
            f.level_name = str(getattr(health, "level_name", "L%d" % f.level))
        if supervisor is not None:
            f.mode = str(getattr(supervisor, "mode", "track"))

        f.rtf = float(rtf)
        f.overruns = int(overruns)
        f.n_cmd = int(n_cmd)
        f.n_state = int(n_state)
        return f


__all__ = ["VizFrame", "FrameCollector", "FlatGround", "RaycastGround", "make_ground"]
