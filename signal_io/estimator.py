"""腿式里程计 —— 从 Go2 腿部几何 + 关节状态估出**世界系**基座线速度。

为什么需要它: 训练观测里的 global_linvel (qvel[0:3]) 是 oracle；真机没有这个
传感器，只能用"接触腿在世界系不动"的假设反解基座速度:

    v_world = -omega_world x p_foot_world - R_world_body * (J(q) @ dq_leg)

其中 p_foot 是**机体系**足端位置，J = d p_foot_body / d q_leg，R 由 IMU 四元数给出。
接触判据用 LowState.foot_force (N)。

几何全部从 models/go2/go2.xml 解析 (不写死数值)，FK 与 MuJoCo 的 geom_xpos
逐点对齐 (见 tests/test_estimator.py)。只依赖 numpy + 标准库。
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import numpy as np

from .msg import NUM_JOINT, LowState

__all__ = ["LegKinematics", "LegOdometry", "LEG_ORDER", "quat_to_mat"]

LEG_ORDER: Tuple[str, ...] = ("FR", "FL", "RR", "RL")


# ---------------------------------------------------------------- 小工具


def quat_to_mat(q_wxyz) -> np.ndarray:
    """(w,x,y,z) -> 3x3 旋转矩阵 (机体系 -> 世界系)。自动归一化。"""
    q = np.asarray(q_wxyz, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _vec(s: Optional[str], n: int) -> np.ndarray:
    if s is None or not s.strip():
        return np.zeros(n, dtype=np.float64)
    v = np.array([float(t) for t in s.replace(",", " ").split()], dtype=np.float64)
    if v.size < n:
        v = np.concatenate([v, np.zeros(n - v.size)])
    return v[:n]


def _skew(u: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -u[2], u[1]],
                     [u[2], 0.0, -u[0]],
                     [-u[1], u[0], 0.0]], dtype=np.float64)


def _axis_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕单位轴 axis 转 angle 的旋转矩阵 (Rodrigues)。"""
    u = np.asarray(axis, dtype=np.float64)
    n = float(np.linalg.norm(u))
    if n < 1e-12:
        return np.eye(3)
    u = u / n
    K = _skew(u)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def _axis_rot_d(axis: np.ndarray, angle: float) -> np.ndarray:
    """d/d(angle) [axis_rot(axis, angle)] = [axis]_x @ R。"""
    u = np.asarray(axis, dtype=np.float64)
    n = float(np.linalg.norm(u))
    if n < 1e-12:
        return np.zeros((3, 3))
    u = u / n
    return _skew(u) @ _axis_rot(u, angle)


# ---------------------------------------------------------------- XML 解析


def _default_classes(root: ET.Element) -> Dict[str, Dict[str, dict]]:
    """解析 <default> 类继承树 -> {class: {tag: {attr: value}}}。

    只支持工程里实际用到的语义: <default class="x"> 继承外层 default 的同名
    子元素属性; 最外层无 class 的 <default> 作为全局默认。
    """
    classes: Dict[str, Dict[str, dict]] = {}

    def merged(inherited, elem):
        out = {k: dict(v) for k, v in inherited.items()}
        for child in elem:
            if child.tag == "default":
                continue
            out.setdefault(child.tag, {}).update(child.attrib)
        return out

    def walk(elem, inherited):
        cur = merged(inherited, elem)
        cls = elem.get("class")
        if cls:
            classes[cls] = cur
        for child in elem:
            if child.tag == "default":
                walk(child, cur)

    for elem in root.findall("default"):
        walk(elem, {})
    return classes


def _elem_attrs(elem: ET.Element, tag: str,
                classes: Dict[str, Dict[str, dict]],
                cls: Optional[str]) -> dict:
    """元素属性 = 默认类链属性 + 元素自身属性 (自身优先)。"""
    out: dict = {}
    if cls and cls in classes:
        out.update(classes[cls].get(tag, {}))
    out.update(elem.attrib)
    return out


class LegKinematics:
    """Go2 腿部几何 + 足端正运动学 (机体系)。几何从 go2.xml 解析。

    每条腿的链: {leg}_hip -> {leg}_thigh -> {leg}_calf -> 足端球心，
    关节顺序固定为 (hip, thigh, calf)，腿顺序 FR, FL, RR, RL。
    """

    def __init__(self, xml_path: str) -> None:
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(xml_path)
        self.xml_path = os.path.abspath(xml_path)
        root = ET.parse(xml_path).getroot()
        self._classes = _default_classes(root)

        # ---- 建 body 树 (name -> elem / parent / 生效的 childclass)
        self._bodies: Dict[str, ET.Element] = {}
        self._parent: Dict[str, Optional[str]] = {}
        self._child_class: Dict[str, Optional[str]] = {}

        def walk(elem, inherited_cls, parent_name):
            for child in elem:
                if child.tag != "body":
                    continue
                name = child.get("name")
                if name is None:
                    continue
                self._bodies[name] = child
                self._parent[name] = parent_name
                child_cls = child.get("childclass") or inherited_cls
                self._child_class[name] = child_cls
                walk(child, child_cls, name)

        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("XML 里没有 <worldbody>: %s" % xml_path)
        walk(worldbody, None, None)

        # ---- 根 body: 带 freejoint 的那个
        self.base_body = None
        for name, elem in self._bodies.items():
            if elem.find("freejoint") is not None:
                self.base_body = name
                break
        if self.base_body is None:
            raise ValueError("XML 里找不到带 <freejoint> 的根 body")
        base_elem = self._bodies[self.base_body]

        # ---- 四条腿的几何
        hip_pos, thigh_pos, calf_pos, foot_pos = [], [], [], []
        hip_axis, thigh_axis, calf_axis = [], [], []
        foot_radius, joint_names = [], []
        self._leg_bodies: List[Tuple[str, str, str]] = []

        for leg in LEG_ORDER:
            try:
                hip_e = self._bodies[leg + "_hip"]
                thigh_e = self._bodies[leg + "_thigh"]
                calf_e = self._bodies[leg + "_calf"]
            except KeyError as exc:
                raise ValueError("XML 里缺少 %s 的腿部 body: %s" % (leg, exc))
            if self._parent.get(leg + "_hip") != self.base_body:
                raise ValueError("%s_hip 的父 body 不是 %s" % (leg, self.base_body))
            if self._parent.get(leg + "_thigh") != leg + "_hip":
                raise ValueError("%s_thigh 的父 body 不是 %s_hip" % (leg, leg))
            if self._parent.get(leg + "_calf") != leg + "_thigh":
                raise ValueError("%s_calf 的父 body 不是 %s_thigh" % (leg, leg))
            self._leg_bodies.append((leg + "_hip", leg + "_thigh", leg + "_calf"))

            # body 的 pos 是相对父体的固定平移 (这些 body 都没有 quat)
            hip_pos.append(_vec(hip_e.get("pos"), 3))
            thigh_pos.append(_vec(thigh_e.get("pos"), 3))
            calf_pos.append(_vec(calf_e.get("pos"), 3))
            for body_name in (leg + "_hip", leg + "_thigh"):
                if self._bodies[body_name].get("quat") is not None:
                    raise NotImplementedError(
                        "%s 带非单位 quat，解析器未实现" % body_name)

            # 关节轴
            def joint_axis(body_name: str, body_elem: ET.Element) -> np.ndarray:
                j = body_elem.find("joint")
                if j is None:
                    raise ValueError("%s 里没有 <joint>" % body_name)
                cls = j.get("class") or self._child_class.get(body_name)
                attrs = _elem_attrs(j, "joint", self._classes, cls)
                ax = _vec(attrs.get("axis", "0 1 0"), 3)
                joint_names.append(attrs.get("name", body_name))
                return ax

            hip_axis.append(joint_axis(leg + "_hip", hip_e))
            thigh_axis.append(joint_axis(leg + "_thigh", thigh_e))
            calf_axis.append(joint_axis(leg + "_calf", calf_e))

            # 足端: calf 里名为 <leg> 的碰撞球 geom (class="foot")
            foot_geoms = [g for g in calf_e.findall("geom")
                          if g.get("name") == leg]
            if not foot_geoms:
                foot_geoms = [g for g in calf_e.findall("geom")
                              if g.get("class") == "foot"]
            if not foot_geoms:
                raise ValueError("%s_calf 里找不到足端 geom" % leg)
            fg = foot_geoms[0]
            fcls = fg.get("class") or self._child_class.get(leg + "_calf")
            fattrs = _elem_attrs(fg, "geom", self._classes, fcls)
            foot_pos.append(_vec(fattrs.get("pos"), 3))
            size = _vec(fattrs.get("size"), 1)
            foot_radius.append(float(size[0]))

        self.hip_pos = np.asarray(hip_pos)          # (4,3) 相对 base
        self.thigh_pos = np.asarray(thigh_pos)      # (4,3) 相对 hip
        self.calf_pos = np.asarray(calf_pos)        # (4,3) 相对 thigh
        self.foot_pos = np.asarray(foot_pos)        # (4,3) 相对 calf (球心)
        self.hip_axis = np.asarray(hip_axis)
        self.thigh_axis = np.asarray(thigh_axis)
        self.calf_axis = np.asarray(calf_axis)
        self.foot_radius = np.asarray(foot_radius)
        self.joint_names = tuple(joint_names)
        if self.joint_names != tuple(
                "%s_%s_joint" % (l, s) for l in LEG_ORDER
                for s in ("hip", "thigh", "calf")):
            raise ValueError("关节顺序与约定的 (FR,FL,RR,RL)x(hip,thigh,calf) 不一致: %s"
                             % (self.joint_names,))

    # -------------------------------------------------------- FK / Jacobian

    def foot_positions_body(self, q12: np.ndarray) -> np.ndarray:
        """(12,) 关节角 -> (4,3) 足端球心在**机体系**的位置 (米)。"""
        q = self._q12(q12)
        out = np.empty((4, 3), dtype=np.float64)
        for i in range(4):
            R1 = _axis_rot(self.hip_axis[i], q[i, 0])
            R2 = _axis_rot(self.thigh_axis[i], q[i, 1])
            R3 = _axis_rot(self.calf_axis[i], q[i, 2])
            p = self.hip_pos[i] + R1 @ (
                self.thigh_pos[i] + R2 @ (self.calf_pos[i] + R3 @ self.foot_pos[i]))
            out[i] = p
        return out

    def foot_jacobians_body(self, q12: np.ndarray) -> np.ndarray:
        """(12,) 关节角 -> (4,3,3)  Jacobian d(p_foot_body)/d(hip,thigh,calf)。"""
        q = self._q12(q12)
        out = np.empty((4, 3, 3), dtype=np.float64)
        for i in range(4):
            a1, a2, a3 = self.hip_axis[i], self.thigh_axis[i], self.calf_axis[i]
            q1, q2, q3 = q[i, 0], q[i, 1], q[i, 2]
            R1 = _axis_rot(a1, q1)
            R2 = _axis_rot(a2, q2)
            R3 = _axis_rot(a3, q3)
            dR1 = _axis_rot_d(a1, q1)
            dR2 = _axis_rot_d(a2, q2)
            dR3 = _axis_rot_d(a3, q3)
            # p = hip + R1 (thigh + R2 (calf + R3 foot))
            inner = self.thigh_pos[i] + R2 @ (self.calf_pos[i] + R3 @ self.foot_pos[i])
            mid = self.calf_pos[i] + R3 @ self.foot_pos[i]
            out[i, :, 0] = dR1 @ inner
            out[i, :, 1] = R1 @ (dR2 @ mid)
            out[i, :, 2] = R1 @ R2 @ (dR3 @ self.foot_pos[i])
        return out

    def _q12(self, q12: np.ndarray) -> np.ndarray:
        q = np.asarray(q12, dtype=np.float64).reshape(-1)
        if q.size != NUM_JOINT:
            raise ValueError("q12 必须是 %d 维，得到 %d" % (NUM_JOINT, q.size))
        return q.reshape(4, 3)

    def describe(self) -> dict:
        return {
            "xml_path": self.xml_path,
            "base_body": self.base_body,
            "leg_order": list(LEG_ORDER),
            "hip_pos": self.hip_pos.tolist(),
            "thigh_pos": self.thigh_pos.tolist(),
            "calf_pos": self.calf_pos.tolist(),
            "foot_pos": self.foot_pos.tolist(),
            "foot_radius": self.foot_radius.tolist(),
            "hip_axis": self.hip_axis.tolist(),
            "thigh_axis": self.thigh_axis.tolist(),
            "calf_axis": self.calf_axis.tolist(),
        }


# ---------------------------------------------------------------- 里程计


class LegOdometry:
    """接触腿不动假设下的腿式里程计。输出**世界系**线速度 (与训练 frame="global" 对齐)。

    置信度定义 (0..1，见契约 §3 "接触腿数量/质量"):
      * 接触腿 < 2 -> confidence = 0，返回上一帧滤波值 (不猜、不输出野值)；
      * 否则 confidence = (n_contact / 4) * quality，
        quality = 1 / (1 + (spread / 0.2)^2)，spread = 各接触腿估计值的最大偏差 (m/s)。
    """

    #: 接触腿数量低于该值时不出估计
    MIN_CONTACT_LEGS = 2
    #: 多腿一致性判据的参考尺度 (m/s)
    QUALITY_SCALE = 0.2
    #: 单腿离群剔除阈值 (m/s，相对中位数)
    OUTLIER_TOL = 0.5

    def __init__(self, kin: LegKinematics, contact_force_thresh: float = 30.0,
                 cutoff_hz: float = 30.0, max_accel: float = 50.0) -> None:
        self.kin = kin
        self.contact_force_thresh = float(contact_force_thresh)
        self.cutoff_hz = float(cutoff_hz)
        self.max_accel = float(max_accel)
        self.reset()

    # --------------------------------------------------------

    def reset(self) -> None:
        self._v = np.zeros(3, dtype=np.float64)          # 滤波输出 (世界系)
        self._v_raw = np.zeros(3, dtype=np.float64)      # 本帧未滤波估计
        self._confidence = 0.0
        self._has_estimate = False
        self._stats = {
            "n_update": 0, "n_no_state": 0, "n_low_contact": 0,
            "n_contact_hist": [0, 0, 0, 0, 0],           # 索引 = 接触腿数
            "n_leg_estimates": 0, "n_outliers": 0,
            "n_accel_limited": 0, "n_zero_contact": 0,
            "confidence": 0.0, "n_contact_last": 0,
            "spread_last": 0.0, "v_last": [0.0, 0.0, 0.0],
        }

    # --------------------------------------------------------

    @property
    def confidence(self) -> float:
        """0..1：接触腿数量/一致性质量。"""
        return float(self._confidence)

    @property
    def velocity(self) -> np.ndarray:
        """最近一次输出的世界系线速度 (m/s)。"""
        return self._v.copy()

    # --------------------------------------------------------

    def update(self, state: Optional[LowState], dt: float, t: float) -> np.ndarray:
        """一步更新，返回 (3,) **世界系**线速度。

        state 为 None 或 dt<=0 / 接触腿 < 2 -> confidence=0，返回上一帧值
        (从未估计过则为 0)，并在 stats 里计数。
        """
        dt = float(dt)
        self._stats["n_update"] += 1
        if state is None or not np.isfinite(dt) or dt <= 0.0:
            self._confidence = 0.0
            self._stats["n_no_state"] += 1
            self._stats["confidence"] = 0.0
            self._stats["v_last"] = self._v.tolist()
            return self._v.copy()

        q = state.q12()
        dq = state.dq12()
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
            self._confidence = 0.0
            self._stats["n_no_state"] += 1
            self._stats["confidence"] = 0.0
            return self._v.copy()

        forces = np.asarray(list(state.foot_force)[:4], dtype=np.float64)
        if forces.size < 4:
            forces = np.concatenate([forces, np.zeros(4 - forces.size)])
        contact = np.abs(forces) >= self.contact_force_thresh
        n_contact = int(np.count_nonzero(contact))
        self._stats["n_contact_last"] = n_contact
        self._stats["n_contact_hist"][min(n_contact, 4)] += 1

        if n_contact < self.MIN_CONTACT_LEGS:
            self._confidence = 0.0
            self._stats["n_low_contact"] += 1
            if n_contact == 0:
                self._stats["n_zero_contact"] += 1
            self._stats["confidence"] = 0.0
            self._stats["spread_last"] = 0.0
            self._stats["v_last"] = self._v.tolist()
            return self._v.copy()

        R = quat_to_mat(np.asarray(state.imu_state.quaternion, dtype=np.float64))
        gyro = np.asarray(state.imu_state.gyroscope, dtype=np.float64).reshape(-1)
        omega_world = R @ gyro[:3] if gyro.size >= 3 else np.zeros(3)

        p_foot_body = self.kin.foot_positions_body(q)     # (4,3)
        J = self.kin.foot_jacobians_body(q)               # (4,3,3)

        est = []
        for i in range(4):
            if not contact[i]:
                continue
            p_world = R @ p_foot_body[i]
            dq_leg = dq[3 * i:3 * i + 3]
            v_i = -np.cross(omega_world, p_world) - R @ (J[i] @ dq_leg)
            est.append(v_i)
        if not est:
            self._confidence = 0.0
            return self._v.copy()
        V = np.asarray(est)                                # (n,3)
        self._stats["n_leg_estimates"] += V.shape[0]

        # 中位数 + 离群剔除 (腿间应一致; 单腿跳变/打滑会被剔掉)
        med = np.median(V, axis=0)
        dev = np.linalg.norm(V - med, axis=1)
        keep = dev <= self.OUTLIER_TOL
        if not np.any(keep):                               # 全部离群 -> 取最接近的一条
            keep = dev <= dev.min() + 1e-9
        n_out = int(np.count_nonzero(~keep))
        self._stats["n_outliers"] += n_out
        Vk = V[keep]
        v_raw = Vk.mean(axis=0)
        spread = float(np.max(np.linalg.norm(Vk - v_raw, axis=1))) if Vk.shape[0] else 0.0

        quality = 1.0 / (1.0 + (spread / self.QUALITY_SCALE) ** 2)
        self._confidence = float(np.clip(n_contact / 4.0 * quality, 0.0, 1.0))

        # 一阶低通 (cutoff_hz) + 加速度上限 (防野值)
        if self.cutoff_hz > 0.0:
            alpha = (2.0 * np.pi * self.cutoff_hz * dt) / (
                1.0 + 2.0 * np.pi * self.cutoff_hz * dt)
        else:
            alpha = 1.0
        v_lp = self._v + alpha * (v_raw - self._v)
        dv = v_lp - self._v
        if self.max_accel > 0.0:
            lim = self.max_accel * dt
            n_dv = float(np.linalg.norm(dv))
            if n_dv > lim > 0.0:
                dv *= lim / n_dv
                self._stats["n_accel_limited"] += 1
        self._v = self._v + dv
        self._v_raw = v_raw
        self._has_estimate = True

        self._stats["confidence"] = self._confidence
        self._stats["spread_last"] = spread
        self._stats["v_last"] = self._v.tolist()
        return self._v.copy()

    # --------------------------------------------------------

    def stats(self) -> dict:
        d = dict(self._stats)
        d.update({
            "has_estimate": bool(self._has_estimate),
            "confidence": float(self._confidence),
            "velocity": self._v.tolist(),
            "v_raw": self._v_raw.tolist(),
            "contact_force_thresh": self.contact_force_thresh,
            "cutoff_hz": self.cutoff_hz,
            "max_accel": self.max_accel,
        })
        return d
