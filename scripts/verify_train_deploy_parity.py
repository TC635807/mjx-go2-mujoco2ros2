#!/usr/bin/env python3
"""训练侧 <position> Go2 模型 vs 部署侧 <motor> + wrapper PD Go2 模型：物理一致性验证。

背景
----
训练侧 (mjx-go2-training/models/go2/go2_mjx_position.xml):
    <position kp="20" kv="1" forcerange="-23.7 23.7"/> (knee ±45.43)，ctrlrange = 关节限位
    MuJoCo 内部算 force = kp*(ctrl - q) + kv*(0 - dq)，再 clip 到 forcerange。

部署侧 (mjx-go2-mujoco2ros2/models/go2/go2.xml, unitree_mujoco 官方):
    <motor ctrlrange="-23.7 23.7"/> (knee ±45.43)
    sim_env/robot.py::Go2Sim.write_cmd 每个 500 Hz 仿真步算
        tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff，clip 到 ctrlrange
    控制侧 50 Hz 更新 q_des（零阶保持），dq_des = 0、tau_ff = 0
    （controllers.py::PolicyController.update + config/go2_sim.json policy.kp/kd = 20/1）

两者**应当**严格等价。本脚本用 CPU MuJoCo 实测确认或证伪，并做参数归因。

用法
----
    cd mjx-go2-mujoco2ros2
    python scripts/verify_train_deploy_parity.py --mode all --json /tmp/parity.json
    python scripts/verify_train_deploy_parity.py --mode diff
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from signal_io.msg import NUM_JOINT, LowCmd  # noqa: E402
from sim_env.robot import (MOTOR_JOINTS, TORQUE_LIMITS, Go2Sim,  # noqa: E402
                              RobotSpec, _actuator_of_joint)

PROJECT_DEPLOY_BODY = os.path.join(ROOT, "sim_env", "models", "go2", "go2.xml")
GETUP_TRAIN_BODY = "/home/tc63/mujoco/mjx-go2-training/models/go2/go2_mjx_position.xml"
MENAGERIE_BODY = "/home/tc63/mujoco/models/mujoco_menagerie/unitree_go2/go2.xml"

SIM_DT = 0.002
CTRL_DT = 0.02
KP, KD = 20.0, 1.0

# 关节顺序统一按 MOTOR_JOINTS = FR,FL,RR,RL × (hip,thigh,calf)
DEPLOY_HOME = np.array([0.0, 0.9, -1.8] * 4)            # 部署 keyframe home
TRAIN_HOME = np.array([-0.1, 0.8, -1.5,                  # FR
                       0.1, 0.8, -1.5,                   # FL
                       -0.1, 1.0, -1.5,                  # RR
                       0.1, 1.0, -1.5])                  # RL  (训练 keyframe home)
DEPLOY_HOME_Z = 0.27
TRAIN_HOME_Z = 0.32

LIMITS = np.array(TORQUE_LIMITS, dtype=np.float64)      # 23.7,23.7,45.43 × 4
Q_LO = np.array([-1.0472, -1.5708, -2.7227] * 4)
Q_HI = np.array([1.0472, 3.4907, -0.83776] * 4)
# 训练 env 默认求解器 (envs/go2_walk.py: solver_iterations=2, solver_ls_iterations=5)
TRAIN_ITERS, TRAIN_LS = 2, 5
# MuJoCo CPU 默认（部署侧）
DEPLOY_ITERS, DEPLOY_LS = 100, 50


def _arr(x):
    return [float(v) for v in np.asarray(x).reshape(-1)]


def _fmt(v):
    if isinstance(v, (list, tuple)) and v and isinstance(v[0], str):
        return "[" + ", ".join(v) + "]"
    if isinstance(v, str):
        return v
    a = np.asarray(v, dtype=np.float64).reshape(-1)
    if a.size == 1:
        return "%.6g" % a[0]
    return "[" + ", ".join("%.6g" % x for x in a) + "]"


# ======================================================================================
# XML -> 单文件平地场景（绝对路径，绕开 include/meshdir 的解析陷阱）
# ======================================================================================

def _read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _inline_includes(text, base_dir, depth=0):
    notes = []
    if depth > 4:
        return text, notes
    out, pos = [], 0
    for m in re.finditer(r'<include\s+file="([^"]+)"\s*/>', text):
        out.append(text[pos:m.start()])
        inc = m.group(1)
        p = inc if os.path.isabs(inc) else os.path.join(base_dir, inc)
        if not os.path.isfile(p):
            notes.append("include 找不到: %s" % p)
            out.append(m.group(0))
        else:
            sub = _read(p)
            sub = sub[sub.index(">") + 1: sub.rindex("</mujoco>")]
            sub, n2 = _inline_includes(sub, os.path.dirname(p), depth + 1)
            notes.extend(n2)
            out.append("\n<!-- inlined: %s -->\n%s\n" % (p, sub))
        pos = m.end()
    out.append(text[pos:])
    return "".join(out), notes


def make_standalone_scene(body_path, out_path, name):
    """body + 平地（geom 必须叫 floor：训练侧 sensor 里有 geom2="floor"）。"""
    text = _read(body_path)
    notes = []
    cm = re.search(r"<compiler\b([^>]*?)/>", text, re.S)
    attrs = cm.group(1) if cm else ' angle="radian"'
    if "autolimits" not in attrs:
        attrs += ' autolimits="true"'
    md = re.search(r'meshdir\s*=\s*"([^"]*)"', attrs)
    meshdir = md.group(1) if md else "."
    attrs = re.sub(r'\s*meshdir\s*=\s*"[^"]*"', "", attrs)
    abs_mesh = os.path.normpath(os.path.join(os.path.dirname(body_path), meshdir))
    inner = text[cm.end():] if cm else text
    inner = inner[inner.index(">") + 1:]
    inner = inner[: inner.rindex("</mujoco>")]
    inner, n2 = _inline_includes(inner, os.path.dirname(body_path))
    notes.extend(n2)
    scene = ('<mujoco model="%s">\n'
             '  <compiler%s meshdir="%s"/>\n'
             '%s\n'
             '  <worldbody>\n'
             '    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>\n'
             '    <geom name="floor" size="0 0 0.05" type="plane"/>\n'
             '  </worldbody>\n'
             '</mujoco>\n' % (name, attrs, abs_mesh, inner))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(scene)
    return out_path, notes


def make_converted_position_body(src_body, out_xml, kp=KP, kd=KD):
    """同一份 body XML，只把 <motor ctrlrange=X> 换成 <position kp kv forcerange=X>。

    这是 mjx-go2-training/sim/warmstart_bignet.py 的**逆变换**：树顺序、几何、质量、阻尼全部逐字不动，
    唯一变化就是执行器写法。用它可以把"执行器写法是否等价"与"模型本身是否有差异"彻底分开。
    """
    text = _read(src_body)
    abs_mesh = os.path.normpath(os.path.join(os.path.dirname(src_body),
                                            (re.search(r'meshdir\s*=\s*"([^"]*)"', text).group(1)
                                             if re.search(r'meshdir\s*=\s*"([^"]*)"', text) else ".")))
    text = re.sub(r'meshdir\s*=\s*"[^"]*"', 'meshdir="%s"' % abs_mesh, text, count=1)
    head, sep, tail = text.partition("<actuator>")
    if not sep:
        raise SystemExit("[convert] 找不到 <actuator> 段")
    rest, sep2, after = tail.partition("</actuator>")
    if not sep2:
        raise SystemExit("[convert] 找不到 </actuator>")
    n = len(re.findall(r"<motor\b", rest))
    rest = rest.replace("<motor ", "<position ")
    text = head + sep + rest + sep2 + after
    if n != 12:
        raise SystemExit("[convert] 期望 12 个 <motor>，实际 %d" % n)
    # <motor ctrlrange="X"> -> <position kp kv forcerange="X">
    text = re.sub(r'<motor ctrlrange="([^"]*)"\s*/>',
                  lambda m: '<position kp="%g" kv="%g" forcerange="%s"/>' % (kp, kd, m.group(1)), text)
    text = text.replace('<position kp="%g" kv="%g"/>' % (kp, kd),
                        '<position kp="%g" kv="%g" forcerange="-23.7 23.7"/>' % (kp, kd))
    if text.count("<motor") != 0:
        raise SystemExit("[convert] 还有 %d 处 <motor> 没转换" % text.count("<motor"))
    os.makedirs(os.path.dirname(out_xml), exist_ok=True)
    with open(out_xml, "w", encoding="utf-8") as f:
        f.write(text)
    return out_xml


# ======================================================================================
# 结构差异
# ======================================================================================

def _named(m, kind, n):
    out = {}
    for i in range(n):
        nm = mujoco.mj_id2name(m, kind, i)
        out[nm if nm else "<anon#%d>" % i] = i
    return out


def _same(va, vb, tol=1e-9):
    if isinstance(va, (str, list, tuple)) or isinstance(vb, (str, list, tuple)):
        return list(va) == list(vb)
    a, b = np.asarray(va, dtype=np.float64), np.asarray(vb, dtype=np.float64)
    if a.shape != b.shape:
        return False
    return bool(np.all(np.abs(a - b) <= tol + tol * np.abs(a)))


def structural_diff(ma, mb, na="deploy", nb="train", tol=1e-9):
    rows = []

    def add(group, item, va, vb):
        rows.append({"group": group, "item": item, na: va, nb: vb, "same": _same(va, vb, tol)})

    for f in ("timestep", "iterations", "ls_iterations", "cone", "impratio", "jacobian",
              "solver", "integrator", "tolerance", "o_solref", "o_solimp", "o_friction"):
        if hasattr(ma.opt, f) and hasattr(mb.opt, f):
            va, vb = getattr(ma.opt, f), getattr(mb.opt, f)
            add("option", f, _arr(va) if hasattr(va, "__len__") else va,
                _arr(vb) if hasattr(vb, "__len__") else vb)

    bna = _named(ma, mujoco.mjtObj.mjOBJ_BODY, ma.nbody)
    bnb = _named(mb, mujoco.mjtObj.mjOBJ_BODY, mb.nbody)
    add("tree", "worldbody 顺序", [k for k in sorted(bna, key=bna.get)],
        [k for k in sorted(bnb, key=bnb.get)])
    for nm in sorted(set(bna) & set(bnb)):
        ia, ib = bna[nm], bnb[nm]
        add("body", "%s.mass" % nm, float(ma.body_mass[ia]), float(mb.body_mass[ib]))
        add("body", "%s.inertia" % nm, _arr(ma.body_inertia[ia]), _arr(mb.body_inertia[ib]))
        add("body", "%s.ipos+iquat" % nm, _arr(ma.body_ipos[ia]) + _arr(ma.body_iquat[ia]),
            _arr(mb.body_ipos[ib]) + _arr(mb.body_iquat[ib]))
        add("body", "%s.pos" % nm, _arr(ma.body_pos[ia]), _arr(mb.body_pos[ib]))
        add("body", "%s.quat" % nm, _arr(ma.body_quat[ia]), _arr(mb.body_quat[ib]))

    jna = _named(ma, mujoco.mjtObj.mjOBJ_JOINT, ma.njnt)
    jnb = _named(mb, mujoco.mjtObj.mjOBJ_JOINT, mb.njnt)
    add("tree", "joint 顺序", [k for k in sorted(jna, key=jna.get)],
        [k for k in sorted(jnb, key=jnb.get)])
    for nm in sorted(set(jna) & set(jnb)):
        ia, ib = jna[nm], jnb[nm]
        add("joint", "%s.range" % nm, _arr(ma.jnt_range[ia]), _arr(mb.jnt_range[ib]))
        add("joint", "%s.axis" % nm, _arr(ma.jnt_axis[ia]), _arr(mb.jnt_axis[ib]))
        add("joint", "%s.pos" % nm, _arr(ma.jnt_pos[ia]), _arr(mb.jnt_pos[ib]))
        add("joint", "%s.type" % nm, int(ma.jnt_type[ia]), int(mb.jnt_type[ib]))
        if ma.jnt_type[ia] == mb.jnt_type[ib] and ma.jnt_type[ia] != mujoco.mjtJoint.mjJNT_FREE:
            da, db = int(ma.jnt_dofadr[ia]), int(mb.jnt_dofadr[ib])
            add("joint", "%s.damping" % nm, float(ma.dof_damping[da]), float(mb.dof_damping[db]))
            add("joint", "%s.armature" % nm, float(ma.dof_armature[da]), float(mb.dof_armature[db]))
            add("joint", "%s.frictionloss" % nm, float(ma.dof_frictionloss[da]), float(mb.dof_frictionloss[db]))

    # geom：按 (所属 body 名, 该 body 内第几个 geom) 配对，屏蔽匿名 geom 的下标漂移
    def geom_key(m, g):
        b = m.geom_bodyid[g]
        bnm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "<body%d>" % b
        local = int(np.sum(m.geom_bodyid[:g + 1] == b)) - 1
        gnm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        return "%s#%d%s" % (bnm, local, (":" + gnm) if gnm else "")
    ga = {geom_key(ma, g): g for g in range(ma.ngeom)}
    gb = {geom_key(mb, g): g for g in range(mb.ngeom)}
    add("tree", "geom 数量", ma.ngeom, mb.ngeom)
    only_a = sorted(set(ga) - set(gb))
    only_b = sorted(set(gb) - set(ga))
    if only_a or only_b:
        add("geom", "仅部署有 / 仅训练有", only_a, only_b)
    for nm in sorted(set(ga) & set(gb)):
        ia, ib = ga[nm], gb[nm]
        for f in ("size", "pos", "quat", "friction", "solimp", "solref"):
            add("geom", "geom[%s].%s" % (nm, f), _arr(getattr(ma, "geom_" + f)[ia]),
                _arr(getattr(mb, "geom_" + f)[ib]))
        for f, cast in (("condim", int), ("priority", int), ("margin", float),
                        ("type", int), ("contype", int), ("conaffinity", int)):
            add("geom", "geom[%s].%s" % (nm, f), cast(getattr(ma, "geom_" + f)[ia]),
                cast(getattr(mb, "geom_" + f)[ib]))

    ana = _named(ma, mujoco.mjtObj.mjOBJ_ACTUATOR, ma.nu)
    anb = _named(mb, mujoco.mjtObj.mjOBJ_ACTUATOR, mb.nu)
    add("tree", "actuator 顺序", [k for k in sorted(ana, key=ana.get)],
        [k for k in sorted(anb, key=anb.get)])
    for nm in sorted(set(ana) & set(anb)):
        ia, ib = ana[nm], anb[nm]
        add("actuator", "act[%s].gaintype+biastype" % nm,
            [int(ma.actuator_gaintype[ia]), int(ma.actuator_biastype[ia])],
            [int(mb.actuator_gaintype[ib]), int(mb.actuator_biastype[ib])])
        add("actuator", "act[%s].gainprm" % nm, _arr(ma.actuator_gainprm[ia]), _arr(mb.actuator_gainprm[ib]))
        add("actuator", "act[%s].biasprm" % nm, _arr(ma.actuator_biasprm[ia]), _arr(mb.actuator_biasprm[ib]))
        add("actuator", "act[%s].ctrlrange+limited" % nm,
            _arr(ma.actuator_ctrlrange[ia]) + [int(ma.actuator_ctrllimited[ia])],
            _arr(mb.actuator_ctrlrange[ib]) + [int(mb.actuator_ctrllimited[ib])])
        add("actuator", "act[%s].forcerange+limited" % nm,
            _arr(ma.actuator_forcerange[ia]) + [int(ma.actuator_forcelimited[ia])],
            _arr(mb.actuator_forcerange[ib]) + [int(mb.actuator_forcelimited[ib])])

    if ma.nkey and mb.nkey:
        add("keyframe", "home.qpos", _arr(ma.key_qpos[0]), _arr(mb.key_qpos[0]))
        add("keyframe", "home.ctrl", _arr(ma.key_ctrl[0]), _arr(mb.key_ctrl[0]))
    add("model", "nq/nv/nu/nbody/ngeom",
        [ma.nq, ma.nv, ma.nu, ma.nbody, ma.ngeom], [mb.nq, mb.nv, mb.nu, mb.nbody, mb.ngeom])
    return rows


_GROUP_ZH = {"option": "求解器选项", "body": "刚体(质量/惯量/位姿)", "joint": "关节",
             "geom": "几何/接触", "actuator": "执行器", "keyframe": "keyframe",
             "tree": "树顺序", "model": "模型规模"}


def _collapse(item):
    """把 geom[FL_calf#5:FL].solimp 折成 geom[*].solimp，便于归类同类差异。"""
    for pre in ("geom[", "act["):
        if item.startswith(pre) and "]." in item:
            return pre + "*]." + item.split("].", 1)[1]
    return item


def diff_summary(rows, na="deploy", nb="train"):
    """把逐元素差异折叠成"差异类别"（值完全相同的一类合并计数）。"""
    buckets = {}
    for r in rows:
        if r["same"]:
            continue
        key = (r["group"], _collapse(r["item"]), json.dumps(r[na], ensure_ascii=False),
               json.dumps(r[nb], ensure_ascii=False))
        b = buckets.setdefault(key, {"group": r["group"], "field": _collapse(r["item"]),
                                     na: r[na], nb: r[nb], "count": 0, "examples": []})
        b["count"] += 1
        if len(b["examples"]) < 3:
            b["examples"].append(r["item"])
    return sorted(buckets.values(), key=lambda x: (x["group"], -x["count"], x["field"]))


def diff_table_text(rows, na="deploy", nb="train"):
    buckets = diff_summary(rows, na, nb)
    out = ["| 组 | 差异类别 | %s | %s | 元素数 | 例 |" % (na, nb), "|---|---|---|---|---|---|"]
    for b in buckets:
        out.append("| %s | %s | %s | %s | %d | %s |" % (
            _GROUP_ZH.get(b["group"], b["group"]), b["field"], _fmt(b[na]), _fmt(b[nb]),
            b["count"], ", ".join(b["examples"])))
    out.append("")
    out.append("逐元素共 %d 项对比，**%d 项不同**；折叠为 %d 类差异。" % (
        len(rows), sum(1 for r in rows if not r["same"]), len(buckets)))
    return "\n".join(out)


# ======================================================================================
# 两套物理语义
# ======================================================================================

class Engine:
    """统一接口。kind='motor' 走部署语义；kind='position' 走训练语义。"""

    def __init__(self, scene, kind, kp=KP, kd=KD, sim_dt=SIM_DT, ctrl_dt=CTRL_DT,
                 iters=None, ls=None, patch=None, limits=None, disable_contact=False):
        assert kind in ("motor", "position")
        self.kind = kind
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.model.opt.timestep = float(sim_dt)
        if iters is not None:
            self.model.opt.iterations = int(iters)
        if ls is not None:
            self.model.opt.ls_iterations = int(ls)
        if disable_contact:
            self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        if patch:   # patch 最后应用，保证 patch 里的设置不被 iters/ls 覆盖
            patch(self.model)
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.decim = max(1, int(round(ctrl_dt / self.dt)))
        self.kp, self.kd = float(kp), float(kd)
        self.limits = np.asarray(limits if limits is not None else LIMITS, dtype=np.float64)
        self.act_id = np.array([_actuator_of_joint(self.model, j) for j in MOTOR_JOINTS], dtype=int)
        self.qpos_adr = np.array([self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in MOTOR_JOINTS], dtype=int)
        self.dof_adr = np.array([self.model.jnt_dofadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in MOTOR_JOINTS], dtype=int)
        if kind == "position":
            self.set_gains(kp, kd)
        self.tau_applied = np.zeros(NUM_JOINT)

    def set_gains(self, kp, kd):
        """部署侧改 kp/kd = 训练侧改 <position kp kv>。

        实测确认 MuJoCo 的 <position kp kv> 编译结果是
            gainprm = [kp, 0, 0], biasprm = [0, -kp, -kv]
        => force = kp*ctrl - kp*q - kv*dq = kp*(ctrl-q) - kv*dq，与 wrapper 的
           tau = kp*(q_des-q) + kd*(0-dq) 逐项相同（kv == kd）。
        """
        self.kp, self.kd = float(kp), float(kd)
        if self.kind == "position":
            for a in range(self.model.nu):
                self.model.actuator_gainprm[a, 0] = kp
                self.model.actuator_biasprm[a, 1] = -kp
                self.model.actuator_biasprm[a, 2] = -kd

    def reset(self, home=DEPLOY_HOME, z=DEPLOY_HOME_Z):
        """按**关节名**摆位，屏蔽两侧 body 树顺序差异。"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qpos[2] = float(z)
        self.data.qpos[3] = 1.0
        self.data.qpos[self.qpos_adr] = np.asarray(home, dtype=np.float64)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.tau_applied = np.zeros(NUM_JOINT)

    def write_cmd(self, q_des):
        q = np.asarray(self.data.qpos[self.qpos_adr], dtype=np.float64)
        dq = np.asarray(self.data.qvel[self.dof_adr], dtype=np.float64)
        q_des = np.asarray(q_des, dtype=np.float64)
        if self.kind == "position":
            self.data.ctrl[self.act_id] = q_des
            lo = np.where(self.model.actuator_forcelimited[self.act_id],
                          self.model.actuator_forcerange[self.act_id, 0], -np.inf)
            hi = np.where(self.model.actuator_forcelimited[self.act_id],
                          self.model.actuator_forcerange[self.act_id, 1], np.inf)
            tau = np.clip(self.kp * (q_des - q) - self.kd * dq, lo, hi)
        else:
            # = Go2Sim.write_cmd: tau = kp*(q_des-q) + kd*(0-dq) + 0，再 clip 到 ctrlrange
            tau = np.clip(self.kp * (q_des - q) - self.kd * dq, -self.limits, self.limits)
            self.data.ctrl[self.act_id] = tau
        self.tau_applied = tau
        return tau

    def force_now(self):
        """用 MuJoCo 自己的 actuator_force 读回当前 ctrl 产生的力矩（会做一次 mj_forward）。"""
        mujoco.mj_forward(self.model, self.data)
        return np.asarray(self.data.actuator_force[self.act_id], dtype=np.float64)

    def step(self, n=1):
        mujoco.mj_step(self.model, self.data, n)

    def joint_state(self):
        return (np.asarray(self.data.qpos[self.qpos_adr], dtype=np.float64),
                np.asarray(self.data.qvel[self.dof_adr], dtype=np.float64))


def action_program(kind, home, n_ctrl, seed=0, rand_s=3.0, amp=0.12):
    t = np.arange(n_ctrl) * CTRL_DT
    if kind in ("stand", "settle"):
        return np.tile(home, (n_ctrl, 1)), "固定站立 %.1f s (q_des=部署 home)" % (n_ctrl * CTRL_DT)
    if kind == "settle5":
        return np.tile(TRAIN_HOME, (n_ctrl, 1)), "固定站立 %.1f s (q_des=训练 home)" % (n_ctrl * CTRL_DT)
    if kind == "rand_then_stand":
        rng = np.random.default_rng(seed)
        sig = np.zeros((n_ctrl, NUM_JOINT))
        for j in range(NUM_JOINT):
            for _ in range(3):
                sig[:, j] += (rng.uniform(0.4, 1.0) * amp) * np.sin(
                    2 * np.pi * rng.uniform(0.3, 1.5) * t + rng.uniform(0.0, 2 * np.pi))
        mask = (t < rand_s)[:, None].astype(float)
        return (np.clip(home[None, :] + sig * mask, Q_LO, Q_HI),
                "%.1f s 平滑随机 (幅值 %.2f rad) + %.1f s 固定站立" % (rand_s, amp, n_ctrl * CTRL_DT - rand_s))
    if kind == "small":
        rng = np.random.default_rng(seed)
        sig = np.zeros((n_ctrl, NUM_JOINT))
        for j in range(NUM_JOINT):
            sig[:, j] = 0.02 * np.sin(2 * np.pi * rng.uniform(0.3, 1.0) * t + rng.uniform(0, 2 * np.pi))
        return np.clip(home[None, :] + sig, Q_LO, Q_HI), "小信号 ±0.02 rad"
    raise ValueError(kind)


def run(eng, actions, sim_s):
    eng.reset()
    n_sim = int(round(sim_s / eng.dt))
    t = np.empty(n_sim + 1)
    q = np.empty((n_sim + 1, NUM_JOINT))
    dq = np.empty((n_sim + 1, NUM_JOINT))
    base = np.empty((n_sim + 1, 3))
    quat = np.empty((n_sim + 1, 4))
    tau = np.empty((n_sim + 1, NUM_JOINT))
    t[0] = 0.0
    q[0], dq[0] = eng.joint_state()
    base[0] = eng.data.qpos[:3]
    quat[0] = eng.data.qpos[3:7]
    tau[0] = eng.tau_applied
    tgt = actions[0]
    for k in range(1, n_sim + 1):
        if (k - 1) % eng.decim == 0:
            tgt = actions[min((k - 1) // eng.decim, len(actions) - 1)]
        eng.write_cmd(tgt)
        eng.step(1)
        t[k] = k * eng.dt
        q[k], dq[k] = eng.joint_state()
        base[k] = eng.data.qpos[:3]
        quat[k] = eng.data.qpos[3:7]
        tau[k] = eng.tau_applied
    return {"t": t, "q": q, "dq": dq, "base": base, "quat": quat, "tau": tau}


def tilt_deg(quat):
    w, x, y, z = quat
    r22 = 1 - 2 * (x * x + y * y)
    return float(np.degrees(np.arccos(np.clip(r22, -1, 1))))


def compare(ra, rb):
    dq = np.abs(ra["q"] - rb["q"])
    ddq = np.abs(ra["dq"] - rb["dq"])
    db = np.abs(ra["base"] - rb["base"])
    over = np.where(dq.max(axis=1) > 0.05)[0]
    return {
        "n_steps": int(len(ra["t"])),
        "max_dq": float(dq.max()),
        "rms_dq": float(np.sqrt((dq ** 2).mean())),
        "max_dq_per_joint": _arr(dq.max(axis=0)),
        "max_ddq": float(ddq.max()),
        "max_dbase_xyz": _arr(db.max(axis=0)),
        "max_dtau": float(np.abs(ra["tau"] - rb["tau"]).max()),
        "divergence_time_0.05rad": (float(ra["t"][over[0]]) if over.size else None),
        "final": {
            "a_base_z": float(ra["base"][-1, 2]), "b_base_z": float(rb["base"][-1, 2]),
            "a_tilt_deg": tilt_deg(ra["quat"][-1]), "b_tilt_deg": tilt_deg(rb["quat"][-1]),
            "a_min_base_z": float(ra["base"][:, 2].min()), "b_min_base_z": float(rb["base"][:, 2].min()),
            "a_final_q": _arr(ra["q"][-1]), "b_final_q": _arr(rb["q"][-1]),
            "a_xy": _arr(ra["base"][-1, :2]), "b_xy": _arr(rb["base"][-1, :2]),
        },
    }


def brief(c):
    f = c["final"]
    return ("max|dq|=%.3e rad  max|ddq|=%.3e  max|dtau|=%.3e N·m | 末态 deploy z=%.4f tilt=%.2f° "
            "/ train z=%.4f tilt=%.2f° | 首次 |dq|>0.05: %s" % (
                c["max_dq"], c["max_ddq"], c["max_dtau"], f["a_base_z"], f["a_tilt_deg"],
                f["b_base_z"], f["b_tilt_deg"],
                ("%.2f s" % c["divergence_time_0.05rad"]) if c["divergence_time_0.05rad"] is not None else "未超过"))


# ======================================================================================
# 各模式
# ======================================================================================

def cmd_diff(args, M, out):
    print("=" * 96)
    print("M1. 结构差异")
    print("=" * 96)
    ma = mujoco.MjModel.from_xml_path(M["deploy_scene"])
    mb = mujoco.MjModel.from_xml_path(M["train_scene"])
    rows = structural_diff(ma, mb)
    txt = diff_table_text(rows)
    print(txt)
    out["diff"] = {"rows": rows, "n_rows": len(rows),
                   "n_diff": sum(1 for r in rows if not r["same"]), "table": txt,
                   "summary": diff_summary(rows)}
    groups = {}
    for r in rows:
        if not r["same"]:
            groups.setdefault(r["group"], []).append(r["item"])
    print("\n差异按组统计:")
    for g in ("tree", "actuator", "geom", "joint", "body", "option", "keyframe", "model"):
        if g in groups:
            print("  %-8s %3d 项: %s" % (g, len(groups[g]), ", ".join(groups[g][:10])
                                        + (" ..." if len(groups[g]) > 10 else "")))
    out["diff"]["groups"] = groups


def cmd_actuator(args, M, out):
    print("=" * 96)
    print("M2. 执行器语义：<position kp kv forcerange> vs <motor ctrlrange> + Python PD")
    print("=" * 96)
    dep = Engine(M["deploy_scene"], "motor")
    trn = Engine(M["train_scene"], "position")
    print("[M2.1] MuJoCo 编译后的执行器定义")
    for tag, m in (("deploy", dep.model), ("train", trn.model)):
        for nm in ("FR_hip", "FR_calf"):
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, nm)
            print("  %-6s %-8s gaintype=%d biastype=%d gainprm=%s biasprm=%s ctrlL=%d %s forceL=%d %s" % (
                tag, nm, m.actuator_gaintype[a], m.actuator_biastype[a],
                _fmt(m.actuator_gainprm[a][:3]), _fmt(m.actuator_biasprm[a][:3]),
                m.actuator_ctrllimited[a], _fmt(m.actuator_ctrlrange[a]),
                m.actuator_forcelimited[a], _fmt(m.actuator_forcerange[a])))
    print("  <motor>    : force = gainprm[0]*ctrl = 1*ctrl, ctrl 被 clip 到 ctrlrange = 力矩限幅")
    print("  <position> : force = kp*ctrl - (kp+kv)*q - kv*dq, force 被 clip 到 forcerange")

    print()
    print("[M2.2a-1] 同一份 body，只把 <motor ctrlrange> 换成 <position kp kv forcerange>（树完全一致）")
    conv_body = make_converted_position_body(M["deploy_body"],
                                            os.path.join(out["workdir"], "go2_position_conv.xml"))
    conv_scene = os.path.join(os.path.dirname(conv_body), "conv_scene.xml")
    make_standalone_scene(conv_body, conv_scene, "conv")
    dep_c = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
    trn_c = Engine(conv_scene, "position", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
    rng = np.random.default_rng(0)
    dep_c.reset(); trn_c.reset()
    mx_q = mx_tau = 0.0
    for k in range(1500):
        tgt = np.clip(TRAIN_HOME + 0.15 * rng.standard_normal(NUM_JOINT), Q_LO, Q_HI)
        dep_c.write_cmd(tgt); trn_c.write_cmd(tgt)
        fa, fb = dep_c.force_now(), trn_c.force_now()
        mx_tau = max(mx_tau, float(np.abs(fa - fb).max()))
        dep_c.step(1); trn_c.step(1)
        mx_q = max(mx_q, float(np.abs(dep_c.joint_state()[0] - trn_c.joint_state()[0]).max()))
    print("  1500 步(3 s): max|dtau| = %.3e N·m   max|dq| = %.3e rad" % (mx_tau, mx_q))
    ok = mx_q < 1e-9
    print("  -> %s" % ("执行器写法严格等价 ✓  (wrapper PD == <position kp kv>)" if ok else "执行器写法**不等价** ✗"))
    out["actuator"] = {"equiv_same_tree": {"max_dtau": mx_tau, "max_dq": mx_q, "pass": bool(ok)}}

    print()
    print("[M2.2a-2] 同树对照做不成（真训练模型树顺序不同）时，用真训练模型近似复现")
    dep_a = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
    trn_a = Engine(M["train_scene"], "position", iters=DEPLOY_ITERS, ls=DEPLOY_LS,
                   patch=_full_revert_to_deploy)
    rng = np.random.default_rng(0)
    dep_a.reset(); trn_a.reset()
    mx_q = mx_tau = 0.0
    for k in range(1500):
        tgt = np.clip(DEPLOY_HOME + 0.15 * rng.standard_normal(NUM_JOINT), Q_LO, Q_HI)
        dep_a.write_cmd(tgt); trn_a.write_cmd(tgt)
        # 两侧都让 MuJoCo 自己算 actuator_force 再比，避免拿"公式"跟"公式"比
        fa, fb = dep_a.force_now(), trn_a.force_now()
        mx_tau = max(mx_tau, float(np.abs(fa - fb).max()))
        dep_a.step(1); trn_a.step(1)
        qa, _ = dep_a.joint_state(); qb, _ = trn_a.joint_state()
        mx_q = max(mx_q, float(np.abs(qa - qb).max()))
    print("  1500 步(3 s): max|dtau| = %.3e N·m   max|dq| = %.3e rad" % (mx_tau, mx_q))
    print("  （这是把训练模型除执行器外全部改成部署取值后的残差；树顺序不同带来浮点级差异，会随时间放大）")
    out["actuator"]["equiv_cross_tree_reverted"] = {"max_dtau": mx_tau, "max_dq": mx_q}

    print()
    print("[M2.2a-3] 无接触对照（关掉 CONTACT）：排除接触求解器顺序敏感性")
    for tag, sc, kd_ in (("同树 <position>", conv_scene, "position"),
                         ("真训练模型(其余改回部署)", M["train_scene"], "position")):
        dep_n = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS, disable_contact=True)
        trn_n = Engine(sc, kd_, iters=DEPLOY_ITERS, ls=DEPLOY_LS, disable_contact=True,
                       patch=(None if sc == conv_scene else _full_revert_to_deploy))
        dep_n.reset(home=TRAIN_HOME, z=TRAIN_HOME_Z)
        trn_n.reset(home=TRAIN_HOME, z=TRAIN_HOME_Z)
        mx = 0.0
        rngn = np.random.default_rng(0)
        for k in range(1500):
            tgt = np.clip(TRAIN_HOME + 0.15 * rngn.standard_normal(NUM_JOINT), Q_LO, Q_HI)
            dep_n.write_cmd(tgt); trn_n.write_cmd(tgt)
            dep_n.step(1); trn_n.step(1)
            mx = max(mx, float(np.abs(dep_n.joint_state()[0] - trn_n.joint_state()[0]).max()))
        print("  %-26s 1500 步 max|dq| = %.3e rad" % (tag, mx))
        out["actuator"].setdefault("no_contact", {})[tag] = mx
    print("  -> 无接触还差很多 => 模型本身有物理差异；无接触几乎为 0 => 残差纯粹来自接触求解顺序")

    print()
    print("[M2.2b] 按各自出厂配置（部署: 硬接触+100/50；训练: 软接触+2/5）")
    dep_b = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
    trn_b = Engine(M["train_scene"], "position", iters=TRAIN_ITERS, ls=TRAIN_LS)
    dep_b.reset(); trn_b.reset()
    mx_q2 = 0.0
    for k in range(1500):
        tgt = np.clip(DEPLOY_HOME + 0.15 * rng.standard_normal(NUM_JOINT), Q_LO, Q_HI)
        dep_b.write_cmd(tgt); trn_b.write_cmd(tgt)
        dep_b.step(1); trn_b.step(1)
        qa, _ = dep_b.joint_state(); qb, _ = trn_b.joint_state()
        mx_q2 = max(mx_q2, float(np.abs(qa - qb).max()))
    print("  1500 步(3 s): max|dq| = %.3e rad" % mx_q2)
    out["actuator"]["as_shipped_max_dq_3s"] = mx_q2

    print()
    print("[M2.3] 脚本 PD 与项目 wrapper Go2Sim.write_cmd 是否一致")
    sim = Go2Sim(M["deploy_scene"], spec=RobotSpec(name="go2"), sim_dt=SIM_DT)
    mine = Engine(M["deploy_scene"], "motor", limits=sim.torque_limits)
    q0 = np.zeros(19); q0[2] = DEPLOY_HOME_Z; q0[3] = 1.0; q0[7:19] = DEPLOY_HOME
    sim.data.qpos[:] = q0; sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    mine.reset()
    worst = 0.0
    for k in range(200):
        qd = np.clip(DEPLOY_HOME + 0.4 * rng.standard_normal(NUM_JOINT), Q_LO, Q_HI)
        cmd = LowCmd()
        for i in range(NUM_JOINT):
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.kp, m.kd, m.tau = 0x01, float(qd[i]), 0.0, KP, KD, 0.0
        worst = max(worst, float(np.abs(sim.write_cmd(cmd) - mine.write_cmd(qd)).max()))
        sim.step(1); mine.step(1)
    print("  max|tau_wrapper - tau_script| = %.3e N·m" % worst)
    out["actuator"]["wrapper_pd_max_diff"] = worst

    print()
    print("[M2.4] forcerange 是否真的会触发（漏写 forcerange 的风险量化）")
    print("  解析: |tau|max = kp*|q_des-q| + kd*|dq|，hip/thigh 上限 23.7、knee 45.43")
    for kp in (20.0, 50.0):
        for tag, patch in (("有 forcerange", None),
                            ("无 forcerange", lambda m: m.actuator_forcerange.__setitem__(
                                slice(None), np.array([-1e10, 1e10])))):
            eng = Engine(M["train_scene"], "position", kp=kp, kd=KD, patch=patch,
                         iters=DEPLOY_ITERS, ls=DEPLOY_LS)
            eng.reset()
            peak = np.zeros(NUM_JOINT)
            rng2 = np.random.default_rng(2)
            for k in range(250):   # 0.5 s 大幅随机目标（模拟策略输出抖动）
                tgt = np.clip(DEPLOY_HOME + 0.9 * rng2.standard_normal(NUM_JOINT), Q_LO, Q_HI)
                eng.write_cmd(tgt)
                peak = np.maximum(peak, np.abs(eng.force_now()))
                eng.step(1)
            print("  kp=%-4g %-14s |tau| 峰值 = %s  (hip/thigh 上限 23.7, knee 45.43)" % (
                kp, tag, _fmt(np.round(peak, 3))))
    print("  -> 训练侧写 forcerange 时 MuJoCo 才 clip 力矩；autolimits 只把 ctrlrange 设成关节限位")


def tilt_series(quat):
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    r22 = 1 - 2 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1, 1)))


def settled(r, t_lo, t_hi):
    m = (r["t"] >= t_lo) & (r["t"] <= t_hi)
    return {"z": float(r["base"][m, 2].mean()),
            "tilt": float(tilt_series(r["quat"][m]).mean()),
            "q": _arr(r["q"][m].mean(axis=0)),
            "xy": _arr(r["base"][m, :2].mean(axis=0)),
            "z_std": float(r["base"][m, 2].std())}


def cmd_harness(args, M, out):
    print("=" * 96)
    print("M0. 装置自检：复现任务书给出的实测值 (训练 home + PD kp=20 kd=1 -> z≈0.268 m, 倾角≈1.58°)")
    print("=" * 96)
    n_ctrl = int(round(6.0 / CTRL_DT))
    acts, _ = action_program("stand", TRAIN_HOME, n_ctrl)
    res = {}
    for tag, scene, kind, it, ls in (("部署(go2.xml <motor>)", M["deploy_scene"], "motor", DEPLOY_ITERS, DEPLOY_LS),
                                     ("训练(go2_mjx_position.xml)", M["train_scene"], "position", TRAIN_ITERS, TRAIN_LS)):
        eng = Engine(scene, kind, iters=it, ls=ls)
        eng.reset(home=TRAIN_HOME, z=TRAIN_HOME_Z)
        r = eng.reset  # noqa
        rr = run(eng, acts, 6.0)
        s = settled(rr, 4.0, 6.0)
        s["z_series"] = {("%.1f" % tt): float(rr["base"][int(round(tt / eng.dt)), 2])
                         for tt in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)}
        s["tilt_series"] = {("%.1f" % tt): float(tilt_series(rr["quat"][int(round(tt / eng.dt)):int(round(tt / eng.dt)) + 1])[0])
                             for tt in (0.5, 1.0, 2.0, 4.0, 6.0)}
        s["z_min"] = float(rr["base"][:, 2].min())
        res[tag] = s
        print("  %-28s 沉降后 z = %.4f m (std %.4f), 倾角 = %.2f°, xy = %s" % (
            tag, s["z"], s["z_std"], s["tilt"], _fmt(s["xy"])))
        print("      z(t) 采样: %s m   |  z_min = %.4f" % (
            {k: round(v, 4) for k, v in s["z_series"].items()}, s["z_min"]))
    out["harness"] = res


def cmd_trajectory(args, M, out):
    print("=" * 96)
    print("M3. 轨迹等价性（同一初始状态 + 同一串 q_des）")
    print("=" * 96)
    dep = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
    trn = Engine(M["train_scene"], "position", iters=TRAIN_ITERS, ls=TRAIN_LS)
    fg_d, fg_t = _foot_geoms(dep.model)[0], _foot_geoms(trn.model)[0]
    print("部署: 阻尼 %.3f 摩擦 %.3f armature %.4f | 足端 condim %d solimp %s friction %s" % (
        dep.model.dof_damping[dep.dof_adr[0]], dep.model.dof_frictionloss[dep.dof_adr[0]],
        dep.model.dof_armature[dep.dof_adr[0]], dep.model.geom_condim[fg_d],
        _fmt(dep.model.geom_solimp[fg_d]), _fmt(dep.model.geom_friction[fg_d])))
    print("训练: 阻尼 %.3f 摩擦 %.3f armature %.4f | 足端 condim %d solimp %s friction %s" % (
        trn.model.dof_damping[trn.dof_adr[0]], trn.model.dof_frictionloss[trn.dof_adr[0]],
        trn.model.dof_armature[trn.dof_adr[0]], trn.model.geom_condim[fg_t],
        _fmt(trn.model.geom_solimp[fg_t]), _fmt(trn.model.geom_friction[fg_t])))
    print("求解器: 部署 %d/%d | 训练 %d/%d" % (dep.model.opt.iterations, dep.model.opt.ls_iterations,
                                            trn.model.opt.iterations, trn.model.opt.ls_iterations))
    res = {}
    for kind, sim_s, seed in (("settle", 3.0, 0), ("small", 1.0, 1),
                              ("rand_then_stand", 6.0, 0), ("settle5", 5.0, 0)):
        n_ctrl = int(round(sim_s / CTRL_DT))
        acts, desc = action_program(kind, DEPLOY_HOME, n_ctrl, seed=seed)
        ra, rb = run(dep, acts, sim_s), run(trn, acts, sim_s)
        c = compare(ra, rb); c["desc"] = desc
        res[kind] = c
        print("\n-- %s" % desc)
        print("   " + brief(c))
        print("   末态关节偏差 (deg): %s" % _fmt(np.degrees(np.array(c["final"]["a_final_q"])
                                                          - np.array(c["final"]["b_final_q"]))))
        print("   末态 xy 漂移 deploy %s / train %s" % (_fmt(c["final"]["a_xy"]), _fmt(c["final"]["b_xy"])))
    out["trajectory"] = res


def cmd_sensitivity(args, M, out):
    print("=" * 96)
    print("M4. kp/kd 敏感性扫描（3 s 固定站立，q_des = 部署 home）")
    print("=" * 96)
    kps, kds = [20.0, 30.0, 50.0], [0.5, 1.0, 1.5]
    sim_s = 3.0
    n_ctrl = int(round(sim_s / CTRL_DT))
    res = {}
    for tgt_name, tgt in (("deploy_home", DEPLOY_HOME), ("train_home", TRAIN_HOME)):
        acts, _ = action_program("stand", tgt, n_ctrl)
        print()
        print("  目标姿态 q_des = %s" % tgt_name)
        print("  %-5s %-5s | %-9s %-9s | %-9s %-9s | %-11s %-11s | %s" % (
            "kp", "kd", "dep_z", "trn_z", "dep_tilt", "trn_tilt", "max|dq|", "末态dq(deg)", "dz(m)"))
        rows = {}
        for kp in kps:
            for kd in kds:
                dep = Engine(M["deploy_scene"], "motor", kp=kp, kd=kd, iters=DEPLOY_ITERS, ls=DEPLOY_LS)
                trn = Engine(M["train_scene"], "position", kp=kp, kd=kd, iters=TRAIN_ITERS, ls=TRAIN_LS)
                c = compare(run(dep, acts, sim_s), run(trn, acts, sim_s))
                f = c["final"]
                c["final_dq_deg"] = float(np.degrees(np.abs(np.array(f["a_final_q"])
                                                           - np.array(f["b_final_q"]))).max())
                c["dz"] = f["b_base_z"] - f["a_base_z"]
                rows["kp%g_kd%g" % (kp, kd)] = c
                print("  %-5g %-5g | %-9.4f %-9.4f | %-9.2f %-9.2f | %-11.3e %-11.2f | %+.4f" % (
                    kp, kd, f["a_base_z"], f["b_base_z"], f["a_tilt_deg"], f["b_tilt_deg"],
                    c["max_dq"], c["final_dq_deg"], c["dz"]))
        res[tgt_name] = rows
    out["sensitivity"] = {"program": "3s stand", "targets": res}


def cmd_ablation(args, M, out):
    print("=" * 96)
    print("M5. 参数归因：从训练模型出发逐项改回部署取值，看 沉降平衡 + 短时漂移 怎么变")
    print("=" * 96)
    # (a) 静态可复现量：从训练 reset 条件出发站 5 s，取 4~5 s 的稳定均值
    sim_s = 5.0
    n_ctrl = int(round(sim_s / CTRL_DT))
    acts_stand, _ = action_program("stand", TRAIN_HOME, n_ctrl)
    # (b) 短时漂移：0.5 s 小信号（混沌放大之前）
    acts_small, _ = action_program("small", TRAIN_HOME, int(round(0.5 / CTRL_DT)), seed=3)

    def run_pair(patch=None, it=TRAIN_ITERS, ls=TRAIN_LS):
        dep = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
        trn = Engine(M["train_scene"], "position", iters=it, ls=ls, patch=patch)
        rs_d, rs_t = run(dep, acts_stand, sim_s), run(trn, acts_stand, sim_s)
        sd, st = settled(rs_d, 4.0, 5.0), settled(rs_t, 4.0, 5.0)
        dep2 = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
        trn2 = Engine(M["train_scene"], "position", iters=it, ls=ls, patch=patch)
        rr = compare(run(trn2, acts_small, 0.5), run(dep2, acts_small, 0.5))
        return {"dep_settled": sd, "trn_settled": st,
                "dz": st["z"] - sd["z"], "dtilt": st["tilt"] - sd["tilt"],
                "dq_deg_max": float(np.degrees(np.abs(np.array(st["q"]) - np.array(sd["q"]))).max()),
                "short_max_dq": rr["max_dq"]}

    base = run_pair()
    print("  基线（训练原样 vs 部署原样），q_des = 训练 home，初始 z=%.2f" % TRAIN_HOME_Z)
    print("    部署 沉降 z=%.4f tilt=%.2f° | 训练 沉降 z=%.4f tilt=%.2f° | dz=%+.4f m dtilt=%+.2f°"
          % (base["dep_settled"]["z"], base["dep_settled"]["tilt"],
             base["trn_settled"]["z"], base["trn_settled"]["tilt"], base["dz"], base["dtilt"]))
    print("    沉降关节角最大差 = %.2f°, 0.5 s 小信号漂移 max|dq| = %.3e rad"
          % (base["dq_deg_max"], base["short_max_dq"]))

    def abl_foot_soft_to_hard(m):
        for g in _foot_geoms(m):
            m.geom_condim[g] = 6
            m.geom_solimp[g] = np.array([0.9, 0.95, 0.001, 0.5, 2.0])

    def abl_foot_condim(m):
        for g in _foot_geoms(m):
            m.geom_condim[g] = 6

    def abl_foot_solimp(m):
        for g in _foot_geoms(m):
            m.geom_solimp[g] = np.array([0.9, 0.95, 0.001, 0.5, 2.0])

    def abl_iters(m):
        m.opt.iterations = DEPLOY_ITERS
        m.opt.ls_iterations = DEPLOY_LS

    def abl_margin(m):
        m.geom_margin[:] = 0.001

    def abl_both(m):
        abl_foot_soft_to_hard(m)
        abl_margin(m)
        abl_iters(m)

    ablations = [
        ("足端 软接触(condim3, solimp0.023) -> 部署硬接触", abl_foot_soft_to_hard),
        ("  A. 只改 condim 3 -> 6", abl_foot_condim),
        ("  B. 只改 solimp 0.023 -> 0.001 (仍 condim3)", abl_foot_solimp),
        ("geom margin 0 -> 0.001", abl_margin),
        ("求解器 iterations 2/5 -> 100/50", abl_iters),
        ("足端 + margin + 求解器 全部改回部署", abl_both),
    ]
    rows = []
    hdr = "  %-46s %9s %9s %9s %11s %11s" % ("改动", "dz(m)", "dtilt(°)", "dq(°)", "0.5s|dq|", "相对基线")
    print("\n" + hdr)
    for name, fn in ablations:
        r = run_pair(patch=fn)
        red = 100.0 * (1 - r["short_max_dq"] / max(base["short_max_dq"], 1e-30))
        r["name"] = name
        r["short_reduction_pct"] = red
        rows.append(r)
        print("  %-46s %+9.4f %+9.2f %9.2f %11.3e %10.1f%%" % (
            name, r["dz"], r["dtilt"], r["dq_deg_max"], r["short_max_dq"], red))
    out["ablation"] = {"baseline": base, "rows": rows}


def cmd_pose(args, M, out):
    print("=" * 96)
    print("M6. 姿态契约（keyframe home 不同 = 策略接口差异，不是物理差异）")
    print("=" * 96)
    print("  部署 keyframe home (FR,FL,RR,RL x hip,thigh,calf): %s  z=%.2f" % (
        _fmt(DEPLOY_HOME), DEPLOY_HOME_Z))
    print("  训练 keyframe home                               : %s  z=%.2f" % (
        _fmt(TRAIN_HOME), TRAIN_HOME_Z))
    print("  姿态差 (deg): %s" % _fmt(np.degrees(TRAIN_HOME - DEPLOY_HOME)))
    n_ctrl = int(round(3.0 / CTRL_DT))
    res = {}
    for tag, home in (("deploy_home", DEPLOY_HOME), ("train_home", TRAIN_HOME)):
        acts, _ = action_program("stand", home, n_ctrl)
        d = Engine(M["deploy_scene"], "motor", iters=DEPLOY_ITERS, ls=DEPLOY_LS)
        t = Engine(M["train_scene"], "position", iters=TRAIN_ITERS, ls=TRAIN_LS)
        rd, rt = run(d, acts, 3.0), run(t, acts, 3.0)
        z = lambda r: float(r["base"][-1, 2])
        tg = lambda r: tilt_deg(r["quat"][-1])
        res[tag] = {"deploy_z": z(rd), "deploy_tilt": tg(rd), "train_z": z(rt), "train_tilt": tg(rt),
                    "deploy_min_z": float(rd["base"][:, 2].min()), "train_min_z": float(rt["base"][:, 2].min())}
        print("  q_des = %-11s -> 部署 z=%.4f tilt=%.2f°  |  训练 z=%.4f tilt=%.2f°" % (
            tag, z(rd), tg(rd), z(rt), tg(rt)))
    out["pose"] = {"deploy_home": _arr(DEPLOY_HOME), "train_home": _arr(TRAIN_HOME),
                   "delta_deg": _arr(np.degrees(TRAIN_HOME - DEPLOY_HOME)), "settle": res}


def _foot_geoms(m):
    return [g for g in range(m.ngeom)
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE and m.geom_priority[g] > 0]


def _revert_foot_to_deploy(m):
    for g in _foot_geoms(m):
        m.geom_condim[g] = 6
        m.geom_solimp[g] = np.array([0.9, 0.95, 0.001, 0.5, 2.0])


def _full_revert_to_deploy(m):
    """把训练模型里除执行器写法外的物理差异全部改回部署取值（margin/足端/求解器）。"""
    _revert_foot_to_deploy(m)
    m.geom_margin[:] = 0.001
    m.opt.iterations = DEPLOY_ITERS
    m.opt.ls_iterations = DEPLOY_LS


# ======================================================================================

def _file_id(path):
    import hashlib
    h = hashlib.sha256(open(path, "rb").read()).hexdigest()
    return {"path": path, "sha256": h, "mtime": time.strftime(
        "%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path)))}


def build_models(args, workdir):
    notes = []
    dep_scene = os.path.join(workdir, "deploy_scene.xml")
    make_standalone_scene(args.deploy_body, dep_scene, "deploy")
    notes.append("deploy body = %s" % args.deploy_body)
    train_body = args.train_body or (GETUP_TRAIN_BODY if os.path.isfile(GETUP_TRAIN_BODY)
                                     else MENAGERIE_BODY)
    tr_scene = os.path.join(workdir, "train_scene.xml")
    make_standalone_scene(train_body, tr_scene, "train")
    notes.append("train  body = %s" % train_body)
    for x in (_file_id(args.deploy_body), _file_id(train_body)):
        notes.append("  sha256 %s  %s" % (x["sha256"][:16], x["path"]))
    return {"deploy_body": args.deploy_body, "train_body": train_body,
            "deploy_scene": dep_scene, "train_scene": tr_scene,
            "ids": [_file_id(args.deploy_body), _file_id(train_body)]}, notes


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="all",
                    choices=["harness", "diff", "actuator", "trajectory", "sensitivity",
                             "ablation", "pose", "all"])
    ap.add_argument("--deploy-body", default=PROJECT_DEPLOY_BODY)
    ap.add_argument("--train-body", default=None)
    ap.add_argument("--workdir", default=None)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    t0 = time.time()
    workdir = args.workdir or tempfile.mkdtemp(prefix="go2_parity_")
    os.makedirs(workdir, exist_ok=True)
    out = {"workdir": workdir, "sim_dt": SIM_DT, "ctrl_dt": CTRL_DT,
           "deploy_iters": [DEPLOY_ITERS, DEPLOY_LS], "train_iters": [TRAIN_ITERS, TRAIN_LS]}
    M, notes = build_models(args, workdir)
    out["models"] = M
    print("工作目录:", workdir)
    for n in notes:
        print("  " + n)
    print()

    todo = {"harness": cmd_harness, "diff": cmd_diff, "actuator": cmd_actuator,
            "trajectory": cmd_trajectory, "sensitivity": cmd_sensitivity,
            "ablation": cmd_ablation, "pose": cmd_pose}
    order = ["harness", "diff", "actuator", "trajectory", "sensitivity", "ablation",
             "pose"] if args.mode == "all" else [args.mode]
    for k in order:
        todo[k](args, M, out)
        print()
    out["elapsed_s"] = time.time() - t0
    print("[parity] 用时 %.1f s" % out["elapsed_s"])
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2, default=float)
        print("[parity] JSON ->", args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
