#!/usr/bin/env python
"""stuck_forensics.py — 部署侧 CPU MuJoCo 的“脚卡死/原地卡住”力学取证 (500Hz 物理 / 50Hz 策略)。

严格判据 (默认): >= --window 1.2s 内基座平面位移 < --disp-eps 0.02m, 且 cmd=0.4、
tilt<--tilt-max 30°、未摔倒、已过 --warmup。恢复判据: 最近 1.0s 位移 > 0.15m
(避免“抖一下”被当成恢复)。事件窗口 dump 接触力学证据 (100Hz), 聚合后由
analyze_stuck.py 分类; 本脚本也给出初判 a/b/c/d + 子类。

关键区分 (本脚本聚合里都有原始量, 分析脚本可重分类):
  * 被卡的那个足是否真的“动不了” (foot_xy_disp < 0.02) —— 否则是策略原地抽搐(d)
  * (a) a_vgroove   : 球足被相邻瓦片(tc_*) V 槽/窄缝托住 (>=2 tc 接触、对立法向、球心沉到名义面下)
  * (b) b_wall_side : 球足楔在台阶立面(bx_*)/瓦片竖侧面; 或小腿/腹部顶在立面上 (记 sub)
  * (c) c_penetration: 被卡的足持续穿透 (>4mm) 或球心低于所接触 box 顶面
  * (d) d_other     : 无上述“被卡足”证据 (= 策略放弃/原地抽搐/高心/其它)
另外打 init 标记: 事件 onset 早于 --init-s, 典型是出生嵌进地形 (uniform spawn 常见)。

每 500Hz 物理步记录: 基座位姿/速度、关节 q/dq、上一拍策略动作、施加力矩、四足球心世界
坐标、足-地形接触 (geom 名/面/穿透 dist/世界法向/法向力/切向力/接触点/box 顶面高度)、
足球心下方 hfield 名义高度 (hfield_data 双线性插值)、非足部位(腹/小腿/髋)与地形接触。

用法:
  python scripts/stuck_forensics.py --scene hyb2 --spawn ring --seeds 0:16 --duration 15 \
      --out /tmp/t1_wedge/hyb2_ring.json
  # --spawn ring(观察器/外圈出生, 同 /tmp/go2_diag.py)
  #         uniform(训练评测口径, 同 /tmp/eval_deploy_terrain.py) / origin
  # --phase seed(默认, 每 seed 固定相位, 可复现) / fixed / seq(reset 序列, 不可重跑复现)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import mujoco  # noqa: E402

from signal_io.config import load_config, robot_spec_of  # noqa: E402
from signal_io.msg import LowCmd  # noqa: E402
from sim_env.robot import Go2Sim  # noqa: E402
from ros2_controller import run_controller as rc  # noqa: E402
from ros2_controller.health import HealthMonitor  # noqa: E402
from ros2_controller.safety import SafetySupervisor  # noqa: E402

FOOT_NAMES = ("FL", "FR", "RL", "RR")
FOOT_R = 0.022
FACE_NAME = {0: "+x", 1: "-x", 2: "+y", 3: "-y", 4: "+z_top", 5: "-z_bottom"}
TWOPI = 2.0 * np.pi


def _name(m, objtype, gid):
    n = mujoco.mj_id2name(m, objtype, int(gid))
    return n or ("<g%d>" % int(gid))


def _quat_rpy(q):
    w, x, y, z = (float(v) for v in q)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


class Terrain(object):
    def __init__(self, sim):
        self.sim = sim
        self.m = sim.model
        self.d = sim.data
        m = self.m
        self.base_body = int(sim.base_body)
        self.body_names = {}
        for b in range(int(m.nbody)):
            self.body_names[b] = _name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        self.foot_gids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_NAMES]
        self.foot_set = set(self.foot_gids)
        self.hf = None
        for g in range(int(m.ngeom)):
            if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_HFIELD):
                nr, nc = int(m.hfield_nrow[g]), int(m.hfield_ncol[g])
                flat = np.asarray(m.hfield_data, dtype=np.float64).reshape(-1)
                hd = flat[:nr * nc].reshape(nr, nc)
                self.hf = dict(gid=int(g), nrow=nr, ncol=nc, data=hd,
                               sx=float(m.hfield_size[g][0]), sy=float(m.hfield_size[g][1]),
                               elev=float(m.hfield_size[g][2]),
                               ox=float(self.d.geom_xpos[g][0]), oy=float(self.d.geom_xpos[g][1]))
                break
        robot_bodies = set()
        for b in range(int(m.nbody)):
            cur = int(b)
            while cur != 0:
                if cur == self.base_body:
                    robot_bodies.add(b)
                    break
                cur = int(m.body_parentid[cur])
        robot_groups = {int(m.geom_group[g]) for g in range(int(m.ngeom))
                        if int(m.geom_bodyid[g]) in robot_bodies}
        all_groups = {int(m.geom_group[g]) for g in range(int(m.ngeom))}
        self.env_mask = np.zeros(max(6, max(all_groups) + 1), dtype=np.uint8)
        for g in (all_groups - robot_groups):
            self.env_mask[g] = 1
        self.gid = np.zeros(1, dtype=np.int32)
        self.box = {}
        for g in range(int(m.ngeom)):
            if int(m.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_BOX):
                continue
            R = np.asarray(self.d.geom_xmat[g], dtype=np.float64).reshape(3, 3)
            p = np.asarray(self.d.geom_xpos[g], dtype=np.float64)
            sz = np.asarray(m.geom_size[g], dtype=np.float64)
            self.box[g] = dict(name=_name(m, mujoco.mjtObj.mjOBJ_GEOM, g), pos=p.copy(),
                               R=R.copy(), size=sz.copy(), n=R @ np.array([0.0, 0.0, 1.0]),
                               topc=p + R @ np.array([0.0, 0.0, float(sz[2])]))
        # 机器人非足碰撞 geom: (gid, body_name, desc)
        self.body_gids = {}
        for g in range(int(m.ngeom)):
            if int(m.geom_bodyid[g]) in robot_bodies and g not in self.foot_set \
                    and int(m.geom_group[g]) == 3:
                desc = "%s:type%d:%s" % (self.body_names[int(m.geom_bodyid[g])],
                                         int(m.geom_type[g]),
                                         "x".join("%.3f" % v for v in m.geom_size[g] if v > 0))
                self.body_gids[g] = desc
        self.all_box_gids = list(self.box.keys())

    def hfield_z(self, x, y):
        hf = self.hf
        if hf is None:
            return 0.0
        u = (x - hf["ox"] + hf["sx"]) / (2.0 * hf["sx"])
        v = (y - hf["oy"] + hf["sy"]) / (2.0 * hf["sy"])
        if u < 0.0 or u > 1.0 or v < 0.0 or v > 1.0:
            return 0.0
        ci = u * (hf["ncol"] - 1); ri = v * (hf["nrow"] - 1)
        j0 = int(np.floor(ci)); i0 = int(np.floor(ri))
        j1 = min(j0 + 1, hf["ncol"] - 1); i1 = min(i0 + 1, hf["nrow"] - 1)
        fj = ci - j0; fi = ri - i0
        h = (hf["data"][i0, j0] * (1 - fi) * (1 - fj) + hf["data"][i1, j0] * fi * (1 - fj)
             + hf["data"][i0, j1] * (1 - fi) * fj + hf["data"][i1, j1] * fi * fj)
        return float(h * hf["elev"])

    def ground_env(self, x, y):
        d = mujoco.mj_ray(self.m, self.d, np.array([x, y, 3.0]), np.array([0.0, 0.0, -1.0]),
                          self.env_mask, 1, -1, self.gid)
        if d is None:
            return 0.0
        return max(0.0, 3.0 - float(d))

    def box_top_z(self, g, x, y):
        b = self.box.get(int(g))
        if b is None:
            return None
        n = b["n"]
        if abs(n[2]) < 1e-6:
            return None
        tc = b["topc"]
        return float(tc[2] - (n[0] * (x - tc[0]) + n[1] * (y - tc[1])) / n[2])

    def box_face(self, g, p):
        b = self.box.get(int(g))
        if b is None:
            return -1, 0.0, False
        q = b["R"].T @ (np.asarray(p, dtype=np.float64) - b["pos"])
        ratio = np.abs(q) / np.maximum(b["size"], 1e-9)
        k = int(np.argmax(ratio))
        face = 2 * k + (0 if q[k] >= 0 else 1)
        return face, float(ratio[k]), bool(np.sort(ratio)[-2] > 0.9 * ratio[k])

    def box_sdf(self, g, p):
        b = self.box.get(int(g))
        if b is None:
            return 1.0
        q = b["R"].T @ (np.asarray(p, dtype=np.float64) - b["pos"])
        d = np.abs(q) - b["size"]
        return float(np.linalg.norm(np.maximum(d, 0.0)) + min(float(np.max(d)), 0.0))

    def spawn_penetration(self, feet):
        worst = 0.0
        for p in feet:
            for g in self.all_box_gids:
                worst = max(worst, FOOT_R - self.box_sdf(g, p))
        return worst


class Forensics(object):
    def __init__(self, scene, cfg, cmd, profile):
        self.scene = scene
        self.cfg = cfg
        self.cmd = np.asarray(cmd, dtype=np.float64)
        self.profile = profile
        spec = robot_spec_of(cfg)
        self.sim = Go2Sim(scene, spec=spec, sim_dt=cfg["sim_dt"],
                          foot_soft_contact=bool(getattr(spec, "foot_soft_contact", False)))
        self.ctrl = rc.build_controller("policy", cfg, self.sim, cli_cmd=list(cmd), linvel="oracle")
        self.health = HealthMonitor(cfg)
        limits = np.asarray(self.sim.torque_limits, dtype=np.float64)
        self.sup = SafetySupervisor(self.sim.stand_pose, limits=limits,
                                    ramp_rate=cfg.get("ramp_rate", 200.0))
        self.terr = Terrain(self.sim)
        self.dec = max(1, int(round(cfg["control_dt"] / cfg["sim_dt"])))
        self.dt = float(cfg["sim_dt"])
        self.f6 = np.zeros(6)

    def _record(self, t, tau, hs_level):
        sim, d, m, terr = self.sim, self.sim.data, self.sim.model, self.terr
        q = np.asarray(d.qpos, dtype=np.float64)
        v = np.asarray(d.qvel, dtype=np.float64)
        roll, pitch = _quat_rpy(q[3:7])
        tilt = max(abs(np.degrees(roll)), abs(np.degrees(pitch)))
        feet = np.asarray(d.geom_xpos[terr.foot_gids], dtype=np.float64).copy()
        contacts = []
        body_contacts = []
        fq = [0.0, 0.0, 0.0, 0.0]
        for i in range(int(d.ncon)):
            con = d.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            isf1, isf2 = g1 in terr.foot_set, g2 in terr.foot_set
            if not isf1 and not isf2:
                rb = g1 if g1 in terr.body_gids else (g2 if g2 in terr.body_gids else -1)
                if rb >= 0:
                    og = g2 if rb == g1 else g1
                    gname = _name(m, mujoco.mjtObj.mjOBJ_GEOM, og)
                    if og in terr.box:
                        face, _r, _e = terr.box_face(og, con.pos)
                    else:
                        face = -1
                    body_contacts.append([terr.body_gids[rb], gname, face,
                                          round(float(con.dist), 4)])
                continue
            fi = terr.foot_gids.index(g1 if isf1 else g2)
            og = g2 if isf1 else g1
            normal = np.asarray(con.frame, dtype=np.float64).reshape(3, 3)[0].copy()
            if isf1:
                normal = -normal
            mujoco.mj_contactForce(m, d, i, self.f6)
            fn = float(self.f6[0]); ft = float(np.linalg.norm(self.f6[1:3]))
            fq[fi] += fn
            gname = _name(m, mujoco.mjtObj.mjOBJ_GEOM, og)
            if og in terr.box:
                face, _ratio, edge = terr.box_face(og, con.pos)
                tz = terr.box_top_z(og, feet[fi, 0], feet[fi, 1])
            else:
                face, edge, tz = -1, False, None
            contacts.append([fi, gname, face, round(float(con.dist), 4),
                             round(float(normal[0]), 3), round(float(normal[1]), 3),
                             round(float(normal[2]), 3), round(fn, 2), round(ft, 2),
                             round(float(con.pos[0]), 4), round(float(con.pos[1]), 4),
                             round(float(con.pos[2]), 4),
                             None if tz is None else round(tz, 4), 1 if edge else 0])
        hf = [round(terr.hfield_z(float(feet[k, 0]), float(feet[k, 1])), 4) for k in range(4)]
        la = self.ctrl.last_action
        return dict(
            t=round(float(t), 4), xyz=[round(float(q[0]), 4), round(float(q[1]), 4), round(float(q[2]), 4)],
            quat=[round(float(x), 5) for x in q[3:7]],
            q=[round(float(x), 5) for x in q], qv=[round(float(x), 4) for x in v],
            rpy=[round(float(np.degrees(roll)), 2), round(float(np.degrees(pitch)), 2)],
            tilt=round(tilt, 2), vxy=[round(float(v[0]), 3), round(float(v[1]), 3)],
            feet=[[round(float(x), 4) for x in feet[i]] for i in range(4)],
            fq=[round(x, 2) for x in fq], hf=hf,
            la=[round(float(x), 3) for x in (np.zeros(12) if la is None else la)],
            tau=[round(float(x), 2) for x in tau], level=int(hs_level),
            c=contacts, bc=body_contacts,
        )

    def run_seed(self, seed, spawn, duration, opts):
        sim, ctrl, health, sup = self.sim, self.ctrl, self.health, self.sup
        sim.reset()
        x, y, yaw = spawn
        g = self.terr.ground_env(x, y)
        sim.data.qpos[0] = x
        sim.data.qpos[1] = y
        sim.data.qpos[2] = 0.32 + g + 0.01
        sim.data.qpos[3:7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        sim.data.qvel[:] = 0.0
        mujoco.mj_forward(sim.model, sim.data)
        sim.tick = 0
        sim.time = 0.0
        feet0 = np.asarray(sim.data.geom_xpos[self.terr.foot_gids], dtype=np.float64).copy()
        spawn_pen = self.terr.spawn_penetration(feet0)
        spawn_sink = float(np.max([self.terr.hfield_z(float(p[0]), float(p[1])) + FOOT_R - p[2]
                                   for p in feet0]))
        ctrl.command[:] = self.cmd
        ctrl.reset()
        phase = None
        if opts["phase"] == "seed":
            phase = float(TWOPI * ((seed * 0.6180339887498949 + 0.13) % 1.0))
        elif opts["phase"] == "fixed":
            phase = 0.0
        if phase is not None and hasattr(ctrl.obs, "set_phase"):
            ctrl.obs.set_phase(phase)
        ctrl.obs._cmd = self.cmd.copy()
        health.reset(0.0)
        sup.reset()
        st = sim.read_state()
        sim._last_state = st

        dt = self.dt
        n_steps = int(round(duration / dt))
        W = max(1, int(round(opts["window"] / dt)))
        Wmove = max(1, int(round(1.0 / dt)))
        warm = int(round(opts["warmup"] / dt))
        hold_steps = int(round(opts["max_hold"] / dt))
        merge_steps = int(round(opts["merge_gap"] / dt))
        buf_len = int(round(opts["buffer_s"] / dt))
        pos_hist = np.zeros((n_steps, 2))
        buf = []
        cmdv = LowCmd().zero()

        fell = False; t_fall = None; tilt_max = 0.0
        vsum = 0.0; nv = 0; slow_s = 0.0; slow_events = 0; in_slow = False
        events = []; state = None; suppressed = False
        ncon_sum = 0; last_event_end = 0
        # 全run机制统计: 足球被两个近竖直面互顶(缝/楔)夹住
        pinch = [dict(on=False, s=0.0, n=0) for _ in range(4)]
        max_pen = [0.0] * 4
        max_fn = [0.0] * 4
        traj = []
        t_start = time.perf_counter()
        for k in range(n_steps):
            st = sim.read_state()
            sim._last_state = st
            t = float(sim.time)
            hs = health.check(st, t)
            if k % self.dec == 0:
                cmdv = ctrl.update(st, t)
            safe = sup.process(cmdv, hs, t, dt)
            tau = sim.write_cmd(safe)
            sim.step(1)
            rec = self._record(t + dt, tau, hs.level)
            rec["k"] = k
            buf.append(rec)
            if len(buf) > buf_len:
                buf.pop(0)
            xyz = rec["xyz"]
            pos_hist[k, 0] = xyz[0]; pos_hist[k, 1] = xyz[1]
            tilt = rec["tilt"]
            tilt_max = max(tilt_max, tilt)
            vn = (rec["vxy"][0] ** 2 + rec["vxy"][1] ** 2) ** 0.5
            vsum += vn; nv += 1; ncon_sum += len(rec["c"])
            if not fell and (tilt > 60.0 or xyz[2] < 0.15):
                fell = True; t_fall = round(float(t + dt), 3)
            # 缝/楔夹住检测: 同一足两个接触, 法向近水平(|n_xy|>0.7)且互相对立(dot<-0.6),
            # 且两侧都吃上力(>30N) -> 球被夹在缝里
            cs_by_foot = {i: [] for i in range(4)}
            for c in rec["c"]:
                cs_by_foot[int(c[0])].append(c)
                fd = int(c[0])
                if -c[3] > max_pen[fd]:
                    max_pen[fd] = -c[3]
                if c[7] > max_fn[fd]:
                    max_fn[fd] = c[7]
            for fi in range(4):
                cs = cs_by_foot[fi]
                hit = False
                for ii in range(len(cs)):
                    for jj in range(ii + 1, len(cs)):
                        a, b = cs[ii], cs[jj]
                        h1 = np.hypot(a[4], a[5]); h2 = np.hypot(b[4], b[5])
                        if h1 > 0.7 and h2 > 0.7 and (a[4] * b[4] + a[5] * b[5]) / (h1 * h2) < -0.6                                 and a[7] > 30.0 and b[7] > 30.0:
                            hit = True
                if hit:
                    pinch[fi]["s"] += dt
                    if not pinch[fi]["on"]:
                        pinch[fi]["n"] += 1
                        pinch[fi]["on"] = True
                else:
                    pinch[fi]["on"] = False
            if len(traj) == 0 or (t + dt) - traj[-1][0] >= 0.5:
                traj.append([round(float(t + dt), 2), round(float(xyz[0]), 3),
                             round(float(xyz[1]), 3), round(float(xyz[2]), 3),
                             round(float(tilt), 1), 0.0])
            if vn < 0.05 and tilt < 25.0:
                slow_s += dt
                if not in_slow:
                    slow_events += 1; in_slow = True
            else:
                in_slow = False
            if fell or k < W or k < warm:
                continue
            disp_w = float(np.linalg.norm(pos_hist[k] - pos_hist[k - W]))
            disp_move = (float(np.linalg.norm(pos_hist[k] - pos_hist[k - Wmove]))
                         if k >= Wmove else 1.0)
            active = (disp_w < opts["disp_eps"] and tilt < opts["tilt_max"] and xyz[2] > 0.15)
            moving = disp_move > 0.15
            if suppressed:
                if moving:
                    suppressed = False
                    last_event_end = k
                continue
            if state is None:
                if active:
                    start_step = max(k - W, last_event_end, warm)
                    state = dict(start_step=start_step, trigger_step=k, last_active=k,
                                 still_stuck=False,
                                 start_t=round(float((start_step + 1) * dt), 3))
                    events.append(state)
                continue
            if active:
                state["last_active"] = k
                if k - state["trigger_step"] > hold_steps:
                    state["still_stuck"] = True
                    self._finalize(state, buf, pos_hist, dt, W, opts, n_steps)
                    last_event_end = k
                    state = None; suppressed = True
                continue
            if k - state["last_active"] >= merge_steps or moving:
                self._finalize(state, buf, pos_hist, dt, W, opts, n_steps)
                last_event_end = state["last_active"]
                state = None
                if not moving:
                    suppressed = True
        if state is not None:
            self._finalize(state, buf, pos_hist, dt, W, opts, n_steps)
        for row in traj:
            row[5] = round(float(np.linalg.norm(np.array(row[1:3]) - np.array([x, y]))), 3)
        wall = time.perf_counter() - t_start
        end_xyz = [round(float(sim.data.qpos[0]), 3), round(float(sim.data.qpos[1]), 3)]
        evs = [e for e in events if "class" in e]
        return dict(
            seed=int(seed), spawn=[round(float(x), 3), round(float(y), 3), round(float(yaw), 3)],
            phase=None if phase is None else round(float(phase), 4),
            terrain=opts["terrain"], scene=os.path.basename(self.scene), profile=self.profile,
            cmd=[float(c) for c in self.cmd], duration=duration, dt=dt,
            spawn_pen=round(float(spawn_pen), 4), spawn_sink_hf=round(spawn_sink, 4),
            fell=bool(fell), t_fall=t_fall, tilt_max=round(float(tilt_max), 1),
            v_mean=round(float(vsum / max(1, nv)), 3),
            ncon_mean=round(float(ncon_sum / max(1, nv)), 2),
            final_xy=end_xyz,
            dist_xy=round(float(np.linalg.norm(np.array(end_xyz) - np.array([x, y]))), 3),
            slow_s=round(float(slow_s), 2), slow_events=int(slow_events),
            n_events=len(evs), stuck_s=round(float(sum(e["duration"] for e in evs)), 2),
            events=evs, wall_s=round(float(wall), 2), traj=traj,
            pinch_s=[round(pp["s"], 2) for pp in pinch],
            pinch_n=[int(pp["n"]) for pp in pinch],
            pinch_s_total=round(float(sum(pp["s"] for pp in pinch)), 2),
            pinch_n_total=int(sum(pp["n"] for pp in pinch)),
            max_pen=[round(v, 4) for v in max_pen],
            max_fn=[round(v, 1) for v in max_fn],
        )

    def _finalize(self, state, buf, pos_hist, dt, W, opts, n_steps):
        k = state["last_active"]
        start_step = max(0, state["start_step"])
        start_t = state["start_t"]
        state["duration"] = round(float((k - start_step) * dt), 3)
        state["end_t"] = round(float(start_t + state["duration"]), 3)
        k2 = min(n_steps - 1, k + int(round(1.0 / dt)))
        state["moved_after"] = (round(float(np.linalg.norm(pos_hist[k2] - pos_hist[k])), 3)
                                if k2 > k else None)
        t_lo = start_t - 0.5
        t_hi = start_t + min(1.5, state["duration"] + 0.3)
        win = [r for r in buf if t_lo <= r["t"] <= t_hi]
        if not win:
            win = [r for r in buf if abs(r["t"] - start_t) < 0.5]
        anchor = min(win, key=lambda r: abs(r["t"] - (start_t + 0.2))) if win else None
        state["pose"] = None if anchor is None else dict(
            t=anchor["t"], qpos=anchor["q"], qvel=anchor["qv"],
            xyz=anchor["xyz"], quat=anchor["quat"], feet=anchor["feet"])
        ds = win[::5] if len(win) > 260 else win
        state["window"] = ds
        state["agg"] = aggregate(state, ds, dt)
        state["class"] = classify(state["agg"], opts, state)


# ---------------------------------------------------------------- 聚合
def aggregate(state, recs, dt):
    n = max(1, len(recs))
    per = [dict(f_tc=0, f_two=0, f_bx=0, f_side=0, f_top=0, f_opp=0, f_pen4=0,
                f_below=0, sink=[], fq=[], nc=[], xyz=[], pairs={}) for _ in range(4)]
    base_xyz = []; tilt_max = 0.0; levels = set(); ncon = []
    body_pairs = {}; n_body = 0
    for r in recs:
        base_xyz.append(r["xyz"]); tilt_max = max(tilt_max, r["tilt"])
        levels.add(r["level"]); ncon.append(len(r["c"]))
        if r["bc"]:
            n_body += 1
        for b in r["bc"]:
            key = (b[0], b[1], FACE_NAME.get(b[2], "?"))
            e = body_pairs.setdefault(key, dict(n=0, dist_sum=0.0, dist_min=9.9))
            e["n"] += 1; e["dist_sum"] += b[3]; e["dist_min"] = min(e["dist_min"], b[3])
        feet = r["feet"]; hf = r["hf"]
        byfoot = {i: [] for i in range(4)}
        for c in r["c"]:
            byfoot[int(c[0])].append(c)
        for i in range(4):
            p = per[i]; cs = byfoot[i]
            p["xyz"].append(feet[i]); p["fq"].append(r["fq"][i]); p["nc"].append(len(cs))
            p["sink"].append(hf[i] + FOOT_R - feet[i][2])
            n_tc = sum(1 for c in cs if c[1].startswith("tc_"))
            if n_tc >= 1: p["f_tc"] += 1
            if n_tc >= 2: p["f_two"] += 1
            if any(c[1].startswith("bx_") for c in cs): p["f_bx"] += 1
            has_side = has_opp = has_pen = below = has_top = False
            for c in cs:
                gname, face, dist, nx, ny, fn = c[1], c[2], c[3], c[4], c[5], c[7]
                if face in (0, 1, 2, 3) or np.hypot(nx, ny) > 0.4:
                    has_side = True
                if face == 4:
                    has_top = True
                if -dist > 0.004:
                    has_pen = True
                tz = c[12]
                if tz is not None and feet[i][2] < tz:
                    below = True
                for c2 in cs:
                    if c2 is c:
                        continue
                    h1 = np.hypot(c[4], c[5]); h2 = np.hypot(c2[4], c2[5])
                    if h1 > 0.15 and h2 > 0.15:
                        dxy = (c[4] * c2[4] + c[5] * c2[5]) / (h1 * h2)
                        if dxy < -0.3 and (c[7] > 2.0 or c2[7] > 2.0):
                            has_opp = True
                key = (gname, FACE_NAME.get(face, "?"))
                e = p["pairs"].setdefault(key, dict(n=0, ds=0.0, dmin=9.9, fnmax=0.0,
                                                    ns=[0.0, 0.0, 0.0]))
                e["n"] += 1; e["ds"] += dist; e["dmin"] = min(e["dmin"], dist)
                e["fnmax"] = max(e["fnmax"], fn)
                e["ns"][0] += nx; e["ns"][1] += ny; e["ns"][2] += c[6]
            if has_side: p["f_side"] += 1
            if has_top: p["f_top"] += 1
            if has_opp: p["f_opp"] += 1
            if has_pen: p["f_pen4"] += 1
            if below: p["f_below"] += 1
    axyz = np.asarray(base_xyz) if base_xyz else np.zeros((1, 3))
    # 运动性: 基座位移 / 足最大位移 / 足最大速度 (100Hz 记录)
    foot_speed_max = 0.0
    if len(recs) >= 2:
        for i in range(4):
            fz = np.array([r["feet"][i][:2] for r in recs])
            foot_speed_max = max(foot_speed_max,
                                 float(np.max(np.linalg.norm(np.diff(fz, axis=0), axis=1)))
                                 / max(len(recs) - 1, 1) / max(dt, 1e-9))
    dq_abs = 0.0
    if recs:
        dq_abs = float(np.mean([np.mean(np.abs(r["qv"][6:18])) for r in recs]))
    out = dict(
        n_rec=int(n), base_xy_disp=round(float(np.linalg.norm(axyz[-1, :2] - axyz[0, :2])), 4),
        base_z_min=round(float(axyz[:, 2].min()), 3),
        tilt_max=round(float(tilt_max), 1),
        ncon_mean=round(float(np.mean(ncon)), 2) if ncon else None,
        health_levels=sorted(levels),
        max_foot_speed=round(foot_speed_max, 3), mean_abs_dq=round(dq_abs, 3),
        body_contact_frac=round(n_body / n, 3), body_pairs=[], per_foot=[])
    for (bdesc, gname, fname), e in sorted(body_pairs.items(),
                                           key=lambda kv: kv[1]["n"], reverse=True)[:8]:
        out["body_pairs"].append(dict(robot=bdesc, geom=gname, face=fname, n=e["n"],
                                      frac=round(e["n"] / n, 3),
                                      dist_mean=round(e["dist_sum"] / max(1, e["n"]), 4),
                                      dist_min=round(e["dist_min"], 4)))
    for i in range(4):
        p = per[i]
        sh = np.asarray(p["sink"]) if p["sink"] else np.zeros(1)
        fxyz = np.asarray(p["xyz"]) if p["xyz"] else np.zeros((1, 3))
        pairs = []
        for (gname, fname), e in sorted(p["pairs"].items(),
                                        key=lambda kv: (kv[1]["fnmax"], kv[1]["n"]),
                                        reverse=True)[:10]:
            pairs.append(dict(geom=gname, face=fname, n=e["n"],
                              dist_mean=round(e["ds"] / max(1, e["n"]), 4),
                              dist_min=round(e["dmin"], 4), fn_max=round(e["fnmax"], 1),
                              n_mean=[round(e["ns"][0] / max(1, e["n"]), 2),
                                      round(e["ns"][1] / max(1, e["n"]), 2),
                                      round(e["ns"][2] / max(1, e["n"]), 2)]))
        out["per_foot"].append(dict(
            foot=FOOT_NAMES[i], n_contacts=round(float(np.mean(p["nc"])), 2),
            frac_tc=round(p["f_tc"] / n, 3), frac_two_tc=round(p["f_two"] / n, 3),
            frac_bx=round(p["f_bx"] / n, 3), frac_side=round(p["f_side"] / n, 3),
            frac_top=round(p["f_top"] / n, 3), frac_opp=round(p["f_opp"] / n, 3),
            frac_pen4=round(p["f_pen4"] / n, 3),
            frac_below_plane=round(p["f_below"] / n, 3),
            sink_hf_med=round(float(np.median(sh)), 4),
            sink_hf_p95=round(float(np.percentile(sh, 95)), 4),
            fq_mean=round(float(np.mean(p["fq"])), 1),
            xy_disp=round(float(np.max(np.linalg.norm(fxyz[:, :2] - fxyz[0, :2], axis=1))), 4),
            z_range=round(float(np.ptp(fxyz[:, 2])), 4),
            pairs=pairs))
    return out


def classify(agg, opts, state):
    pf = agg["per_foot"]
    static = {p["foot"]: p["xy_disp"] < 0.02 for p in pf}
    pen = [p["foot"] for p in pf if static[p["foot"]] and
           (p["frac_pen4"] > 0.05 or p["frac_below_plane"] > 0.10)]
    vg = []
    for p in pf:
        if not static[p["foot"]] or p["foot"] in pen:
            continue
        if p["frac_two_tc"] > 0.15 and (p["frac_opp"] > 0.03 or p["sink_hf_med"] > 0.002):
            vg.append(p["foot"])
    side = [p["foot"] for p in pf if static[p["foot"]] and p["frac_side"] > 0.2
            and p["foot"] not in pen and p["foot"] not in vg]
    body_side = [b for b in agg["body_pairs"]
                 if b["face"] in ("+x", "-x", "+y", "-y") and b["frac"] > 0.1]
    if pen:
        cls, sub = "c_penetration", "foot"
    elif vg:
        cls, sub = "a_vgroove", "foot"
    elif side or body_side:
        cls, sub = "b_wall_side", ("foot_side" if side else "body/calf_side")
    else:
        cls, sub = "d_other", ("thrash" if agg["max_foot_speed"] > 0.08 else "frozen")

    def score(p):
        return (p["frac_pen4"] * 5 + p["frac_below_plane"] * 4 + p["frac_two_tc"] * 3
                + p["frac_opp"] * 3 + p["frac_side"] + p["fq_mean"] * 0.01)
    blame = max(pf, key=score)["foot"]
    init = bool(state["start_t"] < opts.get("init_s", 3.0))
    return dict(cls=cls, sub=sub, blame_foot=blame, init=init,
                pen_feet=pen, vg_feet=vg, side_feet=side,
                body_side=[b["robot"] + "|" + b["geom"] + "|" + b["face"] for b in body_side],
                base_xy_disp=agg["base_xy_disp"], tilt_max=agg["tilt_max"],
                max_foot_speed=agg["max_foot_speed"],
                static_feet=[f for f, s in static.items() if s])


# ---------------------------------------------------------------- spawn
def _load_meta(terrain):
    man = json.load(open(os.path.join(_ROOT, "sim_env/models/go2/terrains.json")))
    for e in man["terrains"]:
        if e["name"] == terrain:
            return e
    raise KeyError("未知地形 %r" % terrain)


def make_spawns(meta, seeds, mode):
    out = {}
    fh = float(meta.get("field_half") or 0.0)
    hh = float(meta.get("hfield_half") or fh)
    kind = meta.get("kind", "flat")
    if mode == "uniform":
        rng = np.random.default_rng(0)
        seq = {}
        for s in range(max(seeds) + 1):
            seq[s] = (float(rng.uniform(-fh, fh)), float(rng.uniform(-fh, fh)),
                      float(rng.uniform(-np.pi, np.pi)))
        for s in seeds:
            out[s] = seq[s]
    elif mode == "ring":
        for s in seeds:
            rng = np.random.default_rng(s)
            if kind == "flat" or (fh <= 0 and hh <= 0):
                out[s] = (0.0, 0.0, 0.0)
                continue
            if kind == "hfield":
                r = float(rng.uniform(0.0, 0.6)); th = float(rng.uniform(-np.pi, np.pi))
                yaw = float(rng.uniform(-np.pi, np.pi))
            else:
                r_in = min(fh + 0.4, max(0.0, hh - 0.4))
                r_out = max(r_in + 0.2, min(fh + 1.3, hh - 0.2))
                r = float(rng.uniform(r_in, r_out)); th = float(rng.uniform(-np.pi, np.pi))
                yaw = float(th + np.pi + rng.uniform(-0.5, 0.5))
            out[s] = (r * np.cos(th), r * np.sin(th), yaw)
    elif mode == "origin":
        for s in seeds:
            out[s] = (0.0, 0.0, 0.0)
    else:
        raise ValueError(mode)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="hyb2")
    ap.add_argument("--profile", default="go2_v25b")
    ap.add_argument("--seeds", default="0:16")
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--cmd", type=float, nargs=3, default=[0.4, 0.0, 0.0])
    ap.add_argument("--spawn", default="ring", choices=["ring", "uniform", "origin"])
    ap.add_argument("--phase", default="seed", choices=["seed", "fixed", "seq"])
    ap.add_argument("--window", type=float, default=1.2)
    ap.add_argument("--disp-eps", type=float, default=0.02)
    ap.add_argument("--tilt-max", type=float, default=30.0)
    ap.add_argument("--warmup", type=float, default=2.5)
    ap.add_argument("--init-s", type=float, default=3.0)
    ap.add_argument("--max-hold", type=float, default=2.5)
    ap.add_argument("--merge-gap", type=float, default=0.4)
    ap.add_argument("--buffer-s", type=float, default=4.0)
    ap.add_argument("--scan", default="raycast")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    opts = dict(terrain=a.scene, window=a.window, disp_eps=a.disp_eps, tilt_max=a.tilt_max,
                warmup=a.warmup, max_hold=a.max_hold, merge_gap=a.merge_gap,
                buffer_s=a.buffer_s, init_s=a.init_s, phase=a.phase)
    cfg = load_config(os.path.join(_ROOT, "config", "go2_sim.json"),
                      overrides={"robot": "go2", "policy_profile": a.profile})
    cfg["policy"]["height_scan_source"] = a.scan
    if a.scene.endswith(".xml"):
        scene = a.scene if os.path.isabs(a.scene) else os.path.join(_ROOT, a.scene)
        meta = dict(name=os.path.basename(a.scene), kind="flat", field_half=0.0, hfield_half=0.0)
    else:
        meta = _load_meta(a.scene)
        scene = meta["scene"]
        if not os.path.isabs(scene):
            scene = os.path.join(_ROOT, scene)
        opts["terrain"] = meta["name"]
    seeds = (list(range(int(a.seeds.split(":")[0]), int(a.seeds.split(":")[1])))
             if ":" in a.seeds else [int(s) for s in a.seeds.split(",") if s.strip()])
    spawns = make_spawns(meta, seeds, a.spawn)
    print("[forensics] terrain=%s spawn=%s phase=%s seeds=%s dur=%.1f window=%.2fs "
          "disp<%.3f tilt<%.0f warmup=%.1fs" %
          (opts["terrain"], a.spawn, a.phase, seeds, a.duration, a.window, a.disp_eps,
           a.tilt_max, a.warmup), flush=True)
    runs = []
    t0 = time.perf_counter()
    for s in seeds:
        # 每个 seed 重建 sim/controller: 同一进程里复用会残留 obs 滤波器/高度图衰减状态,
        # 实测同 seed 同相位结果不可复现 (dist 4.08 vs 3.25); 重建后跨进程逐位一致。
        f = Forensics(scene, cfg, a.cmd, a.profile)
        r = f.run_seed(s, spawns[s], a.duration, opts)
        runs.append(r)
        print("  seed %2d spawn(%+.2f,%+.2f) pen0=%.3f sink0=%.3f fell=%-5s tilt=%5.1f "
              "v=%+.3f dist=%5.2f slow=%.1fs pinch=%.2fs/%d maxpen=%.3f ev=%d stuck=%.1fs %s" %
              (s, spawns[s][0], spawns[s][1], r["spawn_pen"], r["spawn_sink_hf"], r["fell"],
               r["tilt_max"], r["v_mean"], r["dist_xy"], r["slow_s"], r["pinch_s_total"],
               r["pinch_n_total"], max(r["max_pen"]), r["n_events"],
               r["stuck_s"], [(e["class"]["cls"], e["class"]["sub"], e["class"]["init"])
                              for e in r["events"]]), flush=True)
    wall = time.perf_counter() - t0
    out = dict(meta=dict(terrain=opts["terrain"], profile=a.profile,
                         cmd=[float(c) for c in a.cmd], duration=a.duration, spawn=a.spawn,
                         phase=a.phase, seed_list=seeds, window=a.window,
                         disp_eps=a.disp_eps, tilt_max=a.tilt_max, warmup=a.warmup,
                         init_s=a.init_s, scene=os.path.basename(scene),
                         script="scripts/stuck_forensics.py", wall_s=round(float(wall), 1)),
               runs=runs)
    summ = dict(n=len(runs), fell=sum(1 for r in runs if r["fell"]),
                seeds_with_event=sum(1 for r in runs if r["n_events"] > 0),
                seeds_with_noninit_event=sum(1 for r in runs
                                             if any(not e["class"]["init"] for e in r["events"])),
                n_events=sum(r["n_events"] for r in runs),
                n_init_events=sum(1 for r in runs for e in r["events"] if e["class"]["init"]),
                n_noninit_events=sum(1 for r in runs for e in r["events"]
                                     if not e["class"]["init"]),
                stuck_s_mean=round(float(np.mean([r["stuck_s"] for r in runs])), 2),
                v_mean=round(float(np.mean([r["v_mean"] for r in runs])), 3),
                dist_mean=round(float(np.mean([r["dist_xy"] for r in runs])), 2),
                slow_s_mean=round(float(np.mean([r["slow_s"] for r in runs])), 2),
                spawn_pen_mean=round(float(np.mean([r["spawn_pen"] for r in runs])), 4),
                pinch_s_total_mean=round(float(np.mean([r["pinch_s_total"] for r in runs])), 2),
                seeds_with_pinch=sum(1 for r in runs if r["pinch_n_total"] > 0),
                pinch_n_total=sum(r["pinch_n_total"] for r in runs),
                max_pen_mean=round(float(np.mean([max(r["max_pen"]) for r in runs])), 4),
                cls_counter={}, noninit_cls_counter={})
    for r in runs:
        for e in r["events"]:
            c = e["class"]["cls"]
            summ["cls_counter"][c] = summ["cls_counter"].get(c, 0) + 1
            if not e["class"]["init"]:
                summ["noninit_cls_counter"][c] = summ["noninit_cls_counter"].get(c, 0) + 1
    out["summary"] = summ
    print("[forensics] " + json.dumps(summ, ensure_ascii=False), flush=True)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as fh:
            json.dump(out, fh, ensure_ascii=False)
        print("[forensics] -> %s (%.1f MB)" % (a.out, os.path.getsize(a.out) / 1e6), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
