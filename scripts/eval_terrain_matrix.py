#!/usr/bin/env python
"""地形矩阵验收评测 —— 固定 spawn 协议 / 固定指令 / 严格卡死判据。

用途: 部署侧(CPU MuJoCo 500Hz + 50Hz 策略) 的地形鲁棒性**统一口径**评测:
  * 同一份 seeds 序列、同一出生协议(外圈平地环形出生/朝向场地中心, 平地=原点)
  * 同一指令(默认 0.4 m/s), 同一 profile
  * 每个 episode 记录: 存活(摔倒=倾角>FALL_TILT_DEG 或 z<FALL_Z)/前进距离/速度/倾角/
    **严格卡死**(>=STUCK_WIN 内位移 <STUCK_DISP 且 cmd>0.1 且未摔倒 -> 一次事件)/髋角统计/
    动作统计 -> 用来对比"髋摆动大/动作不自然"是否改善。

结果: JSONL(每 episode 一行) + summary JSON + 可选 TSV 行, 便于 before/after 对比。

用法:
  # 16 地形 x 24 seeds, 4 进程
  python scripts/eval_terrain_matrix.py --terrains all --seeds 24 --duration 15 --jobs 4 \
      --out logs/matrix_v25b.json
  # 候选碰撞场景(与 terrains.json 同结构, scene 指向 /tmp 下候选场景)
  python scripts/eval_terrain_matrix.py --manifest /tmp/t2_coll/cand_A.json \
      --terrains hyb2,stg1 --seeds 24

注意: 纯 CPU, 不占 GPU; 入口已强制 BLAS 单线程(见文件顶部)。
"""
import os
import sys

# OpenBLAS 多线程会让 1024-512 小矩阵前向从 0.2ms 变成 12ms(实测), 必须在 numpy 前设置
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

import argparse
import json
import multiprocessing as mp
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from signal_io.config import load_config, robot_spec_of  # noqa: E402
from signal_io.msg import LowCmd  # noqa: E402
from sim_env.robot import Go2Sim  # noqa: E402
from ros2_controller import run_controller as rc  # noqa: E402
from ros2_controller.health import HealthMonitor  # noqa: E402
from ros2_controller.safety import SafetySupervisor  # noqa: E402

MANIFEST = os.path.join(ROOT, "sim_env", "models", "go2", "terrains.json")
FALL_TILT_DEG = 20.0   # 与历次部署评测一致(>20° 判摔)
FALL_Z = 0.16
STUCK_WIN_S = 1.0      # 严格卡死窗口
STUCK_DISP = 0.03      # 窗口内位移阈值 (m)
STUCK_SPEED = 0.05     # 窗口内平均速度阈值 (m/s)


def load_manifest(path):
    with open(path, encoding="utf-8") as fh:
        m = json.load(fh)
    entries = m["terrains"] if isinstance(m, dict) and "terrains" in m else m
    return {e["name"]: e for e in entries}


def _rpy_deg(q):
    w, x, y, z = q[3:7]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(np.degrees(roll)), float(np.degrees(pitch))


def _env_geom_mask(sim):
    m = sim.model
    base = int(sim.base_body)
    robot = set()
    for b in range(int(m.nbody)):
        cur = b
        while cur != 0:
            if cur == base:
                robot.add(b)
                break
            cur = int(m.body_parentid[cur])
    rg = {int(m.geom_group[g]) for g in range(m.ngeom)
          if int(m.geom_bodyid[g]) in robot}
    ag = {int(m.geom_group[g]) for g in range(m.ngeom)}
    env = sorted(ag - rg)
    return np.array([1 if g in env else 0 for g in range(6)], dtype=np.uint8)


def _ground_at(sim, mask, x, y):
    gid = np.zeros(1, dtype=np.int32)
    d = mujoco.mj_ray(sim.model, sim.data, np.array([x, y, 3.0]),
                      np.array([0.0, 0.0, -1.0]), mask, 1, -1, gid)
    return 0.0 if d is None or d >= 3.0 else 3.0 - float(d)


def _ground_safe(sim, mask, x, y, spread=0.15):
    """出生高度 = 中心 + 四足偏移点下方地形最高点 + 2cm 余量。

    旧实现只按中心点 raycast: 若中心落在两块瓦片/两个 box 之间的低点, 四足会被
    放进地形里 2~8cm (T1 实测 uniform 出生: 6/16 假卡死, 前进 -54%)。取邻域
    最高点保证整机出生在地面之上, 自由落体沉降 2cm 内无害。
    """
    hs = [_ground_at(sim, mask, x, y)]
    for dx, dy in ((spread, 0.0), (-spread, 0.0), (0.0, spread), (0.0, -spread)):
        hs.append(_ground_at(sim, mask, x + dx, y + dy))
    return max(hs) + 0.02


def _spawn(sim, meta, mask, seed):
    """出生协议 (与 sim_env 观察器/历次评测一致)。

    hybrid/boxgeom: 外圈平地环形出生 + 朝向场地中心(避免出生在竖面上);
    hfield: 中心附近 0.6m 内随机; flat: 原点。
    固定 seeds 保证 before/after 完全同一出生点。
    """
    rng = np.random.default_rng(seed)
    sim.reset()
    kind = meta.get("kind", "flat")
    fh = float(meta.get("field_half") or 0.0)
    hh = float(meta.get("hfield_half") or fh)
    if kind == "flat" or (fh <= 0 and hh <= 0):
        return np.array(sim.data.qpos[:2], dtype=np.float64)
    if kind == "hfield":
        r = float(rng.uniform(0.0, 0.6))
        th = float(rng.uniform(-np.pi, np.pi))
        yaw = float(rng.uniform(-np.pi, np.pi))
    else:
        r_in = min(fh + 0.4, max(0.0, hh - 0.4))
        r_out = max(r_in + 0.2, min(fh + 1.3, hh - 0.2))
        r = float(rng.uniform(r_in, r_out))
        th = float(rng.uniform(-np.pi, np.pi))
        yaw = float(th + np.pi + rng.uniform(-0.5, 0.5))
    x, y = r * np.cos(th), r * np.sin(th)
    base_z = float(sim.data.qpos[2])
    ground = _ground_safe(sim, mask, x, y)
    sim.data.qpos[0] = x
    sim.data.qpos[1] = y
    sim.data.qpos[2] = base_z + ground + 0.01
    c, s = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    sim.data.qpos[3:7] = [c, 0.0, 0.0, s]
    sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    sim.tick = 0
    sim.time = 0.0
    return np.array([x, y], dtype=np.float64)


def run_episode(sim, ctrl, cfg, meta, mask, seed, cmd, duration):
    p0 = _spawn(sim, meta, mask, seed)
    ctrl.command[:] = np.asarray(cmd, dtype=np.float64)
    ctrl.reset()
    ctrl.obs._cmd = np.asarray(cmd, dtype=np.float64)
    health = HealthMonitor(cfg)
    health.reset(0.0)
    limits = np.asarray(sim.torque_limits, dtype=np.float64)
    sup = SafetySupervisor(sim.stand_pose, limits=limits,
                           ramp_rate=cfg.get("ramp_rate", 200.0),
                           mode_kp=float(cfg.get("safety", {}).get("mode_kp", 20.0)),
                           mode_kd=float(cfg.get("safety", {}).get("mode_kd", 1.0)))
    sup.reset()
    st = sim.read_state()
    sim._last_state = st
    cmdv = LowCmd().zero()
    dec = max(1, int(round(float(cfg["control_dt"]) / sim.dt)))
    n_steps = int(round(duration / sim.dt))
    dt = float(sim.dt)

    qpos_hist = []   # 每 0.05s 的 xy, 用于严格卡死检测
    tilts = []
    speeds = []
    hip_q = []
    hip_act = []
    n_clamped = 0
    fell = False
    t_fall = None
    dist_hist = []
    for k in range(n_steps):
        st = sim.read_state()
        sim._last_state = st
        hs = health.check(st, float(sim.time))
        if k % dec == 0:
            cmdv = ctrl.update(st, float(sim.time))
            if ctrl.last_action is not None:
                a = np.asarray(ctrl.last_action)
                hip_act.append(np.abs(a[[0, 3, 6, 9]]))
        safe = sup.process(cmdv, hs, float(sim.time), sim.dt)
        sim.write_cmd(safe)
        sim.step(1)
        if k % max(1, int(round(0.1 / dt))) == 0:
            q = sim.data.qpos
            qpos_hist.append((float(q[0]), float(q[1])))
            r, p = _rpy_deg(q)
            tl = max(abs(r), abs(p))
            tilts.append(tl)
            speeds.append(float(np.linalg.norm(sim.data.qvel[0:2])))
            dist_hist.append(float(np.linalg.norm(q[:2] - p0)))
            qj = np.asarray(sim.data.qpos[sim.qpos_adr])
            hip_q.append(np.abs(qj[[0, 3, 6, 9]]))
            if not fell and (tl > FALL_TILT_DEG or q[2] < FALL_Z):
                fell = True
                t_fall = float(sim.time)
    qh = np.asarray(qpos_hist)
    tilts = np.asarray(tilts)
    speeds = np.asarray(speeds)
    dists = np.asarray(dist_hist)
    hipq = np.asarray(hip_q) if hip_q else np.zeros((1, 4))
    hipa = np.asarray(hip_act) if hip_act else np.zeros((1, 4))
    # 严格卡死: 滑动窗口内位移 < STUCK_DISP 且窗口均速 < STUCK_SPEED 且未摔倒
    nwin = max(1, int(round(STUCK_WIN_S / 0.1)))
    stuck_s = 0.0
    stuck_events = 0
    in_stuck = False
    for i in range(nwin, len(qh)):
        d = float(np.linalg.norm(qh[i] - qh[i - nwin]))
        sp = float(np.mean(speeds[i - nwin:i]))
        bad = (d < STUCK_DISP and sp < STUCK_SPEED and tilts[i] < 30.0
               and float(np.linalg.norm(np.asarray(cmd)[:2])) > 0.1)
        if bad:
            stuck_s += 0.1
            if not in_stuck:
                stuck_events += 1
                in_stuck = True
        else:
            in_stuck = False
    return dict(
        seed=int(seed),
        spawn=[round(float(p0[0]), 2), round(float(p0[1]), 2)],
        fell=bool(fell),
        t_fall=None if t_fall is None else round(t_fall, 2),
        dist=round(float(dists[-1]) if len(dists) else 0.0, 3),
        v_mean=round(float(speeds.mean()) if len(speeds) else 0.0, 3),
        tilt_max=round(float(tilts.max()) if len(tilts) else 0.0, 1),
        tilt_p95=round(float(np.percentile(tilts, 95)) if len(tilts) else 0.0, 1),
        stuck_s=round(stuck_s, 1),
        stuck_events=int(stuck_events),
        hip_q_mean=[round(float(v), 1) for v in np.degrees(hipq.mean(axis=0))],
        hip_q_p95=[round(float(v), 1) for v in np.degrees(np.percentile(hipq, 95, axis=0))],
        hip_act_p95=[round(float(v), 2) for v in np.percentile(hipa, 95, axis=0)],
        n_calls=int(getattr(ctrl, "n_calls", 0)),
        n_clamped=int(getattr(ctrl, "n_clamped", 0)),
        x=round(float(sim.data.qpos[0]), 2),
        y=round(float(sim.data.qpos[1]), 2),
    )


_W = {}


def _worker_init(payload):
    _W.update(payload)


def _worker_run(task):
    terrain, meta, seeds, cmd, duration = task
    cfg = _W["cfg"]
    scene = meta["scene"]
    if not os.path.isabs(scene):
        scene = os.path.join(ROOT, scene)
    spec = robot_spec_of(cfg)
    sim = Go2Sim(scene, spec=spec, sim_dt=cfg["sim_dt"],
                 foot_soft_contact=bool(getattr(spec, "foot_soft_contact", False)))
    ctrl = rc.build_controller("policy", cfg, sim, cli_cmd=cmd, linvel="oracle")
    mask = _env_geom_mask(sim)
    out = []
    for seed in seeds:
        r = run_episode(sim, ctrl, cfg, meta, mask, seed, cmd, duration)
        r["terrain"] = terrain
        out.append(r)
    return out


def aggregate(rows):
    n = len(rows)
    if n == 0:
        return {}
    surv = 100.0 * sum(1 for r in rows if not r["fell"]) / n
    hipq = np.asarray([r["hip_q_p95"] for r in rows], dtype=np.float64)
    hipa = np.asarray([r["hip_act_p95"] for r in rows], dtype=np.float64)
    return dict(
        n=n,
        survival=round(surv, 1),
        dist_mean=round(float(np.mean([r["dist"] for r in rows])), 2),
        v_mean=round(float(np.mean([r["v_mean"] for r in rows])), 3),
        tilt_max_mean=round(float(np.mean([r["tilt_max"] for r in rows])), 1),
        tilt_p95_mean=round(float(np.mean([r["tilt_p95"] for r in rows])), 1),
        stuck_events=round(float(np.mean([r["stuck_events"] for r in rows])), 2),
        stuck_s=round(float(np.mean([r["stuck_s"] for r in rows])), 2),
        hip_q_p95_mean=[round(float(v), 1) for v in hipq.mean(axis=0)],
        hip_act_p95_mean=[round(float(v), 2) for v in hipa.mean(axis=0)],
    )


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrains", default="all",
                    help="逗号分隔地形名, 'all' = 清单全部(除 flat 可选)")
    ap.add_argument("--no-flat", action="store_true", help="all 时去掉 flat")
    ap.add_argument("--profile", default="go2_v25b", help="策略档 (config/go2_sim.json)")
    ap.add_argument("--manifest", default=MANIFEST, help="地形清单 json (可用候选场景清单)")
    ap.add_argument("--scene-map", default="", help="JSON 文件: {terrain: /abs/scene.xml} 覆盖场景")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--cmd", type=float, nargs=3, default=[0.4, 0.0, 0.0])
    ap.add_argument("--scan", default="raycast", choices=["raycast", "ideal", "nominal"])
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--chunk", type=int, default=6, help="每个任务的 seeds 数(模型只加载一次)")
    ap.add_argument("--out", default="", help="summary JSON 输出路径 (JSONL 同名 .jsonl)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    man = load_manifest(args.manifest)
    scene_map = {}
    if args.scene_map:
        with open(args.scene_map, encoding="utf-8") as fh:
            scene_map = json.load(fh)
    if args.terrains == "all":
        names = [n for n in man if n != "flat"] if args.no_flat else list(man)
    else:
        names = [n.strip() for n in args.terrains.split(",") if n.strip()]
    names = [n for n in names if n in man]
    if not names:
        print("[matrix] 没有可用地形", file=sys.stderr)
        return 2
    for n in names:
        if n in scene_map:
            man[n] = dict(man[n], scene=scene_map[n])

    cfg = load_config(os.path.join(ROOT, "config", "go2_sim.json"),
                      overrides={"robot": "go2", "policy_profile": args.profile})
    cfg["policy"]["height_scan_source"] = args.scan
    cmd = [float(v) for v in args.cmd]
    seeds = list(range(args.seed0, args.seed0 + args.seeds))
    tasks = []
    for n in names:
        for i in range(0, len(seeds), max(1, args.chunk)):
            tasks.append((n, man[n], seeds[i:i + max(1, args.chunk)], cmd, args.duration))
    t0 = time.perf_counter()
    rows = []
    if args.jobs <= 1:
        _worker_init(dict(cfg=cfg))
        for t in tasks:
            rows.extend(_worker_run(t))
    else:
        with mp.Pool(processes=args.jobs, initializer=_worker_init,
                     initargs=(dict(cfg=cfg),)) as pool:
            for part in pool.imap_unordered(_worker_run, tasks):
                rows.extend(part)
                if not args.quiet:
                    print("[matrix] %d/%d episodes" % (len(rows), len(seeds) * len(names)),
                          flush=True)
    wall = time.perf_counter() - t0
    per = {}
    for n in names:
        rs = [r for r in rows if r["terrain"] == n]
        per[n] = aggregate(rs)
    summary = dict(profile=args.profile, scan=args.scan, seeds=args.seeds, seed0=args.seed0,
                   duration=args.duration, cmd=cmd, wall_s=round(wall, 1),
                   manifest=args.manifest, scene_map=args.scene_map, per_terrain=per,
                   overall=aggregate(rows))
    print(json.dumps(summary["overall"], ensure_ascii=False))
    print("%-12s %6s %6s %7s %7s %8s %8s" % ("terrain", "surv%", "v", "dist", "tilt95",
                                             "stuck_ev", "stuck_s"))
    for n in names:
        a = per[n]
        print("%-12s %6.1f %6.3f %7.2f %7.1f %8.2f %8.2f" % (
            n, a["survival"], a["v_mean"], a["dist_mean"], a["tilt_p95_mean"],
            a["stuck_events"], a["stuck_s"]))
    if args.out:
        outp = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
        os.makedirs(os.path.dirname(outp), exist_ok=True)
        with open(outp, "w", encoding="utf-8") as fh:
            json.dump(summary, fh, ensure_ascii=False, indent=1)
        with open(outp + "l", "w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False) + "\n")
        print("[matrix] 写出", outp, "(+ .jsonl)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
