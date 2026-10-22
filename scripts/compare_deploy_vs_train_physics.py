#!/usr/bin/env python3
"""部署物理 vs 训练物理: 用 eval_go2.py 的同一套判据跑同一个策略, 量化性能差 (task-5 §3)。

做法
----
**不重写判据** —— 直接 subprocess 调 mjx-go2-training/sim/eval_go2.py, 只换 --xml / --foot-contact,
其余参数逐字相同。于是"差异"只可能来自被测模型, 不可能来自两套指标实现。

四组对照 (默认):
    train / flat      models/go2/scene_mjx_position.xml    <position> + 软接触(训练 XML 原样)
    deploy/ flat      <deploy>/models/go2/scene_flat.xml   <motor> + wrapper PD + 软接触(强制)
    train / terrain   models/go2/scene_mjx_terrain.xml     <position> + hfield 地形
    deploy/ terrain   由本脚本生成: 部署 body + 同一份 hfield (保证地形逐点相同)

另有两组"无策略"对照 (--dummy hold_home): 去掉策略这个变量, 直接量两套物理在纯站姿下的差别。

顺带做一次**接口互检**: 用 eval_go2 自己的 Go2Sim 在两个模型上摆到同一物理状态, 比较 91 维
obs。这是"策略放进部署模型不会静默错位"的直接证据 (部署模型 body 树顺序是 FL,FR,RL,RR,
而观测/动作需要 FR,FL,RR,RL —— eval_go2 用 --dof-order actuator 做置换, 必须验证置换是对的)。

    python scripts/compare_deploy_vs_train_physics.py \
        --policy /home/tc63/mujoco/mjx-go2-training/policies/go2_smoke.pkl --seeds 8
    # 正式策略训练完后: 只换 --policy 即可
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GETUP = "/home/tc63/mujoco/mjx-go2-training"
EVAL_SCRIPT = os.path.join(GETUP, "sim", "eval_go2.py")
TRAIN_FLAT = os.path.join(GETUP, "models", "go2", "scene_mjx_position.xml")
TRAIN_TERRAIN = os.path.join(GETUP, "models", "go2", "scene_mjx_terrain.xml")
DEPLOY_FLAT = os.path.join(PROJECT_ROOT, "sim_env", "models", "go2", "scene_flat.xml")
DEPLOY_BODY = os.path.join(PROJECT_ROOT, "sim_env", "models", "go2", "go2.xml")

METRIC_KEYS = [
    ("survive_rate_env", "存活率(env判据) %", 100.0),
    ("survive_rate_tilt", "存活率(tilt判据) %", 100.0),
    ("fall_time_mean_s", "平均跌落时间 s", 1.0),
    ("vx_mean", "vx 均值 m/s", 1.0),
    ("track_err_ss", "稳态跟踪误差 m/s", 1.0),
    ("z_rel_mean", "躯干离地高度 m", 1.0),
    ("pitch_mean_deg", "pitch 均值 deg", 1.0),
    ("pitch_max_deg", "pitch 最大 deg", 1.0),
    ("angvel_rms", "角速度 RMS rad/s", 1.0),
    ("tau_abs_max", "|tau| 峰值 N·m", 1.0),
    ("dist_forward_mean", "前进距离 m", 1.0),
]


def _read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def build_deploy_terrain(out_path, train_terrain=None):
    """部署 body + 训练地形 hfield 的单文件场景 (保证地形逐点相同)。

    train_terrain: 训练侧地形场景 XML; None = 旧 scene_mjx_terrain.xml。
    v6 用 models/go2/scene_mjx_terrain_mix.xml (n=128 混合地形)。
    """
    terrain_path = str(train_terrain or TRAIN_TERRAIN)
    scene = _read(terrain_path)
    body = _read(DEPLOY_BODY)
    cm = re.search(r"<compiler\b([^>]*?)/>", body, re.S)
    attrs = cm.group(1) if cm else ' angle="radian" autolimits="true"'
    md = re.search(r'meshdir\s*=\s*"([^"]*)"', attrs)
    abs_mesh = os.path.normpath(os.path.join(os.path.dirname(DEPLOY_BODY),
                                             md.group(1) if md else "."))
    attrs = re.sub(r'\s*meshdir\s*=\s*"[^"]*"', "", attrs)
    if "autolimits" not in attrs:
        attrs += ' autolimits="true"'
    inner = body[cm.end():] if cm else body
    inner = inner[inner.index(">") + 1: inner.rindex("</mujoco>")]
    n = len(re.findall(r'<include\s+file="[^"]*"\s*/>', scene))
    if n != 1:
        raise SystemExit("地形场景里应恰好有 1 个 <include>，实际 %d" % n)
    scene = re.sub(r'<include\s+file="[^"]*"\s*/>', lambda m: inner, scene, count=1)
    # hfield 贴图: 场景里写的是相对 meshdir 的文件名 -> 换成训练侧绝对路径
    m_png = re.search(r'<hfield\b[^>]*?file="([^"]+)"', scene)
    if not m_png:
        raise SystemExit("地形场景里找不到 hfield file=...: %s" % terrain_path)
    png_rel = m_png.group(1)
    cands = [
        os.path.join(os.path.dirname(terrain_path), png_rel),
        os.path.join(os.path.dirname(terrain_path), "assets", png_rel),
    ]
    abs_png = next((os.path.normpath(c) for c in cands if os.path.isfile(c)), None)
    if abs_png is None:
        raise SystemExit("找不到 hfield 贴图 %r (候选: %s)" % (png_rel, cands))
    scene = scene.replace('file="%s"' % png_rel, 'file="%s"' % abs_png)
    scene = re.sub(r"(<mujoco[^>]*>)",
                   lambda m: m.group(1) + '\n  <compiler%s meshdir="%s"/>' % (attrs, abs_mesh),
                   scene, count=1)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(scene)
    return out_path


def run_eval(tag, xml, terrain, foot_contact, policy, dummy, args, outdir):
    cmd = [sys.executable, EVAL_SCRIPT,
           "--xml", xml, "--terrain", terrain,
           "--solver", "cpu", "--foot-contact", foot_contact,
           "--home-pose", "unitree_rl_gym", "--spawn", "origin",
           "--spawn-z", str(args.spawn_z), "--reset-noise", str(args.reset_noise),
           "--cmd", str(args.cmd[0]), str(args.cmd[1]), str(args.cmd[2]),
           "--seeds", str(args.seeds), "--duration", str(args.duration),
           "--out", outdir, "--tag", tag, "--no-plot", "--quiet"]
    if policy:
        cmd += ["--policy", policy]
    else:
        cmd += ["--policy", "none", "--dummy", dummy]
    env = dict(os.environ)
    env.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    env.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.15")
    env.setdefault("JAX_PLATFORM_NAME", "cpu")
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    jp = os.path.join(outdir, tag + ".json")
    if p.returncode != 0 or not os.path.isfile(jp):
        print(p.stdout[-3000:], p.stderr[-3000:])
        raise SystemExit("eval_go2 运行失败: %s" % tag)
    with open(jp) as f:
        d = json.load(f)
    d["_tag"] = tag
    d["_xml"] = xml
    return d


def xcheck_obs(args, workdir):
    """接口互检: 两个模型摆到同一物理状态, 比较 91 维 obs。"""
    sys.path.insert(0, os.path.join(GETUP, "sim"))
    import eval_go2 as E  # noqa: E402

    parser = E.build_parser()
    def mk(xml):
        a = parser.parse_args(["--xml", xml, "--solver", "cpu", "--home-pose",
                               "unitree_rl_gym", "--spawn", "origin",
                               "--spawn-z", str(args.spawn_z), "--reset-noise", "0",
                               "--foot-contact", "xml", "--seeds", "1", "--duration", "1",
                               "--quiet", "--no-plot"])
        return E.Go2Sim(xml, E.finalize_args(a))

    tr = mk(TRAIN_FLAT)
    dp = mk(DEPLOY_FLAT)
    cmd = np.array(args.cmd, dtype=np.float64)
    st_tr = tr.reset(7, cmd, "origin")
    st_dp = dp.reset(7, cmd, "origin")
    o_tr = tr.obs(st_tr, True, True)
    o_dp = dp.obs(st_dp, True, True)
    d = np.abs(o_tr.astype(np.float64) - o_dp.astype(np.float64))
    # 分解：哪一段不一样
    segs = {"linvel": (0, 3), "angvel": (3, 6), "gravity": (6, 9), "cmd": (9, 12),
            "q-home": (12, 24), "dq": (24, 36), "last_act": (36, 48),
            "scan": (48, 83), "phase": (83, 91)}
    per = {k: float(d[a:b].max()) for k, (a, b) in segs.items()}
    print("  接口互检 (eval_go2 自己的 Go2Sim, 同物理状态 -> 91 维 obs):")
    print("    dof_order  : train=%s / deploy=%s (置换 %s)" % (
        tr.dof_order, dp.dof_order,
        "已应用" if dp.dof_from_act is not None else "不需要"))
    print("    max|Δobs|  : %.3e   (逐段 %s)" % (float(d.max()), per))
    ok = float(d.max()) < 1e-5
    print("    -> %s" % ("接口一致 ✓ (策略放进部署模型不会静默错位)" if ok else "**接口不一致** ✗"))
    return {"dof_order_train": tr.dof_order, "dof_order_deploy": dp.dof_order,
            "max_abs_diff": float(d.max()), "per_segment": per, "pass": bool(ok)}


def table(rows, title):
    print()
    print("  " + title)
    print("  %-26s %14s %14s %12s" % ("指标", "训练物理(position)", "部署物理(motor+PD)", "差"))
    print("  " + "-" * 72)
    for key, name, scale in METRIC_KEYS:
        try:
            a = float(rows["train"][key]) * scale
            b = float(rows["deploy"][key]) * scale
        except (KeyError, TypeError):
            continue
        print("  %-26s %14.4f %14.4f %+12.4f" % (name, a, b, b - a))
    # 地形分层通过率
    for side, r in (("train", rows["train"]), ("deploy", rows["deploy"])):
        bins = (r.get("terrain_report") or {}).get("bins") or []
        hit = [x for x in bins if x.get("n_reached")]
        if hit:
            print("  [%s] 地形分层:" % side)
            for x in hit:
                pr = x.get("pass_rate")
                print("    h∈[%.2f,%.2f) 到达 %d 轮, 通过 %s" % (
                    x["bin_lo_m"], x["bin_hi_m"], x["n_reached"],
                    "n/a" if pr is None else "%.1f%%" % (100 * pr)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=os.path.join(GETUP, "policies", "go2_smoke.pkl"))
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--cmd", type=float, nargs=3, default=[0.5, 0.0, 0.0])
    ap.add_argument("--spawn-z", type=float, default=0.32)
    ap.add_argument("--reset-noise", type=float, default=0.1)
    ap.add_argument("--out", default=os.path.join(PROJECT_ROOT, "logs", "go2_parity_eval"))
    ap.add_argument("--workdir", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--skip-terrain", action="store_true")
    ap.add_argument("--skip-dummy", action="store_true")
    args = ap.parse_args()

    workdir = args.workdir or os.path.join(PROJECT_ROOT, "logs", "go2_parity_eval", "_scenes")
    os.makedirs(args.out, exist_ok=True)
    deploy_terrain = build_deploy_terrain(os.path.join(workdir, "deploy_terrain.xml"))
    stag = os.path.splitext(os.path.basename(args.policy))[0]

    print("=" * 96)
    print("部署物理 vs 训练物理 (判据: sim/eval_go2.py)")
    print("=" * 96)
    print("  策略        :", args.policy, "(smoke/未训练时结论只作链路验证)")
    print("  回合        :%d 轮 x %.1f s   指令 %s" % (args.seeds, args.duration, args.cmd))
    print("  共同设置    : --solver cpu (100/50) --home-pose unitree_rl_gym "
          "--spawn origin --spawn-z %.2f --reset-noise %.2f" % (args.spawn_z, args.reset_noise))
    print("  训练 XML    :", os.path.relpath(TRAIN_FLAT, PROJECT_ROOT))
    print("  部署 XML    :", os.path.relpath(DEPLOY_FLAT, PROJECT_ROOT))
    print("  部署地形 XML:", os.path.relpath(deploy_terrain, PROJECT_ROOT), "(部署 body + 同一份 hfield)")
    print()
    print("  " + "-" * 92)
    xc = xcheck_obs(args, workdir)
    print("  " + "-" * 92)

    out = {"policy": args.policy, "seeds": args.seeds, "duration": args.duration,
           "cmd": list(args.cmd), "xcheck": xc, "runs": {}, "dummy": {}}

    def pair(name, xml_tr, xml_dp, terrain, fc_tr, fc_dp):
        a = run_eval("%s_%s_train" % (stag, name), xml_tr, terrain, fc_tr,
                     args.policy, None, args, args.out)
        b = run_eval("%s_%s_deploy" % (stag, name), xml_dp, terrain, fc_dp,
                     args.policy, None, args, args.out)
        out["runs"][name] = {"train": a, "deploy": b}
        print()
        print("  [%s] 策略 %s" % (name, os.path.basename(args.policy)))
        print("    训练: 存活(env) %.1f%%  vx %+.3f  z %.3f  pitch %+.2f°  |tau|max %.1f  前进 %.2f m" % (
            100 * a["survive_rate_env"], a["vx_mean"], a["z_rel_mean"],
            a["pitch_mean_deg"], a["tau_abs_max"], a["dist_forward_mean"]))
        print("    部署: 存活(env) %.1f%%  vx %+.3f  z %.3f  pitch %+.2f°  |tau|max %.1f  前进 %.2f m" % (
            100 * b["survive_rate_env"], b["vx_mean"], b["z_rel_mean"],
            b["pitch_mean_deg"], b["tau_abs_max"], b["dist_forward_mean"]))
        return a, b

    print()
    print("  " + "-" * 92)
    print("  A. 平地对照")
    print("  " + "-" * 92)
    pair("flat", TRAIN_FLAT, DEPLOY_FLAT, "flat", "xml", "soft")
    table(out["runs"]["flat"], "平地: 同一策略 / 同一判据, 只换物理模型")

    # 第三行: 部署模型 + 硬接触 = **本任务改动之前** run_sim/probe 的默认物理
    c = run_eval("%s_flat_deployhard" % stag, DEPLOY_FLAT, "flat", "xml",
                 args.policy, None, args, args.out)
    out["runs"]["flat_deployhard"] = {"train": out["runs"]["flat"]["train"], "deploy": c}
    print()
    print("  [flat/改前的部署默认] 部署 XML + 硬接触(condim6):")
    print("    存活(env) %.1f%%  vx %+.3f  z %.3f  pitch %+.2f°  |tau|max %.1f  前进 %.2f m" % (
        100 * c["survive_rate_env"], c["vx_mean"], c["z_rel_mean"],
        c["pitch_mean_deg"], c["tau_abs_max"], c["dist_forward_mean"]))
    table(out["runs"]["flat_deployhard"],
          "平地: 训练物理 / 部署物理(软接触=改后) / 部署物理(硬接触=改前)")

    if not args.skip_terrain:
        print()
        print("  " + "-" * 92)
        print("  B. 地形对照 (同一份 hfield)")
        print("  " + "-" * 92)
        pair("terrain", TRAIN_TERRAIN, deploy_terrain, "terrain", "xml", "soft")
        table(out["runs"]["terrain"], "地形: 同一策略 / 同一判据, 只换物理模型")

    if not args.skip_dummy:
        print()
        print("  " + "-" * 92)
        print("  C. 无策略对照 (--dummy hold_home): 去掉策略变量, 只量两套物理")
        print("  " + "-" * 92)
        a = run_eval("%s_dummy_train" % stag, TRAIN_FLAT, "flat", "xml", None,
                     "hold_home", args, args.out)
        b = run_eval("%s_dummy_deploy" % stag, DEPLOY_FLAT, "flat", "soft", None,
                     "hold_home", args, args.out)
        c2 = run_eval("%s_dummy_deployhard" % stag, DEPLOY_FLAT, "flat", "xml", None,
                      "hold_home", args, args.out)
        out["dummy"] = {"train": a, "deploy": b, "deploy_hard": c2}
        print("    训练物理 z=%.4f pitch=%+.2f° | 部署物理(软) z=%.4f pitch=%+.2f° | 部署物理(硬) z=%.4f pitch=%+.2f°" % (
            a["z_rel_mean"], a["pitch_mean_deg"], b["z_rel_mean"], b["pitch_mean_deg"],
            c2["z_rel_mean"], c2["pitch_mean_deg"]))
        table(out["dummy"], "纯站姿 (hold_home): 两套物理的直接差别")

    print()
    print("=" * 96)
    print("  JSON 明细:", args.out)
    print("=" * 96)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2, default=str)
        print("  [json]", args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
