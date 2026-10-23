#!/usr/bin/env python3
"""地形清单解析与仿真出生点（ROS 2 仿真节点 / 观察器共用）。"""

from __future__ import annotations

import json
import os
from typing import Dict, Optional

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TERRAIN_MANIFEST = os.path.join(ROOT, "sim_env", "models", "go2", "terrains.json")


def load_terrain_manifest(path: str = TERRAIN_MANIFEST) -> Dict[str, dict]:
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as fh:
        man = json.load(fh)
    return {e["name"]: e for e in man.get("terrains", []) if "name" in e}


def resolve_terrain(name: str, manifest: Optional[Dict[str, dict]] = None) -> dict:
    man = load_terrain_manifest() if manifest is None else manifest
    e = man.get(name)
    if e is None:
        raise SystemExit("未知地形 %r; 可选: %s (清单: %s)"
                         % (name, ", ".join(sorted(man)), TERRAIN_MANIFEST))
    e = dict(e)
    if not os.path.isabs(e["scene"]):
        e["scene"] = os.path.join(ROOT, e["scene"])
    if not os.path.isfile(e["scene"]):
        raise SystemExit("地形场景不存在: %s" % e["scene"])
    return e


def ray_ground_height(sim, x: float, y: float, top: float = 3.0) -> float:
    import mujoco

    pnt = np.array([x, y, top], dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(sim.model, sim.data, pnt, vec, None, 1, -1, geomid)
    if dist < 0.0 or dist >= top:
        return 0.0
    return float(top - dist)


def spawn_on_terrain(sim, meta: dict, rng=None) -> None:
    """把机器人放到该地形上（外圈平地出生；hfield 在中心附近出生）。"""
    rng = np.random.default_rng() if rng is None else rng
    sim.reset()
    kind = meta.get("kind", "flat")
    fh = float(meta.get("field_half") or 0.0)
    hh = float(meta.get("hfield_half") or fh)
    if kind == "flat" or (fh <= 0.0 and hh <= 0.0):
        return
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
    ground = ray_ground_height(sim, x, y)
    sim.data.qpos[0] = x
    sim.data.qpos[1] = y
    sim.data.qpos[2] = base_z + ground + 0.01
    c, s = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    sim.data.qpos[3:7] = [c, 0.0, 0.0, s]
    sim.data.qvel[:] = 0.0
    import mujoco

    mujoco.mj_forward(sim.model, sim.data)
    sim.tick = 0
    sim.time = 0.0


__all__ = ["TERRAIN_MANIFEST", "load_terrain_manifest", "resolve_terrain",
           "ray_ground_height", "spawn_on_terrain"]
