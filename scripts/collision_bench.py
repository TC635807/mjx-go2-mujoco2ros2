#!/usr/bin/env python
"""T2 碰撞表示基准: 生成候选地形碰撞场景 + 几何/感知/性能/机制/整机严格卡死评测。

背景 (实测): CPU MuJoCo 3.12 的 mj_ray **跳过 rgba alpha==0 的 geom**。部署现状把 hfield 设为
视觉 only (group=3, 与机器人碰撞组相同 -> 被非机器人 group 掩码排除), 599 个隐藏倾斜瓦片
是 rgba="1 1 1 0" -> 任何 env 掩码 raycast 都看不到它们, 只看到 floor_plane。因此
_ground_at() 一类出生高度 raycast 在粗糙/斜坡区返回 0, 导致 uniform 出生时机器人嵌进地形
(实测 mean -2.2cm, 37.5% 出生点误差>2cm); 而 perception 的 SimLidar 用 self_occlusion=True
(geomgroup=None) 能看到视觉 hfield, 高度扫描本身没有瞎。

候选 (均在 /tmp 下生成, 不改部署 16 个 scene):
  C0     现状: 0.2m 网格、精确尺寸 (0.2x0.2) 倾斜 box 瓦片 + 训练 box geoms + plane
  C1sX   同一网格/同一数量, 瓦片 xy 半尺寸 x X (X=1.02/1.05/1.10): 相邻瓦片重叠,
         封住精确尺寸时的缝 -> 数量/性能零代价; 上包络浮空量实测 +0.4mm/1%
  C2     连续三角棱柱 mesh (每格 2 个凸三棱柱, 共享边, 顶点取自 hfield) -> 无缝隙
  C3     真 hfield 碰撞 (CPU MuJoCo 球-hfield 每接触会重复 1~6 个同点接触, condim6 下
         滚动/扭转摩擦被放大 -> 走不动, 实测存活 45.8% vs 70.8%, 仅作对照)

用法:
  # 1) 生成候选场景 (默认 /tmp/t2_coll/scenes, 自动复制 go2.xml + assets 软链)
  python scripts/collision_bench.py gen --prefixes hyb2,stg1

  # 2) 几何保真/感知一致性 + 出生高度精度
  python scripts/collision_bench.py validate --prefix hyb2

  # 3) 性能 (加载/mj_step/ncon/内存, 3 次中位)
  python scripts/collision_bench.py perf --prefix hyb2

  # 4) 机制: 足型球 (r=0.022, 2kg) 落在瓦片缝上 -> embedded/trapped/side-contact
  python scripts/collision_bench.py mech --prefix hyb2

  # 5) 整机严格卡死 (T1 口径: ring spawn, 24 seeds x 15s, warmup 2.5s 后 1.2s 窗位移<2cm)
  python scripts/collision_bench.py strict --prefix hyb2 --seeds 24 --duration 15

  # 6) lead 统一口径 (调用 scripts/eval_terrain_matrix.py, 候选清单自动生成)
  python scripts/collision_bench.py matrix --prefix hyb2 --seeds 24 --duration 15 --jobs 4

输出: 场景 -> /tmp/t2_coll/scenes; 数据 -> /tmp/t2_coll/data/*.json
"""
import argparse
import json
import os
import re
import shutil
import sys
import time

for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np

DEPLOY = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
TRAIN = os.path.abspath(os.path.join(DEPLOY, "..", "mjx-go2-training"))
MODELS = os.path.join(TRAIN, "models", "go2")
ASSETS = os.path.join(MODELS, "assets")
OUT = os.environ.get("T2_OUT", "/tmp/t2_coll")
SCENES = os.path.join(OUT, "scenes")
DATA = os.path.join(OUT, "data")
FLAT_HALF = 4.2
HF_HALF = 6.0

sys.path.insert(0, DEPLOY)


# ---------------------------------------------------------------- 训练地形解析高度
class TerrainOracle:
    """训练侧 terrain_height 的等价实现: max(hfield 双线性, boxgrid 查表)。

    hfield 表来自 CPU MuJoCo 载入的训练场景 (与 envs/go2_walk.py 同源同约定;
    已用训练场景 mj_ray 自检: mean |err| 1e-5 m)。
    """

    def __init__(self, prefix):
        import mujoco
        sp = os.path.join(MODELS, "scene_mjx_terrain_%s.xml" % prefix)
        self.m = mujoco.MjModel.from_xml_path(sp)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        hid = 0
        self.nrow = int(self.m.hfield_nrow[hid])
        self.ncol = int(self.m.hfield_ncol[hid])
        self.size = np.array(self.m.hfield_size[hid])
        adr = int(self.m.hfield_adr[hid])
        self.table = np.asarray(
            self.m.hfield_data[adr:adr + self.nrow * self.ncol],
            dtype=np.float64).reshape(self.nrow, self.ncol)
        self.elev = float(self.size[2])
        self.gpos = np.array(self.m.geom_pos[self.m.geom("floor").id])
        self.bg = None
        jp = os.path.join(ASSETS, prefix + ".json")
        if os.path.isfile(jp):
            j = json.load(open(jp))
            if j.get("kind") == "boxgrid":
                self.bg = dict(gw=float(j["grid_w"]), n=int(j["n"]),
                               x0=float(j["x_min"]), y0=float(j["y_min"]),
                               H=np.asarray(j["heights"], np.float64).reshape(j["n"], j["n"]))
        self.mask = np.zeros(6, np.uint8)
        self.mask[0] = 1
        self.gid = np.zeros(1, np.int32)

    def hfield_height(self, xy):
        p = np.asarray(xy, np.float64).reshape(-1, 2)
        u = (p[:, 0] - self.gpos[0]) / self.size[0]
        v = (p[:, 1] - self.gpos[1]) / self.size[1]
        col = np.clip((u + 1.0) * 0.5 * (self.ncol - 1), 0.0, self.ncol - 2.0)
        row = np.clip((v + 1.0) * 0.5 * (self.nrow - 1), 0.0, self.nrow - 2.0)
        r0 = np.floor(row).astype(int)
        c0 = np.floor(col).astype(int)
        fr = row - r0
        fc = col - c0
        t = self.table
        h = ((1 - fr) * ((1 - fc) * t[r0, c0] + fc * t[r0, c0 + 1])
             + fr * ((1 - fc) * t[r0 + 1, c0] + fc * t[r0 + 1, c0 + 1]))
        return h * self.elev

    def boxgrid_height(self, xy):
        p = np.asarray(xy, np.float64).reshape(-1, 2)
        if self.bg is None:
            return np.zeros(len(p))
        g = self.bg
        i = np.floor((p[:, 0] - g["x0"]) / g["gw"]).astype(int)
        j = np.floor((p[:, 1] - g["y0"]) / g["gw"]).astype(int)
        ok = (i >= 0) & (i < g["n"]) & (j >= 0) & (j < g["n"])
        h = np.zeros(len(p))
        h[ok] = g["H"][i[ok], j[ok]]
        return h

    def height(self, xy):
        lead = np.asarray(xy).shape[:-1]
        p = np.asarray(xy, np.float64).reshape(-1, 2)
        return np.maximum(self.hfield_height(p), self.boxgrid_height(p)).reshape(lead)


# ---------------------------------------------------------------- 场景生成
SCENE_TMPL = """<mujoco model="go2 terrain {prefix} cand {cand}">
  <!-- 由 mjx-go2-mujoco2ros2/scripts/collision_bench.py 生成 (T2 碰撞表示 A/B), 不要手改。
       prefix={prefix} cand={cand} {note} -->
  <include file="go2_copy.xml"/>
  <statistic center="0 0 0.1" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
      rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="box_mat" rgba="0.55 0.45 0.35 1"/>
    <hfield name="terrain" file="{png}" size="{sx:.3f} {sy:.3f} {elev:.4f} {base:.4f}"/>
{meshes}
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
{visual_hfield}
    <geom name="floor_plane" type="plane" size="0 0 0.05" material="groundplane"
      pos="0 0 -0.002" friction="1.0 0.02 0.01"/>
{collision}
{boxes}
  </worldbody>
</mujoco>
"""

VISUAL_HFIELD = """    <geom name="floor_vis" type="hfield" hfield="terrain" material="groundplane"
      pos="0 0 0" friction="1.0 0.02 0.01" contype="0" conaffinity="0" group="3"/>"""


def _quat_from_z(nv):
    nv = np.asarray(nv, np.float64)
    nv = nv / max(float(np.linalg.norm(nv)), 1e-12)
    c = float(nv[2])
    if c > 1.0 - 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    if c < -1.0 + 1e-9:
        return (0.0, 1.0, 0.0, 0.0)
    ax = np.cross(np.array([0.0, 0.0, 1.0]), nv)
    ax = ax / max(float(np.linalg.norm(ax)), 1e-12)
    ang = float(np.arccos(np.clip(c, -1.0, 1.0)))
    s = np.sin(ang / 2.0)
    return (float(np.cos(ang / 2.0)), float(ax[0] * s), float(ax[1] * s), float(ax[2] * s))


def parse_training_scene(prefix):
    sp = os.path.join(MODELS, "scene_mjx_terrain_%s.xml" % prefix)
    txt = open(sp, encoding="utf-8").read()
    m = re.search(r"<hfield name=.terrain. file=.([^\"]+). size=.([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+).", txt)
    boxes = ["    " + l.strip() for l in txt.splitlines()
             if re.match(r"<geom name=.(bx|pb)_\d+.", l.strip())]
    return dict(prefix=prefix, png=os.path.join(ASSETS, m.group(1)),
                sx=float(m.group(2)), sy=float(m.group(3)),
                elev=float(m.group(4)), base=float(m.group(5)), boxes=boxes)


def prep_scene_dir():
    """候选场景目录: go2.xml 本地副本 + assets 软链 (MuJoCo 的 meshdir 相对主 XML 解析)。"""
    os.makedirs(SCENES, exist_ok=True)
    os.makedirs(DATA, exist_ok=True)
    src = os.path.join(DEPLOY, "sim_env", "models", "go2")
    copy = os.path.join(SCENES, "go2_copy.xml")
    if not os.path.isfile(copy) or os.path.getmtime(copy) < os.path.getmtime(os.path.join(src, "go2.xml")):
        shutil.copyfile(os.path.join(src, "go2.xml"), copy)
    link = os.path.join(SCENES, "assets")
    if not os.path.exists(link):
        os.symlink(os.path.join(src, "assets"), link)


def tile_grid(o, cell=0.2):
    """C0 网格: 与原 export_deploy_terrains.hfield_collision_tiles 同样的 raycast 采样。"""
    import mujoco
    m, d = o.m, o.d
    hf_gid = [g for g in range(int(m.ngeom))
              if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_HFIELD][0]
    sx, sy = o.size[0], o.size[1]
    n = max(2, int(round(2.0 * sx / cell)))
    xs = o.gpos[0] - sx + (np.arange(n) + 0.5) * cell
    ys = o.gpos[1] - sy + (np.arange(n) + 0.5) * cell
    H = np.zeros((n, n))
    K = np.zeros((n, n), bool)
    # v33: 视觉 hfield/网格 = group1, 碰撞 box/瓦片 = group4 (旧版是 group0)
    mask = np.array([1, 1, 0, 0, 1, 0], np.uint8)
    gid = np.zeros(1, np.int32)
    for i in range(n):
        for j in range(n):
            dist = mujoco.mj_ray(m, d, np.array([xs[i], ys[j], 3.0]),
                                 np.array([0.0, 0.0, -1.0]), mask, 1, -1, gid)
            if dist is None or dist >= 3.0:
                H[i, j] = 0.0
                K[i, j] = False
            else:
                H[i, j] = 3.0 - float(dist)
                K[i, j] = (int(gid[0]) == hf_gid)
    nz = H > 0.002
    grow = nz.copy()
    grow[1:, :] |= nz[:-1, :]
    grow[:-1, :] |= nz[1:, :]
    grow[:, 1:] |= nz[:, :-1]
    grow[:, :-1] |= nz[:, 1:]
    return xs, ys, H, (K & grow), cell


def gen_tiles(o, scale=1.0, thickness=0.12, alpha=0.0):
    """倾斜 box 瓦片。scale=1 -> 现状 (精确尺寸); scale>1 -> 相邻重叠封缝。"""
    xs, ys, H, keep, cell = tile_grid(o)
    n = H.shape[0]
    eps = cell * 0.5 * scale
    rows = []
    for i in range(n):
        for j in range(n):
            if not keep[i, j]:
                continue
            h = H[i, j]
            ip, im = min(i + 1, n - 1), max(i - 1, 0)
            jp, jm = min(j + 1, n - 1), max(j - 1, 0)
            gx = (H[ip, j] - H[im, j]) / max((ip - im) * cell, 1e-9)
            gy = (H[i, jp] - H[i, jm]) / max((jp - jm) * cell, 1e-9)
            nv = np.array([-gx, -gy, 1.0])
            q = _quat_from_z(nv)
            nn = nv / np.linalg.norm(nv)
            cx = xs[i] - nn[0] * thickness * 0.5
            cy = ys[j] - nn[1] * thickness * 0.5
            cz = h - nn[2] * thickness * 0.5
            rows.append(
                '    <geom name="tc_%03d_%03d" type="box" pos="%.4f %.4f %.4f" '
                'size="%.5f %.5f %.5f" quat="%.6f %.6f %.6f %.6f" '
                'rgba="1 1 1 %g" contype="1" conaffinity="1" friction="1.0 0.02 0.01"/>'
                % (i, j, cx, cy, cz, eps, eps, thickness * 0.5,
                   q[0], q[1], q[2], q[3], alpha))
    return rows, len(rows), cell


def gen_mesh(o, res=0.2, thickness=0.12, alpha=0.0):
    """连续三角棱柱 mesh: 顶点取自 hfield, 每格 2 个凸三棱柱 (相邻共享边, 无缝隙)。"""
    sx, sy = o.size[0], o.size[1]
    nx = int(round(2 * sx / res)) + 1
    ny = int(round(2 * sy / res)) + 1
    xs = o.gpos[0] - sx + np.arange(nx) * res
    ys = o.gpos[1] - sy + np.arange(ny) * res
    XX, YY = np.meshgrid(xs, ys, indexing="ij")
    HH = o.hfield_height(np.stack([XX.ravel(), YY.ravel()], -1)).reshape(nx, ny)
    nonflat = np.abs(HH) > 0.002
    g = nonflat.copy()
    g[1:, :] |= nonflat[:-1, :]
    g[:-1, :] |= nonflat[1:, :]
    g[:, 1:] |= nonflat[:, :-1]
    g[:, :-1] |= nonflat[:, 1:]
    meshes = []
    geoms = []
    k = 0

    def prism(pts):
        nonlocal k
        name = "pm_%05d" % k
        vs = " ".join("%.5f" % v for p in pts for v in p)
        meshes.append('    <mesh name="%s" vertex="%s"/>' % (name, vs))
        geoms.append('    <geom name="%s" type="mesh" mesh="%s" pos="0 0 0" '
                     'rgba="1 1 1 %g" contype="1" conaffinity="1" friction="1.0 0.02 0.01"/>'
                     % (name, name, alpha))
        k += 1

    for i in range(nx - 1):
        for j in range(ny - 1):
            if not g[i, j]:
                continue
            h00, h10, h01, h11 = HH[i, j], HH[i + 1, j], HH[i, j + 1], HH[i + 1, j + 1]
            x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
            if (i + j) % 2 == 0:
                tris = [((x0, y0, h00), (x1, y0, h10), (x1, y1, h11)),
                        ((x0, y0, h00), (x1, y1, h11), (x0, y1, h01))]
            else:
                tris = [((x0, y0, h00), (x1, y0, h10), (x0, y1, h01)),
                        ((x1, y0, h10), (x1, y1, h11), (x0, y1, h01))]
            for tri in tris:
                prism(list(tri) + [(p[0], p[1], p[2] - thickness) for p in tri])
    return meshes, geoms, k


def hfield_collision_rows():
    return ['    <geom name="floor" type="hfield" hfield="terrain" material="groundplane"\n'
            '      pos="0 0 0" friction="1.0 0.02 0.01" contype="1" conaffinity="1" group="0"/>',
            VISUAL_HFIELD]


def write_scene(prefix, cand, info, rows, meshes=None, hf_group=3, note=""):
    png_local = os.path.join(SCENES, "terrain_%s.png" % prefix)
    if not os.path.isfile(png_local):
        shutil.copyfile(info["png"], png_local)
    vh = "" if cand.startswith("C3") else VISUAL_HFIELD.replace(
        'group="3"', 'group="%d"' % hf_group)
    txt = SCENE_TMPL.format(
        prefix=prefix, cand=cand, note=note, png=png_local,
        sx=info["sx"], sy=info["sy"], elev=info["elev"], base=info["base"],
        meshes="\n".join(meshes or []), visual_hfield=vh,
        collision="\n".join(rows), boxes="\n".join(info["boxes"]))
    p = os.path.join(SCENES, "scene_%s_%s.xml" % (prefix, cand))
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(txt)
    return p


def cmd_gen(args):
    prep_scene_dir()
    scales = [float(s) for s in args.c1_scales.split(",")]
    for prefix in args.prefixes.split(","):
        info = parse_training_scene(prefix)
        o = TerrainOracle(prefix)
        done = []
        rows, n, cell = gen_tiles(o, 1.0, alpha=0.0)
        done.append(write_scene(prefix, "C0", info, rows, note="现状: %d 瓦片" % n))
        rows_v, _, _ = gen_tiles(o, 1.0, alpha=0.01)
        done.append(write_scene(prefix, "C0v", info, rows_v, note="瓦片 ray 可见"))
        done.append(write_scene(prefix, "C0p", info, rows, hf_group=1,
                                note="视觉 hfield ray 可见 (出生高度正确)"))
        for s in scales:
            tag = "C1s%d" % int(round(s * 100))
            rows_s, _, _ = gen_tiles(o, s, alpha=0.0)
            done.append(write_scene(prefix, tag, info, rows_s,
                                    note="重叠瓦片 scale=%.2f (数量=%d, 位置不变)" % (s, n)))
            rows_sv, _, _ = gen_tiles(o, s, alpha=0.01)
            done.append(write_scene(prefix, tag + "v", info, rows_sv,
                                    note="重叠瓦片 scale=%.2f + ray 可见" % s))
        meshes, geoms, np_ = gen_mesh(o, res=args.c2_res, alpha=0.0)
        done.append(write_scene(prefix, "C2", info, geoms, meshes=meshes,
                                note="连续三角棱柱 mesh res=%.2f, %d 棱柱" % (args.c2_res, np_)))
        meshes2, geoms2, _ = gen_mesh(o, res=args.c2_res, alpha=0.01)
        done.append(write_scene(prefix, "C2v", info, geoms2, meshes=meshes2,
                                note="连续三角棱柱 mesh + ray 可见"))
        done.append(write_scene(prefix, "C3", info, hfield_collision_rows(),
                                note="真 hfield 碰撞 (对照)"))
        print("[gen] %s -> %d 场景:" % (prefix, len(done)))
        for p in done:
            print("   ", p)
    return 0


# ---------------------------------------------------------------- 场景加载/raycast 工具
def load_scene(scene):
    import mujoco
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def env_geom_mask(m):
    import mujoco
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base < 0:
        base = 1
    robot = set()
    for b in range(int(m.nbody)):
        cur = b
        while cur != 0:
            if cur == base:
                robot.add(b)
                break
            cur = int(m.body_parentid[cur])
    rg = {int(m.geom_group[g]) for g in range(m.ngeom) if int(m.geom_bodyid[g]) in robot}
    ag = {int(m.geom_group[g]) for g in range(m.ngeom)}
    env = sorted(ag - rg)
    return np.array([1 if g in env else 0 for g in range(6)], np.uint8)


def ray_heights(m, d, mask, xy, z0=5.0):
    import mujoco
    gid = np.zeros(1, np.int32)
    out = np.full(len(xy), np.nan)
    for k, (x, y) in enumerate(xy):
        dist = mujoco.mj_ray(m, d, np.array([x, y, z0]), np.array([0.0, 0.0, -1.0]),
                             mask, 1, -1, gid)
        if dist is not None and dist < z0 + 0.6:
            out[k] = z0 - float(dist)
    return out


def scene_list(prefix, cands):
    out = []
    for c in cands:
        p = os.path.join(SCENES, "scene_%s_%s.xml" % (prefix, c))
        if os.path.isfile(p):
            out.append((c, p))
    return out


def stats(a):
    a = np.asarray(a)
    return dict(n=int(len(a)), mean=round(float(a.mean()), 4),
                mae=round(float(np.abs(a).mean()), 4),
                p95_abs=round(float(np.percentile(np.abs(a), 95)), 4),
                max_abs=round(float(np.abs(a).max()), 4))


def cmd_validate(args):
    o = TerrainOracle(args.prefix)
    rng = np.random.default_rng(7)
    pts = rng.uniform(-FLAT_HALF, FLAT_HALF, (60000, 2))
    h = o.height(pts)
    bg = o.boxgrid_height(pts) if o.bg is not None else np.zeros(len(pts))
    tile_pts = pts[(h > 0.002) & (bg < 1e-6)][:3000]
    r2 = np.random.default_rng(0)
    spawn_pts = np.asarray([(float(r2.uniform(-FLAT_HALF, FLAT_HALF)),
                             float(r2.uniform(-FLAT_HALF, FLAT_HALF)))
                            for _ in range(24)])
    res = {}
    for tag, p in scene_list(args.prefix, args.cands.split(",")):
        m, d = load_scene(p)
        mask = env_geom_mask(m)
        hs = ray_heights(m, d, mask, tile_pts)
        ok = ~np.isnan(hs)
        ht = o.height(tile_pts)
        g = ray_heights(m, d, mask, spawn_pts, z0=3.0)
        g = np.where(np.isnan(g), 0.0, g)
        eg = g - o.height(spawn_pts)
        res[tag] = dict(ngeom=int(m.ngeom), nmesh=int(m.nmesh),
                        tile_region=stats(hs[ok] - ht[ok]) if ok.any() else dict(n=0),
                        uniform_spawn=dict(mean=round(float(eg.mean()), 4),
                                           max_abs=round(float(np.abs(eg).max()), 4),
                                           frac_gt_2cm=round(float((np.abs(eg) > 0.02).mean()), 3)))
        print(tag, json.dumps(res[tag]))
    json.dump(res, open(os.path.join(DATA, "validate_%s.json" % args.prefix), "w"),
              ensure_ascii=False, indent=1)
    return 0


def cmd_perf(args):
    import mujoco
    res = {}
    for tag, p in scene_list(args.prefix, args.cands.split(",")):
        loads = []
        samples = []
        ncon_peak = 0
        mem = 0.0
        ng = nm = 0
        for _ in range(3):
            t0 = time.perf_counter()
            m = mujoco.MjModel.from_xml_path(p)
            loads.append((time.perf_counter() - t0) * 1000)
            ng, nm = int(m.ngeom), int(m.nmesh)
            mem = float(m.nbuffer) / 1e6
            d = mujoco.MjData(m)
            mujoco.mj_resetDataKeyframe(m, d, 0)
            mujoco.mj_forward(m, d)
            for _ in range(1000):
                mujoco.mj_step(m, d)
            ts = np.empty(1000)
            for k in range(1000):
                t0 = time.perf_counter()
                mujoco.mj_step(m, d)
                ts[k] = time.perf_counter() - t0
                ncon_peak = max(ncon_peak, int(d.ncon))
            samples.append(ts)
        allts = np.concatenate(samples)
        res[tag] = dict(load_ms=round(float(np.median(loads)), 1),
                        step_ms_median=round(1000 * float(np.median(allts)), 3),
                        step_ms_p95=round(1000 * float(np.percentile(allts, 95)), 3),
                        ncon_static_peak=int(ncon_peak), mem_mb=round(mem, 1),
                        ngeom=ng, nmesh=nm)
        print(tag, json.dumps(res[tag]))
    json.dump(res, open(os.path.join(DATA, "perf_%s.json" % args.prefix), "w"),
              ensure_ascii=False, indent=1)
    return 0


# ---------------------------------------------------------------- 机制: 足型球落缝
def _parse_tiles(scene):
    txt = open(scene).read()
    out = []
    pat = (r'<geom name="tc_(\d+)_(\d+)" type="box" pos="([-\d.]+) ([-\d.]+) ([-\d.]+)" '
           r'size="([\d.]+) ([\d.]+) ([\d.]+)" quat="([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)"')
    for m in re.finditer(pat, txt):
        px, py, pz = [float(m.group(k)) for k in (3, 4, 5)]
        hx, hy, hz = [float(m.group(k)) for k in (6, 7, 8)]
        q = np.array([float(m.group(k)) for k in (9, 10, 11, 12)])
        w, x, y, z = q / np.linalg.norm(q)
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        top = np.array([px, py, pz]) + R @ np.array([0, 0, hz])
        corners = [np.array([px, py, pz]) + R @ np.array([sx * hx, sy * hy, hz])
                   for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        out.append(dict(i=int(m.group(1)), j=int(m.group(2)), px=px, py=py,
                        top=top, n=R @ np.array([0, 0, 1.0]), corners=corners))
    return out


def _envelope(tiles, x, y):
    env = np.full(np.shape(x), -0.002, dtype=np.float64)
    x = np.asarray(x, np.float64).reshape(-1)
    y = np.asarray(y, np.float64).reshape(-1)
    for t in tiles:
        n = t["n"]
        if abs(n[2]) < 1e-6:
            continue
        z = t["top"][2] - (n[0] * (x - t["top"][0]) + n[1] * (y - t["top"][1])) / n[2]
        xs = [c[0] for c in t["corners"]]
        ys = [c[1] for c in t["corners"]]
        m = ((x >= min(xs) - 1e-6) & (x <= max(xs) + 1e-6)
             & (y >= min(ys) - 1e-6) & (y <= max(ys) + 1e-6))
        env[m] = np.maximum(env[m], z[m])
    return env


def cmd_mech(args):
    import mujoco
    ref = os.path.join(SCENES, "scene_%s_C0.xml" % args.prefix)
    tiles0 = _parse_tiles(ref)
    centers = {(t["i"], t["j"]): (t["px"], t["py"]) for t in tiles0}
    cand = []
    for (i, j), (x, y) in centers.items():
        cand.append((x, y))
        for di, dj in ((1, 0), (0, 1)):
            if (i + di, j + dj) in centers:
                x2, y2 = centers[(i + di, j + dj)]
                cand.append(((x + x2) / 2, (y + y2) / 2))
    rng = np.random.default_rng(0)
    idx = rng.choice(len(cand), min(args.ntrial, len(cand)), replace=False)
    pts = [cand[k] for k in idx]
    res = {}
    for tag, p in scene_list(args.prefix, args.cands.split(",")):
        txt = open(p).read().replace('<include file="go2_copy.xml"/>', "")
        tiles = _parse_tiles(p)
        rows = []
        for (x, y) in pts:
            env = float(_envelope(tiles, np.array([x]), np.array([y]))[0])
            z0 = env + 0.022 + 0.04
            trial = txt.replace("</worldbody>",
                '  <body pos="%f %f %f"><freejoint/>\n'
                '    <geom name="probe_ball" type="sphere" size="0.022" mass="2.0" condim="6" '
                'friction="0.4 0.02 0.01" priority="1"/>\n  </body>\n</worldbody>'
                % (x, y, z0))
            m = mujoco.MjModel.from_xml_string(trial)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            for _ in range(500):
                mujoco.mj_step(m, d)
            z_settle = float(d.qpos[2])
            ncon = int(d.ncon)
            side = 0
            for k in range(d.ncon):
                c = d.contact[k]
                names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                         for g in (c.geom1, c.geom2)]
                if "probe_ball" not in names:
                    continue
                if float(c.frame[0]) ** 2 + float(c.frame[1]) ** 2 > 0.25:
                    side += 1
            bid = -1
            for b in range(1, m.nbody):
                if any((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "") == "probe_ball"
                       for g in range(m.ngeom) if m.geom_bodyid[g] == b):
                    bid = b
            x0, y0 = float(d.qpos[0]), float(d.qpos[1])
            disp = 0.0
            if args.push and bid > 0:
                for _ in range(250):
                    d.xfrc_applied[bid, 0] = 6.0
                    mujoco.mj_step(m, d)
                disp = float(np.hypot(d.qpos[0] - x0, d.qpos[1] - y0))
            rows.append(dict(x=round(x, 2), y=round(y, 2), env=round(env, 4),
                             pen=round(float((env + 0.022) - z_settle), 4),
                             disp=round(disp, 4), side=int(side), ncon=ncon))
        pen = np.asarray([r["pen"] for r in rows])
        disp = np.asarray([r["disp"] for r in rows])
        res[tag] = dict(n=len(rows),
                        frac_embedded=round(float((pen > 0.005).mean()), 3),
                        frac_side_contact=round(float(np.mean([r["side"] > 0 for r in rows])), 3),
                        frac_trapped=round(float((disp < 0.02).mean()), 3),
                        pen_mean=round(float(pen.mean()), 4),
                        pen_p95=round(float(np.percentile(pen, 95)), 4),
                        rows=rows)
        print(tag, json.dumps({k: v for k, v in res[tag].items() if k != "rows"}))
    json.dump(res, open(os.path.join(DATA, "mech_%s.json" % args.prefix), "w"),
              ensure_ascii=False, indent=1)
    return 0


# ---------------------------------------------------------------- 整机严格卡死 / matrix
def _ring_spawn(sim, mask, fh, hh, seed):
    import mujoco
    rng = np.random.default_rng(seed)
    sim.reset()
    r_in = min(fh + 0.4, max(0.0, hh - 0.4))
    r_out = max(r_in + 0.2, min(fh + 1.3, hh - 0.2))
    r = float(rng.uniform(r_in, r_out))
    th = float(rng.uniform(-np.pi, np.pi))
    yaw = float(th + np.pi + rng.uniform(-0.5, 0.5))
    x, y = r * np.cos(th), r * np.sin(th)
    gid = np.zeros(1, np.int32)
    dist = mujoco.mj_ray(sim.model, sim.data, np.array([x, y, 3.0]),
                         np.array([0.0, 0.0, -1.0]), mask, 1, -1, gid)
    g = 0.0 if dist is None or dist >= 3.0 else 3.0 - float(dist)
    sim.data.qpos[0] = x
    sim.data.qpos[1] = y
    sim.data.qpos[2] = 0.32 + g + 0.01
    sim.data.qpos[3:7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    sim.tick = 0
    sim.time = 0.0


def _rpy_deg(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return float(roll), float(pitch)


def cmd_strict(args):
    import mujoco
    from signal_io.config import load_config, robot_spec_of
    from signal_io.msg import LowCmd
    from sim_env.robot import Go2Sim
    import ros2_controller.run_controller as rc
    from ros2_controller.health import HealthMonitor
    from ros2_controller.safety import SafetySupervisor
    cfg = load_config(os.path.join(DEPLOY, "config", "go2_sim.json"),
                      overrides={"robot": "go2", "policy_profile": "go2_v25b"})
    cfg["policy"]["height_scan_source"] = "raycast"
    spec = robot_spec_of(cfg)
    res = {}
    for tag, p in scene_list(args.prefix, args.cands.split(",")):
        sim = Go2Sim(p, spec=spec, sim_dt=cfg["sim_dt"], foot_soft_contact=False)
        ctrl = rc.build_controller("policy", cfg, sim, cli_cmd=list(args.cmd), linvel="oracle")
        mask = env_geom_mask(sim.model)
        health = HealthMonitor(cfg)
        sup = SafetySupervisor(sim.stand_pose,
                               limits=np.asarray(sim.torque_limits, np.float64),
                               ramp_rate=cfg.get("ramp_rate", 200.0))
        rows = []
        for seed in range(args.seeds):
            _ring_spawn(sim, mask, FLAT_HALF, HF_HALF, seed)
            ctrl.command[:] = np.array(args.cmd)
            ctrl.reset()
            ctrl.obs._cmd = np.array(args.cmd)
            health.reset(0.0)
            sup.reset()
            st = sim.read_state()
            sim._last_state = st
            cmdv = LowCmd().zero()
            hist = []
            fell = False
            n_steps = int(round(args.duration / sim.dt))
            for k in range(n_steps):
                st = sim.read_state()
                sim._last_state = st
                hs = health.check(st, float(sim.time))
                if k % 10 == 0:
                    cmdv = ctrl.update(st, float(sim.time))
                safe = sup.process(cmdv, hs, float(sim.time), sim.dt)
                sim.write_cmd(safe)
                sim.step(1)
                if k % max(1, int(round(0.1 / sim.dt))) == 0:
                    q = sim.data.qpos
                    r, pch = _rpy_deg(q[3:7])
                    tl = max(abs(r), abs(pch))
                    hist.append((float(sim.time), float(q[0]), float(q[1]), tl, float(q[2]),
                                 float(np.linalg.norm(sim.data.qvel[0:2]))))
                    if not fell and (tl > 20.0 or q[2] < 0.16):
                        fell = True
            H = np.asarray(hist)
            win = int(round(1.2 / 0.1))
            stuck_s = 0.0
            events = []
            in_stuck = False
            for i in range(win, len(H)):
                if H[i, 0] < 2.5 or fell:
                    continue
                dd = float(np.hypot(*(H[i, 1:3] - H[i - win, 1:3])))
                ok = dd < 0.02 and H[i, 3] < 30.0 and H[i, 4] > 0.15
                if ok:
                    stuck_s += 0.1
                    if not in_stuck:
                        events.append(dict(t=round(float(H[i, 0]), 2),
                                           x=round(float(H[i, 1]), 2),
                                           y=round(float(H[i, 2]), 2)))
                        in_stuck = True
                else:
                    in_stuck = False
            dist = float(np.hypot(H[-1, 1] - H[0, 1], H[-1, 2] - H[0, 2])) if len(H) else 0.0
            rows.append(dict(seed=seed, fell=bool(fell), dist=round(dist, 2),
                             v_mean=round(float(H[:, 5].mean()), 3),
                             tilt_max=round(float(H[:, 3].max()), 1),
                             stuck_s=round(stuck_s, 1), stuck_events=len(events), events=events))
            print("  %s seed %2d fell=%-5s dist=%.2f v=%.3f tilt=%.1f stuck=%.1fs/%d"
                  % (tag, seed, fell, dist, rows[-1]["v_mean"], rows[-1]["tilt_max"],
                     stuck_s, len(events)), flush=True)
        n = len(rows)
        res[tag] = dict(scene=os.path.basename(p), n=n,
                        survival=round(100.0 * sum(1 for r in rows if not r["fell"]) / n, 1),
                        dist_mean=round(float(np.mean([r["dist"] for r in rows])), 2),
                        v_mean=round(float(np.mean([r["v_mean"] for r in rows])), 3),
                        tilt_max_mean=round(float(np.mean([r["tilt_max"] for r in rows])), 1),
                        stuck_events=sum(r["stuck_events"] for r in rows),
                        stuck_seed_frac=round(100.0 * sum(1 for r in rows if r["stuck_events"] > 0) / n, 1),
                        stuck_s_total=round(float(sum(r["stuck_s"] for r in rows)), 1),
                        per_seed=rows)
        print(tag, json.dumps({k: v for k, v in res[tag].items() if k != "per_seed"}))
    json.dump(res, open(os.path.join(DATA, "strict_%s.json" % args.prefix), "w"),
              ensure_ascii=False, indent=1)
    return 0


def cmd_matrix(args):
    """调用 lead 的统一口径脚本 scripts/eval_terrain_matrix.py (候选清单自动生成)。"""
    mpath = os.path.join(DATA, "cand_manifest.json")
    entries = [dict(name="flat",
                    scene=os.path.join(DEPLOY, "sim_env/models/go2/scene_flat.xml"),
                    kind="flat", field_half=0.0, hfield_half=0.0)]
    names = []
    for prefix in args.prefixes.split(","):
        for tag, p in scene_list(prefix, args.cands.split(",")):
            name = "%s_%s" % (prefix, tag)
            names.append(name)
            entries.append(dict(name=name, scene=p, kind="hybrid",
                                field_half=FLAT_HALF, hfield_half=HF_HALF))
    json.dump(dict(terrains=entries), open(mpath, "w"), ensure_ascii=False, indent=1)
    cmd = [sys.executable, os.path.join(DEPLOY, "scripts", "eval_terrain_matrix.py"),
           "--manifest", mpath, "--terrains", ",".join(names),
           "--seeds", str(args.seeds), "--duration", str(args.duration),
           "--jobs", str(args.jobs), "--cmd", "0.4", "0", "0",
           "--out", os.path.join(DATA, "matrix_%s.json" % args.prefix)]
    print("运行:", " ".join(cmd))
    os.execv(sys.executable, cmd)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    default_cands = "C0,C0v,C0p,C1s102,C1s105,C1s110,C2v,C3"
    g = sub.add_parser("gen")
    g.add_argument("--prefixes", default="hyb2,stg1")
    g.add_argument("--c1-scales", default="1.02,1.05,1.10")
    g.add_argument("--c2-res", type=float, default=0.2)
    g.set_defaults(func=cmd_gen)
    for name, fn in (("validate", cmd_validate), ("perf", cmd_perf),
                     ("mech", cmd_mech), ("strict", cmd_strict)):
        s = sub.add_parser(name)
        s.add_argument("--prefix", default="hyb2")
        s.add_argument("--cands", default=default_cands)
        if name == "mech":
            s.add_argument("--ntrial", type=int, default=60)
            s.add_argument("--push", action="store_true", default=True)
            s.add_argument("--no-push", dest="push", action="store_false")
        if name == "strict":
            s.add_argument("--seeds", type=int, default=24)
            s.add_argument("--duration", type=float, default=15.0)
            s.add_argument("--cmd", type=float, nargs=3, default=[0.4, 0, 0])
        s.set_defaults(func=fn)
    s = sub.add_parser("matrix")
    s.add_argument("--prefixes", default="hyb2,stg1")
    s.add_argument("--cands", default=default_cands)
    s.add_argument("--seeds", type=int, default=24)
    s.add_argument("--duration", type=float, default=15.0)
    s.add_argument("--jobs", type=int, default=4)
    s.set_defaults(func=cmd_matrix)

    args = ap.parse_args(argv)
    os.makedirs(SCENES, exist_ok=True)
    os.makedirs(DATA, exist_ok=True)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())