"""perception.py —— 点云 → 局部高程图 → 35 点机体系高度扫描（仿真 / 真机共用一条管线）。

契约：docs/architecture.md §6（签名以契约为准）。

    SimLidar (MuJoCo ray-cast)   ─┐
                                  ├─→ HeightMapper.update(points_world, t)
    PointCloud2（真机，以后接）  ─┘            │
                                              ├─→ ground_height(xy_world)   （双线性）
                                              └─→ scan(base_pos, base_quat)  (35,) → obs[48:83]

**训练侧定义是唯一真相**（mjx-go2-training/envs/go2_walk.py::height_scan_features）：

    grid     : nx=7, ny=5, x∈[-0.3,0.9], y∈[-0.4,0.4]
               np.meshgrid(gx, gy, indexing="ij").ravel() → x 外层、y 内层，index = i*ny+j
    xy_world = Rz(yaw) @ xy_local + base_xy            # 只按 yaw 旋转，不含 roll/pitch
    d_m      = base_z - h_ground(xy_world) - 0.28
    feature  = clip(d_m, ±0.35) * 5.0

本模块逐项复刻上式；任何一处改动（例如用完整姿态旋转、clip 乘完再截断、base 取别的值）
都会让策略看到的地形特征整体偏置。

约定：
  * 世界系 x 前 / y 左 / z 上；机体系同向，原点在躯干（free joint 位置）。
  * 点云一律 **世界系** (N,3)；真机侧把 PointCloud2 解成同样形状即可复用整条管线。
  * 运行时只依赖 numpy（+ 仿真侧 mujoco，且只在 SimLidar 里 import）。
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------- 契约常量（训练侧）
SCAN_NX = 17
SCAN_NY = 11
SCAN_X_RANGE = (-0.4, 1.2)
SCAN_Y_RANGE = (-0.5, 0.5)
SCAN_BASE = 0.268          # 标称站高 (m)，训练 base_height_target 同值
SCAN_CLIP_M = 0.35        # 米制差值截断 (m)
SCAN_SCALE = 5.0          # 米制差 → 特征
NOMINAL_BASE_HEIGHT = 0.268

__all__ = [
    "HeightMapper", "SimLidar", "scan_grid_body",
    "SCAN_NX", "SCAN_NY", "SCAN_X_RANGE", "SCAN_Y_RANGE",
    "SCAN_BASE", "SCAN_CLIP_M", "SCAN_SCALE",
]


def scan_grid_body(nx: int = SCAN_NX, ny: int = SCAN_NY,
                   x_range: Tuple[float, float] = SCAN_X_RANGE,
                   y_range: Tuple[float, float] = SCAN_Y_RANGE) -> np.ndarray:
    """(N,2) 机体系采样点，**x 外层、y 内层**（index = i*ny + j）。

    与训练 np.meshgrid(gx, gy, indexing="ij").ravel() 以及
    obs.WalkObsBuilder.scan_grid() 必须逐点相同。
    """
    gx = np.linspace(float(x_range[0]), float(x_range[1]), int(nx))
    gy = np.linspace(float(y_range[0]), float(y_range[1]), int(ny))
    GX, GY = np.meshgrid(gx, gy, indexing="ij")
    return np.stack([GX.ravel(), GY.ravel()], axis=-1)


class HeightMapper:
    """世界系点云 → 稀疏局部高程图 → 双线性地面高度 → 35 点训练口径高度扫描。

    高程图是**世界系稀疏格网**：格点 (i, j) 对应世界坐标 (i*resolution, j*resolution)，
    每个格点存该格内点云的最大 z 与最后一次观测时间 t。这样 update() 不需要基座位姿，
    scan() 时再把机体系采样点按 yaw 旋到世界系查询——与训练侧 terrain_height(xy_world)
    的调用方式一致。

    格点超时（t - t_obs > decay_s）视为未知；未知格点在双线性时按可用权重重新归一化，
    四个格点全缺时用半径 fill_radius_cells 内最近的已知格点"保持填充"（hold），
    再不行才用 default_height（0.0 = 世界地面平面）。
    """

    # key 打包：i,j 各加 OFF 偏移后折成一维
    _OFF = 1 << 20
    _SPAN = 1 << 21

    def __init__(self, nx: int = SCAN_NX, ny: int = SCAN_NY,
                 x_range: Tuple[float, float] = SCAN_X_RANGE,
                 y_range: Tuple[float, float] = SCAN_Y_RANGE,
                 resolution: float = 0.05, decay_s: float = 1.0,
                 base: float = SCAN_BASE, clip_m: float = SCAN_CLIP_M,
                 scale: float = SCAN_SCALE, *, max_cells: int = 262144,
                 fill_radius_cells: int = 4, default_height: float = 0.0,
                 fill_local: bool = False) -> None:
        self.nx = int(nx)
        self.ny = int(ny)
        self.x_range = (float(x_range[0]), float(x_range[1]))
        self.y_range = (float(y_range[0]), float(y_range[1]))
        self.resolution = float(resolution)
        self.decay_s = float(decay_s)
        self.base = float(base)
        self.clip_m = float(clip_m)
        self.scale = float(scale)
        self.max_cells = int(max_cells)
        self.fill_radius_cells = int(fill_radius_cells)
        self.default_height = float(default_height)
        # v6: 完全未观测的格点填"局部地面估计"(= 当前基座高度 - 标称站高)
        # 而不是世界 z=0。训练侧 v6 遮挡模型把未观测格点填成特征 0 (中性平地先验),
        # 而特征 0 等价于 h_ground = z_base - base —— 即这里的 fill_local 语义。
        # 旧行为 (fill_local=False) 在楼梯上会系统性偏 0.2m, 是真机高程图的
        # "未知格默认值" bug 的仿真等价物。
        self.fill_local = bool(fill_local)
        if self.resolution <= 0.0:
            raise ValueError("resolution 必须 > 0")
        if self.nx < 1 or self.ny < 1:
            raise ValueError("nx/ny 必须 ≥ 1")

        self._grid: Dict[int, list] = {}      # packed(i,j) -> [z_max, t_obs]
        self._t_last = -np.inf
        self._t_pruned = -np.inf
        self.n_points_total = 0
        self.n_points_last = 0
        self.n_prunes = 0
        self.last_scan_observed = 0
        self._grid_points = scan_grid_body(self.nx, self.ny, self.x_range, self.y_range)

    # ------------------------------------------------------------ 复位
    def reset(self) -> None:
        """清空高程图（换场地 / 重新开始 episode 时调用）。"""
        self._grid.clear()
        self._t_last = -np.inf
        self._t_pruned = -np.inf
        self.n_points_total = 0
        self.n_points_last = 0
        self.n_prunes = 0
        self.last_scan_observed = 0

    # ------------------------------------------------------------ 内部工具
    @classmethod
    def _pack(cls, i: int, j: int) -> int:
        return (i + cls._OFF) * cls._SPAN + (j + cls._OFF)

    def scan_grid(self) -> np.ndarray:
        """(35,2) 机体系采样点，与 obs.WalkObsBuilder.scan_grid() 逐点一致。"""
        return self._grid_points.copy()

    def _fresh(self, cell: Optional[list]) -> bool:
        return cell is not None and (self._t_last - cell[1]) <= self.decay_s + 1e-12

    def _prune(self, t: float, force: bool = False) -> None:
        if not force and not ((t - self._t_pruned) > self.decay_s or len(self._grid) > self.max_cells):
            return
        dead = [k for k, c in self._grid.items() if (t - c[1]) > self.decay_s + 1e-12]
        for k in dead:
            del self._grid[k]
        if len(self._grid) > self.max_cells:
            # 极端情况（一直往一个方向走）：丢掉最旧的 25%，保内存有界
            order = sorted(self._grid.items(), key=lambda kv: kv[1][1])
            for k, _ in order[: max(1, self.max_cells // 4)]:
                del self._grid[k]
        self._t_pruned = t
        self.n_prunes += 1

    # ------------------------------------------------------------ 更新
    def update(self, points_world: np.ndarray, t: float) -> None:
        """(N,3) 世界系点云 → 更新高程图（格点取 max，超时衰减）。

        NaN/Inf 点丢弃；每个点就近投到最近格点（round(x/resolution)）。
        """
        pts = np.asarray(points_world, dtype=np.float64)
        if pts.size == 0:
            pts = pts.reshape(0, 3)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("points_world 必须是 (N,3)，收到 %r" % (pts.shape,))
        t = float(t)
        self._t_last = t
        self._prune(t)

        good = np.isfinite(pts).all(axis=1)
        pts = pts[good]
        self.n_points_last = int(pts.shape[0])
        self.n_points_total += int(pts.shape[0])
        if pts.shape[0] == 0:
            return

        res = self.resolution
        i = np.floor(pts[:, 0] / res + 0.5).astype(np.int64)
        j = np.floor(pts[:, 1] / res + 0.5).astype(np.int64)
        z = pts[:, 2]
        lim = self._OFF - 2
        inside = (np.abs(i) < lim) & (np.abs(j) < lim)
        if not inside.all():
            i, j, z = i[inside], j[inside], z[inside]
        if i.size == 0:
            return

        packed = (i + self._OFF) * self._SPAN + (j + self._OFF)
        uniq, inv = np.unique(packed, return_inverse=True)
        zmax = np.full(uniq.size, -np.inf, dtype=np.float64)
        np.maximum.at(zmax, inv.reshape(-1), z)

        for key, zz in zip(uniq.tolist(), zmax.tolist()):
            cell = self._grid.get(key)
            if self._fresh(cell):
                if zz > cell[0]:
                    cell[0] = zz
                cell[1] = t
            else:
                self._grid[key] = [zz, t]

    # ------------------------------------------------------------ 查询
    def _lookup(self, ii: np.ndarray, jj: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """批量查格点，返回 (值, 是否已知)。未知（缺失或超时）处值为 0。"""
        vals = np.zeros(ii.shape[0], dtype=np.float64)
        known = np.zeros(ii.shape[0], dtype=bool)
        grid = self._grid
        span = self._SPAN
        off = self._OFF
        decay = self.decay_s + 1e-12
        t_last = self._t_last
        for k in range(ii.shape[0]):
            cell = grid.get((int(ii[k]) + off) * span + (int(jj[k]) + off))
            if cell is not None and (t_last - cell[1]) <= decay:
                vals[k] = cell[0]
                known[k] = True
        return vals, known

    def _nearest_fresh(self, i: int, j: int) -> Optional[float]:
        r = self.fill_radius_cells
        if r <= 0:
            return None
        best_d2 = (r + 1) ** 2
        best: Optional[float] = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                d2 = di * di + dj * dj
                if d2 >= best_d2:
                    continue
                cell = self._grid.get(self._pack(i + di, j + dj))
                if self._fresh(cell):
                    best_d2 = d2
                    best = cell[0]
        return best

    def _node_values(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 (双线性权重和, 加权值, 每点已知格点数)。"""
        res = self.resolution
        fx = xy[:, 0] / res
        fy = xy[:, 1] / res
        i0 = np.floor(fx).astype(np.int64)
        j0 = np.floor(fy).astype(np.int64)
        tx = fx - i0
        ty = fy - j0
        wsum = np.zeros(xy.shape[0])
        vsum = np.zeros(xy.shape[0])
        nknown = np.zeros(xy.shape[0], dtype=np.int64)
        for di, dj, w in ((0, 0, (1.0 - tx) * (1.0 - ty)),
                          (1, 0, tx * (1.0 - ty)),
                          (0, 1, (1.0 - tx) * ty),
                          (1, 1, tx * ty)):
            vals, known = self._lookup(i0 + di, j0 + dj)
            vsum += np.where(known, w * vals, 0.0)
            wsum += np.where(known, w, 0.0)
            nknown += known.astype(np.int64)
        return wsum, vsum, nknown

    def ground_height(self, xy_world: np.ndarray,
                      fallback_height: Optional[float] = None) -> np.ndarray:
        """(M,2) 世界系 xy → (M,) 地面高度，双线性插值。

        未知格点按可用权重重新归一化；整格未知时用最近已知格点保持填充；
        再无观测则取 fallback_height（默认 default_height，默认 0.0 m）。
        """
        xy = np.asarray(xy_world, dtype=np.float64).reshape(-1, 2)
        fb = self.default_height if fallback_height is None else float(fallback_height)
        wsum, vsum, nknown = self._node_values(xy)
        out = np.full(xy.shape[0], fb, dtype=np.float64)
        ok = wsum > 0.0
        out[ok] = vsum[ok] / wsum[ok]
        if (~ok).any():
            res = self.resolution
            for k in np.nonzero(~ok)[0]:
                v = self._nearest_fresh(int(np.floor(xy[k, 0] / res)),
                                        int(np.floor(xy[k, 1] / res)))
                if v is not None:
                    out[k] = v
        return out

    def observed(self, xy_world: np.ndarray) -> np.ndarray:
        """(M,) bool：该查询点是否**直接**落在至少一个新鲜格点的双线性邻域内。"""
        xy = np.asarray(xy_world, dtype=np.float64).reshape(-1, 2)
        _, _, nknown = self._node_values(xy)
        return nknown > 0

    def scan(self, base_pos: np.ndarray, base_quat_wxyz: np.ndarray) -> np.ndarray:
        """(35,) 训练口径高度扫描。

        base_pos (3,) 世界系基座位置；base_quat_wxyz (4,) wxyz。
        只按 yaw 旋转（与训练一致），d = z_base - h_ground - 0.28，
        clip(±0.35) 后 ×5.0。负 = 前方地面更高。
        """
        bp = np.asarray(base_pos, dtype=np.float64).reshape(3)
        q = np.asarray(base_quat_wxyz, dtype=np.float64).reshape(4)
        nq = float(np.linalg.norm(q))
        if nq > 1e-12:
            q = q / nq
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        g = self._grid_points
        xw = c * g[:, 0] - s * g[:, 1] + bp[0]
        yw = s * g[:, 0] + c * g[:, 1] + bp[1]
        xy = np.stack([xw, yw], axis=-1)
        self.last_scan_observed = int(self.observed(xy).sum())
        # v6: 未观测格点填"局部地面" (特征≈0), 与训练侧遮挡模型 fill 语义一致
        fb = (float(bp[2]) - self.base) if self.fill_local else None
        h = self.ground_height(xy, fallback_height=fb)
        d_m = bp[2] - h - self.base
        return np.clip(d_m, -self.clip_m, self.clip_m) * self.scale

    # ------------------------------------------------------------ 统计
    def stats(self) -> dict:
        n_fresh = sum(1 for c in self._grid.values() if self._fresh(c))
        return {
            "n_cells": len(self._grid),
            "n_cells_fresh": int(n_fresh),
            "n_points_total": int(self.n_points_total),
            "n_points_last": int(self.n_points_last),
            "t_last": None if not np.isfinite(self._t_last) else float(self._t_last),
            "decay_s": self.decay_s,
            "resolution": self.resolution,
            "base": self.base,
            "clip_m": self.clip_m,
            "scale": self.scale,
            "last_scan_observed": int(self.last_scan_observed),
            "last_scan_points": int(self._grid_points.shape[0]),
        }


class SimLidar:
    """仿真侧点云源：从 Go2Sim 的 MuJoCo 模型 ray-cast 出 (N,3) 世界系点云。

    只读使用 sim.model / sim.data / sim.base_body，不修改 robot.py，也不写 data。

    传感器安装：位于基座 xy 正上方、离**脚下地面** height_above_ground 米（"height above
    ground" 的字面语义）。地面高度由一条向下探测射线得到（射线同样屏蔽机器人本体），
    探测不到时退回标称站高 0.28 m。这样机器人上下台阶时点云的参考面跟着地面走。

    射线方向：方位角 hfov 均匀分层、仰角 vfov 均匀分层（层内抖动，受 seed 控制，
    同一 seed + 位姿 → 完全可复现）。方位 0 = 机体系 +x（前方），仰角 0 = 水平。

    注意（视场盲区，实测）：vfov 下限 -0.5 rad、安装高 0.30 m 时，最近可见地面在
    r = 0.30/tan(0.5) ≈ 0.55 m，近场（脚下 0.55 m 内）无回波，HeightMapper 会用
    最近已知格点保持填充。要覆盖近场需把 vfov 下限压到约 -1.3 rad 或降低安装高度。
    """

    def __init__(self, sim, n_rays: int = 2000, hfov: Sequence[float] = (0.0, 2 * np.pi),
                 vfov: Sequence[float] = (-0.5, 0.2), max_range: float = 8.0,
                 height_above_ground: float = 0.30, seed: int = 0, *,
                 nominal_base_height: float = NOMINAL_BASE_HEIGHT,
                 mount_body: Optional[Sequence[float]] = None,
                 min_range: float = 0.05,
                 self_occlusion: bool = False, multiray: bool = True) -> None:
        import mujoco  # 仿真侧才需要；真机路径不 import

        self._mujoco = mujoco
        self.sim = sim
        self.n_rays = int(n_rays)
        self.hfov = (float(hfov[0]), float(hfov[1]))
        self.vfov = (float(vfov[0]), float(vfov[1]))
        self.max_range = float(max_range)
        self.height_above_ground = float(height_above_ground)
        self.nominal_base_height = float(nominal_base_height)
        self.seed = int(seed)
        # v6 (真机 L1 口径):
        #   mount_body      —— 机体系安装位置 (m)。给了就用"机体固定安装"语义
        #                      (origin = base_pos + R_base @ mount_body), 否则回退到
        #                      旧的 "base xy + 0.30m above local ground" 语义。
        #   min_range       —— 小于它的回波丢弃 (L1 最小测距 0.05m / 高程图
        #                      min_valid_distance 0.10m)。
        #   self_occlusion  —— True: 射线打到机器人自身 geom 时**不产生点**
        #                      (= 被机体/腿遮挡), 而不是像旧实现那样把机器人 geom
        #                      从 ray-cast 里排除掉 (旧实现 = 无自遮挡)。
        self.mount_body = (None if mount_body is None
                           else np.asarray(mount_body, dtype=np.float64).reshape(3))
        self.min_range = float(min_range)
        self.self_occlusion = bool(self_occlusion)
        if self.n_rays < 1:
            raise ValueError("n_rays 必须 ≥ 1")

        self._dirs_body = self._make_dirs()
        self._geomgroup, self._robot_bodies = self._resolve_geom_filter()
        # 自遮挡: ray-cast 不再排除机器人 group, 改由 sample() 里"命中机器人 body
        # 就丢弃该射线"来实现遮挡语义。
        self._cast_mask = None if self.self_occlusion else self._geomgroup
        # multiray: 用 mujoco.mj_multiRay 批量投射（同一原点）。逐条 mj_ray 在 3000 条
        # 射线时 ~28 ms/帧，是观察器卡顿的主因之一；批量版语义相同、成本低一个量级。
        self.multiray = bool(multiray)
        self._bodyexclude = int(sim.base_body)
        self._gid = np.zeros(1, dtype=np.int32)
        self.n_calls = 0
        self.n_rays_cast = 0
        self.n_hits = 0
        self.n_occluded = 0

    # ------------------------------------------------------------ 构造
    def _make_dirs(self) -> np.ndarray:
        h0, h1 = self.hfov
        v0, v1 = self.vfov
        hs, vs = h1 - h0, v1 - v0
        if hs <= 0.0 or vs <= 0.0:
            raise ValueError("hfov/vfov 必须递增")
        n_el = max(1, int(round(np.sqrt(self.n_rays * vs / hs))))
        n_az = max(1, int(np.ceil(self.n_rays / n_el)))
        rng = np.random.default_rng(self.seed)
        az = h0 + (np.arange(n_az) + rng.random(n_az)) * hs / n_az
        el = v0 + (np.arange(n_el) + rng.random(n_el)) * vs / n_el
        AZ = np.repeat(az, n_el)
        EL = np.tile(el, n_az)
        k = min(self.n_rays, AZ.size)
        AZ, EL = AZ[:k], EL[:k]
        return np.stack([np.cos(EL) * np.cos(AZ),
                         np.cos(EL) * np.sin(AZ),
                         np.sin(EL)], axis=-1)

    def _resolve_geom_filter(self) -> Tuple[Optional[np.ndarray], frozenset]:
        """自动挑出"只有环境 geom"的 group，避免射线打到机器人自己身上。

        机器人树 = base_body 及其所有子孙 body。含机器人 geom 的 group 全部排除；
        若没有任何干净 group（例如整机与地面同组），退化为不过滤 + 命中后按 body 丢弃。
        """
        m = self.sim.model
        base = int(self.sim.base_body)
        robot = set()
        for b in range(int(m.nbody)):
            cur = b
            while cur != 0:
                if cur == base:
                    robot.add(b)
                    break
                cur = int(m.body_parentid[cur])
        robot_groups = set()
        all_groups = set()
        for g in range(int(m.ngeom)):
            grp = int(m.geom_group[g])
            all_groups.add(grp)
            if int(m.geom_bodyid[g]) in robot:
                robot_groups.add(grp)
        env_groups = sorted(all_groups - robot_groups)
        mask = np.array([1 if g in env_groups else 0 for g in range(6)], dtype=np.uint8)
        if not mask.any():
            mask = None  # type: ignore[assignment]
        return mask, frozenset(robot)

    @property
    def geom_group_mask(self) -> Optional[np.ndarray]:
        """ray-cast 允许的 geom group 掩码（None = 不过滤，靠命中后按 body 丢弃）。"""
        return None if self._geomgroup is None else self._geomgroup.copy()

    @property
    def robot_bodies(self) -> frozenset:
        """机器人树 body 集合（这些 body 上的 geom 永远不算点云）。"""
        return self._robot_bodies

    # ------------------------------------------------------------ 采样
    def _sensor_origin(self, base: np.ndarray) -> np.ndarray:
        if self.mount_body is not None:
            # 机体固定安装: base_pos + R_base @ mount_body (基座完整姿态)
            rot = np.asarray(self.sim.data.xmat[self.sim.base_body],
                             dtype=np.float64).reshape(3, 3)
            return base + rot @ self.mount_body
        mj = self._mujoco
        down = np.array([0.0, 0.0, -1.0])
        dist = mj.mj_ray(self.sim.model, self.sim.data, base, down,
                         self._geomgroup, 1, self._bodyexclude, self._gid)
        if dist is not None and 0.0 <= float(dist) <= self.max_range:
            ground_z = base[2] - float(dist)
        else:
            ground_z = base[2] - self.nominal_base_height
        return np.array([base[0], base[1], ground_z + self.height_above_ground])

    def sample(self) -> np.ndarray:
        """(N,3) 世界系命中点（未命中的射线被丢弃，N ≤ n_rays）。

        multiray=True 时走 mujoco.mj_multiRay 批量投射（同原点/多方向）。逐条 mj_ray
        在 3000 条射线时要 ~28 ms/帧（观察器 RTF 0.39 的主因之一），批量版语义相同。
        """
        mj = self._mujoco
        m = self.sim.model
        d = self.sim.data
        base = np.asarray(d.xpos[self.sim.base_body], dtype=np.float64).reshape(3)
        rot = np.asarray(d.xmat[self.sim.base_body], dtype=np.float64).reshape(3, 3)
        origin = self._sensor_origin(base)
        dirs = np.ascontiguousarray(self._dirs_body @ rot.T, dtype=np.float64)
        n_dir = int(dirs.shape[0])
        if self.multiray and n_dir > 0:
            gid = np.zeros(n_dir, dtype=np.int32)
            dist = np.zeros(n_dir, dtype=np.float64)
            mj.mj_multiRay(m, d, origin, dirs.reshape(-1), self._cast_mask, True,
                           self._bodyexclude, gid, dist, None, n_dir,
                           float(self.max_range))
            ok = (dist >= self.min_range) & (dist <= self.max_range)
            gid, dist, dirs_ok = gid[ok], dist[ok], dirs[ok]
            if len(self._robot_bodies) and len(dist):
                bodies = np.asarray(m.geom_bodyid)[gid]
                occ = np.isin(bodies, np.fromiter(self._robot_bodies, dtype=np.int64))
                self.n_occluded += int(np.count_nonzero(occ))
                dist = dist[~occ]
                dirs_ok = dirs_ok[~occ]
            hit = origin + dirs_ok * dist[:, None]
            self.n_calls += 1
            self.n_rays_cast += n_dir
            self.n_hits += int(hit.shape[0])
            return hit
        # 逐条回退路径（老版本语义, 用于对照/调试）
        hit = np.empty((n_dir, 3), dtype=np.float64)
        n = 0
        for k in range(n_dir):
            dist = mj.mj_ray(m, d, origin, dirs[k], self._cast_mask, 1,
                             self._bodyexclude, self._gid)
            if dist is None:
                continue
            dist = float(dist)
            if dist < self.min_range or dist > self.max_range:
                continue
            g = int(self._gid[0])
            if g >= 0 and int(m.geom_bodyid[g]) in self._robot_bodies:
                self.n_occluded += 1     # v6: 被自身机体/腿挡住 -> 该方向无回波
                continue
            hit[n] = origin + dirs[k] * dist
            n += 1
        self.n_calls += 1
        self.n_rays_cast += n_dir
        self.n_hits += n
        return hit[:n].copy()

    def stats(self) -> dict:
        return {
            "n_rays": self.n_rays,
            "n_calls": self.n_calls,
            "n_rays_cast": self.n_rays_cast,
            "n_hits": self.n_hits,
            "hit_rate": (self.n_hits / self.n_rays_cast) if self.n_rays_cast else 0.0,
            "height_above_ground": self.height_above_ground,
            "max_range": self.max_range,
            "hfov": self.hfov,
            "vfov": self.vfov,
            "seed": self.seed,
            "mount_body": (None if self.mount_body is None
                           else [round(float(v), 5) for v in self.mount_body]),
            "min_range": self.min_range,
            "self_occlusion": self.self_occlusion,
            "multiray": bool(self.multiray),
            "n_occluded": int(self.n_occluded),
        }
