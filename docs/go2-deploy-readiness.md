# Go2 部署侧就绪报告 (task-5)

> 目标: 把"训练出的 Go2 策略放进部署框架就能闭环跑"这条链路打通, 并量化"部署物理 vs 训练物理"的差别。
> 全部 CPU 实测 (mujoco 3.12.0 / CPU Newton), 未使用 GPU。
> 训练侧文件: `mjx-go2-training/models/go2/go2_mjx_position.xml` (未改动, 只读)。
> 本任务**没有修改** mjx-go2-training 下的任何文件。

---

## 0. 一页结论

| # | 事项 | 结论 |
|---|---|---|
| 1 | 部署侧评测物理统一到训练物理 | ✅ `config/go2_sim.json` 的 `robots.go2` 档默认 `foot_soft_contact=true` + `solver 100/50`; `run_sim.py`/`probe_policy_go2.py` 改成三态 flag (显式 > config > 框架默认)。**go2 逐项不变** (§2) |
| 2 | 站姿换成能站住的开源惯例姿态 | ✅ go2 标称站姿 = unitree_rl_gym home + 出生 z=0.32。实测 10 s 稳定: z=0.2808 m, 倾角 0.89° (旧姿态 0.1998 m / 6.82°)。**go2 仍取自己的 keyframe** (§3) |
| 3 | 端到端闭环 | ✅ 部署框架 (motor+PD) 闭环已跑通; **接口互检 max|Δobs| = 0.000e+00** (91 维逐段全 0)。策略本身还是 smoke 检查点 (221k 步, 未训练), 数字待正式策略重跑 (§6) |
| 4 | height_scan 来源 | ⚠️ **leg_fk 给的是"单个标量", 35 个分量完全相同, 没有任何空间信息**; 与引擎真值比 31.5% 的分量差 > 2 cm (p95 7.2 cm, max 21.9 cm)。raycast(HeightMapper) 中位误差 1.6 mm 但近场盲区导致 16.8% 仍 > 2 cm (§5) |

---

## 1. 改动清单 (逐处 + go2 为何不受影响)

| 文件 | 改动 | go2 为何不受影响 |
|---|---|---|
| `config/go2_sim.json` | `robots.go2` 新增 `stand_pose` / `stand_height` / `foot_soft_contact` / `solver_iterations` / `solver_ls_iterations` / `scan_base` / `home_q`; 新增 `policy_profiles.{go2,go2}` + `policy_profile=auto` | `robots.go2` 段**只显式写出与 obs.py 既有默认逐位相同的 `scan_base=0.28` / `home_q`**, 不写 stand_pose/foot_soft_contact/solver_*; 顶层 `policy` 块一字未动 |
| `src/go2_bridge/robot.py` | `RobotSpec` 新增 `stand_pose_from_xml=True` / `stand_height=None`; `Go2Sim` 只在 `stand_pose_from_xml` 为真时才用 XML keyframe 覆盖站姿; `reset()` 优先用显式 height | 两个新字段的默认值就是原有行为 (True/None) ⇒ go2 路径**代码路径完全一致** |
| `src/go2_bridge/config.py` | `load_config` 把 `robots.<name>` 的 6 个键提到 cfg 顶层; 解析 `policy_profile` (auto = 当前 robot) 并把选中档 merge 进 `cfg["policy"]`; 导出 `cfg["_train_scan_base"] / ["_train_home_q"]` | go2 档 = `policy_profiles` 里没有 go2 键时直接用顶层 `policy` 块 (本仓库就没写 go2 键) ⇒ 逐字不变 |
| `src/go2_bridge/node.py` | `SimNode` 新增 `solver_ls_iterations=None` 透传给 `Go2Sim` | 新参数默认 None ⇒ 不传给模型, 行为不变 |
| `scripts/run_sim.py` | `--soft-feet/--hard-feet` 改三态 (`default=None`), 新增 `--iters/--ls-iters`; 打印站姿/接触/求解器 | `--robot go2` ⇒ cfg 的 `foot_soft_contact=False` ⇒ 与改动前同为硬接触; 求解器同为 100/50 (实测见 §2) |
| `scripts/probe_policy_go2.py` | 同三态 flag + `--ls-iters`; height_scan base 用 `cfg["_train_scan_base"]` (=0.28/0.268); obs 的 home 段用 `cfg["_train_home_q"]`; 出生姿态用被测机型 stand_pose | `--robot go2` ⇒ `_train_scan_base=0.28`、`_train_home_q` 与原来的 `TRAINING_HOME_Q` 逐位相同, 软接触/迭代也同为 False/None |
| `scripts/probe_policy_controller.py` | go2 走 config 档, go2 **保留硬编码软接触**; 新增 `--policy-profile` | 明确分支: `if args.robot == "go2"` 才读 config, go2 走 else 原值 |
| `scripts/run_controller.py` | obs 的 `scan_base`/`home_q` 按 **train_robot** 取; 新增 `--policy-profile`; 借模型的 `Go2Sim` 在 config 给了 `stand_pose` 时带上 spec —— 于是**安全监督器的下蹲姿态跟着新站姿走** (修前恒为旧 keyframe 的 `(0,0.9,-2.15)`) | 没给 `stand_pose` (go2) 时 `spec=None`, 与改动前逐字相同; 实测 go2 下蹲姿仍是 `[0,0.8,-1.85]`; `train_robot=go2` 时 obs 常量亦逐位相同 |
| **新增** `scripts/check_go2_stand.py` | 站姿可站立性自检 (§3) | 只读, 只跑 go2 |
| **新增** `scripts/compare_deploy_vs_train_physics.py` | 部署物理 vs 训练物理对照 (§6) | 只读, 不改其它文件 |
| **新增** `scripts/probe_height_scan_sources.py` | leg_fk vs 真值 vs raycast (§5) | 只读 |
| **新增产物** `models/policies/go2_smoke.npz/.json/.equiv.npz` | smoke 策略的纯 numpy 导出 (`scripts/export_go2_policy.py` 本身机型无关) | 新文件, 不动 go2 的 npz |

**go2 回归证据** (改动后实测):

```
$ python scripts/run_sim.py --robot go2 --target sim_udp --duration 2 --no-realtime
[sim] robot = go2 | 力矩上限 [23.7, 23.7, 35.55] (hip/thigh, knee)
[sim] 站姿   = [0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 1.0, -1.5, 0.0, 1.0, -1.5] | 出生 z = None
[sim] 足端接触 = XML 原始(硬 condim=6) | 求解器 = 100 / 50 | 信号劣化 = sim_clean
```

`出生 z = None` 表示沿用 go2 keyframe 的 z; 站姿 = go2 keyframe 的 (0,0.8,-1.5)/(0,1.0,-1.5);
软接触 False、100/50 —— 与改动前逐项相同。

---

## 2. 接触 / 求解器档位 (item 1)

**为什么必须统一**: 训练模型的足端是 `condim=3 + solimp=[0.9,0.95,0.023,0.5,2.0]` (软接触);
部署模型 `go2.xml` 自带 `condim=6 + MuJoCo 默认 solimp` (硬接触)。
`docs/go2-train-deploy-parity.md` §5 实测: 不统一会让沉降关节角差 7.84°、0.5 s 小信号漂移从
0.0152 rad 涨到 0.209 rad (13 倍)。改了之后两项都回到 0.05° / 0.0067 rad。

**求解器档位的选择**: 训练真实跑的是 MJX warp 的 2/5 次迭代, CPU Newton **复刻不了**这个语义。
实测把 2/5 直接搬到 CPU: 站姿在 8.28 s 时 `QACC = NaN` 发散 (倾角冲到 173°, 见 §3 表)。
所以 CPU 评测统一取**收敛**的 100/50, 并把这一点写进 config 注释, 不留歧义。

```
$ python scripts/run_sim.py --target sim_udp --duration 2 --no-realtime
[sim] 站姿   = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5] | 出生 z = 0.32
[sim] 足端接触 = soft(训练同款 condim=3) | 求解器 = 100 / 50 | 信号劣化 = sim_clean
```

优先级: **命令行显式 flag > config 的 robots.<robot> 段 > 框架默认**。
`--hard-feet` 可以随时退回部署 XML 原始接触去复现"改之前"的行为。

---

## 3. 站姿 (item 2)

**问题**: 部署 `go2.xml` 的 keyframe home = 每腿 `(0, 0.9, -1.8)` 是**真机出厂站立姿态**, 不是训练姿态。
kp=20/kd=1 下站不住 (沉降到 0.20 m 且继续掉)。

**修复**: `config/go2_sim.json` 的 `robots.go2.stand_pose` =
FR(-0.1, 0.8, -1.5) / FL(+0.1, 0.8, -1.5) / RR(-0.1, 1.0, -1.5) / RL(+0.1, 1.0, -1.5), `stand_height=0.32`。
关节顺序固定为 `MOTOR_JOINTS = FR,FL,RR,RL × (hip,thigh,calf)`。

```bash
$ python scripts/check_go2_stand.py --duration 10 --json /tmp/go2_stand.json
```

| 姿态 / 接触档 | z_end | z_min | tilt_end | tilt_max | drift_x | 10 s 内站住? |
|---|---|---|---|---|---|---|
| 部署 keyframe home (0,0.9,-1.8) + 硬接触 **[改前默认]** | 0.1998 | 0.1997 | 6.82° | 7.14° | −0.0976 | 勉强 (z 贴着下界 0.19) |
| unitree_rl_gym home + 硬接触 | 0.2810 | 0.2811 | 0.63° | 1.66° | −0.0540 | 是 |
| **unitree_rl_gym home + 软接触 [改后默认]** | **0.2808** | 0.2809 | **0.89°** | **1.58°** | −0.0519 | **是** |
| unitree_rl_gym home + 软接触 + 2/5 迭代 (MJX warp 档) | 0.1987 | 0.2042 | 19.23° | **173.04°** | −0.1538 | **否, 8.28 s 发散** |
| config 实际取值 (照抄 cfg) | 0.2808 | 0.2809 | 0.89° | 1.58° | −0.0519 | 是 |

改后 10 s 时间序列 (新默认档): z 稳定在 0.2808–0.2815 m, 倾角 0.77° → 0.88° 缓慢收敛, xy 漂移 −5 cm 后停住。

---

## 4. obs 的四个常量 (Lead 必查清单)

Lead 已逐项核对: 91 = 48+35+8 布局 / 网格 nx=7 ny=5 x∈[−0.3,0.9] y∈[−0.4,0.4] meshgrid(ij) x 外层 /
trot=[0,π,π,0]↔[FR,FL,RR,RL] / 步频 1.8 Hz / 相位归一化 [−π,π) / 动作=12 维绝对位置目标经 ctrlrange clamp —— 全部一致。
剩下的 4 个常量:

| # | 常量 | 改前 | 改后 | 落在哪里 |
|---|---|---|---|---|
| 1 | `height_scan.base` | 0.28 (Go2 值) | **0.268** | `config/go2_sim.json` 的 `robots.go2.scan_base` → `run_controller.py` 构造 `WalkObsConfig(scan_base=...)` |
| 2 | `home_q` (obs 12:24 段) | Go2 home (hip=0) | **Go2 训练 home (hip=∓0.1, thigh 0.8/1.0, calf −1.5)** | `robots.go2.home_q` → `WalkObsBuilder(..., home_q=...)` |
| 3 | 策略档 | robot=go2 却用 go2 策略且 `train_robot=go2` | **`policy_profiles.go2`: npz=go2 策略, train_robot=go2, kp=20, kd=1, action_clip=true** | `config/go2_sim.json`; `auto` 按 `--robot` 选档, `--policy-profile go2` 可退回"Go2 策略跑 Go2 机器人"那套移植验证 |
| 4 | height_scan 来源 | `leg_fk` | 见 §5 (语义 + 怎么切 raycast + 实测差距) | `robots.go2` 的 `policy.height_scan_source` |

分布类型 (第 3 项): 训练 `train_go2.py` 用 `distribution_type="tanh_normal"`, 但 `eval_go2.py` 对 pkl 的自动识别给出 `dist=normal`。
**判据以 eval_go2.py 的自动识别为准, 不要手工套 tanh** —— `export_go2_policy.py` 的 numpy 前向用的也是
`NormalDistribution.mode() == loc` (无 tanh), 并且与 jax 前向逐元素对到 `1.998e-15` (见 §6), 这条链路自洽。

改后实测 (部署框架闭环, `probe_policy_controller.py --robot go2`):

```
[ctrl] 训练机型  = go2 | 策略档 = go2 | 动作夹紧范围 [-1.047 -1.571 -2.723 ...] -> [1.047 3.491 -0.838 ...]
[ctrl] obs home  = [-0.1  0.8 -1.5  0.1  0.8 -1.5 -0.1  1.  -1.5  0.1  1.  -1.5] | height_scan.base = 0.268
```

---

## 5. height_scan: leg_fk 到底给的是什么 (item 4)

### 5.1 语义

| | 定义 | 代码 |
|---|---|---|
| **训练口径 (真值)** | `feat_i = clip(z_base − h_ground(xy_i) − 0.268, ±0.35) × 5`, `xy_i` 是 35 个逐点采样位置 (按 yaw 旋到世界系) | `go2_walk.py:714` / `eval_go2.py::height_scan` |
| **部署默认 leg_fk** | `zc = median(−foot_z_body + r)` (取**接触腿**的 FK), 然后 `feat_i = clip(zc − 0 − 0.268, ±0.35) × 5` —— **35 个点填同一个数** | `controllers.py:205-251` |
| **raycast (有激光时)** | `SimLidar` 点云 → `HeightMapper.update()` (世界系稀疏格网, res 0.05 m) → `HeightMapper.scan()` 逐点查 → 同训练公式 | `perception.py` |

leg_fk 的代码注释自己写明了: *"平地假设：35 个采样点的地面高度都是 0（真地形走 perception.HeightMapper）"*
—— 它假设地面是平的, 所以输出必然是常数向量。

### 5.2 实测 (81 个地形位置 × 35 维, 每处先站 0.2 s 让足端接触)

```bash
$ python scripts/probe_height_scan_sources.py --grid 9 --json /tmp/go2_scan_sources.json
```

| | 特征 mean | std | min | max | 与真值差 p50 | p95 | max | 折成米 p50 / p95 / max | > 2 cm 占比 |
|---|---|---|---|---|---|---|---|---|---|
| **引擎真值** | 0.060 | **0.166** | −0.991 | +1.009 | — | — | — | — | — |
| **leg_fk** | 0.106 | 0.041 | 0.012 | 0.191 | 0.050 | 0.361 | 1.093 | 1.0 / 7.2 / 21.9 cm | **31.5%** |
| **raycast** | 0.120 | 0.283 | −1.067 | +1.457 | **0.008** | 0.367 | 1.500 | **0.16** / 7.3 / 30.0 cm | 16.8% |

* `leg_fk` 在**同一个位置**的 35 个分量互相之间的最大极差 = **`0.000e+00`** —— 它是常数向量,
  等于把"我大概多高"这一个标量复制 35 份。真值在同位置的跨分量极差在 std 0.166 量级 (约 16 cm)。
  ⇒ **地形上策略拿不到任何"前方多高"的信息, 这不是噪声而是信息缺失**。
* raycast 中位数好三个数量级 (1.6 mm), 但尾巴很重: `last_scan_observed = 17/35` —— 35 个采样点里
  只有 17 个落在被观测到的格点内, 其余走 `hold/default=0` 兜底。根因是 `SimLidar` 注释里写明的近场盲区:
  `vfov` 下限 −0.5 rad、安装高 0.30 m ⇒ 最近可见地面在 0.55 m 外。

### 5.3 怎么切到 raycast

单进程仿真 (可直接用, 已数值验证):

```python
from go2_bridge.perception import HeightMapper, SimLidar
hm = HeightMapper(base=0.268)                        # base 必须跟 scan_base 一致
lidar = SimLidar(sim, n_rays=2000, vfov=(-1.3, 0.2),     # 近场要覆盖, 见下
                 max_range=8.0, height_above_ground=0.30, nominal_base_height=0.268)
def height_scan_fn(state, t):                       # 挂到 PolicyController(height_scan_fn=...)
    hm.update(lidar.sample(), t)
    return hm.scan(sim.data.qpos[:3], sim.data.qpos[3:7])
```

`PolicyController` 一拿到 `height_scan_fn` 就直接用它 (`_scan_source` 标 `external`), 不再走 leg_fk ——
这个 hook 是现成的 (`controllers.py:126/234`)。

⚠️ **ROS2 / 真机路径**: 控制器进程里**没有** MuJoCo 仿真对象, 所以不能像上面那样直接 `SimLidar(sim)`。
真机上 `height_scan_fn` 必须由**传感器**喂: PointCloud2 订阅回调 → `HeightMapper.update(points_world, t)` →
`HeightMapper.scan(qpos[:3], qpos[3:7])`。`HeightMapper` 接口本身就是按这个设计的 (与训练同定义:
yaw-only 旋转 / base / clip / ×5 全在类里)。这一步**尚未接线**, 见 §8 R3。

---

## 6. 端到端 (item 3)

### 6.1 接口互检: 部署模型的 obs == 训练模型的 obs (逐位)

用 `eval_go2.py` 自己的 `Go2Sim` 在两个模型上摆到**同一物理状态**, 比较 91 维 obs:

```
dof_order  : train=model / deploy=actuator (置换已应用)
max|Δobs|  : 0.000e+00
逐段       : linvel 0 / angvel 0 / gravity 0 / cmd 0 / q-home 0 / dq 0 / last_act 0 / scan 0 / phase 0
-> 接口一致 ✓
```

这是"策略放进部署模型不会静默错位"的直接证据 —— 部署模型 body 树是 `FL,FR,RL,RR` 而观测/动作需要
`FR,FL,RR,RL`, 置换一旦错就会在这里暴露。**全 0 说明置换是对的。**

### 6.2 策略闭环: 部署物理 vs 训练物理 (同一份判据)

```bash
$ python scripts/compare_deploy_vs_train_physics.py --seeds 8 --json /tmp/go2_parity_eval.json
```

判据**不重写** —— 直接 subprocess 调 `sim/eval_go2.py`, 只换 `--xml` / `--foot-contact`,
其余参数逐字相同 (`--solver cpu --home-pose unitree_rl_gym --spawn origin --spawn-z 0.32 --reset-noise 0.1`)。

**(a) 无策略对照 (`--dummy hold_home`)** —— 去掉策略变量, 只量两套物理 (8 轮 × 15 s):

| 指标 | 训练物理 (position) | 部署物理 (motor+PD, 软接触=改后) | 部署物理 (硬接触=改前) |
|---|---|---|---|
| 存活率 (env 判据) | 100% | 100% | 100% |
| 躯干离地高度 z | 0.2684 m | 0.2691 m | 0.2714 m |
| 与训练的高度差 | — | **+0.7 mm** | +3.0 mm |
| pitch 均值 | +1.47° | +0.01° | +1.23° |
| 角速度 RMS | 0.060 rad/s | 0.059 | 0.058 |
| **|tau| 峰值** | 9.07 N·m | 9.48 N·m | 9.00 N·m |
| 前进距离 | −0.052 m | −0.065 m | −0.048 m |

⇒ **纯站姿下两套物理只差 0.7 mm**, 与 `docs/go2-train-deploy-parity.md` 证到的执行器逐位等价一致;
把接触档换成软接触后, 与训练的高度差从 3.0 mm 收敛到 0.7 mm (item 1 的直接收益)。
另注: 训练模型 0.2684 m 与 Lead 给的 0.267 m 口径吻合, 口径问题闭环。

**(b) 策略闭环 (go2_smoke.pkl, 221k 步未训练, 8 轮 × 15 s)**:

| 指标 | 训练物理 (position) | 部署物理 (motor+PD) |
|---|---|---|
| 存活率 (env 判据) / (tilt 判据) | 0.0% / 100% | 0.0% / 100% |
| 平均跌落时间 | 0.525 s | 0.645 s |
| vx 均值 | −0.272 m/s | −0.366 m/s |
| 稳态跟踪误差 | 0.963 m/s | 0.993 m/s |
| 躯干离地高度 | 0.2894 m | 0.2790 m |
| pitch 均值 / 最大 | +6.99° / 24.0° | +0.83° / 23.4° |
| 角速度 RMS | 2.57 rad/s | 4.60 rad/s |
| **|tau| 峰值** | 23.70 N·m | 41.49 N·m |
| 前进距离 | −0.155 m | −0.246 m |
| 地形分层通过率 | h∈[0,0.05) 8/8 到达, 0% 通过 | 同左 |

地形档 (同一份 hfield, 部署 body + 训练 terrain.png) 结论同形: 存活 0%/0%, 跌落 0.617/0.628 s,
前进 −0.254/−0.264 m, 分层通过率 0%/0%。

**这份数字不能用来下"部署物理造成多少性能损失"的结论** —— 策略在 0.5 s 内两侧都倒了,
测到的全是"未训练策略乱甩"的混沌差异 (|tau| 峰值 23.7 vs 41.5 也只是两条不同崩溃轨迹的峰值)。
链路是通的, **有效数字等正式策略 (§8 R1)**。

### 6.3 部署框架自己的闭环 (motor + LowCmd + PD)

```bash
$ python scripts/probe_policy_controller.py --robot go2 --duration 8 --json /tmp/probe_ctrl_go2.json
```

* 站姿/出生: `[-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,1.0,-1.5, 0.1,1.0,-1.5]`, z=0.32, 软接触, 100/50 ✅
* 策略: `models/policies/go2_smoke.npz` (obs 91), `train_robot=go2`, 动作夹紧范围 = Go2 关节范围 ✅
* obs: `home_q` = Go2 训练 home, `height_scan.base` = 0.268 ✅
* 安全监督器: `crouch_pose` 跟着新站姿 = `[-0.1,0.8,-1.85, 0.1,0.8,-1.85, -0.1,1.0,-1.85, 0.1,1.0,-1.85]` (修前恒为 `[0,0.9,-2.15]` 的旧 keyframe 姿态)
* 预摆 1.5 s (75 拍) 稳定: z 0.32 → 0.281, 倾角 0.38°; 1.5 s 交给策略后 2.4 s 起倾角爬到 19°、4.8 s 翻倒
  —— 与 eval_go2 上 smoke 策略 0.4–0.6 s 倒是一致的 (smoke = 221k 步, 目标 40M)。

策略导出 (纯 numpy, 部署侧不依赖 jax/brax):

```bash
$ python scripts/export_go2_policy.py --pkl <getup>/policies/go2_smoke.pkl \
      --out models/policies/go2_smoke.npz --name go2_smoke
[export] obs_dim = 91 (privileged 162), layers = [128, 128]
[export] numpy(f64) vs jax(f64)          max|diff| = 1.998e-15   <- 公式正确
[export] numpy(f64) vs jax(f32,highest)  max|diff| = 1.451e-06   <- 同精度等价
[export] 错误地套上 normalize 的偏离       max|diff| = 3.006e+00   <- 千万别这么做
[export] 等价性: PASS
```

`scripts/export_go2_policy.py` 本身是机型无关的: obs 维度/层宽从权重形状反推, `--pkl/--out/--name` 都可传,
不需要为 Go2 另写一个导出器。

---

## 7. 复现命令汇总

```bash
cd /home/tc63/mujoco/mjx-go2-mujoco2ros2
export XLA_PYTHON_CLIENT_PREALLOCATE=false XLA_PYTHON_CLIENT_MEM_FRACTION=0.15 JAX_PLATFORM_NAME=cpu
PY=/home/tc63/mujoco/.venv/bin/python

# item 2 站姿可站立性
$PY scripts/check_go2_stand.py --duration 10 --json /tmp/go2_stand.json

# item 1 接触/求解器档位: 看启动那几行
$PY scripts/run_sim.py --target sim_udp --duration 2 --no-realtime           # go2 新默认
$PY scripts/run_sim.py --robot go2 --target sim_udp --duration 2 --no-realtime   # go2 回归

# 策略导出 (纯 numpy)
$PY scripts/export_go2_policy.py --pkl /home/tc63/mujoco/mjx-go2-training/policies/go2_smoke.pkl \
    --out models/policies/go2_smoke.npz --name go2_smoke

# 部署框架闭环 (motor + LowCmd + PD)
$PY scripts/probe_policy_go2.py --robot go2 --duration 8 --linvel oracle
$PY scripts/probe_policy_controller.py --robot go2 --duration 8 --json /tmp/probe_ctrl_go2.json

# item 3 部署物理 vs 训练物理 (判据 = eval_go2.py)
$PY scripts/compare_deploy_vs_train_physics.py --seeds 8 --json /tmp/go2_parity_eval.json

# item 4 height_scan 来源对照
$PY scripts/probe_height_scan_sources.py --grid 9 --json /tmp/go2_scan_sources.json
```

**正式策略到位后只需要换策略路径**:

```bash
$PY scripts/compare_deploy_vs_train_physics.py \
    --policy /home/tc63/mujoco/mjx-go2-training/policies/go2_walk_policy.pkl \
    --seeds 20 --json /tmp/go2_parity_eval_walk.json
# 并把 config/go2_sim.json 的 policy_profiles.go2.npz 换成导出的正式 npz
```

---

## 8. 残余风险 / 待办

| # | 风险 | 现状 | 建议 |
|---|---|---|---|
| **R1** | **策略质量未知** | go2_smoke 是 221k/40M 步的早期检查点, 两侧都在 0.5 s 内倒; "部署物理 vs 训练物理"的性能差**还没有有效数字** | 正式 `go2_walk_policy.pkl` 到位后按 §7 最后一段重跑 (Lead 会通知) |
| **R2** | **leg_fk 的 35 维扫描是常数向量** | 地形上 31.5% 的分量 > 2 cm 偏差 (p95 7.2 cm)。策略在地形上等于"闭着眼" | 训练若依赖 height_scan, 部署侧必须走 raycast/点云; 至少把 `height_scan_source` 的语义写清 (已写进 config) |
| **R3** | **raycast 没接到 ROS2/真机** | `PolicyController(height_scan_fn=)` 是现成 hook, 但 `run_controller.py` 里没有建它 (控制器进程里没有仿真对象, 必须由 PointCloud2 喂) | 接一个 PointCloud2 → `HeightMapper.update()` 回调; 并把 `HeightMapper(base=0.268)` 与训练对齐 |
| **R4** | raycast 近场盲区 | `vfov` 下限 −0.5 rad / 安装高 0.30 m ⇒ 0.55 m 内无回波, 35 点里只有 17 点被直接观测, 其余走 hold/0 兜底 (p95 7.3 cm, max 30 cm) | 把 `vfov` 下限压到 ≈ −1.3 rad 或降低安装高; 并给"未观测格点"更好的先验 (而不是 0) |
| **R5** | 求解器语义差 | 训练是 MJX warp 2/5, CPU 一律 100/50 | 已写进 config 注释; 若要验证训练物理本身, 得在 GPU 上跑 MJX (本任务按约束未用 GPU) |
| **R6** | 部署 XML 无足端 site | eval_go2 报 `footsites=geom-fallback`, 足高/滑移统计有 ~2 cm 系统偏差 (生存/速度/倾角指标不受影响) | 若要用足端高度类指标, 给部署模型补 4 个足端 site (或按 FK 换算) |
| **R8** | go2 的 SafetySupervisor 力矩上限用的是框架默认值 (knee 45.43) 而不是 go2 真实值 (35.55) | 既有行为, 本任务**刻意没改** (改了会动 go2 的默认值); go2 因为默认值恰好等于真实值 (45.43) 所以无影响 | 若要修: 同样给借模型的 `Go2Sim` 带上 `robot_spec_of(cfg)`, 但这会改 go2 的安全层数值, 需你点头 |
| **R7** | 部署模型 body 树顺序 ≠ 电机顺序 | 靠 `--dof-order actuator` 置换兜住 (互检 0.0) | 任何新写的 obs/动作代码都必须按**关节名**索引, 不要按下标 |


