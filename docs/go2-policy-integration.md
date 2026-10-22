# Go2 策略接入 mjx-go2-mujoco2ros2 + 可视化检查

> 结论: 策略已经接进框架的闭环 (config → npz → 观测契约)，命令行一条就能看:
> `bash scripts/view_go2_policy.sh`
> 实测 15 s 走 6.63 m, 躯干 z 0.297 m, tilt 2.52°, 健康等级 L0, 0 次降级。

---

## 1. 接进去了什么

| 项 | 值 |
|---|---|
| 策略 | `models/policies/go2_walk_policy.npz` (纯 numpy, 来自 `policies/go2_walk_policy_hard.pkl`, 37.26M 步) |
| 配置档 | `config/go2_sim.json` → `robots.go2` + `policy_profiles.go2` (`policy_profile=auto` 按 `--robot go2` 选中) |
| 训练机型 | `train_robot: "go2"` (关节范围 / 观测 home / scan_base 全按它取) |
| home_q | `[-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,1.0,-1.5, 0.1,1.0,-1.5]` (= 训练 XML keyframe ctrl) |
| scan_base | 0.268 (Go2 是 0.28) |
| 关节序 | `FR,FL,RR,RL × (hip,thigh,calf)` —— 与训练执行器序一致; 部署模型 body 树是 `FL,FR,RL,RR`, 由 `--dof-order actuator` 侧做置换 |
| 执行器 | 策略输出 12 维**绝对位置目标** → 框架按 `tau = kp·(q_des−q) + kd·(dq_des−dq)`, kp=20 kd=1 |
| 足端接触 | **硬接触** (`foot_soft_contact: false`) —— v2 策略就是硬接触训出来的 |

## 2. 这一轮修掉的两个**真问题**

### 2.1 观测 0:3 的线速度用了世界系, 而 Go2 策略要机体系

- `envs/go2_walk.py` 用 `command_config.frame="body"` → obs 0:3 = `R^T @ v_world`
  (当初为了支持 vy/wz 指令必须用机体系, 否则"边走边转"不可实现)。
- 框架 `obs.py` 原来写死 `base_lin_vel_world` (= Go2 的 world 语义)。
- 平地小偏航时两者几乎相同, **所以它照样能走** —— 但一转向就会看到错的速度, 会明显退化。

修法: `WalkObsConfig.linvel_frame` (`world`/`body`), 按 `train_robot` 取
(go2→world, go2→body), 在 `obs.observe()` 里用 IMU 四元数把世界系速度转到机体系。
`config/go2_sim.json` 里 `robots.go2.linvel_frame="body"`, `robots.go2.linvel_frame="world"`。

### 2.2 曲线图里"估计 vx"恒为 0 (看着像里程计全废)

`viz/frame.py` 的 `FrameCollector.collect()` 只填了 `base_lin_vel_truth`, **从没填
`base_lin_vel_world`** → 汇总图的"估计 vx"是一条 0 线, 误差恒等于 −真值。
现改为取控制器实际喂给策略的 `ctrl._vel`。
修完: `vx_est_mean = 0.443`, `vx_err_mean_abs = 0.022` (之前是 0 和 0.458)。

## 3. 怎么用

```bash
cd mjx-go2-mujoco2ros2

# ① 3D 窗口 + 曲线 + 仪表盘 (WSLg 下 DISPLAY=:0 可用, 实测渲染 428 fps)
bash scripts/view_go2_policy.sh

# ② 长一点 / 无 GUI / 障碍场景 / 落盘
bash scripts/view_go2_policy.sh --duration 30
bash scripts/view_go2_policy.sh --headless
bash scripts/view_go2_policy.sh --obstacles
bash scripts/view_go2_policy.sh --record logs/go2_view     # CSV + report.md + summary.png

# ③ 换指令 (机体系: vx m/s, vy m/s, wz rad/s) —— 训练覆盖 vx∈[0,1.0], vy±0.4, wz±1.5
bash scripts/view_go2_policy.sh --cmd 0.3 0.2 0.5
```

窗口键盘: `r` 重置 · 空格 暂停 · `h` 3D 叠加 · `f` 曲线 · `i` 仪表盘 · `q` 退出。

### 3.1 关键开关为什么这么设

| 开关 | 值 | 理由 |
|---|---|---|
| `--linvel` | **oracle** | 训练 obs 0:3 就是 `qvel[0:3]` 真值; 仿真里有真值就该用真值, 否则把"里程计误差"混进了"策略能力"的测量 |
| `--scan` | **nominal** | 平地档: 特征 = `clip(z−0−0.268, ±0.35)×5 ≈ 0`, 与训练平地一致。`leg_fk` 是**恒定标量**(35 个点同值, 零地形信息)且带 −0.12 偏置 |
| `--scene` | flat | `obstacles` 场景没有接雷达真值, 35 维扫描仍是 nominal → 看不出地形信息, 只能算"盲走"检查 |

## 4. 实测数字 (15 s, 指令 vx=0.5)

| 指标 | 值 |
|---|---|
| 前进 | x 0 → **6.63 m** (平均 0.44 m/s) |
| 躯干高度 | 均 0.289 m (0.252–0.320) |
| 倾角 | 最大 9.27°, RMS 3.20° |
| 速度跟踪 | `vx_est_mean = 0.443`, 平均绝对误差 **0.022 m/s** |
| 力矩 | `|tau|max = 32.5 N·m` (关节上限 23.7/45.4) |
| 动作 | `|a|max = 3.18 rad`, 653/675 步被夹到关节范围 (训练侧同样如此, 属已知现象) |
| 健康 | L0 全程, 0 次降级, 0 次安全层切换 |

## 5. 还没接的部分 (真机路径, 与本次可视化无关)

1. **高度扫描来自雷达**: 现在 `height_scan_source` 只有 `nominal` / `leg_fk`, 后者是恒定标量。
   单进程下 `SimLidar + HeightMapper(base=0.268)` 已数值验证 (见
   `docs/go2-deploy-readiness.md` §5), 但 **ROS2/真机路径还没接线** ——
   控制器进程里没有仿真对象, 必须由 `PointCloud2 -> HeightMapper.update` 喂进
   `PolicyController(height_scan_fn=...)` 这个现成 hook。
2. **速度估计**: `--linvel estimator` 走 `LegOdometry`, 实测会发散 (真值 0.5 时估出 −0.18)。
   真机只能用这条, 所以这是上真机前必须调通的一项 (接触判据 `foot_force ≥ 30 N` 在真机上是否可靠)。
3. **工程链路**: `run_controller.py --robot go2 --linvel estimator` + ROS2 话题已经在框架里,
   但还没在真机/RTF=1 的 ROS2 回路上联调过。
