# 训练 ↔ 部署一致性

本仓与训练仓没有包级依赖，耦合是"训练产物单向导出 + 同一契约两侧各实现一次"。
本文汇总当前已验证的三类一致性：**执行器语义、观测/动作、物理参数**。

## 0. 结论速览

| 问题 | 结论 | 验证脚本 |
|---|---|---|
| 部署的 `<motor>` + PD 是否等价于训练的 `<position kp=20 kv=1>`？ | **严格等价**：1500 步后 `max|Δq| = 1.1e-15 rad`、`max|Δτ| = 4.6e-14 N·m` | `verify_train_deploy_parity.py` |
| 部署观测是否就是训练观测？ | **逐段 `max|diff| = 0`**（243 维，含随机 roll/pitch 0.34 rad） | `verify_train_deploy_obs.py` |
| 策略输出是否一致？ | jax(pkl) vs numpy(npz) 12 维动作 **`max|diff| ≤ 1.15e-6`** | `verify_train_deploy_obs.py` |
| 导出的 npz 本身有没有精度损失？ | f32 highest vs f64 重算 **`max|diff| = 1.45e-6`** → PASS | `export_policy.py` |
| 部署物理与训练物理差多少？ | 同一站姿下只差 **0.7 mm** | `compare_deploy_vs_train_physics.py` |

## 1. 复现

```bash
# 观测 / 动作逐段对账（需要训练仓路径）
python scripts/verify_train_deploy_obs.py

# 执行器 + 轨迹 + 参数归因（纯 CPU）
python scripts/verify_train_deploy_parity.py --mode all

# 部署物理 vs 训练物理（调用训练仓 eval_go2.py 的同一套判据）
python scripts/compare_deploy_vs_train_physics.py
```

训练侧文件默认取 `../mjx-go2-training/models/go2/go2_mjx_position.xml` 与
`../mjx-go2-training/sim/eval_go2.py`，可用 `--train-body` 指定其他版本。

## 2. 执行器语义

训练侧：

```xml
<position kp="20" kv="1" forcerange="-23.7 23.7"/>   <!-- 膝关节 ±45.43 -->
```

MuJoCo 内部：`force = kp·(ctrl − q) + kv·(0 − dq)`，再 clip 到 `forcerange`。

部署侧 `sim_env/robot.py`：

```python
tau = kp * (q_des - q) + kd * (0.0 - dq) + tau_ff
tau = clip(tau, TORQUE_LIMITS)
```

策略路径恒有 `tau_ff = 0`、`dq_des = 0`，`kp = 20`、`kd = 1`，因此与训练侧严格等价。
注意：`<position>` 的 `forcerange` 不能省；只写 `ctrlrange` 不会限制力矩。

## 3. 观测 / 动作

观测 243 维 = 48 本体 + 187 高度扫描（17×11）+ 8 相位。逐段对账：

```text
0:3   线速度 ×2          3:6   角速度 ×0.25      6:9   projected gravity
9:12  指令 ×2           12:24  关节位置 − home   24:36 关节速度
36:48 last_action        48:235 高度扫描          235:243 四腿相位
```

当前结果：**每一段 `max|diff| = 0`**。历史坑：`projected_gravity` 曾把旋转矩阵的
第三列当第三行用，纯 yaw 时看不出来，一有 roll/pitch 重力符号就反了；修复后由
`tests/test_obs.py` 用随机四元数回归。

动作 12 维 = 绝对关节位置目标，先按训练机型关节范围 clamp，再由部署侧 PD 转力矩。
`go2_walk` 训练时 `normalize_observations=False`，部署侧同样**不套** `(x−mean)/std`；
误用 normalizer 会让动作偏差达到数 rad。

## 4. 物理参数

两侧模型逐项对比过质量、惯量、关节限位、阻尼、armature、frictionloss、几何尺寸、
足端摩擦与接触参数。当前约定：

- **足端硬接触**（`foot_soft_contact = false`）：v2/v33d 策略按硬接触训练，
  部署模型保持 `condim=6`、MuJoCo 默认 `solimp`、`margin=0`；
  训练模型已改成与部署模型逐项一致，因此"评测物理 == 训练物理"。
- **求解器**：训练用 MJX warp（2/5 次迭代），CPU MuJoCo 无法复刻该欠解语义；
  部署评测统一用收敛档 `solver_iterations=100 / solver_ls_iterations=50`。
- **body 树顺序**：部署模型是 `FL,FR,RL,RR`，训练模型是 `FR,FL,RR,RL`；
  代码一律按关节名 / `MOTOR_JOINTS` 索引，不能按下标硬取。

## 5. 残余差异（不是 bug，但要知道）

1. CPU Newton（100/50）与 MJX warp（2/5）的接触求解语义不同，地形上的绝对成功率
   不能逐位对比；同策略同地形的相对排名仍可用于选型。
2. 观测里的基座线速度在仿真用 oracle、真机用腿式里程计，误差是 sim2real 的第一号风险。
3. 高度扫描在仿真走 `SimLidar + HeightMapper`（真值 raycast）；真机点云接入前先用
   `--scan leg_fk` / `nominal`。
4. 观测与物理的逐段对账只证明"接口一致"，不证明策略本身在真机上一定成功。
