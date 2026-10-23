# 观察器（ROS 2 节点）

观察器现在是一个真正的 ROS 2 节点：它**不自己跑仿真/控制器**，只订阅话题。
入口脚本会自动拉起整张节点图，然后用真实节点观察：

```bash
# 一键：启动 sim_sensors + perception + controller，再启动 go2_viewer
bash scripts/view_go2_policy.sh
```

```text
scripts/view_go2_policy.sh
  ├─ ros2 launch ros2_ws/launch/sim.launch.py   # sim_sensors + perception + controller
  └─ scripts/view_ros2.py                        # go2_viewer（纯订阅）
```

关闭观察窗口 / Ctrl-C 时脚本会一起结束 launch 里的所有节点。

## 1. 常用命令

```bash
# 平地，一直跑到关窗口
bash scripts/view_go2_policy.sh

# 地形
bash scripts/view_go2_policy.sh --terrain wide_h050
bash scripts/view_go2_policy.sh --terrain mixed1
bash scripts/view_go2_policy.sh --terrain rubble
bash scripts/view_go2_policy.sh --list-terrains

# 时长 / 无 GUI / 落盘
bash scripts/view_go2_policy.sh --duration 20
bash scripts/view_go2_policy.sh --headless --duration 10
bash scripts/view_go2_policy.sh --record logs/go2_view

# 策略档 / 速度指令
PROFILE=go2_v29c bash scripts/view_go2_policy.sh
bash scripts/view_go2_policy.sh --cmd 0.3 0.2 0.5

# 软件渲染提帧率
bash scripts/view_go2_policy.sh --robot-style lite --sync-hz 30
bash scripts/view_go2_policy.sh --follow --trail 200

# 旧版单进程观察器（不启动 ROS 2 节点）
bash scripts/view_go2_policy.sh --local --terrain hyb2

# 真机观察（机器人 DDS 在线；默认 controller=stand / scan=leg_fk）
ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=20 viewer:=true
```

## 2. 观察器订阅什么

| 话题 | 类型 | 用途 |
|---|---|---|
| `/lowstate` | `unitree_go/LowState` | 关节角/角速度/IMU/足端力 |
| `/lowcmd` | `unitree_go/LowCmd` | 关节目标（q_des） |
| `/go2/odom` | `nav_msgs/Odometry` | 基座位姿与速度 |
| `/perception/height_scan` | `std_msgs/Float32MultiArray` | 187 点高度扫描 |
| `/go2/controller/obs` | `std_msgs/Float32MultiArray` | 243 维观测（`--publish-debug`） |
| `/go2/controller/action` | `std_msgs/Float32MultiArray` | 12 维策略原始动作 |
| `/go2/controller/info` | `std_msgs/String`（JSON） | 健康等级、模式、速度/扫描来源、统计 |

launch 里的 `go2_controller` 已默认带 `--publish-debug`，所以 `view_go2_policy.sh`
不需要额外参数。

## 3. 看什么

一个 MuJoCo 窗口 + 一个数据窗口（matplotlib / QtAgg，可缩放、可拖走）：

| 区域 | 内容 |
|---|---|
| 3D 叠加 | 187 个高度采样点（按特征值着色）、足端接触、速度估计/真值箭头、指令箭头、轨迹拖尾 |
| 实时曲线 | 关节力矩、基座高度与倾斜、线速度（估计 vs 真值）、动作幅值 |
| 文字 HUD | 健康等级 L0-L3、模式、tilt、z、vx、tau_max、输入来源 |
| 终端仪表盘 | `--headless` 时 ANSI 原地刷新，不依赖 GUI |

高度点颜色语义：`clip(z_base − h_ground − 0.268, ±0.35) × 5`；负=前方地面更高（上坡/台阶），
正=下陷，约 0=平地。它不是"到地面的距离"，而是相对标称站高的偏差。

## 4. 参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `--terrain` | flat | 31 个地形之一（`--list-terrains`） |
| `--duration` | 0 | 观察时长（秒）；0 = 直到关窗口/Ctrl-C |
| `--headless` | 关 | 不开 3D 窗口，只出终端仪表盘 |
| `--record DIR` | 关 | CSV + 关键帧 + `summary.png` + `report.md` |
| `--follow` | 关 | 相机跟随躯干 |
| `--trail N` | 120 | 轨迹拖尾点数 |
| `--robot-style` | full | `lite` = 只显示碰撞几何（软件渲染推荐） |
| `--lite` | on | 关阴影/反射等昂贵视觉效果 |
| `--plots` | window | `window` = 独立 matplotlib 曲线窗；`off` = 不画 |
| `--sync-hz` | 60 | 3D 刷新频率 |
| `--plot-hz` | 3 | 曲线刷新频率 |

没有 GUI 时自动退化为终端仪表盘；`--local` 则回到旧版单进程观察器
（`scripts/view_policy.py`，不启动 ROS 2 节点，适合没有 ROS 2 的机器）。

## 5. 实现

```text
scripts/view_go2_policy.sh      启动 launch + viewer 的入口（--local 回退）
scripts/view_go2_policy_local.sh 旧单进程观察器入口（view_policy.py）
scripts/view_ros2.py            go2_viewer 节点：订阅话题 -> VizFrame -> MuJoCo 窗口
signal_io/ros2_debug.py         控制器调试发布（obs/action/info）
sim_env/viz/frame.py            VizFrame / FrameCollector
sim_env/viz/scene.py            3D 叠加（扫描点/足端/箭头/轨迹）
sim_env/viz/live.py             独立曲线窗口（支持 7x5 / 17x11 扫描）
sim_env/viz/record.py           Recorder / 终端仪表盘
```

观察器只消费话题和 `VizFrame`，不读仿真或控制器内部对象，所以同一份代码可以观察
仿真节点和真机节点。
