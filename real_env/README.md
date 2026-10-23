# 真机 Go2 部署指南（sim2real）

本目录是真机入口。控制代码与仿真共用同一套：`ros2_controller/`（50 Hz 策略）
+ `signal_io/`（消息 / IO / 感知）+ `config/go2_sim.json`（target、profile）。
**唯一的区别是 `--target`：数据源从 MuJoCo 换成真机 DDS。**

> 状态（诚实说明）：本仓已在仿真 + 消息层 + ROS 2 多节点上完成验证；
> **尚未在真实 Go2 上跑过**。下面每一条都来自官方源码/文档或社区可复现项目，
> 上机前请按 §3 的只读自检脚本逐项确认。调研来源见
> [docs/go2-sim2real-references.md](../docs/go2-sim2real-references.md)。

## 0. 一页结论

1. **不需要自己写 lowstate/lowcmd 的 hardware bridge。** 机器人固件就在 DDS 上收发
   `rt/lowstate` / `rt/lowcmd`；装好 ROS 2 + `unitree_go` 消息包 + `rmw_cyclonedds_cpp` 后，
   ROS 2 侧直接看到 **`/lowstate`、`/lowcmd`**（rmw 会自动补 `rt/` 前缀）。
   写 `/rt/lowcmd` 会变成 DDS 的 `rt/rt/lowcmd`，**真机收不到**。
2. **CRC 已实现，纯 Python**：`signal_io/go2_crc.py` 按固件的 812 字节 IDL 内存布局算
   CRC32（poly 0x04C11DB7 / init 0xFFFFFFFF / MSB-first），并用官方 C 库
   `crc_amd64.so` 的 4 组 gold value 做回归。**不是** CDR 序列化字节，也不是 zlib CRC32。
3. **进低层控制前必须释放 sport mode**（遥控器 L2+R2 进 debug mode，或程序
   MotionSwitcher.ReleaseMode / RobotStateClient.ServiceSwitch("sport_mode",0)）。
   释放后机器人自带 `/sportmodestate`（位置/速度）**不可读**，因此**不能**依赖真机 odom。
4. **L1 雷达**：`/utlidar/cloud`（PointCloud2，**frame = utlidar_lidar**，约 11 Hz），
   不是机体系、也不是世界系；`/utlidar/robot_odom` / `/utlidar/robot_pose` 在很多固件上
   **长期没有数据**（厂商建议接 LIO）。所以本项目真机默认 `--scan leg_fk`（平地可用），
   地形感知需要先补状态估计/外参标定（§6）。
5. **控制分层保持 500 Hz**：50 Hz 策略算 12 维关节位置目标，500 Hz 零阶保持下发 LowCmd
   （与仿真同一套代码，见 `signal_io/node.py`）。

## 1. 真机通信栈

```
                    ROS 2 话题（本仓看到的）          DDS 话题（固件）      方向
  /lowstate   unitree_go/msg/LowState      <->  rt/lowstate     机器人 -> PC   500 Hz
  /lowcmd     unitree_go/msg/LowCmd        <->  rt/lowcmd       PC -> 机器人   500 Hz（本仓）
  /utlidar/cloud        sensor_msgs/PointCloud2  <-> rt/utlidar/cloud   ~11 Hz
  /utlidar/lidar_state  unitree_go/LidarState    <-> rt/utlidar/lidar_state
  /api/motion_switcher/request|response  unitree_api/Request/Response  释放/查询运控
```

- 消息包在 `ros2_ws/src/unitree_go`（colcon build 后 `source ros2_ws/install/setup.bash`）。
- `unitree_go` 的 LowCmd **没有** `mode_pr` / `mode_machine`（那是 G1/H1 的 `unitree_hg`）。
  必须设置的是：`head=[0xFE,0xEF]`、`level_flag=0xFF`、20 个电机 `mode=0x01`，
  以及 `crc`（由 `go2_crc.lowcmd_crc` 计算并写入）。
- QoS：lowcmd/lowstate 用默认可靠 QoS（depth=10）；点云用 `qos_profile_sensor_data`
  （BEST_EFFORT），否则可能收不到。

## 2. 网络拓扑与 DDS 配置

| 项 | 值 |
|---|---|
| PC 网段 | 静态 `192.168.123.xxx/24`（例：192.168.123.99），**无 DHCP** |
| 机器人运控 | `192.168.123.161`（`ping` 通才算物理链路 OK） |
| Go2 EDU 机载算力板 | 常见 `192.168.123.18`（可选：把控制程序跑在机器人上） |
| DDS domain | 真机 **0**；仿真 1（本项目 `io_profiles` 已分开，别撞） |
| 网卡 | 有线网卡名（`ip link` 查，常见 `enp3s0`/`eth0`），写进 `config` 的 `interface` |

`apply_io_env()` 会自动在 `rclpy.init()` 之前设置：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

> 网卡写法必须是 `<Interfaces><NetworkInterface .../>`。旧的 `<NetworkInterfaceAddress>`
> 在 CycloneDDS 11（ROS 2 Lyrical 自带）里已被标记 deprecated（会警告、未来可能移除；
> 本机某次与 AllowMulticast 组合还触发过 assert，行为不稳定）。
> 另：`ROS_LOCALHOST_ONLY` 必须未设置或 0（否则 DDS 只走回环，真机必然不通）。

## 3. 上电前 / 首次上机流程

### 3.1 机器人准备

1. 机器人**吊起来**（离地 5~10 cm，随时可以断电）；
2. 遥控器 **L2+R2** 进入 debug mode（关节进入阻尼态）——官方部署流程（unitree_rl_gym
   deploy_real README）；也可以在程序里用 MotionSwitcher 释放（见 §7）；
3. 网线连接 PC 与机器人，PC 网卡设静态 IP，`ping 192.168.123.161` 通；
4. 确认机器人是 **Go2 EDU / R&D 系列**：官方 MotionSwitcher 低层控制仅对 EDU 开放
   （AIR/PRO 不支持低层）。

### 3.2 只读自检（不发送任何指令）

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages ROS_DOMAIN_ID=0

# 只订阅：/lowstate 频率、tick、电机 mode、/lowcmd 是否有别的进程在发 + CRC 校验、
# L1 点云 frame_id/频率/xyz 分布、odom/sportmodestate 是否有数据
scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
```

期望：`/lowstate` ≈ 500 Hz、电机 mode 全 0x01（debug mode 后）、`/lowcmd` 条数 0
（没有别的进程占用）。任何一项不符先解决，再进下一步。

### 3.3 零力矩 → 站姿 → 策略（吊起 → 落地）

```bash
# 1) 零力矩（吊起）：确认状态链路 + 电机能进入伺服
scripts/run_with_ros2.sh ros2_controller/run_controller.py \
    --target real_ros --controller zero --scan leg_fk --duration 10

# 2) 站姿 PD（仍吊起，或地面空旷）：看关节是否平滑走到站姿
scripts/run_with_ros2.sh ros2_controller/run_controller.py \
    --target real_ros --controller stand --scan leg_fk --duration 15

# 3) 落地、确认站姿稳定后，再跑策略（先给很小速度指令）
scripts/run_with_ros2.sh ros2_controller/run_controller.py \
    --target real_ros --controller policy --policy-profile go2_v33d \
    --scan leg_fk --linvel estimator --cmd 0.2 0 0 --duration 20
```

完整节点图（perception + controller，可加观察器）：

```bash
ros2 launch ros2_ws/launch/real.launch.py controller:=zero duration:=10
ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
ros2 launch ros2_ws/launch/real.launch.py controller:=policy viewer:=true
```

> launch 默认 `scan:=leg_fk`（真机没有可靠 odom 时的安全默认）。
> 若已确认 `/utlidar/robot_pose` 有数据，可切 `scan:=topic`
> （此时 perception 节点把 L1 点云转到世界系并生成 187 点高度扫描）。

## 4. 控制链路（和仿真同一套）

- **50 Hz 策略**：`ros2_controller/controller.py`，观测 243 维
  （48 本体 + 187 高度扫描 + 8 相位），动作 12 维关节位置目标；
  `kp=20, kd=1`（= 训练模型 `<position kp=20 kv=1>`，**不要**改成 0.5，会破坏训练/部署一致性）。
- **500 Hz 下发**：`signal_io/node.py::ControllerNode` 每 2 ms 把最近一条 LowCmd
  零阶保持发出；`Ros2Transport` 负责写 `head/level_flag/mode/crc`。
- **时间基**：`LowState.tick` 在仿真里 = 物理步数（2 ms/步），在真机固件里按 1 ms 递增；
  `io_profiles.*.tick_dt` 决定 `sim_time = tick * tick_dt`，控制器的 prepose/ramp/health 都靠它。
- **CRC**：`signal_io/go2_crc.py`，812 字节布局；单测 `tests/test_go2_crc.py` 与官方
  `crc_amd64.so` 逐字节对账（gold value：全零 0x4B5A4880、官方默认 0x9BEBAE54…）。
  仿真 `sim_ros` 也打开 `crc_check`：控制器算 CRC、sim 侧校验，**这条路径在仿真里就被验证过**。

## 5. 安全

- **看门狗（重要）**：`io_profiles.*.watchdog_ms` 只作用于**仿真侧** `SafetyLimiter`
  （仿真里 SimNode 收不到 LowCmd 就清零力矩）。**真机方向没有"指令掉线自动卸力"的软件看门狗**，
  固件对 lowcmd 断流的阈值/动作官方未公开（只有 `LowState.bit_flag` bit0 表示指令超时）。
- **真机保护链**：`HealthMonitor`（LowState 断流按**墙钟**计 age：>40 ms L1、>80 ms L2、
  >250 ms L3 锁存；另查 NaN/姿态/电机 lost/温度/bit_flag）→ `SafetySupervisor`
  （track → crouch → damping → estop）。阈值用 `config.health_profiles.real_*` 覆盖。
- **退出收尾**：真机 `--target real_*` 在 duration 到点 / Ctrl-C / 异常时先发 100 ms
  阻尼指令（`kp=0, kd=8`）再断流（`--no-damping` 可关）；没有状态时不会乱发。
- **没收到 LowState 时**：duration 仍然生效，1 s 后打印明确错误并继续等（不会静默挂死）。
- **官方没有公开"lowcmd 掉指令多久后自动阻尼"的阈值**（只有 `LowState.bit_flag` bit0
  表示指令超时状态）。不要依赖固件保护：程序退出/异常时显式发阻尼
  （`kp=0, kd≈8`）或调用 SportClient.Damp()。
- **建议**：真机首次运行全程吊起 + 手放急停；用 `--json` 落盘 summary，
  观察 tilt / tau / foot_force / health level。

## 6. 感知 / 高度扫描（真机最大风险）

- 训练侧 187 点高度扫描 = `clip(z_base - h_ground(xy_i) - 0.268, ±0.35) * 5`，
  `xy_i` 是机体系 17×11 网格（x∈[-0.4,1.2], y∈[-0.5,0.5]）。
- 真机 L1：`/utlidar/cloud`，frame=`utlidar_lidar`，约 11 Hz（21600 点/s），
  **单帧不完整**；社区唯一公开的 Go2 真机高度图样例（UESTC）需要缓存合并多帧、
  做坐标轴/旋转标定（x*-1、swap xy、Rz(-π/5.5)），再体素化成高度图。
- 本项目现状：`go2_perception --points-frame lidar` 按 `p_base = R(rpy)·p_lidar + t`
  变换（`--lidar-mount` / `--lidar-rpy`，默认取 `config.perception.mount_body/mount_rpy_body`），
  `--odom-type pose|odom` 支持 PoseStamped/Odometry。**但外参来源互相矛盾，必须真机标定**：
  官方 unitree_ros `radar_joint` 是前装 `xyz=(0.28945,0,-0.046825) rpy=(0,2.8782,0)`（≈165°，倒装）；
  社区 go2_ros2_sdk URDF 是顶装 `xyz≈(0.0018,0.0001,0.2536) rpy=0`。两者不能同时为真。
- 外参标定方法（UESTC 同款）：先录一段 `/utlidar/cloud`，在已知平面（地面）上求 R/t；
  或与仿真 `/go2/lidar/points`（世界系）对比同一场景点云求对齐。**标定前不要开真感知**。
- 在 odom 不可用/未标定时控制器自动回退 `leg_fk`（所有采样点填同一个离地高度标量）——
  平地近似正确（特征≈0），**地形上不可信**；控制器会在回退 >2 s 时打印显式警告。
- 真机默认 `scan:=leg_fk`（launch 默认）；只有确认 `/utlidar/robot_pose` 有数据且外参标定后
  才 `scan:=topic perception:=true`。
- 落地路线：先平地 + `leg_fk` 跑通低层链路 → 接 LIO/EKF 提供 odom/高度 →
  用 UESTC 的方法对齐点云与训练网格 → 再开 `--scan topic`。
- 参考实现：[docs/go2-sim2real-references.md](../docs/go2-sim2real-references.md)。

## 7. 释放 / 恢复 sport mode

低层指令必须在 sport mode 释放后才会生效。三种方式（任选其一）：

1. **遥控器 L2+R2** 进 debug mode（最简单，官方部署流程用的就是它）；
2. **MotionSwitcher RPC**：`CheckMode()` → 若返回的 `name` 非空则 `ReleaseMode()`，
   循环到为空（官方 `unitree_sdk2/go2.h` 注释："Run this function before publishing to
   rt/lowcmd"）；退出时 `SelectMode(原 mode 名)` 恢复；
3. **RobotStateClient.ServiceSwitch("sport_mode", 0/1)**（开/关）。

> 当前仓库的 Python 3.14 环境跑不了 `unitree_sdk2py`（cyclonedds Python 绑定不可用），
> 所以 1 是首选；2/3 可在 Python ≤3.13 或 C++/机载 Jetson 上用。以后如果要程序化接管，
> 建议照抄 EasyGO2 的 authority 状态机（ReleaseMode → 等工厂指令静默 → 500 Hz 边界接管
> → 退出 SelectMode 交还）。

## 8. 两种真机 IO

| target | 依赖 | 说明 |
|---|---|---|
| `real_ros` | rclpy + `unitree_go`（`ros2_ws/`） | **推荐**；ROS 2 话题 `/lowstate` / `/lowcmd`，与仿真同构 |
| `real_sdk` | `unitree_sdk2py` + cyclonedds | 官方 SDK；Python 3.14 下不可用（launch 会显式警告并回退 real_ros），见 `docs/ros2-setup.md` §8 |

两者都从 `config/go2_sim.json` 的 `io_profiles` 读 `domain_id` / `interface` /
`cmd_topic` / `state_topic` / `tick_dt` / `crc_check`；切 target 不改控制器代码。

## 9. 已知边界 / 未验证项（上机前必读）

- **未在真实 Go2 上验证**：需要实测 `/lowstate` 实际频率、释放 sport mode 后的电机 mode、
  CRC 错误时的固件行为（可能静默丢帧：发 mode=1 但 `motor_state.mode` 一直 0）。
- **电机 mode**：官方 Go2 例程用 `0x01`；少数项目用 `0x0A`。若 0x01 无法使能，
  用最小 lowcmd 实验确认（不要盲目切换）。
- **ROS 2 Lyrical + CycloneDDS 11 与机器人固件（CycloneDDS 0.10.2）的互通性未实测**；
  官方 unitree_ros2 只覆盖 Foxy/Humble。先在 `lo` + unitree_mujoco/仿真验证，
  再上真机；若发现不通，优先考虑在机器人机载 Jetson（原生 Linux）或降级 ROS 2 环境。
- **WSL2 不建议用于真机 DDS**（官方无支持；NAT 网络不参与机器人网段组播发现）。
  必须在 WSL2 里试的话，最小判据是 `ros2 topic list` 能看到 `/lowstate`。
- 真机 odom / sportmodestate 在低层控制下不可依赖（§0.3/§0.4）。
- **已修**（真机首测前）：无 LowState 时 duration 生效；断流后 health 按墙钟升级并切
  阻尼/急停；退出发 100 ms 阻尼；电机 lost/温度/`bit_flag` 参与健康判定；
  ROS 2 每拍构造 LowCmd 从 0.72 ms 降到 0.013 ms（500 Hz 预算压力大减）。
- **仍待真机确认**：固件 CRC 校验失败的具体表现；lowcmd 断流后的固件行为；
  500 Hz 在非 RT/WSL2 上的抖动（实测 `ros2 launch` 三节点下 controller p95 1–9 ms，
  建议真机绑核/RT 或 C++ 发送）。
- 本仓默认策略 `go2_v33d`（1536-768，20.97M 步）；回退 `go2_v29c`。
  **不要**用 `go2_v24` / `go2_v25a` 上真机（仿真强但部署闭环会绕圈）。

## 10. 站姿与接触参数（与仿真一致）

- **标称站姿** `FR, FL, RR, RL × (hip, thigh, calf)` =
  `[-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]`，
  来自 unitree_rl_gym / 官方部署惯例；不用 `go2.xml` keyframe `(0, 0.9, -1.8)`
  （实测 kp=20/kd=1 下站不住）。站姿写在 `config/go2_sim.json` 的 `robots.go2.stand_pose`。
- **足端硬接触**：`foot_soft_contact=false`（v2/v33d 策略按硬接触训练，
  训练/部署模型足端参数已逐项对齐）。
- **高度扫描**：`leg_fk` 是"单个标量"，训练侧是逐点；地形上的差异见
  `docs/go2-sim-fidelity.md`。真机建议先平地，再按 §6 接点云。
