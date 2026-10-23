# Go2 sim2real 实战项目调研（参考实现与踩坑）

> 目的：为 `real_env/` 的真机实现找**可核对的公开证据**，不照抄博客。
> 调研方法：本地浅克隆仓库读源码 + 官方文档/README，逐条带证据（repo:path:line / URL）。
> 调研日期：2026-10-23。结论只保留与本项目相关的部分。

## 1. 哪些项目真的在 Go2 真机上跑过

| 项目 | 链路 | 真机证据 | 对本项目最有用的点 |
|---|---|---|---|
| [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | ROS 2 + rmw_cyclonedds_cpp | 官方例程 | **ROS 2 话题 /lowstate /lowcmd、CYCLONEDDS_URI 写法、CRC 的 C 实现** |
| [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | 原生 DDS | 官方例程 | **812 字节 LowCmd CRC 布局（纯 Python）**、sport mode 释放、L1 开关 |
| [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | DDS 仿真 | 官方 sim2sim | domain/网卡分离；**"释放 sport mode 后 SportModeState 不可读"** |
| [Teddy-Liao/walk-these-ways-go2](https://github.com/Teddy-Liao/walk-these-ways-go2) | Python 策略 50 Hz + C++ 500 Hz(unitree_sdk2) | 实机视频 | 遥控 L2+B 阻尼急停、姿态超限保护、退出恢复 sport mode |
| [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) | C++ + unitree_sdk2 | 支持表 Go2 Real ✅ | 50 Hz 策略 / 200 Hz 指令分层、保持型 decimation |
| [Glowing-Torch/Go2-ROS2-deploy](https://github.com/Glowing-Torch/Deploy-an-RL-policy-on-the-Unitree-Go2-robot) | **纯 ROS 2** + 500 Hz C++ | 实机 GIF | **与本项目最像**：ROS 2 话题 + 500 Hz 低层 + CRC 在 ROS 2 消息上算 |
| [ShizukuNekuma/EasyGO2](https://github.com/ShizukuNekuma/EasyGO2) | ROS 2 + ONNX | 文档级真机流程 | **接管/交还状态机、stale 超时、低层频率 500 Hz、足端力标定** |
| [UESTC-REVERIE/unitree_go2_deploy](https://github.com/UESTC-REVERIE/unitree_go2_deploy) | unitree_sdk2py | 实机视频 | **唯一公开的 Go2 真机雷达高度图**（点云→459 格） |
| [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) | C++ + unitree_sdk2 | 官方部署代码 | 训练侧自动导出部署契约（obs/action/gain/joint map） |
| [TheoBounac/Deploy_SimToReal_RL_Go2](https://github.com/TheoBounac/Deploy_SimToReal_RL_Go2) | sdk2py + ROS 2 EKF | 实机 GIF | 200 Hz 策略 + 外部 Kalman 测速 |

**不要当证据的**：`Improbable-AI/walk-these-ways` 只支持 Go1；`CAI23sbP/go2_parkour_deploy`
宣称真机但 sim2real 代码未实现（只有 `lo` 分支）；`unitree_rl_gym` 当前 master 的
`deploy_real` 只剩 G1/H1/H1_2，Go2 部署要看历史版本/派生仓。

## 2. 硬件通信：与本项目 `real_env/` 逐条对账

| 事实 | 证据 | 本项目处理 |
|---|---|---|
| 真机 DDS 话题 `rt/lowcmd` / `rt/lowstate`；ROS 2 侧必须写 `/lowcmd` / `/lowstate`（rmw 补 `rt/`） | unitree_ros2 `example/src/src/low_level_ctrl.cpp:16`、`read_low_state.cpp:24` | ✅ `io_profiles` 的 ROS 2 档用 `/lowcmd` `/lowstate`；raw DDS 档用 `rt/` |
| LowCmd 必须带 `head=[0xFE,0xEF]`、`level_flag=0xFF`、20 电机 `mode=0x01`、合法 `crc` | sdk2py `go2_stand_example.py:88-94`；unitree_ros2 `go2_stand_example.cpp:96-110` | ✅ `Ros2Transport._to_ros_cmd` / `DdsTransport._to_idl_cmd` |
| CRC 覆盖 812 字节 IDL 布局去末尾 crc 的 808 字节（202 字），**不是 CDR / 不是 zlib** | sdk2py `utils/crc.py:19,185-193`；unitree_ros2 `common/motor_crc.cpp:3-39` | ✅ `signal_io/go2_crc.py` + 官方向量回归 `tests/test_go2_crc.py` |
| Go2 的 unitree_go LowCmd **没有** `mode_pr` / `mode_machine`（那是 unitree_hg/G1） | sdk2py IDL `_LowCmd_.py` | ✅ hasattr 保护，不写这两个字段 |
| 进低层前必须释放 sport mode：`CheckMode→ReleaseMode` 循环，或 `ServiceSwitch("sport_mode",0)`，或遥控器 **L2+R2** debug mode | unitree_rl_gym `deploy_real/README.md:25-27`；unitree_sdk2 `go2.h:18-33` | 📄 写入 `real_env/README.md` §3/§7（本环境 Python 3.14 跑不了 SDK，首选 L2+R2） |
| 释放后 `rt/sportmodestate` 不可读 | unitree_mujoco `readme.md:22` | ✅ 真机默认 `--scan leg_fk`，不依赖真机 odom/速度 |
| 官方没有 lowcmd 掉指令保护阈值，只有 `LowState.bit_flag` bit0=指令超时 | unitree 官方 Basic_services | ✅ 自建 HealthMonitor + SafetySupervisor；真机阈值 `health_profiles` 放宽 |
| L1：`/utlidar/cloud` PointCloud2，frame=`utlidar_lidar`（雷达系），约 11 Hz | unitree_ros2 `README.md:351-366`；L1 官方参数 21600 点/s | ✅ `perception_node --points-frame lidar`；外参需真机标定 |
| `/utlidar/robot_odom`、`/utlidar/robot_pose` 在部分固件上长期为空 | 厂商论坛/社区实测（论坛 1831；厂商建议接 LIO） | ✅ 真机默认 leg_fk；launch 仍保留 `scan:=topic` 选项 |
| 网段 192.168.123.0/24、PC 静态 IP、domain 0、`<Interfaces><NetworkInterface .../>` | unitree_ros2 `README.md:86,99-101`、`setup.sh:5-8` | ✅ `apply_io_env()`；旧 `NetworkInterfaceAddress` 在 CycloneDDS 11 会 assert 崩溃（本机实测） |
| 主流频率：策略 50 Hz、电机指令 200–500 Hz | rl_sar/EasyGO2/Teddy/Glowing | ✅ 50 Hz 策略 + 500 Hz 零阶保持（与仿真同代码） |

## 3. 高度扫描：唯一的真机公开样例

官方/主流 Go2 策略（unitree_rl_lab、robot_lab、rl_sar、rl_gym Go2）**都是盲走**：
观测 45 维纯本体，策略不使用高度扫描。唯一公开的 Go2 真机高度图是 UESTC 的 navila 变体：

- 订阅 `rt/utlidar/cloud`（PointCloud2），**单帧不完整**：缓存合并 ≥20000 点 / 4 帧；
- 对标训练侧 IsaacLab `height_map_lidar`：`xyz[:,0]*=-1`、`xyz=xyz[:,[1,0,2]]`、
  再绕 z 转 `theta=-π/5.5`（**必须离线对比 Isaac 点云 vs 真机点云后确定**）；
- 体素 0.06 m，x∈[-0.8,0.2]、y∈[-0.8,0.8] → 17×27=459 格；每格 z 最小值，
  <0.05 m 置 0、inf 置 0，再 3×3 max-pool；
- 观测 909 维 = 45 本体 + 459 高度 + 45×9 历史。

对本项目：187 点 raycast 扫描在真机没有先例，必须先平地 + `leg_fk` 跑通链路，
再补 odom/EKF + 点云外参标定，才能开 `--scan topic`。

## 4. 安全与状态机（值得照抄的做法）

- **接管/交还**（EasyGO2）：dry-run 默认关闭输出 → 检测唯一 lowcmd 发布者 →
  `ReleaseMode` → 等工厂指令静默 → 在 500 Hz 边界切到自有控制 → 退出 `SelectMode` 交还。
- **超时参数**（EasyGO2）：`stale_lowstate=0.2 s`、`stale_policy=0.1 s`、
  `minimum_external_low_cmd_hz=100`、键盘心跳 0.75 s。
- **姿态保护**：`|roll|/|pitch| > 0.8 rad` → 阻尼（Teddy）；unitree_rl_lab 用 1.0 rad。
- **退出**：先发阻尼命令（`kp=0,kd≈8` 或 SportClient.Damp()）再退进程，
  并恢复 sport mode；不要直接 `rclcpp::shutdown`。

## 5. 常见坑清单

1. CRC：按 CDR 字节或 zlib.crc32 算 → 固件静默丢帧（发 `mode=1` 但
   `motor_state.mode` 一直 0）。
2. 话题名：ROS 2 侧写 `/rt/lowcmd` → DDS `rt/rt/lowcmd`，收不到。
3. sport mode：忘记释放（指令被覆盖）或忘记恢复（退出后机器人不会自己站）。
4. odom 假设：低层控制下机器人自带 odom/sportmodestate 不可用。
5. 关节顺序：训练（Isaac 顺序）与电机顺序（SDK）不同，必须显式映射
   （社区常用 `[3,0,9,6,4,1,10,7,5,2,11,8]`）；本项目在 `obs.py`/`config` 已固定。
6. 增益/站姿：`kp/kd`、`default_angles`、`action_scale`、`scan base` 必须来自训练侧；
   本项目 `kp=20/kd=1` 与训练模型 `<position kp=20 kv=1>` 严格等价，**不要改成 0.5**。
7. 频率/实时性：Python 主循环同时推理 + 发布容易造成指令空档；本项目已把
   50 Hz 推理与 500 Hz 下发拆开（`ControllerNode`）。
8. 机型限制：官方 MotionSwitcher 低层控制仅对 **Go2 EDU / R&D** 开放；AIR/PRO 不适用。
9. 新固件差异：社区报告 v1.1.7 下 WebRTC 通道 joint state 降到 1 Hz、LiDAR ~7 Hz
   （低层 DDS 通道不受此限，但说明固件行为会变）。
10. WSL2：官方无支持，NAT 网络不参与机器人网段组播发现；真机建议原生 Linux / 机载 Jetson。

## 6. 参考链接

- unitree_ros2: https://github.com/unitreerobotics/unitree_ros2
- unitree_sdk2_python: https://github.com/unitreerobotics/unitree_sdk2_python
- unitree_mujoco: https://github.com/unitreerobotics/unitree_mujoco
- unitree_rl_gym: https://github.com/unitreerobotics/unitree_rl_gym
- unitree_rl_lab: https://github.com/unitreerobotics/unitree_rl_lab
- walk-these-ways-go2: https://github.com/Teddy-Liao/walk-these-ways-go2
- rl_sar: https://github.com/fan-ziqi/rl_sar
- Glowing ROS2 deploy: https://github.com/Glowing-Torch/Deploy-an-RL-policy-on-the-Unitree-Go2-robot
- EasyGO2: https://github.com/ShizukuNekuma/EasyGO2
- UESTC Go2 deploy: https://github.com/UESTC-REVERIE/unitree_go2_deploy
- TheoBounac: https://github.com/TheoBounac/Deploy_SimToReal_RL_Go2
- go2_ros2_sdk（社区话题名参考）: https://github.com/abizovnuralem/go2_ros2_sdk
