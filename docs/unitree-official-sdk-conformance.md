# 宇树官方 SDK 要求符合性审计（mjx-go2-mujoco2ros2）

> 审计对象：/home/tc63/mujoco/mjx-go2-mujoco2ros2 工作区当前状态
> 审计日期：2026-10-25
> 官方依据：宇树文档中心 + 官方 GitHub 仓库（出处见 §1）
> 审计方式：读代码 → 对照官方源码/文档 → 本机跑官方 CRC 回归 → 在**官方推荐栈 Docker 环境**里做 ROS 2 闭环、官方 Python SDK 闭环、以及 ROS 2 ↔ 官方 SDK 的双向 DDS 互通实测（§5）

---

## 0. 一句话结论

**消息契约、CRC、话题映射、控制时序、安全收尾这些“和机器人固件打交道”的部分是对的，并且逐字节/逐位对齐官方实现；但宿主机的运行栈（Ubuntu 26.04 + ROS 2 Lyrical + CycloneDDS 11.0.1 + Python 3.14）整体偏离宇树官方支持范围**，官方 Python SDK 路径在宿主上根本跑不了，而那部分代码里也确实藏着一个从未被执行到的 bug（§3.1）。

已按官方推荐栈（Ubuntu 22.04 + ROS 2 Humble + CycloneDDS 0.10.x + Python 3.10）交付 Docker 环境，在其中把 ROS 2 路径、官方 SDK 路径、双向 DDS 互通全部跑通（§5、docker/README.md）。

| 维度 | 判定 |
|---|---|
| unitree_go 消息定义（LowCmd/LowState/...） | ✅ 与官方仓库逐字节相同 |
| LowCmd 固定头 / 电机 mode / 电机顺序 | ✅ 与官方例程一致 |
| LowCmd CRC（812 B 布局 + crc32_core） | ✅ 与官方 crc_amd64.so 逐值一致 |
| 话题名 / RMW / domain / 网卡绑定 | ✅ 与官方 setup.sh / 例程一致 |
| 控制时序（50 Hz 策略 + 500 Hz 下发） | ✅ |
| 释放运控服务（低层前提） | ⚠️ 文档正确，但宿主只能手动 L2+R2；容器已补程序化脚本 |
| 退出/急停阻尼 | ✅ 优于官方例程 |
| **运行栈版本（OS / ROS 2 / CycloneDDS / Python）** | ❌ **不在官方支持范围**（已用 Docker 对齐） |
| 官方 Python SDK（--target real_sdk/sim_sdk） | ⚠️→✅ 宿主不可用；容器内修复 1 个 bug 后可用 |
| 真机实测 | ❌ 尚未上真机（项目自述，见 real_env/README.md） |

---

## 1. 官方要求（出处）

| # | 官方要求 | 出处 |
|---|---|---|
| 1 | 推荐 **Ubuntu 20.04 或 22.04**；不支持 Mac/Windows；不支持在 Go2 内置电脑上开发 | [Quick_start](https://support.unitree.com/home/zh/developer/Quick_start) |
| 2 | ROS 2 支持：Ubuntu 20.04 + **Foxy**；Ubuntu 22.04 + **Humble（recommend）** | [unitree_ros2 README](https://github.com/unitreerobotics/unitree_ros2) |
| 3 | 机器人固件 CycloneDDS = **0.10.2**；Humble 下不需要重编 CycloneDDS | 同上 §2 |
| 4 | Docker 开发环境：.devcontainer/Dockerfile-humble（ros2:humble-full） | 同上 |
| 5 | 系统依赖：cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev | Quick_start |
| 6 | PC 有线网卡静态 **192.168.123.x/24**；**不能**把 PC 设成 192.168.123.161（内置电脑 IP）；ping 192.168.123.161 通 | Quick_start |
| 7 | 低层控制前**必须关闭主运控服务**：mcf（固件 >1.1.6）/sport_mode（<1.1.6）/wheeled_sport（Go2-W），否则多运控并存会失控 | Quick_start |
| 8 | LowCmd：head=[0xFE,0xEF]、MotorCmd.mode Foc=0x01 / stop=0x00、12 电机顺序 FR/FL/RR/RL、crc 32 位校验 | [Basic_services](https://support.unitree.com/home/zh/developer/Basic_services) |
| 9 | LowState：tick 按 **1 ms** 递增；foot_force 顺序 0-FR/1-FL/2-RR/3-RL；bit_flag 位定义（bit0 运控命令超时、bit5 遥控超时、bit7 电机超时…） | Basic_services |
| 10 | DDS 话题 rt/lowstate / rt/lowcmd；ROS 2 侧官方例程用 /lowstate / /lowcmd | unitree_ros2 README / 例程 |
| 11 | CycloneDDS 配置：RMW_IMPLEMENTATION=rmw_cyclonedds_cpp + CYCLONEDDS_URI 里 NetworkInterface name=... multicast=default | unitree_ros2 setup.sh |
| 12 | 官方 Python SDK：**Python >= 3.8**、**cyclonedds == 0.10.2**、pip install -e . | [unitree_sdk2_python README](https://github.com/unitreerobotics/unitree_sdk2_python) / setup.py |
| 13 | 官方 C++ SDK：Ubuntu 20.04 / gcc 9.4 / cmake >= 3.10 | [unitree_sdk2 README](https://github.com/unitreerobotics/unitree_sdk2) |
| 14 | 程序化释放运控：MotionSwitcherClient.CheckMode → ReleaseMode（go2_stand_example）；遥控器 **L2+R2** 进 debug mode 是官方部署流程 | Quick_start / [unitree_rl_gym deploy_real](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md) |
| 15 | 二次开发支持：AIR/PRO 无、X 基础支持、EDU 支持 | Go2 产品参数页（文档中心） |
| 16 | **unitree_sdk2 与 ROS 2 环境存在二进制冲突**：SDK2 要占用 CycloneDDS，不能在初始化了 ROS/ROS2 的环境里启动（同一进程/同一脚本里二者互斥）；官方推荐同时安装 unitree_sdk2 | [ROS2_service](https://support.unitree.com/home/zh/developer/ROS2_service) |

> ROS2_service 页同时复核了：测试过的组合仍是 Ubuntu 20.04+foxy / 22.04+humble、机器人用 cyclonedds 0.10.2、
> 低层话题为 lowstate/lowcmd（ROS 2 侧）、高层为 sportmodestate（含 lf/sportmodestate 低频版）、
> 控制走 /api/sport/request（高层）与 /lowcmd（低层）、L1 点云 frame=utlidar_lidar —— 与本审计其余结论一致。

---

## 2. 逐项对照

| # | 官方要求 | 项目实现 | 判定 |
|---|---|---|---|
| 1 | unitree_go 消息定义 | ros2_ws/src/unitree_go 与官方 unitree_ros2 v0.3.0 的 cyclonedds_ws/src/unitree/unitree_go **diff -r 无差异** | ✅ |
| 2 | LowCmd 固定头 / mode | signal_io/transport/ros2.py::_to_ros_cmd 写 head=FE EF、level_flag=0xFF、motor_cmd[i].mode 来自控制器（0x01） | ✅ |
| 3 | CRC 布局/算法 | signal_io/go2_crc.py：812 B、202 个 uint32、poly 0x04C11DB7/init 0xFFFFFFFF/MSB-first/无 final xor；与官方 unitree_sdk2py/utils/crc.py 的 __packFmtLowCmd 完全一致 | ✅ |
| 4 | 12 电机顺序 + 20 槽位 | signal_io/msg.py / config：FR/FL/RR/RL × hip/thigh/calf，NUM_MOTOR=20 | ✅ |
| 5 | 500 Hz 低层 / 50 Hz 策略 | signal_io/node.py::ControllerNode.run（decim=10，每 2 ms 下发） | ✅ |
| 6 | 真机 tick 1 ms | config.go2_sim.json 的 real_ros.tick_dt = 0.001 | ✅ |
| 7 | 话题 /lowstate /lowcmd | config.io_profiles.*；不写 rt/ 前缀（rmw 自动补） | ✅ |
| 8 | RMW + 网卡绑定 | signal_io/config.py::apply_io_env：RMW_IMPLEMENTATION=rmw_cyclonedds_cpp、CYCLONEDDS_URI 用 Interfaces/NetworkInterface（与官方 setup.sh 同款） | ✅ |
| 9 | domain：真机 0 / 仿真区分 | real_* = 0、sim_* = 1 | ✅ |
| 10 | QoS：低层默认 depth=10、点云 BEST_EFFORT | transport/ros2.py（10）、perception_node.py / go2_real_check.py（qos_profile_sensor_data） | ✅ |
| 11 | bit_flag 语义 | ros2_controller/health.py：bit0/bit7 → L2；与官方位定义一致 | ✅ |
| 12 | 释放运控 | real_env/README.md 给了 3 种方式；但宿主 SDK 不可用 → 实际只能手动 L2+R2 | ⚠️ |
| 13 | 退出阻尼 | node.py::emergency_stop 发 100 ms kp=0/kd=8 再断流（官方例程只是停发） | ✅ |
| 14 | 网络说明 | real_env/README.md §2：192.168.123.99/24、domain 0、有线网卡 | ✅（.161 表述见 §3.6） |
| 15 | L1 雷达外参 | config.perception.mount_body/mount_rpy_body = **(0.28945, 0, -0.046825)/(0, 2.8782, 0)**，与官方 unitree_ros/go2_description.urdf 的 radar_joint **完全一致** | ✅ |
| 16 | **运行栈版本** | 宿主：Ubuntu 26.04 + ROS 2 Lyrical + CycloneDDS **11.0.1** + Python **3.14** | ❌ |
| 17 | 官方 Python SDK 路径 | 宿主不可用；容器内可用（修复 §3.1） | ⚠️→✅ |
| 18 | 官方 C++ SDK | 项目未使用（ROS 2 路径替代）；容器内已装好 | ➖ |
| 19 | 真机实测 | 项目自述“尚未在真实 Go2 上跑过”（real_env/README.md 状态说明） | ⚠️ |

---

## 3. 发现的问题

### 3.1 【已修】raw DDS 后端解析 LowCmd 崩溃（官方 SDK 路径的真实 bug）

signal_io/transport/dds.py::DdsTransport._on_cmd 里：

```python
reserve=int(getattr(src, "reserve", 0)),
```

但官方 unitree_sdk2py 的 MotorCmd_.reserve 是 types.array[types.uint32, 3]（Python 里是 **list**），int(list) 直接 TypeError。

- **影响**：--target sim_sdk 的仿真侧每收到一帧 LowCmd 就在 SDK 的 reader 线程里抛一次异常，n_cmd 恒为 0；real_sdk 的仿真/桥接方向同理。控制器方向只收 LowState，不受影响。
- **为什么一直没发现**：宿主 Python 3.14 下 unitree_sdk2py 根本 import 不了，这条路径从未被执行（README 也写明“Python 3.14 暂不可用”）。
- **修复**：新增 _scalar_or_first()，标量/IDL 数组都能安全转 int；修复后 python scripts/verify.py --target sim_sdk **16/16 通过**（见 §5）。

### 3.2 【环境级，最主要】运行栈不在官方支持范围

| 维度 | 项目宿主实际 | 宇树官方要求/基线 | 风险 |
|---|---|---|---|
| OS | Ubuntu 26.04 (WSL2) | 20.04 / 22.04，不支持 WSL/Windows | 官方无支持；真机 DDS 在 WSL2 下未验证 |
| ROS 2 | **Lyrical Luth** | Foxy（20.04）/ **Humble（22.04，recommend）** | 官方 unitree_ros2 未测试 Lyrical |
| CycloneDDS | **11.0.1**（Lyrical 自带） | 机器人固件 **0.10.2**；Humble apt 为 0.10.x | **跨大版本互通未验证**（项目 real_env/README.md §9 自己也列了这条） |
| Python | **3.14.4** | Python SDK 要求 >=3.8 且 cyclonedds==0.10.2 | cyclonedds Python 绑定在 3.14 不可用 → 官方 SDK 路径不可用 |

处置：交付官方推荐栈 Docker 环境（docker/），在里面把整条链路验证通过（§5）。

### 3.3 【部署流程】程序化释放运控服务缺失

- 官方低层例程（go2_stand_example）在程序里用 MotionSwitcherClient：CheckMode → 若非空则 ReleaseMode 循环；退出时可选 SelectMode 恢复。
- 项目文档写明了这件事，但宿主 Python 3.14 跑不了 SDK → 实际只能**手动遥控器 L2+R2**。
- 处置：容器内新增 docker/scripts/release_sport_mode.py（**默认只读**，--yes 才释放，--restore <mcf|sport_mode> 恢复），对齐官方流程。

### 3.4 【脚本】scripts/run_with_ros2.sh 硬编码 /opt/ros/lyrical

官方 Humble 环境下这层 wrapper 直接失效（ros2 launch 起不来）。已改为：ROS_SETUP / ROS_DISTRO 自动选择，新增 UNITREE_ROS2_SRC（官方工作区）、GO2_PYTHON（venv python）、GO2_SKIP_PROJECT_WS（容器里跳过宿主 py3.14 构建的 ros2_ws/install）、GO2_SKIP_PY_DIST。**无这些环境变量时行为与原来完全一致**。

### 3.5 【配置】真机网卡名写死 eth0

容器 / WSL mirrored 网络里网卡名和宿主机不同（容器里常见 eth1/eth2）。已在 signal_io/config.py::load_config 增加环境变量覆盖：GO2_INTERFACE / GO2_DOMAIN_ID（文件/CLI 优先级不变，只是多一层 env 覆盖）。

### 3.6 【文档细节，不影响功能】

- real_env/README.md 把 192.168.123.161 称为“机器人运控”；官方 Quick_start 明确它是 **Go2 内置电脑的 IP**（运控服务跑在这台电脑上）。功能上不影响。
- “官方 MotionSwitcher 低层控制仅对 EDU 开放”：官方产品页口径是 AIR/PRO 二次开发“无”、X“基础支持”、EDU“支持”。
- 官方服务名随固件版本变化（mcf >1.1.6 / sport_mode <1.1.6 / wheeled_sport Go2-W），项目文档只提了 sport_mode；建议 checklist 补一行。
- 未使用的 8 个电机槽位项目下发 q=0/kp=kd=0（官方例程 init 用 PosStopF/VelStopF 哨兵）；kp=0 时力矩都等于 0，功能等价。
- README 声称 tests/run_tests.py 14/14 通过；宿主实测 13/14（test_viz_panels 的 15 ms 图像面板阈值实到 15.5 ms），属于环境相关项，与本审计结论无关。

### 3.7 【未验证】真机相关

- 项目尚未上真机（自述）。以下仍需实测：
  - 固件 CRC 校验失败时的行为（官方未公开，最可能是静默丢帧）；
  - rt/lowcmd 断流后固件的动作/阈值（官方文档和 SDK 头文件里都没有明文，项目“未公开”的判断成立）；
  - 释放运控后的电机 mode 取值（官方 Go2 例程用 0x01，少数社区项目用 0x0A）；
  - L1 点云外参/时间同步与 /utlidar/robot_odom 是否可用。
- 建议用容器里的官方 unitree_sdk2 C++ 例程（go2_stand_example 等）与 scripts/go2_real_check.py 做对照实验。

### 3.8 【官方约束】unitree_sdk2 与 ROS 2 的同进程/同环境冲突

官方 [ROS2_service](https://support.unitree.com/home/zh/developer/ROS2_service) 原文：
“Unitree SDK2 与 ROS2 环境存在二进制冲突：Unitree SDK2 需要占用 CycloneDDS，无法在初始化了
ROS/ROS2 环境中启动……也无法在同一脚本中（同时使用）”。

对照本项目：

- **合规**：每个进程只建一种 transport（Ros2Transport 或 DdsTransport），不会在同一进程里同时初始化
  rclpy 与 unitree_sdk2py；sim_sdk/real_sdk 的双进程验收也是两个独立进程。
- **需注意**：容器里 BASH_ENV 会自动 source ROS 2 环境；跑官方 **C++** 例程时要用
  docker/scripts/run_official_sdk2_example.sh（清掉 ROS/colcon 环境、只挂 SDK2 自带 libddsc），
  否则可能与 ROS 2 的 CycloneDDS 抢同一个 DDS 实现。Python SDK 路径（unitree_sdk2py + cyclonedds
  0.10.2 wheel）与 ROS 2 进程互不影响，已在容器内实测互操作（§5 互通 A/B 就是两个独立进程通过 DDS 对话）。

---

## 4. 本次做的修改（最小侵入）

| 文件 | 改动 | 目的 |
|---|---|---|
| signal_io/transport/dds.py | MotorCmd.reserve 支持 IDL 数组（_scalar_or_first） | 修复官方 SDK 收指令崩溃（§3.1） |
| scripts/run_with_ros2.sh | 按 ROS_DISTRO/ROS_SETUP 选 ROS；新增 UNITREE_ROS2_SRC/GO2_PYTHON/GO2_SKIP_PROJECT_WS/GO2_SKIP_PY_DIST | 让官方 Humble 环境能跑本仓脚本（向后兼容） |
| signal_io/config.py | GO2_INTERFACE / GO2_DOMAIN_ID 环境覆盖 | 容器/不同机器不改配置文件即可换网卡/domain |
| docker/**（新增） | Dockerfile / compose / setup / entrypoint / 自检 / DDS 互通测试 / 释放运控脚本 / README | 官方推荐栈环境 |

---

## 5. 官方栈 Docker 环境与实测结果

详见 [docker/README.md](../docker/README.md)。镜像组成：

- ros:humble-ros-base（Ubuntu 22.04.5 + ROS 2 Humble）
- ros-humble-rmw-cyclonedds-cpp（CycloneDDS **0.10.5**，与固件 0.10.2 同系列）
- 官方 unitree_ros2 v0.3.0：unitree_go / unitree_api / unitree_hg（colcon 构建）
- 官方 unitree_sdk2 2.0.2（C++，含预编译 CycloneDDS 与官方例程）
- 官方 unitree_sdk2_python（commit 814556d）+ cyclonedds==0.10.2（官方 pin）
- 本仓 venv：Python 3.10 + numpy + mujoco

**实测（容器内 docker/scripts/verify_conformance.sh）**：

| 项目 | 结果 |
|---|---|
| 官方 Python SDK CRC 类 | ✅ CRC() 可用；对“head=FE EF / level=FF / 20×mode=1 / q=0”算出 **0x57ECF19D**，与本仓 signal_io/go2_crc.py 的 gold value **逐位相同** |
| 本仓单元测试 | ✅ 14/14（无显示器时用 MUJOCO_GL=osmesa） |
| ROS 2 闭环 verify.py --target sim_ros | ✅ **16/16**（RTF 1.000，p95 lag ≈ 0.5 ms，bad_crc=0） |
| 官方 SDK 闭环 verify.py --target sim_sdk | ✅ **16/16**（修复 §3.1 后；宿主 py3.14 上此前无法运行） |
| DDS 互通 A：ROS 2 /lowstate → 官方 SDK rt/lowstate | ✅ 6.02 s 收到 **3008 条 ≈ 499.8 Hz**，字段值/tick/mode 正确 |
| DDS 互通 B：官方 SDK rt/lowcmd（官方 CRC）→ 本仓 ROS 2 仿真 | ✅ SDK 发 4000 条 @500 Hz；仿真侧 rx_cmd=**3990**、**bad_crc=0** → 812 B 布局 + CRC 与官方实现等价 |
| ros2 launch sim.launch.py duration:=8 三节点图 | ✅ 正常结束；仿真 4000 步、bad_crc=0、策略正常跟踪 |
| 官方 C++ SDK | ✅ libunitree_sdk2.a + 官方例程二进制已装（/opt/unitree_robotics） |

> 互通结论的关键依据：官方 C++ SDK 头文件里 TopicTraits<::unitree_go::msg::dds_::LowCmd_>::getTypeName() 返回 "unitree_go::msg::dds_::LowCmd_"，与 ROS 2 生成的 DDS 类型名、cyclonedds-python 转换后的类型名（点号→::）一致，所以两条官方路径可以在 DDS 层直接互通。

### 5.1 补充实测：宿主 Lyrical（CycloneDDS 11.0.1）与官方 0.10.2 **不互通**

用 [docker/scripts/version_probe/](../docker/scripts/version_probe/) 里的探针
（官方 unitree_sdk2 C++ 自带 libddsc = **0.10.2**，作为机器人固件替身）在同一台机器上对比：

| 环境 | ROS 的 CycloneDDS | A：0.10.2 发 rt/lowstate → ROS 2 收 | B：ROS 2 发 /lowcmd → 0.10.2 收 | 结论 |
|---|---|---|---|---|
| 宿主 Ubuntu 26.04 + ROS 2 **Lyrical** | **11.0.1** | ❌ rx=0 | ❌ received 0 | **不通** |
| 容器 `unitree-go2-ros2:humble` | **0.10.5** | ✅ rx=2026 | ✅ received 484 | 通 |

细节：

- 默认网络下 Lyrical 能**发现** `/lowstate`，但报
  `New publisher discovered on topic '/lowstate', offering incompatible QoS. No messages will be received...`，
  一条数据都收不到 —— 正是“topic 可见、hz=0”的静默失败。
- 某些配置下 0.10.2 侧进程会 segfault：`ddsi_xt_type_init_impl with invalid type object`。
- 同一探针、同一 SDK，只换 ROS 栈结果相反 → 问题在 **CycloneDDS 大版本不兼容**，不是配置。

**结论：宿主 Lyrical 栈不能直接上真机**；真机必须用 0.10.x 的 ROS 2 环境（本仓 Docker 官方栈或原生 Ubuntu 22.04 + Humble）。

---

## 6. 上真机 checklist（对齐官方，逐条可执行）

1. **机型确认**：Go2 EDU / X（AIR/PRO 不支持低层）；记录固件版本 → 运控服务名 mcf 还是 sport_mode。
2. **网络**：原生 Ubuntu 22.04（推荐）有线网卡静态 192.168.123.222/24（**不要** .161），ping 192.168.123.161 通。
3. **环境**：docker compose run --rm ros2，进入前设 UNITREE_NIC=<网卡名>、ROS_DOMAIN_ID=0（容器里 ip -4 addr 看 192.168.123.x 的网卡）。
4. **机器人姿态**：吊起离地 5–10 cm；遥控器 **L2+R2** 进 debug mode（关节阻尼），或 python docker/scripts/release_sport_mode.py --iface <网卡> --yes 程序化释放。
5. **只读自检**（不发任何指令）：scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
   期望：/lowstate 约 500 Hz、电机 mode=0x01、/lowcmd 无其它发布者、L1 点云 frame=utlidar_lidar。
6. **分阶段上电**：controller:=zero（10 s）→ stand（15 s）→ policy（30 s，先 --cmd 0.2 0 0）。
7. **收尾**：控制器退出会先发 100 ms 阻尼；异常时立即断电/急停。
8. **CRC 负向实验**（建议做）：故意发 crc=0 的 LowCmd，观察 LowState.motor_state.mode 是否不响应，确认“固件静默丢帧”的行为（这是官方没写明的部分）。
9. **DDS 排障顺序**：网线/网段 → ROS_DOMAIN_ID → 网卡名（CYCLONEDDS_URI）→ ROS_LOCALHOST_ONLY 必须未设置 → ros2 topic list 是否能看到 /lowstate。
10. **WSL2 提醒**：官方不支持 WSL/Windows；本机 WSL2 是 mirrored 网络，理论上能看到 192.168.123.x 网卡，但组播发现未验证。真机建议原生 Ubuntu 22.04（或官方支持的机载环境）。

---

## 7. 结论与建议

1. **代码契约层面合格**：消息/CRC/话题/时序/安全收尾都与官方一致，且 CRC 有官方 .so 逐值回归。
2. **环境层面不合格（宿主）**：Ubuntu 26.04 + Lyrical + CycloneDDS 11 + Python 3.14 属于“能用但官方不背书”，尤其 CycloneDDS 跨大版本对真机是硬风险。
3. **真机部署建议走官方栈**：本仓的 Docker 环境（或原生 Ubuntu 22.04 + Humble）应作为真机基线；Lyrical 宿主只用于仿真开发。
4. **官方 Python SDK 路径**：修复 §3.1 后已在容器内跑通；建议把 sim_sdk/real_sdk 纳入 CI（在 Humble 容器里跑 verify.py --target sim_sdk），否则这类 bug 还会回归。
5. **真机前补齐**：程序化释放运控、CRC 负向实验、断流行为确认、L1 外参/odom 方案。
