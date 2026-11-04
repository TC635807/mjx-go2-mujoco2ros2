# 跨版本 CycloneDDS 互通探针（真机前必做）

## 它回答什么问题

> 我这台机器的 ROS 2（以及它自带的 CycloneDDS）能不能和 Go2 固件的 CycloneDDS **0.10.2** 通信？

宇树官方明确：机器人固件用 **CycloneDDS 0.10.2**；官方只验证过 Ubuntu 20.04 + Foxy / 22.04 + Humble。
如果本机 ROS 2 自带的 CycloneDDS 是别的大版本（例如 ROS 2 Lyrical 自带的 **11.0.1**），
DDS 层可能**根本不通** —— 而且失败是静默的（topic 列表看起来正常、`ros2 topic hz` 却是 0）。

## 原理

用官方 `unitree_sdk2`（C++，自带 `libddsc` = **0.10.2**，与机器人固件同版本）编两个最小程序，
当作“机器人固件替身”，和本机 ROS 2 的 `rclpy` 互相收发：

| 方向 | 等价于 | 期望 |
|---|---|---|
| A) SDK(0.10.2) 发 `rt/lowstate` → ROS 2 订阅 `/lowstate` | 机器人 → PC | 能收到 ≈500 Hz |
| B) ROS 2 发 `/lowcmd` → SDK(0.10.2) 订阅 `rt/lowcmd` | PC → 机器人 | 能收到 |

## 用法

依赖：`g++`、官方 `unitree_sdk2` 源码（含 `include/ lib/ thirdparty/`）、
目标 ROS 2 环境（必须能 `import unitree_go`，即已编译官方 `unitree_go` 消息包）。

```bash
# 宿主（例如 Ubuntu 26.04 + ROS 2 Lyrical）
bash run_version_probe.sh \
  --sdk-src /tmp/_sdk2 \
  --ros-setup /opt/ros/lyrical/setup.bash \
  --ws-setup /path/to/ros2_ws/install/setup.bash \
  --domain 5 --iface lo --seconds 5

# 官方栈容器（Ubuntu 22.04 + Humble）里跑 = 正对照
docker run --rm --network=host -v /path/to/repo:/workspace \
  unitree-go2-ros2:humble bash -c \
  'bash /workspace/docker/scripts/version_probe/run_version_probe.sh --sdk-src /opt/unitree_sdk2 --domain 6 --iface lo'
```

退出码：0 = 双向互通；1 = 不通（不要用这套环境上真机）。

## 实测结果（2026-10-25，本机）

| 环境 | ROS CycloneDDS | A（机器人→PC） | B（PC→机器人） | 结论 |
|---|---|---|---|---|
| 宿主 Ubuntu 26.04 + ROS 2 **Lyrical** | **11.0.1** | ❌ `rx=0` | ❌ `received 0` | **不通**（部分配置下 SDK 侧还会 segfault：`ddsi_xt_type_init_impl with invalid type object`） |
| Docker `unitree-go2-ros2:humble` | **0.10.5** | ✅ `rx=2026` | ✅ `received 484` | 通 |

同一份 SDK 0.10.2、同一个探针，只换 ROS 2 栈，结果就相反 —— 说明问题在 **CycloneDDS 大版本不兼容**，
不是探针或配置问题。

> 补充：在“默认网络、不绑网卡”的配置下，ROS 2 Lyrical 能**发现** `/lowstate` 话题，但会打印
> `New publisher discovered on topic '/lowstate', offering incompatible QoS. No messages will be received...`
> 然后一条也收不到。这正是最危险的一种失败：`ros2 topic list` 看起来一切正常。

## 结论

- 真机（或 unitree_mujoco 等 0.10.2 DDS 端）必须用 **CycloneDDS 0.10.x** 的 ROS 2 环境：
  官方推荐 Ubuntu 22.04 + **Humble**（见 [docker/](../../README.md)）。
- 新 distro（Jazzy/Kilted/Lyrical…）如果要上真机，先跑本探针；不通就换 Humble 或自己编
  CycloneDDS 0.10.x 的 rmw。
