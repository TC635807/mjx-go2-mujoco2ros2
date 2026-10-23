# ROS 2 / CycloneDDS 环境与运行

本仓的 ROS 2 路径已在本机验证：`scripts/verify.py --target sim_ros` 16/16 通过；
`ros2 launch ros2_ws/launch/sim.launch.py` 会启动 sim_sensors / perception / controller
三个真正的 ROS 2 节点，通过 `/lowstate` / `/lowcmd` / `/perception/height_scan` 闭环。

## 0. 现状

- Ubuntu 26.04 (WSL2) + Python 3.14 + ROS 2 **Lyrical Luth**（`/opt/ros/lyrical`）。
- `unitree_go` 消息包已通过 colcon 构建到 `ros2_ws/install`。
- 仿真默认是 3 个 ROS 2 节点（数据源 + 感知 + 控制器）；话题名、消息类型与真机一致。
- `dds` 后端（`unitree_sdk2py` + cyclonedds Python 绑定）在 Python 3.14 下不可用，
  原因见 §6；ROS 2 路径不受影响。

## 1. 安装 ROS 2 Lyrical

Ubuntu 26.04 上 `ros2-apt-source` 没有对应 deb，用官方 key + 手工源（国内建议换清华镜像）：

```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
sudo install -D -m 644 /tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
sudo bash -c "echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu resolute main' > /etc/apt/sources.list.d/ros2.list"
sudo apt update
sudo apt install -y --no-install-recommends \
  ros-lyrical-ros-base ros-lyrical-rmw-cyclonedds-cpp \
  ros-lyrical-rosidl-generator-dds-idl ros-dev-tools
sudo apt install -y python3-empy python3-lark python3-yaml
```

> `ros-lyrical-rosidl-generator-dds-idl` 是必须的：`unitree_go` 要生成 CycloneDDS IDL，
> 少了它 colcon build 会在 `rosidl_generator_dds_idl` 步骤失败。

## 2. 让 venv 看到 ROS 2

本仓 venv 用于仿真（mujoco / numpy），ROS 2 装在系统 Python 上；二者都是 Python 3.14，
ABI 一致，补路径即可。`scripts/run_with_ros2.sh` 已封装：

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
export ROS_DOMAIN_ID=1                 # 仿真；真机是 0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

scripts/run_with_ros2.sh sim_env/run_sim.py --target sim_ros --duration 30 --viewer
scripts/run_with_ros2.sh ros2_controller/run_controller.py --target sim_ros --controller policy --duration 30
```

## 3. 构建 unitree_go 消息

```bash
mkdir -p ros2_ws/src
cp -r third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go ros2_ws/src/
cd ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select unitree_go
```

产物：`unitree_go/msg/LowCmd`、`LowState`、`IMUState`、`MotorCmd`、`MotorState` 等。
字段与 `signal_io/msg.py` 一一对应，`signal_io/transport/ros2.py` 只做字段搬运。

## 4. 跑与真机同构的 ROS 2 节点图

仿真和真机启动的是同一张节点图，只替换数据源节点：

```text
仿真 domain 1                                 真机 domain 0
go2_sim_sensors（MuJoCo 500 Hz）              Go2 固件 DDS + Unitree L1
   /lowstate /go2/odom /go2/lidar/points       /lowstate /lowcmd /utlidar/*
            \                 /              （不需要自写 lowstate/lowcmd bridge，
             go2_perception（PointCloud2 + Odom -> /perception/height_scan）
                              |                rmw_cyclonedds 直接可见）
                    go2_controller（50 Hz）
                              |
                     /lowcmd
```

一键启动（推荐）：

```bash
# 仿真：3 个节点（sim_sensors + perception + controller）
ros2 launch ros2_ws/launch/sim.launch.py duration:=30

# 再叠加观察器节点（go2_viewer）
ros2 launch ros2_ws/launch/sim.launch.py duration:=20 viewer:=true

# 真机：2 个节点（perception + controller），直接订阅机器人 DDS 话题
# 默认 controller:=stand、scan:=leg_fk（安全默认）；完整流程见 real_env/README.md
ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=zero duration:=10
ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
```

也可以直接用观测入口（会自动起上面的节点图 + viewer）：

```bash
bash scripts/view_go2_policy.sh --terrain wide_h050
bash scripts/view_go2_policy.sh --headless --duration 10
```

手工逐节点启动（便于单独调试）：

```bash
# 终端 1：仿真数据源（domain 1 + lo）
scripts/run_with_ros2.sh sim_env/run_sim.py --target sim_ros \
    --publish-sensors --duration 60 --viewer

# 终端 2：感知节点
scripts/run_with_ros2.sh signal_io/perception_node.py --target sim_ros

# 终端 3：控制器（从话题拿高度扫描）
scripts/run_with_ros2.sh ros2_controller/run_controller.py \
    --target sim_ros --controller policy --scan topic --duration 60
```

确认话题：

```bash
ros2 topic list
ros2 topic hz /lowstate             # 约 500 Hz
ros2 topic hz /go2/lidar/points        # 15 Hz（仿真默认）
ros2 topic hz /perception/height_scan  # 15 Hz
ros2 topic info /lowstate -v        # unitree_go/msg/LowState
```

旧的两进程演示脚本仍然保留（不带独立感知节点，控制器自动回退 leg_fk）：

```bash
./scripts/demo_ros2_policy.sh 12 go2
```

## 5. 切真机 Go2

真机完整流程、网络拓扑、安全阈值见 [real_env/README.md](../real_env/README.md)；这里只列最容易踩的点。

1. PC 有线网卡设成 `192.168.123.xxx/24`（静态，无 DHCP），`ping 192.168.123.161` 通。
2. **先释放 sport mode**：遥控器 **L2+R2** 进 debug mode（关节阻尼态），或用
   MotionSwitcher `ReleaseMode` / RobotStateClient `ServiceSwitch("sport_mode",0)`；
   否则低层指令被内置运控覆盖。
3. `ROS_DOMAIN_ID=0`；网卡从 `lo` 换成实际有线网卡，写进 `config` 的 `interface`，
   由 `apply_io_env()` 生成 **新写法** 的 `CYCLONEDDS_URI`：

   ```text
   <CycloneDDS><Domain><General><Interfaces>
     <NetworkInterface name="eth0" priority="default" multicast="default" />
   </Interfaces></General></Domain></CycloneDDS>
   ```

   > 旧写法 `<NetworkInterfaceAddress>` 在 CycloneDDS 11（Lyrical 自带）下已被标记
   > deprecated（会警告、未来可能移除；本机某次组合还触发过 assert）：用
   > `<Interfaces><NetworkInterface .../>` 才稳定且无警告。
   > `ROS_LOCALHOST_ONLY` 必须未设置（否则只走回环）。
4. 同一份控制器代码直接跑；第一次务必吊起来，先 `--controller zero` 确认能收到状态，
   再上 `stand` / `policy`。先跑只读自检（不发送任何指令）：

    ```bash
    scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
    ```

5. 低层状态/指令**不需要自己写 bridge**：机器人固件 DDS 的 `rt/lowstate` / `rt/lowcmd`
   经 `rmw_cyclonedds_cpp` 直接以 ROS 2 话题 `/lowstate` / `/lowcmd` 出现。
   只有 L1 点云（`/utlidar/cloud`）和 odom 需要额外处理（见 §7）。

## 6. ⚠️ 真机 `/lowcmd` 的 CRC（已实现）

真机固件校验 `LowCmd.crc`。算法是宇树的 `crc32_core`：poly `0x04C11DB7`、init `0xFFFFFFFF`、
**逐位 MSB-first、无 final xor**（不是 zlib/Ethernet CRC32）；覆盖范围是
**812 字节 IDL 结构体内存布局**去掉末尾 4 字节 crc 后的 808 字节（202 个 uint32），
**不是** ROS 2 CDR 序列化字节（两者 padding 不同：CDR 里 `motor_cmd[0].mode` 在 offset 22，
固件布局在 24）。

本仓已在 `signal_io/go2_crc.py` 用纯 Python 实现，并做了两层回归：

- `tests/test_go2_crc.py`：4 组 gold value + 官方 `crc_amd64.so` 交叉验证 + 敏感性测试；
- `sim_ros` 也打开 `crc_check`：控制器每拍算 CRC、sim 侧逐条校验（`bad_crc=0` 才算通过）。

必须一起设置的还有固定头：`head=[0xFE,0xEF]`、`level_flag=0xFF`、20 个电机
`mode=0x01`（`Ros2Transport._to_ros_cmd` 已处理）。

CRC 错时最可能的固件行为是**静默丢帧**（发 `mode=1` 但 `motor_state.mode` 一直是 0），
不报错——所以 `go2_real_check.py` 会订阅 `/lowcmd` 并逐条校验 CRC。仿真侧
`Ros2Transport._on_cmd` 对坏 CRC 也**直接丢弃**（与固件语义一致，回归能在 sim 里暴露）。

500 Hz 下发成本：`_to_ros_cmd` 复用预分配消息后从 0.72 ms 降到 **0.013 ms**，
publish ≈0.04 ms；但 `ros2 launch` 三节点 + WSL2 下 controller 抖动 p95 约 1–9 ms，
真机建议绑核 / RT 内核 / C++ 发送，并在 `--json` summary 里看 `pacer.overruns`。

## 7. L1 点云 / odom（真机感知）

- L1 点云：ROS 2 话题 `/utlidar/cloud`（`sensor_msgs/PointCloud2`），
  **frame_id = `utlidar_lidar`（雷达系，不是机体系/世界系）**，周向扫描约 11 Hz、
  单帧不完整（官方参数 21600 点/s）。`go2_perception --points-frame lidar` 按
  `p_base = R(rpy)·p_lidar + t` 变换（`--lidar-mount/--lidar-rpy`）。**外参来源互相矛盾**
  （官方 URDF 前装倒装 rpy≈2.8782 vs 社区 URDF 顶装 rpy=0），必须先真机标定，
  再 `scan:=topic perception:=true`；默认 `scan:=leg_fk` 是安全档。
- odom：`/utlidar/robot_odom`、`/utlidar/robot_pose` 在部分固件上**长期没有数据**；
  且释放 sport mode 后机器人自带 `sportmodestate` 不可读。所以真机默认
  `--scan leg_fk`（平地近似正确），地形感知需先接 LIO/EKF 提供 odom。
- 详细路线与参考实现见 [real_env/README.md](../real_env/README.md) §6 与
  [docs/go2-sim2real-references.md](go2-sim2real-references.md)。

## 8. 为什么 `dds` 后端在 Python 3.14 不可用

`unitree_sdk2py` 依赖 `cyclonedds` 的 Python 绑定。后者在 Python 3.14 下：

1. 编译 `clayer/pysertype.c` 时报 `_Py_IsFinalizing` 未声明（3.13 起改成公开的
   `Py_IsFinalizing`，3.14 删除旧私有 API）；
2. 即使打补丁编译通过，`import cyclonedds` 也会因为 PEP 649（注解延迟求值）
   让它的 IDL 反射机制失效而崩溃：`TypeError: Member TRY_CONSTRUCT1 is not defined`。

ROS 2 的 `rmw_cyclonedds_cpp` 是 C++ 实现，不碰这套 Python 反射，所以本仓走 ROS 2。
等以后环境降到 Python 3.12/3.13，`--target sim_sdk / real_sdk` 可以直接启用。
