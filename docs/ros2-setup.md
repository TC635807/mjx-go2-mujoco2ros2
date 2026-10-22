# 接 ROS 2 / 真机 Go2（本机已按这套步骤装好并跑通）

## 0. 现状

* ROS 2 **Lyrical Luth** 已装在 `/opt/ros/lyrical`（200 个 ros 包）
* `unitree_go` 消息包已 colcon 构建到 `ros2_ws/install`
* `scripts/verify.py --transport ros2` 已 16/16 通过，`ros2 topic list` 里能看到
  `/rt/lowcmd`、`/rt/lowstate`，`ros2 topic hz` = 498 Hz

下面是"从零重装"的完整命令，以及真机切换要注意的事。

## 1. 装 ROS 2 Lyrical

Ubuntu 26.04 (resolute) 上 `ros2-apt-source` 没有对应的 deb，
用官方的 key + 手工写源即可。**国内务必换镜像**：packages.ros.org 实测 25 kB/s，
清华镜像是 900 kB/s。

    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
    sudo install -D -m 644 /tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
    sudo bash -c "echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]     https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu resolute main' > /etc/apt/sources.list.d/ros2.list"
    sudo apt update
    sudo apt install -y --no-install-recommends         ros-lyrical-ros-base ros-lyrical-rmw-cyclonedds-cpp         ros-lyrical-rosidl-generator-dds-idl ros-dev-tools
    sudo apt install -y python3-empy python3-lark python3-yaml

> `ros-lyrical-rosidl-generator-dds-idl` 是必须的：unitree_go 要生成 CycloneDDS 的 IDL，
> 少了它 colcon build 会在 "rosidl_generator_dds_idl" 这步失败。
> `--no-install-recommends` 会省掉一些 Python 依赖，所以 empy/lark/yaml 要单独补。

## 2. 让 venv 里的 Python 看到 ROS 2

我们的 venv 是给训练/仿真用的（mujoco 3.12 + numpy 2.5），ROS 2 装在系统 Python 上。
两者都是 Python 3.14.4，ABI 一致，所以只要补路径即可：

    export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages   # rosidl 要用的 empy/lark
    source /opt/ros/lyrical/setup.bash                              # rclpy 与 C 库

`scripts/run_with_ros2.sh` 把这套封装好了：

    scripts/run_with_ros2.sh scripts/run_sim.py --transport ros2 --duration 30 --viewer

## 3. 构建 unitree_go 消息

    mkdir -p ros2_ws/src
    cp -r third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go ros2_ws/src/
    cd ros2_ws && source /opt/ros/lyrical/setup.bash && colcon build --packages-select unitree_go

产物：`unitree_go/msg/LowCmd`、`LowState`、`IMUState`、`MotorCmd`、`MotorState` …
（字段与 `src/go2_bridge/msg.py` 一一对应，`transport/ros2.py` 做搬运）

## 4. 跑仿真

    source /opt/ros/lyrical/setup.bash
    source ros2_ws/install/setup.bash
    export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
    export ROS_DOMAIN_ID=1                    # 仿真；真机是 0，别撞上
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    # 终端 1
    .venv/bin/python scripts/run_sim.py --transport ros2 --duration 60 --viewer
    # 终端 2
    .venv/bin/python scripts/run_controller.py --transport ros2 --controller stand --duration 60
    # 终端 3（看话题）
    ros2 topic list && ros2 topic hz /rt/lowstate

## 5. 切真机 Go2

1. PC 有线网卡设成 `192.168.123.xxx`（和机器人同网段），确认 `ping 192.168.123.161` 通。
2. **先关掉机器人自带的运控服务**（MotionSwitcher 释放 sport mode），否则低层指令会被覆盖。
   宇树官方 unitree_mujoco 的 readme 也写了这点：关掉运控服务后 `SportModeState` 就读不到了。
3. 换 domain id 与网卡后跑同一份控制器：

       export ROS_DOMAIN_ID=0
       export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

4. **第一次务必把机器人吊起来**，并且先用 `--controller zero` 确认能收到状态，再上 `stand`。

### ⚠️ 真机的 CRC 还没做

真机 `rt/lowcmd` 要求 CRC32（宇树 `crc32_core`，poly 0x04C11DB7），
而 CRC 要按 **CDR 序列化后的字节** 算；rclpy 的 Python 消息对象拿不到那串字节。
所以：

* **仿真 ↔ ROS 2 这条路不受影响**（仿真侧不校验 crc，本框架自己的 CRC 也在 msg.py 里验证过）；
* 真机那条要么用 C++ 的 unitree_ros2 / unitree_sdk2（它们内部算 CRC），
  要么在 Python 里按 LowCmd 的 CDR 布局手工拼一遍（字段定长，可行但要小心对齐）。

## 6. 关于 unitree_sdk2py（dds 后端）

本机 **用不了**：cyclonedds 的 Python 绑定在 Python 3.14 上 import 就崩
（PEP 649 让它的 IDL 注解机制失效），不是配置问题。详见 `docs/backend-notes.md`。
将来若在 Python 3.12/3.13 环境里，`--transport dds` 可以直接用。
