# 真实环境（真机 Go2）部署说明

本目录是真机部署的入口说明。控制代码与仿真共用同一套：
ros2_controller/（50 Hz 控制器）+ signal_io/（统一信号与 IO）+ config/（target 配置），
区别只在 **--target** 与网卡/domain/看门狗。

## 1. 两种真机 IO

| target | 依赖 | 说明 |
|---|---|---|
| real_sdk | unitree_sdk2py + cyclonedds（third_party/） | 官方 SDK，domain 0 + 有线网卡（默认 eth0） |
| real_ros | rclpy + unitree_go 消息包（ros2_ws/） | ROS 2 话题 rt/lowstate / rt/lowcmd |

两者都从 config/go2_sim.json 的 io_profiles 读参数（domain_id、interface、watchdog_ms、crc_check）。
切 target 不改控制器代码。

## 2. 启动前检查

    cd /home/tc63/mujoco/mjx-go2-mujoco2ros2

    # 消息层 / 站立 / 实时性（不接真机也能先跑）
    .venv/bin/python scripts/verify.py --target sim_udp

    # 真机网络/domain 环境（DDS 需要先设环境变量；脚本会打印实际设置）
    bash scripts/bootstrap_dds.sh

    # 站立检查（真机或仿真均可；确认姿态/高度/力矩在范围内）
    .venv/bin/python scripts/check_go2_stand.py

## 3. 真机闭环

    # A) unitree_sdk2py（DDS）
    .venv/bin/python ros2_controller/run_controller.py \
        --target real_sdk --controller policy --policy-profile go2_v25b --duration 20

    # B) ROS 2 两进程（仿真/真机同构，见 docs/ros2-setup.md）
    source /opt/ros/lyrical/setup.bash
    source ros2_ws/install/setup.bash
    # 终端 1: 真机 IO 节点（由 unitree 的 lowstate/lowcmd 桥提供）
    # 终端 2: 控制器
    ./scripts/run_with_ros2.sh ros2_controller/run_controller.py \
        --target real_ros --controller policy --duration 20

## 4. 安全与看门狗（真机比仿真更严）

* io_profiles.real_*：watchdog_ms=20ms（仿真 60ms），掉指令立即卸力；
* HealthMonitor：LowState 超时/NaN/lost/温度 -> L2 阻尼、L3 卸力锁存；
* SafetySupervisor：关节限幅、力矩硬限幅、斜率限幅；
* 首次上真机建议：先 --controller stand，再 policy + 低速指令（command 0.2 m/s），
  用 --json 落盘并观察 tilt / tau / foot_force。

## 5. 已知边界

* 真机目标需要 Go2 硬件与对应 SDK/ROS 2 环境；本仓库本机验证到仿真 + 消息层。
* 真机不要使用 go2_v24 / go2_v25a（仿真强但 CPU 部署闭环会绕圈）；
  默认 go2_v25b 在 oracle/estimator 双速度源下都稳定（见训练仓训练历史）。
