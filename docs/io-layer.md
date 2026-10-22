# IO 层：同一份控制器，切 IO 就能换环境

## 1. 宇树那套的真实结构

先纠正一个常见理解：**不是"ROS 打仿真、SDK 打真机"**。宇树的结构是：

* `unitree_mujoco` 这个仿真器**本身就是一个 DDS 端点**，收发 `unitree_go::msg::dds_::LowCmd_` /
  `LowState_`，话题名就是真机那两个：`rt/lowcmd` / `rt/lowstate`。
  所以"仿真"和"真机"在这条链路上看起来是**同一种东西**，只是网络位置不同。
* 用 `unitree_sdk2`（C++）、`unitree_sdk2py`（Python）、`unitree_ros2`（ROS 2）写的控制器，
  **都能接仿真，也都能接真机**。切换只差初始化参数：

      // unitree_mujoco 官方 example/cpp/stand_go2
      if (argc < 2) ChannelFactory::Instance()->Init(1, "lo");      // 仿真：domain 1 + 回环
      else          ChannelFactory::Instance()->Init(0, argv[1]);   // 真机：domain 0 + 有线网卡

      # unitree_ros2
      source setup_local.sh; export ROS_DOMAIN_ID=1   # 仿真
      source setup.sh;       export ROS_DOMAIN_ID=0   # 真机

所以真实情况是一个 **2×2 矩阵**：

|  | IOROS（rclpy + `unitree_go`） | IOSDK（`unitree_sdk2py` + cyclonedds） |
|---|---|---|
| **MuJoCo 仿真** | `sim_ros` | `sim_sdk` |
| **真机 Go2** | `real_ros` | `real_sdk` |

四个格子里**控制器代码完全相同**，不同的只有：走哪条 IO、domain 号、网卡、看门狗。
宇树的"设计思路"本质就是这一条：**把环境差异全部下沉到 IO 初始化那几行**。

## 2. 本项目的落地：`--target`

`config/go2_sim.json` 里的 `io_profiles` 就是这张表：

    "target": "sim_udp",
    "io_profiles": {
      "sim_udp":  { "name": "udp",  "host": "127.0.0.1", "cmd_port": 50001, "state_port": 50002,
                    "crc_check": true,  "watchdog_ms": 60.0 },
      "sim_ros":  { "name": "ros2", "domain_id": 1, "interface": "lo",
                    "crc_check": false, "watchdog_ms": 60.0 },
      "sim_sdk":  { "name": "dds",  "domain_id": 1, "interface": "lo",
                    "crc_check": false, "watchdog_ms": 60.0 },
      "real_ros": { "name": "ros2", "domain_id": 0, "interface": "eth0",
                    "crc_check": true,  "watchdog_ms": 20.0 },
      "real_sdk": { "name": "dds",  "domain_id": 0, "interface": "eth0",
                    "crc_check": true,  "watchdog_ms": 20.0 }
    }

命令行：

    # 仿真（IOROS）
    .venv/bin/python scripts/run_sim.py        --target sim_ros --duration 60 --viewer
    .venv/bin/python scripts/run_controller.py --target sim_ros --controller lqr --duration 60

    # 真机（IOROS）—— 控制器代码一个字不改，只换 target
    .venv/bin/python scripts/run_controller.py --target real_ros --controller lqr --duration 60

`config.apply_io_env()` 会把 `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI`（网卡）
在 `rclpy.init()` **之前**设好，启动日志里也会打一行 `io_summary`，一眼能看出这次在打谁：

    [ctrl] io  = sim_ros -> IOROS(ROS2/cyclonedds)  [仿真 domain=1 iface=lo]  看门狗 60 ms

## 3. 控制器为什么天然可切换

控制器只认一个函数：

    update(state: LowState, t: float) -> LowCmd

它拿到的是 `LowState`（IMU + 12 个关节 + 足端力），吐出的是 `LowCmd`（12 个 tau/q/kp/kd）。
这两个结构在四个 profile 下**完全一样**（`msg.py` 与 `unitree_go` IDL 一一对应），
所以换 IO 对控制器是透明的。

IO 层本身只有四个方法（`src/go2_bridge/transport/base.py`）：

    publish_state(LowState)      # sim 侧发
    recv_cmd() -> LowCmd | None  # sim 侧收（非阻塞）
    recv_state() -> LowState|None# controller 侧收
    publish_cmd(LowCmd)          # controller 侧发

## 4. 想加自己的 IO（比如真机的 C++ 桥、或自研中间件）

实现这四个方法，然后在 `transport/__init__.py` 的 `make_transport()` 里注册即可，
其余代码（节点、安全层、控制器、验收脚本）全部复用。

## 5. 真机 checklist（`--target real_ros`）

1. 有线网卡设 `192.168.123.xxx`，`ping 192.168.123.161` 通；profile 里的 `interface` 填网卡名。
2. **先关掉机器人自带的运控服务**（MotionSwitcher 释放 sport mode），否则低层指令会被覆盖。
3. 第一次务必吊起来；先 `--controller zero` 确认能收到状态，再上 `stand`。
4. `watchdog_ms` 真机用 20 ms（比仿真的 60 ms 紧），因为真机上丢指令就是真摔。
5. ⚠️ `rt/lowcmd` 的 CRC32 还没做（要按 CDR 序列化后的字节算），见 `docs/results.md` 末尾。
   仿真侧不受影响。
