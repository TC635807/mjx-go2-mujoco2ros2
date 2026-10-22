# 架构：MuJoCo(Go2) ↔ ROS 2 / DDS 双向闭环

## 一句话

MuJoCo 里跑一台 Go2，500 Hz 出 `rt/lowstate`、收 `rt/lowcmd`；
上层控制器 50 Hz 算力矩，中间一层 500 Hz 的电机驱动层做保持与保底。
**仿真和真机共用同一套话题、同一套消息、同一份控制器代码**，切换只改 domain id 和网卡。

## 数据流

    ┌──────────────────────────────┐        rt/lowstate（unitree_go::LowState）      ┌──────────────────────────┐
    │  SimNode（MuJoCo, CPU 单环境）│ ───────────────────────────────────────────────▶│  ControllerNode          │
    │  500 Hz 物理 (sim_dt=2 ms)    │                                                  │  50 Hz 算 LowCmd         │
    │  ← 施加 tau = kp·e + kd·ė + τ │ ◀─────────── rt/lowcmd（LowCmd, CRC32）──────────│  500 Hz 零阶保持发出      │
    └──────────────────────────────┘                                                  └──────────────────────────┘
                ▲                                                                                 ▲
                │ 同一个 Transport 接口：loopback / udp / dds / ros2                              │
                └─────────────────────────────────────────────────────────────────────────────────┘

真机时把左边的 SimNode 换成真机（同一对话题），控制器一行不改。

## 为什么是 50 Hz + 500 Hz 两层

宇树底层 low-level 通道是 500 Hz（官方 `low_level.cpp` 里 `dt = 0.002`），
真机上如果 500 Hz 收不到指令，电机侧会超时保护。所以：

* **500 Hz 层（电机驱动层）**：保持最近一条 LowCmd、按 N·m/s 限斜率、按关节类别夹力矩、60 ms 看门狗。
  实现在 `src/go2_bridge/safety.py`。
* **50 Hz 层（控制层）**：LQR / 状态机 / 策略网络都在这一层，只负责"算出这一拍的 12 个目标力矩"。

这样你的控制理论（LQR 之类）不用去关心 500 Hz 的抖动，而真机又能满足底层时序。

## 消息层

`src/go2_bridge/msg.py` 里 `LowCmd` / `LowState` 的字段与 `unitree_go::msg::dds_::` 一一对应：

* 20 个电机槽位（Go2 用前 12 个），顺序与真机一致：FR / FL / RR / RL × hip / thigh / calf
* `MotorCmd = {mode, q, dq, tau, kp, kd, reserve}`
* `LowState` 带 `imu_state{quaternion, gyroscope, accelerometer, rpy}`、`motor_state[20]`、`foot_force[4]`、`tick`
* CRC32 用宇树 `crc32_core` 同一套算法（poly 0x04C11DB7、MSB-first、init 0xFFFFFFFF、无 final xor），
  `msg.py` 里给了逐位参考实现做对拍，测试保证两者结果完全一致。

**为什么自己写一份而不是直接 import**：本机没有 sudo，装不了 ROS 2 与 CycloneDDS 的系统包，
所以消息层做成零依赖，`loopback` / `udp` 两个后端今天就能跑；
`dds` / `ros2` 后端只做一次字段搬运，装好环境即插即用。

## 四个传输后端

| 后端 | 依赖 | 用途 | 本机验证 |
|---|---|---|---|
| `loopback` | 无 | 单进程调试、单元测试 | ✅ |
| `udp` | 无 | 双进程，和 DDS 同样的报文语义/频率/CRC，用来在没有 ROS 2 的机器上把整条链路跑通 | ✅ |
| `dds` | unitree_sdk2py + cyclonedds | 话题 `rt/lowcmd` / `rt/lowstate` | ❌ Python 3.14 下不可用（PEP 649） |
| `ros2` | rclpy + rmw_cyclonedds_cpp + unitree_go | 真机 / ROS 2 原生，话题与真机一致 | ✅ 16/16 通过 |

切换方式：`--transport {loopback,udp,dds,ros2}`，控制器侧同一份代码。

## 真机（Go2）对应关系

| 仿真 | 真机 |
|---|---|
| `rt/lowcmd` / `rt/lowstate` | 同名 |
| domain_id 1 + interface `lo` | domain_id 0 + 有线网卡（PC 静态 IP 192.168.123.x） |
| `safety.py` 的限幅/看门狗 | 真机上同样需要，且必须 |
| 关节级 PD 写在仿真里 | 真机由电机驱动器算 `kp·(q_des−q) + kd·(dq_des−dq) + tau` |
| 跑低层前不需要特殊操作 | **必须先关掉机器人自带的运控服务（MotionSwitcher / sport mode）** |

## 目录

    config/go2_sim.json      时序 / 传输 / 控制器参数
    models/go2/              Go2 的 MJCF + 网格（取自 unitree_mujoco，执行器已是 <motor>）
    src/go2_bridge/
        msg.py               LowCmd/LowState + CRC32
        robot.py             MuJoCo 封装：加载、施加力矩、打包 LowState
        safety.py            限幅 / 斜率 / 看门狗
        controllers.py       控制器（stand / zero / lqr_stub）
        node.py              SimNode / ControllerNode / Pacer
        transport/           base / local(loopback+udp) / dds / ros2
    scripts/                 run_sim.py / run_controller.py / verify.py
    docs/                    本文件、ros2-setup.md、results.md
