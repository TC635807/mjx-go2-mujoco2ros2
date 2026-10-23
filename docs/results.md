# 验收结果与性能

环境：Ubuntu 26.04 (WSL2) / Python 3.14.4 / MuJoCo 3.12.0 / ROS 2 Lyrical Luth / CPU 单环境
模型：`sim_env/models/go2/scene.xml`（unitree_mujoco 官方 Go2 MJCF，12 个 `<motor>` 力矩执行器）

## 1. 复现

```bash
# 单元测试：14 个纯 Python 测试文件（不需要 pytest）
python tests/run_tests.py

# 验收：消息层 / IMU / PD 站立 / 双进程实时性（不接真机）
python scripts/verify.py --target sim_udp

# ROS 2 双进程（需要 ROS 2 环境，见 docs/ros2-setup.md）
python scripts/verify.py --target sim_ros
```

## 2. 测试与验收

| 项目 | 命令 | 结果 |
|---|---|---|
| 单元测试 | `python tests/run_tests.py` | **14/14 文件通过** |
| 消息 / IMU / PD / 双进程实时性 | `python scripts/verify.py --target sim_udp` | **16/16 通过** |
| ROS 2 双进程 | `python scripts/verify.py --target sim_ros` | **16/16 通过** |
| 观测 / 动作对账 | `scripts/verify_train_deploy_obs.py` | obs 逐段 `max|diff| = 0`；动作 `≤ 1.15e-6` |
| 执行器等价 | `scripts/verify_train_deploy_parity.py --mode all` | `max|Δq| = 1.1e-15 rad`（1500 步） |
| 地形碰撞契约 | `tests/test_terrain_scenes.py` | 31 个地形全部通过 |
| 策略导出等价 | `scripts/export_policy.py` | f32 highest vs f64 `max|diff| = 1.45e-6` |
| ROS 2 节点图（仿真） | `ros2 launch ros2_ws/launch/sim.launch.py` | 3 节点闭环；感知发布 15 Hz 点云，控制器 `height_scan_source=external` |
| ROS 2 观察器 | `bash scripts/view_go2_policy.sh --headless --duration 5` | 自动拉起 3 节点 + viewer；观测到 243 维 obs、187 点扫描、L0 健康 |
| 真机 LowCmd CRC（812 B 布局） | `python tests/test_go2_crc.py` | 4 组官方向量 + `crc_amd64.so` 交叉验证一致 |
| 真机上电自检脚本 | `scripts/go2_real_check.py --target sim_ros` | 识别 500 Hz `/lowstate`、电机 mode、点云 frame、`/lowcmd` CRC |
| ControllerNode 循环/断流/收尾 | `python tests/test_node_loop.py` | 无状态时 duration 生效；断流 L1→L2→L3；退出阻尼 kp=0/kd=8；电机 lost/温度/bit_flag |

覆盖范围：消息 CRC 与往返、观测布局与符号、LiDAR 感知与高程图、腿式里程计、健康层 L0-L3、
安全监督、策略前向（含 `.equiv.npz` 回归）、可视化面板、31 个地形碰撞几何。

## 3. ROS 2 后端实测

```text
本地报文：LowCmd 524 B / LowState 684 B；CRC32 与宇树 crc32_core 逐位参考实现一致
真机报文：LowCmd 812 B（unitree_go IDL 布局）；CRC 与官方 crc_amd64.so 逐字节一致
          仿真闭环 5 s / 2500 条 /lowcmd：控制器算 CRC、sim 侧逐条校验，bad_crc = 0
          坏 CRC 会被 sim 侧丢弃（与真机固件"静默丢帧"语义一致，回归能在 sim 暴露）
          字段：head=[0xFE,0xEF] level_flag=0xFF motor mode=0x01
          500 Hz 下发成本：_to_ros_cmd 0.013 ms（旧实现 0.72 ms）、publish 0.04 ms
          real_ros 无机器人 duration=2：2 s 正常退出 + 明确报错（旧实现会静默挂死）
IMU（站定）：|a| = 9.809 m/s^2；|w| = 0.0017 rad/s；foot_force ≈ [35,35,39,39]
单进程 loopback PD 站立：max tilt 4.3°，mean z 0.258 m；不限速吞吐 ≈3400 Hz
双进程 ROS 2 (500 Hz)：RTF 1.00，p95 周期抖动 4.7 ms，看门狗 0 次误触发
  机器人站住：max tilt 0.9°，mean z 0.266 m
ros2 topic hz /lowstate ≈ 498 Hz，类型 unitree_go/msg/LowState
```

话题名、消息类型与真机低层通道完全一致：

```text
$ ros2 topic list
/parameter_events
/rosout
/lowcmd
/lowstate

$ ros2 topic info /lowstate -v
Type: unitree_go/msg/LowState
Publisher count: 1
Node name: go2_controller
```

## 4. 500 Hz 这一拍的性能拆账

500 Hz 预算 2000 µs。实测（中位 / p95）：

| 环节 | 中位 | p95 | 占预算 |
|---|---:|---:|---:|
| `mujoco.mj_step`（物理一步） | 20.9 µs | 44.5 µs | 1.0% |
| `read_state()`（组 LowState） | 63.4 µs | 140.8 µs | 3.2% |
| `write_cmd()`（12 关节 PD + 限幅） | 37.0 µs | 66.8 µs | 1.8% |
| `LowState.pack()` + CRC32 | 83.0 µs | 133.1 µs | 4.2% |
| `LowState.unpack()` + CRC | 9.2 µs | 15.7 µs | 0.5% |
| `LowCmd.unpack()` | 11.2 µs | 18.7 µs | 0.6% |
| ROS2 `publish(LowState)` | 26.3 µs | 50.3 µs | 1.3% |
| ROS2 `spin_once` | 58.9 µs | 124.8 µs | 2.9% |
| **ROS2 端到端（publish → 回调）** | **233.9 µs** | 445.8 µs | 11.7% |

一个 500 Hz 拍的全部活（物理 + 打包 + 发状态 + 施加力矩）约 **204 µs，占 2 ms 预算的 10.2%**。

> 控制侧另测（优化后）：`_to_ros_cmd` **13 µs**（复用预分配消息；旧实现每电机重新构造
> 一条 20 电机消息 = 0.72 ms）、`publish_cmd` **38 µs**。但 `ros2 launch` 三节点 + WSL2 下
> controller 的 `pacer.overruns` 实测 23%–53%、`lag_p95` 1–9 ms（受启动阶段与同机负载影响）；
> 这是"500 Hz 在非 RT 环境不可靠"的直接证据，真机需绑核/RT 或 C++ 发送。
> 安全侧兜底是：LowState 断流按墙钟升级 L1/L2/L3（见 `tests/test_node_loop.py`）。

结论：

1. 瓶颈不是 Python，而是 DDS 往返与调度抖动；物理本身只花约 21 µs（C 代码）。
2. 50 Hz 控制层每拍有 20 ms，策略前向 0.70 ms（p99 1.45 ms），余量充足。
3. 真机若要在 PC 侧发 500 Hz，建议绑核 / RT 内核 / C++ 节点；仿真有 `Pacer`、零阶保持
   和看门狗兜底，不受影响。

## 5. 后端状态

| 后端 | 状态 | 说明 |
|---|---|---|
| `loopback` | ✅ 已验证 | 单进程，观察器 / verify / 测试 |
| `udp` | ✅ 已验证 | 双进程裸 UDP，报文语义 / 频率 / CRC 与 DDS 一致 |
| `ros2` | ✅ 已验证 | rclpy + `rmw_cyclonedds_cpp`，话题与真机一致 |
| `dds` | ❌ Python 3.14 不可用 | `unitree_sdk2py` 的 cyclonedds Python 绑定 import 即崩，见 `docs/ros2-setup.md` §8 |

## 6. 地形 / 策略评测（摘要）

31 个地形（`sim_env/models/go2/terrains.json`），CPU MuJoCo 48 seed 闭环：

| 指标 | `go2_v29c` | `go2_v32b` | **`go2_v33d`（默认）** |
|---|---|---|---|
| 总存活 | **87.5%** | 83.3% | 86.7%（噪声内） |
| 速度 / 位移 | 0.291 m/s / 3.72 m | 0.340 m/s / 4.45 m | **0.323 m/s / 4.23 m** |
| hyb2 / stg1 / big1 / rubble | **91.7** / 68.8 / **87.5** / 89.6% | 70.8 / 68.8 / 79.2 / 97.9% | 81.2 / **72.9** / 79.2 / **100.0%** |

台阶能力（真竖面，128 env × 300 步）：v33d 在 `wide_h030` 上 1 级 100% / 2 级 98.4%、
`wide_h050` 上 1 级 100% / 2 级 82.8% / 存活 95.3%。

## 7. 已知边界

- 真机验证到仿真 + 消息层 + ROS 2 双进程；**尚未在真实 Go2 上验证**，上机前按
  `real_env/README.md` 走 checklist（只读自检 → 吊起 zero → stand → policy）。
- 真机 `real_ros` 路径已修好：ROS 2 话题名 `/lowstate` `/lowcmd`、LowCmd 固定头
  + 812 B CRC（`signal_io/go2_crc.py`）、CycloneDDS 网卡配置（旧写法 deprecated/不稳定）、
  LowState 断流按墙钟升级 + 退出阻尼、`_to_ros_cmd` 20 倍加速。
- `real_sdk` 在当前 Python 3.14 环境不可用（launch 会显式警告并回退 `real_ros`）。
- 高度扫描：真机默认 `--scan leg_fk`（平地近似正确）；L1 点云接真机前需标定
  外参/轴向 + 可靠 odom，见 `real_env/README.md` §6。
- `dds` 后端需 Python 3.12/3.13 环境。
