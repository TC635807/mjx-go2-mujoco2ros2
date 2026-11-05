# 明天上机（Day-0）作战手册

> 面向：2026-11-05 首次 Go2 sim2real 实验、**以 PC 作主控**（机载电脑路线见 §2.3）。
> 本页是"当天按顺序照做"的一页纸；原理与缺口见
> [safety-audit.md](safety-audit.md)、[real-bringup.md](real-bringup.md)、[portability.md](portability.md)。
>
> 三条硬前提（缺一条就改天）：机器人是 **EDU/R&D**、场地能**吊起**、遥控器在手。

---

## 0. 明天的最小目标（别贪）

| 阶段 | 成功判据 | 不做 |
|---|---|---|
| A. 链路 | `/lowstate` ≈500 Hz、电机 `mode=0x01`、`/lowcmd` 没有别的发布者 | 不动电机 |
| B. 吊起 | `zero` 10 s 无异常；`stand` 关节平滑走到站姿 | 不落地 |
| C. 落地站住 | `policy --cmd 0 0 0` 站 20 s 不摔、`health` 几乎全 L0 | 不上台阶/草地 |
| D. 走 2 m | 发布 `/go2/cmd_vel 0.2 0 0`，直走 2 m 后回零，能停住 | 不做转向/加速 |

四步都过 = 首测成功。**任何一步不对就停在该步排查，不要"再试一次说不定就好了"。**

---

## 1. 时间线（总预算 60~90 min）

| 时间 | 做什么 | 命令 | 期望 / 失败判据 |
|---|---|---|---|
| T-30 | 机器人吊起 5~10 cm、遥控器 **L2+R2**（debug mode）、网线接 PC、电池充满 | — | 关节进入阻尼态 |
| T-20 | PC 有线网卡配静态 IP + 起来 | 见 §2.2 | `ip -br addr` 看到 `192.168.123.x/24` |
| T-15 | `ping 192.168.123.161` | `ping -c3 192.168.123.161` | 通 → 物理链路 OK；不通 → §5.1 |
| T-10 | **环境体检**（只读） | `bash scripts/real_run.sh --stage preflight --iface <网卡>` | `env_check` 无 FAIL；FAIL → 按"修"逐条处理 |
| T-5 | **传感器自检**（只读，不发指令） | 同上（preflight 已含）；单独跑见 §3 | 25 项里 PASS，`lowstate.rate≈500`、`motors.mode` 全 0x01 |
| T+0 | 吊起 `zero` 10 s | `bash scripts/real_run.sh --stage zero` | `n_state>0`、`health` 基本 L0；电机没进伺服 → §5.3 |
| T+5 | 仍吊起 `stand` 15 s | `bash scripts/real_run.sh --stage stand` | 关节 1.5 s 内平滑走到站姿（`ramp`/`start_delay` 已修生效） |
| T+10 | 保持 stand 的同时**缓慢落地**（有人扶/吊带缓降） | — | 落地后腿不抖、不塌 |
| T+15 | 落地站住：`policy --cmd 0 0 0` | `bash scripts/real_run.sh --stage policy --duration 30 --cmd "0 0 0"` | 站住不漂；`level_hist` 几乎全 L0 |
| T+25 | 另开终端给速度 | `bash scripts/run_with_ros2.sh scripts/cmd_vel.py 0.2 0 0 --duration 15` | 直走；3 s 不发消息自动回零 |
| T+35 | 收尾 | 控制器 `Ctrl-C`（会发 100 ms 阻尼） | 看 summary 6 个数（§4） |

> `policy` 阶段**不要**用 `--cmd 0.5 0 0`（配置默认值）直接起步：先 0，站稳后再 0.2。

---

## 2. 主控机怎么选（**先看这节**）

### 2.1 现状（已实测）

* 本机 `/etc/os-release` = **Ubuntu 25.10**，**没有 ROS 2**（`/opt/ros` 不存在、无 `rclpy`/`colcon`）。
  宇树官方只支持 20.04/22.04 + Foxy/Humble，且固件用 CycloneDDS **0.10.2**。
* 但本机 **Docker 已经装好**，且 `unitree-go2-ros2:humble` 镜像**已经构建过**（2.73 GB），
  我实测容器里 rclpy / unitree_go / rmw_cyclonedds_cpp / colcon / mujoco 全可用、能加载本仓模型。

### 2.2 推荐：docker 官方栈当主控（本机不用装 ROS 2）

~~~bash
# 1) 有线网卡起来 + 静态 IP（网卡名以 ip -br addr 为准，本机是 enp0s31f6，当前 DOWN）
sudo ip link set enp0s31f6 up
sudo ip addr add 192.168.123.99/24 dev enp0s31f6
ip -br addr show enp0s31f6          # 应看到 192.168.123.99/24
ping -c3 192.168.123.161            # 机器人运控

# 2) 用官方栈容器（host 网络，能直接看到机器人网卡）
cd /home/wyx/mujoco/mjx-go2-mujoco2ros2
UNITREE_NIC=enp0s31f6 ROS_DOMAIN_ID=0 \
  docker compose -f docker/docker-compose.yml run --rm ros2 bash
# 进容器后：
python3 scripts/env_check.py --target real_ros --iface enp0s31f6
bash scripts/real_run.sh --stage preflight --iface enp0s31f6
~~~

容器里的两条规矩（很重要）：

* **Python 用 `python3`（= /opt/venv，py3.10 + numpy + mujoco），不要用 `.venv/bin/python`**
  —— 后者是宿主机 py3.13 的 venv，容器里 import numpy 会失败。
* `bash scripts/run_with_ros2.sh ...` 是唯一入口（它会自己 source ROS + 设 `GO2_PYTHON`）。

### 2.2b 单文件交付：把「工具链 + 代码」全打包成一个镜像（离线/换机器用）

上面的 compose 用法是 **bind-mount**（代码挂进去，改完立刻生效，适合今天调参）。
如果要**拷到另一台机器 / 没有网络 / 不想让对方装任何东西**，用自包含镜像：

~~~bash
# 源机（本机，约 22 秒）：把仓库 COPY 进镜像
bash docker/build_deploy_image.sh --build --verify

# 导出成单文件（实测 604 MB；镜像各层已压缩，zstd 后约 600 MB）
docker save -o /tmp/go2-deploy.tar go2-deploy:day0

# 目标机：只需要 docker，不需要网络 / pip / colcon / 重建
docker load -i go2-deploy.tar
docker run --rm -it --network=host -e UNITREE_NIC=<接狗网卡> -e ROS_DOMAIN_ID=0 go2-deploy:day0 \
    bash -lc "python3 scripts/env_check.py --target real_ros --iface $UNITREE_NIC"
~~~

| | compose（bind-mount） | 自包含镜像 |
|---|---|---|
| 宿主 `.venv` / `ros2_ws/install` | 容器看得见但不可用 → 必须 `GO2_SKIP_PROJECT_WS=1` | 不存在（`.dockerignore` 排除），不会误用 |
| 改代码 | 保存即生效 | 要重新 `--build` |
| 搬运 | 镜像 + 仓库两份 | **一个 tar** |

> 实测：镜像内（**无任何挂载**）`env_check` = 20 OK / 1 WARN / 0 FAIL（WARN 只是 `ulimit -r=0`）；
> 镜像内直接跑 `sim_ros` 闭环也正常收帧。细节见 [`../docker/DEPLOY.md`](../docker/DEPLOY.md)。

### 2.3 备选：机载电脑（把程序跑在狗上）

* 条件：Jetson/机载算力板 = Ubuntu 22.04 + Humble（`192.168.123.18` 之类）；
  需要**重建** `.venv` 与 `ros2_ws/install`（不能整目录复制，见 [portability.md](portability.md)）。
* 打包 → 目标机装机 → 自检，一条龙：

~~~bash
bash scripts/package_for_onboard.sh --verify          # 生成 /tmp/go2-deploy-*.tar.zst + 离线自检
# 拷到机器人上后：
bash scripts/onboard_setup.sh --with-ros              # 建 venv + pip + colcon build unitree_go
python3 scripts/env_check.py --target real_ros --iface <网卡>
~~~

### 2.4 不推荐：在本机原生装 ROS 2

25.10 不是任何 ROS 2 发行版的 Tier-1 平台，装完也要面对 CycloneDDS 11 与固件 0.10.2 的互通问题
（见 [unitree-official-sdk-conformance.md](unitree-official-sdk-conformance.md)）。**明天别在这上面花时间。**

---

## 3. 每一步到底在查什么

### 3.1 环境体检 `scripts/env_check.py`

~~~bash
python3 scripts/env_check.py --target real_ros --iface enp0s31f6 --json logs/env.json
~~~

查 24 项：python/venv/numpy/mujoco、ROS 2 前缀与 CLI、`rclpy`/`rmw_cyclonedds_cpp`/`libddsc`、
`unitree_go` 消息包、`ROS_DOMAIN_ID`/`RMW`/`CYCLONEDDS_URI`/`ROS_LOCALHOST_ONLY`、目标网卡存在与 IP、
`ping 192.168.123.161`、CPU/实时优先级/磁盘、git 版本、脚本可执行位。
退出码：有 FAIL = 1。**有 FAIL 不要往下走**（现场硬上要显式 `--force`）。

### 3.2 传感器只读自检 `scripts/real_sensor_check.py`

~~~bash
bash scripts/run_with_ros2.sh scripts/real_sensor_check.py \
    --target real_ros --duration 5 --iface enp0s31f6 --json logs/sensor.json

# 只判"链路健不健康"时用轻量模式（不订阅点云/pose/odom，见下面的实测）
bash scripts/run_with_ros2.sh scripts/real_sensor_check.py \
    --target real_ros --duration 6 --iface enp0s31f6 --light
~~~

> **本机实测（2026-11-05，有线直连 192.168.123.161）**：同一根线上
> `ros2 topic hz /lowstate` = **500.01 Hz**、裸 rclpy 订阅 **499.8 Hz**（max gap 3.6 ms），
> 但全量自检只报 **190~275 Hz**（看起来像 FAIL）—— 差别全在本脚本自己的单线程 executor
> 被点云 / odom 解析占住。加 `--light` 后 **496.0 Hz**、`state` 组 PASS。
> 所以：**判链路用 `--light`，点云/雷达项单独跑一次完整模式**（两者都要跑一次）。

**只订阅、绝不发布**（源码级检查 + 建节点后断言 `node.publishers == []`），站立/吊起都能跑。
25 项判定，其中上机必须看：

| 项 | 期望 | 不对就说明 |
|---|---|---|
| `lowstate.rate` | ≈500 Hz | 网卡/domain/CYCLONEDDS_URI 不对 |
| `lowstate.tick` | Δtick 稳定（真机 1 ms 递增） | DDS 丢帧严重 |
| `motors.mode` | 12 个全 `0x01` | 没 release sport mode / CRC 错 / 没进 debug |
| `motors.lost` | 全 0 | 电机通信有问题 |
| `motors.temperature` | < 70 ℃ | 别上机 |
| `imu.rpy/accel/gyro` | 有限、`|accel|≈9.81`、静止 gyro 噪声小 | IMU 没使能 |
| `foot_force` | 4 路非 0（AIR/PRO 可能没有 → SKIP） | 足端传感器缺失（策略不依赖，但要知道） |
| `lowcmd.publisher` | 0（没有别的进程在发） | 有别的控制器在跑，先停掉 |
| `lidar.rate/frame_id` | ≈11 Hz、`utlidar_lidar` | L1 没开或话题名不同 |

`SKIP` ≠ 通过（表示"这个信号本机没有"）；如果它在 `--require` 里，退出码会变 WARN/1。

> 注意：`preflight` 里**传感器自检 FAIL 不会自动阻断**后续阶段（设计如此——控制器在收到第一条 `/lowstate` 之前
> 一条指令都不发，所以"没数据"本身不会让机器人动）。因此**看到 `lowstate.rate FAIL` 必须人工停下排查**，
> 不要因为脚本继续跑了就以为没问题。`env_check` 的 FAIL 才会硬阻断（要 `--force` 才能越过）。

### 3.3 分阶段上机 `scripts/real_run.sh`

~~~bash
bash scripts/real_run.sh --dry-run                    # 先看它要跑什么（本机随时可跑）
bash scripts/real_run.sh --stage preflight            # 只读
bash scripts/real_run.sh --stage zero                 # 吊起，10 s 零力矩
bash scripts/real_run.sh --stage stand                # 吊起，15 s 站姿
bash scripts/real_run.sh --stage policy --duration 30 --cmd "0 0 0"
bash scripts/real_run.sh                              # 全流程（每步要输 yes）
~~~

它做的事：env_check 闸门 → 传感器自检 → 按阶段起 `run_controller.py`（一律经
`scripts/run_with_ros2.sh`）→ 每阶段要人确认 → `--max-duration`（默认 60 s）硬上限 →
日志落 `logs/real_<时间戳>/<stage>.json` → 退出时打印安全提醒。

### 3.4 运行期改速度 `scripts/cmd_vel.py`

控制器订阅 `/go2/cmd_vel`（`geometry_msgs/Twist`），所以**不用重启**就能提速：

~~~bash
bash scripts/run_with_ros2.sh scripts/cmd_vel.py 0 0 0 --duration 5      # 先归零
bash scripts/run_with_ros2.sh scripts/cmd_vel.py 0.2 0 0 --duration 15   # 直走
bash scripts/run_with_ros2.sh scripts/cmd_vel.py --keys                  # w/s/a/d 交互
~~~

三轴被夹在 `vx[-0.2,0.8] / vy[-0.3,0.3] / wz[-0.8,0.8]`（训练命令分布）；
**3 s 没有新消息自动回零**（`--cmd-timeout`）。

#### 实物遥控器直接驱动（`--remote`，2026-11-05 实测标定）

遥控器数据在 `/wirelesscontroller` 上（`lx/ly/rx/ry` + `keys`，实测 **9~10 Hz**），
控制器加 `--remote` 就能把它映射成同一条速度指令（复用上面的限幅/回零）：

~~~bash
bash scripts/go2_docker.sh python3 ros2_controller/run_controller.py --target real_ros \
    --controller policy --cmd 0 0 0 --remote --duration 60
~~~

| 动作 | 轴 | 实测范围 | 映射（默认 `ly,lx,rx` / 增益 `0.8,-0.3,-0.8`） |
|---|---|---|---|
| 左摇杆上/下 | `ly` | +1.00 / -0.92 | `vx = +0.8*ly`（上是前进） |
| 左摇杆左/右 | `lx` | -1.00 / +0.99 | `vy = -0.3*lx`（体坐标 +y=左） |
| 右摇杆左/右 | `rx` | -0.82 / +0.93 | `wz = -0.8*rx`（+wz=左转） |
| 右摇杆上/下 | `ry` | 未用（机身高度/俯仰） | — |

安全：**按住 `L1` = 速度立即归零**；遥控器静默 0.6 s 也自动回零。

### 3.5 守护进程 `scripts/failsafe_guard.py`（可选，第一次默认不真发）

控制器被 `kill -9`/`段错误`/`PC 休眠`时，`finally` 里的阻尼**不会执行**。这个独立进程
订阅 `/lowcmd`，断流 250 ms 就替控制器发 `kp=0/kd=8` 阻尼：

~~~bash
bash scripts/run_with_ros2.sh scripts/failsafe_guard.py --target real_ros --duration 60
# 默认只打印 "WOULD PUBLISH"；确认过 sim_ros 之后才加 --enable-publish
~~~

> 第一次上机建议**只观察不真发**（`n_trips` 应为 0）。它以 250 ms 灵敏度触发，说明指令流本来就不连续。

---

## 4. 每轮结束必看的 6 个数字（summary 里）

| 字段 | 阈值 | 含义 |
|---|---|---|
| `n_state` / `n_cmd` | 都 > 0 且接近 500×时长 | 收/发是否正常 |
| `n_publish_err` | **必须 0** | publish 抛异常（指令没发出去） |
| `max_loop_gap_ms` | 偶发 < 20 ms | 循环停顿（调度/GC/CPU 抢占） |
| `pacer.overruns` | 占比 < 1% | 500 Hz 抖动 |
| `health.level_hist` | L1+L2+L3 合计 < 1% | 健康层是否频繁降级 |
| `transport.bad_crc` + `safety_supervisor.mode_hist` | bad_crc=0；crouch/damping/estop 只在收腿时出现 | CRC/安全层 |

> **CPU/负载警告（今天实测）**：同一台机器上跑 `sim.launch.py`（含 perception 节点 900 射线@15 Hz + 观察器）时，
> 容器里的 controller 会掉到 `state_age≈30 ms` → 触发 L1 `state_jitter` → 下蹲 → 进而 L2 阻尼；
> 去掉 perception、只跑 sim+controller 后立刻恢复：`overruns` 有但 `lag_p95=0.94 ms`、`health` **2999/2999 全 L0**、收帧 ≈493 Hz。
> 所以真机上：**只跑 controller（`--scan leg_fk`），不要同时开 perception / viewer**；
> 必须开时先用 `taskset -c` 把 controller 绑到固定核，并盯 `max_loop_gap_ms`。

~~~bash
# 一条命令看 summary 关键字段
python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print({k:d.get(k) for k in ('n_state','n_cmd','n_publish_err','max_loop_gap_ms')}, d.get('level_hist'), d.get('mode_hist'))" \
    logs/real_*/policy.json
~~~

---

## 5. 出问题按这个顺序查

1. **网线/网段/IP**：`ping 192.168.123.161`；PC 不能设成 `.161`；`ip -br addr` 看网卡是否 UP 且有 `192.168.123.x`。
2. **ROS_DOMAIN_ID=0**（仿真用 1，别串台）；`ROS_LOCALHOST_ONLY` 必须未设置。
3. **网卡**：`CYCLONEDDS_URI` 绑的必须是接狗那张（`GO2_IFACE=<网卡>`；config 里默认写 `eth0`）。
4. **`ros2 topic list` 能看到 `/lowstate` 吗**；`ros2 topic hz /lowstate` ≈500 Hz。
5. **电机不动**：① 释放 sport mode（L2+R2 或 release 脚本）② 电机 `mode=0x01` ③ CRC（本仓已实现并有官方 gold 值）。
   * **L2+R2 没反应的典型原因**：狗还趴着/处于 `mode=0 idle`（`/sportmodestate` 的 `mode`：0=idle、1=balanceStand、5=lieDown、7=damping）。
     正确顺序：**先吊起 → `L2+A` 让它站起来 → 再按 `L2+R2`**（它会立刻变软，吊带被拉紧）。
   * 判据：进了调试模式后 `/lowcmd` 的 **Publisher count 应变成 0**（固件不再喂自己的阻尼指令），
     `/sportmodestate` 可能停发。还没变 = 没进去，别往下走。
   * 遥控器无效时走 App：**单元树 Go2 App → 设置 → 调试模式/开发者选项**。
   * ⚠️ **按 L2+R2 的瞬间四肢疯狂抽搐**（2026-11-05 实测踩过）：当时机上没有任何外部 LowCmd 源，
     机器人自己却在以 500 Hz 发阻尼指令（`/lowcmd` publisher=1、`kp=0 kd=3`）；进调试模式那一刻
     内部指令源停摆/切换，电机在高频换目标 -> 抽搐。**正确顺序**：先把控制器跑起来、确认
     `pacer.hz≈500` 且 `n_cmd` 在涨，**再短按一次** L2+R2。**长按**可能让固件反复进/出调试模式，
     每次切换都重新使能电机，同样会抽。抽搐时关节扭矩接近满额（小腿 45 N·m），**先断电，别碰腿**。
   * ⚠️ **容器里不要 `source /opt/unitree_ros2/setup.sh`**：官方原版写死 `foxy` 和网卡 `enp3s0`，
     一 source 就把入口设好的 `CYCLONEDDS_URI` 覆盖成不存在的网卡，报
     `enp3s0: does not match an available interface` + `rmw_create_node: failed to create domain`。
     入口已经 source 过 `/opt/setup_unitree_humble.sh`，直接敲 `ros2 ...` 即可。
     （`go2-deploy:*` 镜像已把该脚本替换成"重新应用正确环境"的版本，bind-mount 那套仍是原版。）
6. **`bit_flag` bit0 常亮** = 机器人自己报"指令超时" → 查 CRC/网卡/双发布者，不要怀疑策略。
7. **走路时突然下蹲又起来** → 不是策略：看 `health.level_hist`（L1 crouch）与 `pacer.overruns`。
8. **L3 之后再也不动** → L3 是锁存 + 卸力/阻尼，只能重启进程。

---

## 6. 安全红线（写进肌肉记忆）

1. 手不离开遥控器；机器人吊起时**人不要站在它的腿旁边**。
2. 只在落地 + 场地净空 + 有人扶着起步；`--cmd` 从 0 开始。
3. 不用 `--force-unsafe` 去拆健康层/安全层（real_* 已默认拒绝，绕过要显式加）。
4. 不同时跑 `real.launch.py` 和手动 `run_controller.py`（两个 500 Hz 发布者抢 `/lowcmd`）：
   起之前 `ros2 topic info -v /lowcmd | grep -A3 "Publisher count"`。
5. 不用默认 `--duration 20/30` 一把梭；先 10 s。
6. 不用 `--scan topic`（L1 外参未标定，见 [real_env/README.md](../real_env/README.md) §6）。
7. 不用 `--linvel oracle`（独立进程会退回 estimator）、不用 `--no-realtime`。
8. 不要指望低电压保护（**还不存在**，见 safety-audit §5.1）：电池充满、带充电器。
9. `Ctrl-C` 会发 100 ms 阻尼；`kill -9` / 拔网线 / 断电**不会** → 手按急停。
10. 看到 `L3` 先接遥控器 —— 它是"卸力/阻尼"，不是"软停"。
11. **遥控器 `L2 + B` = 阻尼模式 / 软急停（L2 安全模式）** —— 现场第一急停手段。
    ⚠️ 它是否独立于 `mcf` 还没实测（`mcf` 释放期间是否仍生效）：验证办法是跑
    `scripts/probe_lowcmd_sources.py` 时按 `L2+B`，看 `/lowcmd` 上是否出现新的阻尼帧签名。
    验证之前，`mcf` 释放期间请同时依赖「按住 L1 归零」+ 断电。
12. `mcf` 释放（`--release-mode`）之后，属于 `mcf` 的功能（`L2+A` 站起、摇杆走路等）会失效；
    恢复官方控制用 `scripts/go2_service.py --select-mode mcf --yes`。

---

## 7. 今天交付了什么（索引）

| 你要的 | 交付物 | 验证情况 |
|---|---|---|
| ① 实机传感器只读检测 | [scripts/real_sensor_check.py](../scripts/real_sensor_check.py) + [signal_io/real_diag.py](../signal_io/real_diag.py) + [tests/test_real_diag.py](../tests/test_real_diag.py) | 25 项判定；`--selftest` 本机可跑；真实 rclpy 未跑 |
| ② 安全审计（疯车/力矩/失控/断链自动停） | [docs/safety-audit.md](safety-audit.md) + `q_des` 限速 + 危险开关闸门 + [scripts/failsafe_guard.py](../scripts/failsafe_guard.py) | 单测 88 项；guard 未真发 |
| ③ 本机主控启动脚本 | [scripts/real_run.sh](../scripts/real_run.sh)（分阶段 + 闸门 + 日志） | `--dry-run` 本机通过；真机未跑 |
| ④ 环境检查 | [scripts/env_check.py](../scripts/env_check.py) + [scripts/cmd_vel.py](../scripts/cmd_vel.py) | 本机 + 容器内均实测 |
| ⑤ 可迁移性 | [docs/portability.md](portability.md) + [scripts/package_for_onboard.sh](../scripts/package_for_onboard.sh) + [scripts/onboard_setup.sh](../scripts/onboard_setup.sh) | 打包 24 MiB/319 文件 + 7 项离线自检 PASS；目标机未跑 |

**今天在仿真里验证过的真实链路**（不是"应该能行"）：

* 容器内 `sim_ros` 三节点闭环：controller 收到 2153 帧 `/lowstate`，`bad_crc=0`（真机 CRC 812B 布局在闭环里被校验）。
* `real_ros` profile 对着仿真数据源跑 `stand`：`q_slew` 生效（16 次限速，最大单拍 0.135 rad 被压到 0.04），
  `health` L0 为主、无 L2/L3，退出时发 100 ms 阻尼。
* 没有机器人时启动 `real_ros` 控制器：**一条指令都不发**（`n_cmd=0`），1 s 后打印明确错误并按时退出。

---

## 8. 已知缺口（今天没修，明天要认）

1. **低电压无保护**：`power_v` 没有从 ROS 消息搬进内部控制消息（可选检查默认关，恒 SKIP）。
2. **真机力矩限制只有 `q_des` 限速 + 固件电机极限**（我们夹的 `tau` 字段在 position 模式下恒 0，夹了没用）。
3. **进程级崩溃没有"保证"兜底**：guard 是概率性的（它自己也会死；断电/断网时无效）。
4. **L3 锁存后不能运行期恢复**：只能重启进程。
5. **双发布者无软件互斥**：只能人工 `ros2 topic info` 数。
6. **丢包率不可见**：health 只看"多久没收到整帧"，不看 `tick` 连续性。
7. ~~所有 rclpy 代码路径都没在本机跑过；没有验证过真实固件的 `/lowstate` 内容、CRC 被接受、`mode=0x01` 生效~~
   → **已于 2026-11-05 实机验证**：`/lowstate` 500 Hz、12 电机 `mode=0x01`、我们的 LowCmd 被固件接受
   （下发 `kd=0/8` 后关节手感立刻变化）。详见 [§10](#10-实机首测记录2026-11-05go2--本机-docker-主控)。

---

## 9. 相关文档

* [real_env/README.md](../real_env/README.md) —— 真机部署总览（网络/domain/释放运控/上电顺序）
* [real-bringup.md](real-bringup.md) —— 现场 runbook（含容器一节）
* [safety-audit.md](safety-audit.md) —— 安全审计：失效→保护→缺口→残留风险
* [portability.md](portability.md) —— 搬到机载电脑/实验室机器还能不能跑
* [docker/README.md](../docker/README.md) —— 官方推荐栈容器（真机路径）
* [unitree-official-sdk-conformance.md](unitree-official-sdk-conformance.md) —— 与官方 SDK 的逐项对照

---

## 10. 实机首测记录（2026-11-05，Go2 + 本机 docker 主控）

> 本节是**实测结果**，不是计划。§0~§9 是出发前写的；这里记录真实发生了什么、
> 哪些假设被推翻、哪些坑写进了代码。

### 10.1 一句话总结

链路、固件兼容、接管流程全部打通：`/lowstate` 满 500 Hz、我们的 LowCmd 被固件接受、
策略能跑满 120 s 且健康层全程 L0。**卡住的是观测失配**——策略在"无指令"时自行向左前方走，
`n_action_clamped = 99.95%`，速度估计置信度只有 0.307。这是 sim2real 的观测域问题，
不是链路/安全问题。

### 10.2 实测通过的部分（可复现）

| 项 | 结果 |
|---|---|
| 有线链路 | `ping 192.168.123.161` 0.2~0.9 ms；`enp0s31f6` = 192.168.123.222/24 |
| `/lowstate` | **500.0 Hz**，std 0.43 ms，max gap 3.6 ms（裸 rclpy 订阅 499.8 Hz） |
| 容器内环境 | `env_check` **22 OK / 2 WARN / 0 FAIL**（WARN = rtprio、git 未提交） |
| 传感器自检（全量 25 项） | 21 PASS / 4 WARN / 0 FAIL（雷达 15.5 Hz、4004 点/帧、IMU/足端/电量正常） |
| 传感器自检（`--light`） | 15 PASS / 3 WARN / 0 FAIL，`state` 组 PASS（496.0 Hz） |
| 电机 | 12 个 `mode=0x01`、`lost` 无新增、26~41 ℃、tau ≈ 1% 额定 |
| **固件接受我们的 LowCmd** | **是**：下发 `kd=0`/`kd=8` 后关节手感立刻变化，可与固件的 `kd=1/3` 区分 |
| 控制器实跑 | 60 s：`n_state=29997 n_cmd=29998`、500 Hz、`level_hist={L0:29998}` |
| 策略实跑 | 120 s：`n_state=59562 n_cmd=60000`、`level_hist={L0:60000}`、`mode_hist={track:60000}` |
| 遥控器链路 | `/wirelesscontroller` 9~10 Hz；轴与按键实测标定见 §3.4 |

### 10.3 四个被推翻的假设（今天最大的收获）

**① 固件 > 1.1.6 的主运控服务叫 `mcf`，不是 `sport_mode`。**
官方 Quick_start：「`mcf` = Go2 主运动控制服务（软件版本 > 1.1.6）；`sport_mode` = < 1.1.6」，
`MotionSwitcher.CheckMode()` 返回 `{"name":"mcf"}` 印证。**我们一开始关的是 `sport_mode`（旧名字），所以关了没用。**

**② `ServiceSwitch` 关服务是临时的，约 7.5 s 后被看门狗拉起。**
实测：stop 后 `t=+1.5/3.0/4.5/6.0 s` 都是 0，`t=+7.5 s` 自动回到 1。
官方正确做法是 **`MotionSwitcher.ReleaseMode()`**（`go2_stand_example` 就是这么写的）。
释放后 `CheckMode()` 返回 `name=''`，不再自恢复。

**③ 「多个运控同时存在 → 指令混乱 → 失控」是官方明写的。**
这就是两次「四肢抽搐」「上下抖动」的根因：mcf 在发 `kp=40` 位置保持，我们在发 `kp=0/kd=8`，
两个 500 Hz 源交替喂 MCU。**释放 mcf 之后这些现象全部消失。**

**④ 「kd=3 阻尼」不是别人留下的程序，是 mcf 自己的输出。**
释放 mcf 后 `/lowcmd` 兜底签名从 `kp=0/kd=3.0` 变成 `kp=0/kd=1.0`（低层桥弱兜底）；
重启后又变回 `kp=40/kd=1.0`（mcf 位置保持）。中途曾误判为「机载电脑上有残留程序」，
查了 SSH/端口/`bashrunner` —— **是误判，已排除**。

### 10.4 修掉的两个健康层 bug（都上过真机）

**① `motor_state[i].lost` 是累计计数器，被当成当前状态判 L2。**
`RL_thigh.lost = 5`（其余 11 个为 0，q/dq/温度全正常），旧逻辑 `lost != 0 → L2`，
导致**每次启动都 100% L2 → 监督器 crouch→damping → 四脚往里收、站不住**。
修法：改成「1 s 窗口内新增 ≥ 5」或「自基线累计 ≥ 50」才判 L2。

**② `bit_flag` bit0（操作指令超时）单帧就判 L2。**
bit0 会在**我们第一帧 LowCmd 到达之前**闪一下，旧逻辑瞬间 L2，迟滞阶梯要 2 s 才回 L0，
真机表现就是「一启动就 crouch 两秒」。修法：bit0/bit7 加 **0.3 s 去抖**（真断流仍在 300 ms 内升级）。

两处都补了单测（`tests/test_node_loop.py` §4）。

### 10.5 接管流程（现在唯一正确的顺序）

~~~bash
# ① 狗吊起或趴下（ReleaseMode 后 mcf 不再保持姿态）
# ② 先起我们的指令流（zero/damp/policy），确认 n_cmd 在涨
# ③ 再释放 mcf
bash scripts/go2_docker.sh python3 scripts/go2_service.py --release-mode --yes
# ④ 判据：CheckMode name=''、/sportmodestate 停发、level 保持 L0、手感变化
# 恢复官方控制：
bash scripts/go2_docker.sh python3 scripts/go2_service.py --select-mode mcf --yes
~~~

**顺序反了就会抽搐**（先停 mcf、后起指令流 = 中间有一段没有任何指令源）。

### 10.6 策略首测结果与遗留问题

| 指标 | 实测 | 判断 |
|---|---|---|
| `n_state / n_cmd` | 59562 / 60000（500 Hz） | ✅ 链路满速 |
| `level_hist` | `{L0: 60000}` | ✅ 健康层全程绿 |
| `mode_hist` | `{track: 60000}` | ✅ 监督器没介入 |
| **`n_action_clamped`** | **5922/5925 = 99.95%** | ❌ 策略输出几乎全程越界被夹 |
| `action_abs_mean / max` | 0.81 / 1.55 | ❌ 输出幅值偏大 |
| **`linvel_confidence`** | **0.307**（下限 0.3） | ❌ 腿式速度估计基本不可信 |
| `base_lin_vel_world` | `[0.054, -0.031, 0.338]` | ❌ 站立时 z 向 0.34 m/s = 估计在漂 |
| `height_scan_source` | `leg_fk`（训练是 `raycast`） | ⚠️ 分布不同 |
| `overruns` | 18842/60000 = **31%** | ❌ 500 Hz 有 1/3 没踩到 deadline |

**现象**：无遥控器输入时（`last_cmd=[0,0,0]`、`timed_out=true`）机器人仍自行向左前方走。
判断：不是遥控器链路问题（`live_cmd.n_sets=416` 正常），而是**速度估计在漂 → 策略"以为被推动"
→ 输出补偿 → 越界被夹 → 实际走起来**。

**下一步（按优先级）**：
1. `--scan nominal`（平地假设）vs `--scan leg_fk` 的 A/B，看 `n_action_clamped` 是否下降
2. 降 CPU 负载（31% overruns 本身就会毁掉策略表现）
3. 用 `sim_env/run_sim.py --profile real_mild / real_severe` 复现，判断是"噪声域差"还是"漏了真机信号"
4. `--publish-debug` 录 obs，逐段对比 sim vs real

### 10.7 今天新增/改动的工具

| 工具 | 作用 |
|---|---|
| [scripts/go2_service.py](../scripts/go2_service.py) | `--list / --check-mode / --release-mode / --select-mode`：开关机载服务 + 释放/恢复运控 |
| [scripts/go2_docker.sh](../scripts/go2_docker.sh) | 一条命令把任意脚本送进容器跑（宿主机没装 ROS 2） |
| [scripts/probe_lowcmd_sources.py](../scripts/probe_lowcmd_sources.py) | 只读：按帧签名统计 `/lowcmd` 上有几个指令源、各占多少 |
| [scripts/probe_remote.py](../scripts/probe_remote.py) | 只读：实时解遥控器按键位图 + 摇杆轴（L2=bit5、R2=bit4 实测） |
| [scripts/probe_mode_switch.py](../scripts/probe_mode_switch.py) | 只读：抓"切模式瞬间"的 500 Hz 数据（多腿同时抽=服务问题 / 单腿=硬件） |
| `--controller damp --damp-kd N` | 纯阻尼控制器：只改 kd、无位置环，安全验证"控制权是否在我们手里" |
| `--remote*` | 实物遥控器驱动速度指令（映射/增益/死区/失联回零）+ **`L2+B` 按键阻尼急停**（不依赖 mcf） |
| `--light`（传感器自检） | 只订阅低层流，避免点云解析把自己的 `/lowstate` 计数拉低（190~275 Hz → 496 Hz） |
| 1 Hz 进度行（`node.py`） | 真机跑 60 s 能看见 `n_state/n_cmd/Hz/level/mode`，否则全程黑屏 |

### 10.8 下次开工第一件事

1. 跑 §10.6 的 `--scan nominal` A/B，把 `n_action_clamped` 报出来
2. 若 `--scan nominal` 明显更好 → 真机感知源是主要失配源，接下来标定 L1 外参或换扫描方案
3. 若差不多 → 查速度估计器（`linvel_confidence`），必要时给 obs 里的速度项加滤波/零偏补偿

---

## 11. 第二轮实测记录（同日 14:00~16:00，策略部署与摔倒复盘）

> §10 是第一轮（链路/接管）。这一节是第二轮：真机部署策略、摔倒、以及对前面几个
> 判断的**更正**。有些结论是推翻重来的，保留原文以便追溯。

### 11.1 一句话总结

策略能跑、能站、能被遥控器驱动，但有两类问题必须分开修：
**① 策略在 `cmd=0` 时本来就会往前走（仿真同样复现）→ 训练侧问题；**
**② 真机比仿真多一个向左的分量 → 速度估计的零偏（sim2real 侧问题）。**

### 11.2 三个被更正的判断（重要，别再引用旧结论）

**① `n_action_clamped = 100%` 是正常的（我一开始判错了）**

| 运行 | 动作被夹紧 |
|---|---|
| 仿真（同策略 + nominal + 完美平地） | **886/886 = 100.00%** |
| 真机 A（`leg_fk`） | 1425/1425 = 100.00% |
| 真机 B（`nominal`） | 1425/1425 = 100.00% |

策略的关节目标天生就会撞关节限位（action scale + 默认姿态）。这个数**只能做运行间
相对比较**，不能用来判断"观测是否在训练分布"。`scripts/show_summary.py` 里的告警已改掉。

**② 「零指令自己走」不是速度估计的假速度造成的** —— 见 §11.3 的仿真复现。

**③ `leg_fk` 与 `nominal` 对夹紧率没有区别** —— 高程扫描源不是当前瓶颈。

### 11.3 决定性实验：仿真里 `cmd = 0` 也照样往前走

~~~
仿真 · cmd = (0,0,0) · 完美平地 · --scan nominal · 18 s
  -> x = 3.39 m（约 0.19 m/s），upright = True（站着走的，没摔）
  -> level_hist = {L0: 9000}（全程健康）
  -> base_lin_vel 约 [0.02, -0.01, -0.03]（速度估计干净，没有假速度）
~~~

**结论**：仿真里速度估计准、指令为 0，它仍然向前走 0.19 m/s => 这是**策略本身的行为**，
不是 sim2real 失配，也不是估计器偏差。

**推断的原因**：训练命令分布很可能不含零速度（`obs.py` 注释写"训练固定 [0.5,0,0]"，
`policy_profiles.go2.command` 也是 `[0.5,0,0]`，v33d 说明是"70/30 混合命令"）。
`cmd=0` 对策略是分布外输入，它退化到主行为——前进。（训练仓不在本机，此条为推断）

**两个现象要拆开**：

| 现象 | 真机 | 仿真 cmd=0 | 归属 |
|---|---|---|---|
| 向前走约 0.19 m/s | 有 | **有** | 策略/训练 |
| 向左偏 | 有 | 没有（走直线） | 真机速度估计 y 向零偏 = 真正的 sim2real |

### 11.4 摔倒事件复盘（两次，同一天下午）

| | 第一次（14:31 `policy_wide`） | 第二次（15:13 `policy_remote`） |
|---|---|---|
| 遥控器帧数 `n_sets` | 168 | **0（遥控器全程没发数据）** |
| 时长 | 60 s（被 Ctrl-C） | 8.5 s |
| `level_hist` | `{L0:30000}` | `{L0:4237}` |
| `ground_clearance` | — | **0.0127 m（1.3 cm）** |
| `height_scan_mean` | -0.077 | **-0.84** |
| `linvel_source` | estimator/0.307 | **zero/0.0（估计器降级）** |
| 动作 max | 2.34 | 2.66 |

**摔倒链条**：`cmd=0` → 策略自行前进（§11.3）→ 姿态下沉，`ground_clearance` 掉到 1.3 cm
→ **`leg_fk` 高程扫描同时退化（mean -0.84）**，等于告诉策略"前方有 84 cm 的坑" →
速度估计降级成 zero → 观测彻底失真 → 动作加大 → 塌下来。

**教训**：`leg_fk` 在低姿态下会失效，而失效会**放大**摔倒。平地应一律用 `--scan nominal`。

### 11.5 实时性：一次 12 秒冻结（必须知道的坑）

| 指标 | 冻结那次 | 机器重启后 |
|---|---|---|
| `lag_p95` | **12301 ms** | **1.6 ms** |
| `rtf` | 0.69 | 1.00 |
| 实际频率 | 342 Hz | 500.0 Hz |
| `overruns` | 86.7% | 17.7~25.3% |

`lag_p95` 到十几秒 = 进程被冻结（休眠/极端负载）。**这种数据不能用来评价策略。**
现在 `run_controller` 结束前会自检并在 `overruns>10%` 或 `rtf<0.98` 时大声告警。
注意：即便正常状态，这台 i5 笔记本也有约 20% overruns，属于这台机器的上限；跑之前要
关掉浏览器/桌面特效。

### 11.6 遥控器链路的真相（`/wirelesscontroller`）

* `/wirelesscontroller` **只在摇杆偏转/按键时才发帧**（10~20 Hz），回中后静默 →
  运行期间 `n_sets=0` 可能只是"没碰摇杆"，看起来却像代码没接上。
* 轴标定（实测）：左摇杆上下 = `ly`（上 +1.00 / 下 -0.92），左右 = `lx`（左 -1.00 / 右 +0.99），
  右摇杆左右 = `rx`（左 -0.82 / 右 +0.93）。体坐标 `+y=左、+wz=左转`，所以 `vy/wz` 取负。
* 按键位图（实测）：`L2 = bit5 (0x0020)`、`R2 = bit4 (0x0010)`。
* **`L2+B`（软急停/阻尼）是 `mcf` 的功能**：`--release-mode` 之后它失效 → 我们自己实现了
  `KeyGateController`（`--remote-estop-keys L2+B` → 按住输出 `kp=0/kd=8` 纯阻尼，不依赖 mcf）。

### 11.7 这一轮改的代码

| 改动 | 作用 |
|---|---|
| `Ctrl-C` 也保存 summary | 以前中断只写 `{"interrupted": true}`，摔倒数据全丢；现在完整落盘 |
| 遥控器首帧打印 + 进度行 `rmt=` + 3 s 静默告警 | 一眼区分"遥控器没发数据"和"代码没接上" |
| `n_sets` 改为统计收到的帧数 | 按键急停期间也能看出有没有数据 |
| 实时性自检告警 | `overruns>10%` / `rtf<0.98` 时明确提示"数据不可用于评价策略" |
| `scripts/show_summary.py` | summary 人话化；`--all` 做多次运行对照 |
| `--cmd-limits` | 速度限幅可配（默认仍=训练分布，超范围会告警） |
| `go2_docker.sh` 容器内误用提示 | 容器里没有 docker 且有 `docker/` 目录，原来报 `Is a directory` |

### 11.8 待办（按优先级）

1. **训练侧：命令分布必须覆盖零速度**（或单独训 stand 策略 + 部署时按阈值切换）——
   否则"站住"这件事永远做不到
2. **速度估计 y 向零偏补偿**（真机走左前、仿真走直线的差异来源）
3. **`leg_fk` 退化检测 + 平地回退 `nominal`**（`ground_clearance` 异常时自动回退）
4. **`scripts/record_telemetry.py`**：1 kHz 原始时序记录（现在只有 summary，做不了系统辨识）
5. 把 `config/domain_gap.json` 里 `real_mild` 的猜测参数换成实测值（阶跃/扫频/IMU 静止/足端力标定）

### 11.9 现场运维提示

* 电池：31.8 V（满）→ 29.3 V（一下午），26 V 以下不要再跑（**本项目没有低压保护**）。
* 摔倒后髋关节温度会到 **58~62 摄氏度**（阈值 70）——断电凉 5~10 分钟再上电。
* 摔倒后先 `--select-mode mcf` 交回官方运控，再用遥控器 `L2+A` 站起；不行就断电重启。
