# Go2 上机前安全审计（T2）

> **结论先说**：这套代码在**吊装 + 遥控器 debug mode（L2+R2）+ --controller zero/stand**
> 的前提下可以上机首测；**直接落地跑 policy 不行**，见 §6 的 5 条前置动作。
>
> **状态**：本报告基于代码审计 + 仿真/单测验证，**没有在真机上验证过任何一条**
> （本机没装 ROS 2，rclpy 代码只能写不能跑）。凡是"应该在真机成立"的地方都标了
> 「待真机确认」。
>
> **行号说明**：对应 2026-11-04 工作区快照（main@60cc4c1 + T2 改动 + Lead 并行改的
> `signal_io/node.py` / `ros2_controller/controller.py`）。这三份文件仍在改动，
> 以最终 diff 为准；引用时优先看**函数名**再看行号。

---

## 0. 一页结论：最危险的三件事

| # | 事实 | 后果 | 今天的应对 |
|---|---|---|---|
| 1 | 真机是 `<position kp=20 kd=1>`，力矩由**固件**按 `kp*(q_des-q)+kd*(0-dq)` 算；我们夹的 `motor_cmd.tau` 在真机路径上**恒 0** | `safety.py` 的 23.7/23.7/45.43 N·m 硬限幅、200 N·m/s 斜率限幅**只在仿真里有效**；真机上 q_des 一跳，力矩立刻跟着跳（跳 1 rad = 20 N·m） | 新增 `SafetySupervisor.q_slew_rate`，**real_* 默认 20 rad/s**（单拍 ≤ 0.04 rad → 首拍 ≤ 0.8 N·m）；仿真默认关闭 |
| 2 | 控制器进程被 `kill -9` / 段错误 / OOM / DDS 卡死 / PC 休眠后，**没有任何东西替它发阻尼**；`run_controller` 的 finally 只在 Python 正常栈退出时执行 | 机器人可能"保持最后一条指令"（固件行为未知），一直走 | 新增 `scripts/failsafe_guard.py`（独立进程，`/lowcmd` 断了 250 ms 就发 `kp=0/kd=8` 阻尼）；**未验证，默认不真发**，先 sim_ros |
| 3 | **电池/电压完全没有检查**：`unitree_go/LowState` 有 `power_v`，但 `signal_io/transport/ros2.py:250` 明确没搬，`signal_io/msg.py:231` 的 `LowState` 里也没有这个字段 | 低电压时机器人会突然掉电/失能，我们完全看不见 | 只加了"可选检查"（`health.check_battery`，默认关，恒 SKIP）；**缺口未修**，改法见 §5.1 |

另外两件今天已经修掉的真机专属问题（Lead 并行改的，结论照抄进本报告）：

* **时基 bug**：`signal_io/node.py:465` 的 `clock` 取 `st.sim_time`，而真机
  `sim_time = tick*0.001` ≈ **固件开机秒数（几百秒）**。于是传给 `ctrl.update` /
  `supervisor.process` 的 `t` 从几百秒开始：policy 的 `prepose_s=1.5` 窗口
  （`controller.py:377`）和 stand 的 `start_delay 0.5 + ramp 1.0 s`
  （`controller.py:67`）**全部失效**，首拍就是满增益/满策略输出。
  已修：`node.py:480-482` 引入本地时基 `t_ctrl = now - t_ctrl_origin`，health 仍用绝对
  `now` 算 age。仿真 `sim_time` 从 0 起，行为不变。
* **publish 异常会把循环静默带走**：`node.py:508-516` 现在捕获 `publish_cmd` 异常、
  计数（`n_publish_err`）并限频打印；`summary` 里同时给出 `max_loop_gap_ms`
  （`node.py:522-523`）。真机上这两个数非 0 就是"进程活着、指令没发出去"。

---

## 1. 威胁模型与系统边界

    机器人固件 ──rt/lowstate──▶ /lowstate ──▶ HealthMonitor ──▶ Level L0..L3
                                                    │
    策略(50 Hz) ──▶ SafetySupervisor(track/crouch/damping/estop) ──▶ /lowcmd ──▶ 固件
                                                    ▲
    独立进程 failsafe_guard ──(只在 /lowcmd 断流时)──┘

威胁模型（按"会造成什么后果"排）：

| ID | 失效 | 后果等级 | 是否本仓能防 |
|---|---|---|---|
| T1 | 控制器进程死/卡死 | 机器人按最后指令继续动作 | 只有外部进程能兜（今日新增 guard） |
| T2 | /lowstate 断流或不可信（NaN） | 用错误观测算指令 | 能（health） |
| T3 | /lowcmd 链路错（CRC/丢包/双发布者/机器人报指令超时） | 指令不生效或交错 | 部分（CRC 已实现；双发布者只能人工查） |
| T4 | 控制输出异常（q_des 跳变、动作 NaN） | 力矩冲击 / 关节猛打 | 部分（q_des 限速今天开了；NaN 只查了 q） |
| T5 | 姿态异常（摔倒、被抬起、坡上翻） | 继续输出 | 能（tilt>60° → L3；real_* 默认发阻尼而非卸力，见 §3.5） |
| T6 | 电机/电源异常（丢通信、过温、低电压） | 突然失能 | 部分（lost/温度/bit_flag 已判；**电压没判**） |
| T7 | 人为误操作（拆安全层、双发布者、错网卡） | 极高 | 部分（今天加了危险开关闸门） |
| T8 | PC 侧环境（非 RT 内核、休眠、断电、Wi-Fi 抢流量） | 节拍抖动/断流 | 部分（pacer/max_loop_gap 可观测；guard 兜断流） |

**不在威胁模型里**（本仓无法防，上机时用物理手段兜）：机械/齿轮损坏、电池鼓包、
遥控器误触、固件自身 bug、机器人被外力推倒后的连锁动作。

---

## 2. 失效 → 现有保护 → 缺口 → 今天怎么兜 → 残留风险

| # | 失效 | 现有保护（file:line） | 缺口 | 今天怎么兜 | 残留风险 |
|---|---|---|---|---|---|
| 1 | 控制器崩溃 / SIGKILL / 断电 / PC 休眠 | `node.py:399` `emergency_stop()` 只在 run_controller 的 finally 里（`run_controller.py:644-656`），Python 正常退栈才执行 | `kill -9`/段错误/休眠**不会**走 finally；固件对 lowcmd 断流的处理未知 | `scripts/failsafe_guard.py`（新）：独立进程，250 ms 收不到 `/lowcmd` → 500 Hz 发 `kp=0/kd=8` | guard 本身也可能死；PC 断电/网线全断时它也没用。**未验证** |
| 2 | /lowstate 断流 | `health.py:363-369`：40 ms→L1 / 80 ms→L2 / 250 ms→L3（real_ros 档，`config/go2_sim.json:1064-1074`，由 `config.py:336-337` 合并）；`node.py:487` 断流时给 health 传 None | L3 动作：real_* 默认 **damp**（kp=0/kd=8 阻尼，软倒），`zero` 档才是卸力/瘫软；两档都**站不住**，且 L3 锁存、运行期没有 reset → 必须重启进程 | 见 §3.2 的下发内容表；操作员手放遥控器 | 40 ms 抖动就会蹲下（L1），恢复要 1 s 迟滞；PC 调度抖动可能反复触发 |
| 3 | 指令链路（CRC/丢包/bit_flag/双发布者） | 真机 CRC：`go2_crc.py:150` + `transport/ros2.py:380`；仿真侧校验 `transport/ros2.py:322-327`；bit0/bit7 → L2（`health.py:419-421`） | 丢单帧不可见（没有 tick 连续性检查）；**双发布者无软件互斥** | `ros2 topic info /lowcmd` 人工确认 Publisher count（§3.3） | 两个 500 Hz 发布者交错 → 行为不可预测 |
| 4 | 力矩/位置通路 | `controller.py:404-411` 只写 q/kp/kd、tau=0；`safety.py:64-77` 夹 tau（真机恒 0 → 无效）；仿真 `robot.py:191-205` 自己算 PD 并夹 | 真机上**没有任何软件力矩限制** | `SafetySupervisor(q_slew_rate=20)`（real_* 默认开，`run_controller.py:135-153,604-611`）；`prepose` 1.5 s、action clip（`controller.py:402`） | 固件电机极限是最后一道，具体数值未知；kp=20 下 q_des 2.3 rad 就能顶满 calf 45.43 N·m |
| 5 | 姿态 / IMU / 关节 | tilt>60°→L3（`health.py:388`）、imu NaN/维度→L2（`health.py:378-385`）、joint NaN→L3（`health.py:377`） | yaw NaN 未判；dq NaN 只有开了 `max_joint_vel` 才判 | L3 分档：real_* 默认 `damp`（kp=0/kd=8 阻尼，§3.5），`zero` 仍可 config 切回 | L3 两档都不让机器人重新站住；固件 kd 上限可能=5（弱化，待真机确认） |
| 6 | 电机健康 / 电源 | lost→L2、温度 70→L1/85→L2、bit_flag bit0/bit7→L2（`health.py:400-421`） | **电压完全没判**（LowState 里没搬 `power_v`）；`tau_est` 未利用 | `health.check_battery`（默认关，恒 SKIP）；`max_joint_vel`（默认关） | 低电压突然失能；温度只降级不降额 |
| 7 | 危险开关误用 | 今日新增闸门：real_* 上拒绝 `--no-health/--no-supervisor/--no-damping`（`run_controller.py:506-520`） | 没有"运行中禁止"的机制；`--force-unsafe` 一加就回到旧行为 | 必须显式 `--force-unsafe` 并打印理由 | 有人图省事加 `--force-unsafe` |
| 8 | 退出路径 | duration 到点/Ctrl-C/异常都走 finally（`run_controller.py:644-656`）；没收到过 state 时**不发**（`node.py:406` 返回 False） | 只发 100 ms 阻尼，之后固件行为未知 | 结合 guard | 100 ms 之后无人兜底 |
| 9 | 指令掉线自动卸力 | `watchdog_ms=60` **只作用于仿真 SimNode**（`config.py:54-57`、`node.py:199/248`） | 真机方向没有软件看门狗 | guard 的 gap 判定就是这个看门狗 | 见 #1 |

---

## 3. 逐项细节

### 3.1 控制器进程崩溃 / SIGKILL / 断电 / PC 休眠

* 正常退出（duration 到点、Ctrl-C、循环里抛异常）都会走
  `run_controller.py:644-656` 的 finally：real_* 且没 `--no-damping` 时调
  `ControllerNode.emergency_stop()`（`node.py:399`），发 **50 拍 = 100 ms**
  `kp=0/kd=8/q=0` 的阻尼指令，然后 `tr.close()`，之后**不再有 lowcmd**。
* `SIGKILL` / 段错误 / OOM / PC 休眠**不会**执行 finally。
* 固件对 lowcmd 断流后的行为官方没有公开。**因此"最后 100 ms 阻尼"之后的一切都是未知的。**
* 今天能做的兜底只有一个：**另一个进程**（`scripts/failsafe_guard.py`）。它自己也有
  单点故障（它死了就没人兜），所以它是"提高概率"而不是"保证"。

### 3.2 /lowstate 断流：40/80/250 ms 后到底下发什么

阈值来自 `config/go2_sim.json:1064-1074`（real_ros/real_sdk 档）：
`stale_ms=40`、`protect_ms=80`、`estop_ms=250`；由 `config.py:336-337` 合并进
`cfg["health"]`，`run_controller.py` 用它建 `HealthMonitor`。

| 断流时长（real_ros） | 等级 | supervisor 模式 | 实际下发（12 个关节槽 + 后 8 槽） |
|---|---|---|---|
| < 40 ms | L0 | track | `q`=策略输出（经 20 rad/s 限速）、`kp=20 kd=1 tau=0`；后 8 槽全 0（`mode=1`） |
| > 40 ms | L1 | crouch | `q` 在 `crouch_s=2 s` 内 smoothstep 走到 crouch_pose（膝盖各 -0.35 rad），`kp=20 kd=1 tau=0` |
| > 80 ms | L2 | crouch_then_damp | 先按上面蹲 2 s，然后 damping：`q`=当前 q、`kp=0`、`kd=damp_kd`（默认 1，建议 8） |
| > 250 ms | L3 | estop | **real_\* 默认 damp**：20 槽 `mode=0x01 / kp=0 / kd=8 / q=dq=tau=0`（纯关节阻尼）；`estop_policy=zero` 时回到旧的 **20 槽全 0**（含 `mode=0`）= 卸力。head/level_flag 仍是 `FE EF / FF`（合法报文） |

实现位置：`safety.py` 的 `process`/`_select_mode`（模式选择）、`_crouch_cmd`、`_damping_cmd`、
`_estop_cmd`（zero/damp 两档）与 `_zero_cmd`；阈值判定 `health.py:363-369`。

要点（都是坑）：

1. **L3 不是"刹住"**，而是"要么阻尼、要么卸力"，取决于 `safety.estop_policy`：
   * `damp`（**real_* 默认**，`run_controller.py` 的 `resolve_estop_policy`）：
     `mode=1/kp=0/kd=8` → 固件算 `tau=-kd*dq`，只泄能 —— 走路中失联会"阻尼着倒下"。
     注意 Go2 固件的 `kd` 上限**可能只有 5**（kd=8 会被夹到 5），方向不变、只是弱一点
     —— **待真机确认**。
   * `zero`（仿真默认 / config 显式切回）：20 槽全 0，机器人直接瘫软。
   两种都不"站住"：**看到 L3 手放遥控器，别指望它自己恢复**。
2. **L3 锁存且运行期没有 reset**（`HealthMonitor.reset` 只在测试里调）。一次 250 ms
   抖动 = 这次运行报废，只能重启进程（重启又会先发 100 ms 阻尼）。
3. **L1 也会蹲**：40 ms 的空洞就进入 crouch（2 s 插值），恢复需要连续 1 s 满足低一级
   （`recover_s=1.0`）才逐级回落。真机上如果看到"走路时突然下蹲又起来"，先看
   `health.level_hist` 和 `pacer.overruns`，那不是策略问题。
4. L2 的 damping `kd` 默认是 `mode_kd=1`（很软，dq=5 rad/s 只有 5 N·m）。今天加了
   `safety.damp_kd` 旋钮（默认仍 = mode_kd，行为不变），**建议真机配置成 8**。

### 3.3 指令链路：DDS 丢包 / CRC / bit_flag bit0 / 双发布者

* **真机 CRC**：`signal_io/go2_crc.py:150` `lowcmd_crc()` 按 812 字节 unitree_go 布局算
  CRC32（poly 0x04C11DB7、init 0xFFFFFFFF、MSB-first、无 final xor）；
  `transport/ros2.py:380` 每次下发前写入，且有 500 Hz 缓存（`:331-345`）。
  仿真侧 `transport/ros2.py:322-327` 会**真的校验**，所以 sim_ros 闭环能提前暴露
  布局/算法错误 —— 但**这是我们自己校验自己**，不能证明固件认。
  → 待真机确认：机器人收到合法 CRC 后 `motor_state.mode` 是否变 0x01
  （`scripts/go2_real_check.py` 会打印这个）。
* **DDS 丢包**：订阅端用 `_spin_to_latest` 抽干队列（`transport/ros2.py:425-451`），
  所以"延迟越积越大"被解决了，但**丢帧本身不可见**：health 只看"多久没收到整帧"，
  不看 `tick` 是否连续。也就是说 500 Hz 里丢 5 帧（10 ms）→ 完全无感；
  丢 21 帧（42 ms）→ L1 下蹲。**改进方向**：用 `tick` 差算丢包率（§5.6）。
* **bit_flag bit0**（机器人自己报"操作指令超时"，`health.py:417-419`）：这是**最硬的证据**
  —— 说明我们的 lowcmd 没按预期到固件。触发后 L2 → 蹲下 → 2 s 后纯阻尼。
  真机上如果 bit0 一直亮，别怀疑策略，先查 CRC / 网卡 / 双发布者 / DDS domain。
* **双发布者抢 /lowcmd**：`ros2_ws/launch/real.launch.py:122-131` 起 controller 的同时，
  如果又手动 `run_controller.py --target real_ros`，就是**两个 500 Hz 发布者**交错写同一条
  lowcmd，机器人看到的是两条指令的混合，行为不可预测（可能是"两条指令轮流生效"的抖动）。
  本项目 `failsafe_guard.py` 也会往 `/lowcmd` 发（但只在断流时）。
  一键判断（上机现场就该跑）：

      ros2 topic info /lowcmd                     # 看 Publisher count
      ros2 topic info -v /lowcmd | grep -A3 "Publisher count"
      ros2 node list | grep -iE "go2|unitree"     # 谁在发
      ros2 topic hz /lowcmd                       # 频率应该是 ~500 Hz，不是 1000 Hz
      scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

  `go2_real_check.py` 只订阅不发布，它数到的 `/lowcmd 条数 > 0` 说明"有别人在发"。
  **Publisher count ≥ 2 且你在做控制实验 → 先停掉一个再说。**

### 3.4 力矩通路（最重要的一条）

真机 lowcmd 是 **position 模式**：我们只发 `q/kp/kd`（`controller.py:404-411`，
`tau=0`），固件自己算

    tau_total = kp*(q_des - q) + kd*(0 - dq) + tau        # tau 恒 0

所以：

* `SafetyLimiter` 夹的是 `cmd.tau`（`safety.py:64-77`），真机路径上它**一直是 0**，
  夹了等于没夹：**23.7/23.7/45.43 N·m 硬限幅（`robot.py:28`）、200 N·m/s 斜率限幅
  （`config/go2_sim.json:53`）在真机上不生效**。
* 仿真里之所以有效，是因为 `SimNode` 把指令交给 `Go2Sim.write_cmd`
  （`sim_env/robot.py:191-205`），那里**自己**按 PD 算力矩并在 `:199-200` 夹限幅。
  换句话说：这两个保护是"仿真专属"，别把仿真的力矩曲线当成真机证据。
* **真机上唯一有效的软件力矩限制就是 q_des 不许跳**。

首拍力矩表（kp=20，kd 项在 dq=0 时为 0）：

| q_des 单拍跳变 | 首拍 tau = kp·Δq | 说明 |
|---|---|---|
| 0.01 rad | 0.2 N·m | 正常跟踪量级 |
| **0.04 rad** | **0.8 N·m** | **q_slew_rate=20 rad/s 在 2 ms 拍上的上限（今天 real_* 默认）** |
| 0.1 rad | 2.0 N·m | 尚可 |
| **1.0 rad** | **20 N·m** | 题目要算的点：**一个 1 rad 的跳变就是 20 N·m** |
| 1.2 rad | 24 N·m | hip/thigh 的 23.7 N·m 已经顶到（固件侧才夹） |
| **2.0 rad** | **40 N·m** | Lead 举的最坏例子：还没到 calf 的 45.43 N·m |
| 2.3 rad | 46 N·m | calf 也顶满，之后只能靠固件 |

再叠加 kd 项：落地瞬间关节 dq 可能到 5~10 rad/s，`kd=1` 就是 5~10 N·m 的**反向**
阻尼力矩（方向是"阻碍运动"，属于安全方向，但也意味着落地首拍总力矩比表里更大）。

**prepose 阶段**（`controller.py:377-390`，`prepose_s=1.5`、`prepose_kp=20`）：
它把 q 直接写成训练 home（`config` 的 `robots.go2.home_q` = `[-0.1,0.8,-1.5, ...]`），
而机器人实际姿态通常还是固件 home（`0, 0.9, -1.8`）→ calf 首拍 Δq≈0.3 rad = **6 N·m**。

**光有"限速"挡不住这个台阶**：限速只保证"相邻两拍的 q_des 不跳"，而第一条 q_des
相对**机器人真实姿态**的差是另一回事。所以 `SafetySupervisor` 在**每次进入 track
（含进程启动的第一拍）**会把限速历史对齐到**实测 q**（`safety.py` 的
`_seed_slew_from_state`，只在 `q_slew_rate` 打开时生效）：
第一条指令因此从"机器人现在在哪"出发，0.3 rad 按 20 rad/s 在 15 ms 内走完，
首拍力矩 **0.8 N·m** 而不是 6 N·m。仿真默认不开限速，所以这条对仿真没有影响。

结论一句话：**真机上我们只能通过"不让 q_des 跳"和"固件自己的电机极限"来限制力矩。**

### 3.5 姿态 / IMU / 关节：L3 = 阻尼（real_* 默认）或卸力（zero）

* 判定（`health.py:371-395`）：
  * `q12` NaN/长度不对 → **L3**（`:377`）
  * `rpy` 缺失/维度不足 → L2（`:378-381`）；`roll`/`pitch` NaN → L2（`:385`）
  * `|roll|` 或 `|pitch| > 60°` → **L3**（`:388-389`）
  * `yaw` 是 NaN **不判**（只检查了 `rpy[:2]`）—— 缺口很小，记在 §5.12
* **L3 下发**（`safety.py` 的 `_estop_cmd`，立即生效、不过力矩斜率）：
  * `estop_policy="damp"`（real_* 默认）：20 槽 `mode=0x01, kp=0, kd=estop_kd(=8)`，
    `tau=-kd*dq` 纯阻尼。**最适合"站得好好的、只是 LowState 断流/姿态超限"**：
    机器人会软下来而不是自由落体。
  * `estop_policy="zero"`（仿真默认）：20 槽全 0（含 mode=0）= 卸力。
    适合"已经要摔/被抬起乱蹬/关节角 NaN"这种继续出力只会更糟的场合。
  * 两档都**不会让机器人重新站住**，而且 **L3 锁存**（`health.py:353-366`），
    运行期不恢复 → 这次运行结束，重启进程。
  * 姿态 >60° 属于"已经越线"，此时 damp 也只是"软一点地倒"，别指望它救回来。
* `imu` 全 0（没有 IMU 数据但长度正确）**不会**被判错 —— 0 是合法的姿态。
  真机上如果 rpy 一直 0，要怀疑 IMU 没使能，而 health 不会告诉你（§5.13）。

### 3.6 电机健康 / 电源

| 信号 | 判定 | 位置 | 真机是否可用 |
|---|---|---|---|
| `motor_state[i].lost` | 任一 ≠0 → L2 | `health.py:400-405` | 可用（transport 搬了） |
| `motor_state[i].temperature` | ≥70 → L1、≥85 → L2 | `health.py:406-414` | 可用（搬了） |
| `LowState.bit_flag` bit0/bit7 | → L2 | `health.py:417-421` | 可用（搬了） |
| `LowState.power_v` | **完全没搬** | `transport/ros2.py:250`（"暂不搬"）；`msg.py:231-247`（`LowState` 无该字段） | **不可用** |
| `bms_state`（单芯电压/电流/温度） | 没搬 | 同上 | 不可用 |
| `motor_state[i].tau_est` | 没判 | 已搬但没人用 | 可用但未利用（§5.7） |

今天加的两个**可选**检查（默认关闭，保证现有行为逐位不变）：

* `cfg["health"]["check_battery"]`（默认 `False`）+ `v_warn`/`v_min`：
  `health.py:273-294`。用 `getattr(state, "power_v", None)`，**缺字段就 SKIP**，
  `stats()["battery"]` 里把 `n_checked` / `n_skipped` 分开计数 —— "没数据"永远不会
  被当成"电池正常"。
* `cfg["health"]["max_joint_vel"]`（默认 `0`=关，`health.py:295-315`）：
  `|dq|` 超限 → L2；`dq` 是 NaN → L2。Go2 关节额定约 30 rad/s，建议阈值 40+，
  不要用它抓正常高频抖动。

### 3.7 危险开关：`--no-health` / `--no-supervisor` / `--no-damping`

| 开关 | 关掉之后 | 仿真里 | 真机上 |
|---|---|---|---|
| `--no-health` | 不再判 L0..L3（断流/NaN/姿态/温度/bit_flag 全不判），指令照发 | 排障刚需 | **拒绝**（`run_controller.py:506-520`） |
| `--no-supervisor` | 没有 crouch/damping/estop，等于纯直通 | 排障刚需 | **拒绝** |
| `--no-damping` | 退出不发 100 ms 阻尼，机器人停最后一条指令 | 无所谓 | **拒绝** |

要越过闸门必须显式 `--force-unsafe`，并且会打印"你拆了哪张网"。今天之前这三个开关在
real_* 上是完全放开的，一次复制粘贴就可能把安全层关掉 —— 这是 T7 里最容易发生的一种。
（判定逻辑是纯函数 `unsafe_flag_refusals()`，`run_controller.py:156-171`，有单测。）

### 3.8 退出路径

| 路径 | 会发生什么 | 位置 |
|---|---|---|
| `--duration` 到点 | 主循环退出 → summary → finally：real_* 发 100 ms 阻尼 | `run_controller.py:644-656` |
| Ctrl-C | `KeyboardInterrupt` 被捕获 → summary={"interrupted": True} → 同样发阻尼 | 同上 |
| 循环里抛异常 | 异常穿过 try → **finally 仍然执行**（先阻尼），然后 traceback 退出码 1 | 同上 |
| `SIGKILL` / 段错误 / 断电 | **什么都不发**，固件行为未知 | — |
| 从未收到 `/lowstate` | `emergency_stop()` 返回 False → **不发任何指令**（不去动一台没接管过的机器人） | `node.py:406` |
| `--duration 0`（无限） | 只能 Ctrl-C 退出 | — |

注意：`duration` 默认 20 s（`run_controller.py:459`），launch 默认 30 s
（`real.launch.py:145-146`）。**真机首测不要用默认值一把梭**，先 `--duration 10`。

### 3.9 真机没有"指令掉线自动卸力"的软件看门狗

`config.py:54-57` 已经写明：`watchdog_ms=60` **只被 SimNode 的 SafetyLimiter 使用**
（`node.py:199,248`）。真机方向上，如果控制器进程还在但不发 lowcmd（例如 DDS publish
卡住、`publish_cmd` 抛异常），**没有任何东西会替它改发阻尼/卸力**。
今天能做的：

* `node.py:508-516` 捕获 publish 异常并计数（`n_publish_err`），但**它不会自动卸力**；
* `scripts/failsafe_guard.py` 从外部看 "`/lowcmd` 是不是还有新鲜的"，断流 250 ms 就自己发阻尼。
  这是目前唯一的"指令层看门狗"。

### 3.10 （已修）真机时基 bug

见 §0。补充：这个 bug 的隐蔽之处在于**仿真完全正常**（`sim_time` 从 0 起），所以之前
所有仿真验收都过；只有真机上 `tick` 是"固件开机毫秒"时才暴露。修完后
`tests/test_node_loop.py` 有回归；行为差异只出现在"t 很大"的真机场景。

### 3.11 （已修）publish 异常不再静默带走循环

之前 `self.tr.publish_cmd(cmd)` 抛异常会直接跳出 `while`，进程带着 traceback 死掉 ——
但那个 finally 是能跑到阻尼的，所以"死"本身不算最坏；**最坏的是它先跑了很久没发现**。
现在计数 + 限频打印 + 写进 summary（`n_publish_err`、`max_loop_gap_ms`）。
真机验收标准：**这两个数在整个运行期必须都是 0**（`max_loop_gap_ms` 允许偶发 < 20 ms）。

---

## 4. 今天已落地的改动（以及怎么验证的）

| 文件 | 改动 | 默认行为是否变化 | 验证 |
|---|---|---|---|
| `ros2_controller/safety.py` | `SafetySupervisor(q_slew_rate=..., damp_kd=..., estop_policy=...)`：① track 模式 q_des 限速（`_slew_q`）+ 每次进 track 把限速历史对齐实测 q（`_seed_slew_from_state`，首拍不相对真实姿态跳台阶）；② L2 damping 的 kd 可单独配（`damp_kd`）；③ L3 两档 `_estop_cmd`（`zero` 卸力 / `damp` 纯阻尼，默认 `zero`）；④ 模块 docstring 写清"真机 tau 限幅无效" | **不变**（三个新参数默认 `None`/`"zero"`；`zero` 档与旧版逐位相同，`SafetyLimiter` 源码块未动，`test_safety` 的 sha256 + 64 项回归仍过） | `tests/test_safety.py` 64 项全过；`tests/test_failsafe.py` 的 P1 段 |
| `ros2_controller/health.py` | 可选 `check_battery`（+ `v_warn/v_min`，缺字段 SKIP）、可选 `max_joint_vel`（`\|dq\|` 超限→L2、NaN→L2）；`stats()` 暴露 `battery`/`max_joint_vel` | **不变**（默认关闭；65 项阈值/迟滞/锁存测试全过） | `tests/test_health.py` 65 项全过 |
| `ros2_controller/run_controller.py` | ① real_* 上拒绝 `--no-health/--no-supervisor/--no-damping`（`:156-171,506-520`）+ `--force-unsafe`；② q_des 限速取值策略 `resolve_q_slew_rate`（`:135-153`）：real_* 默认 20 rad/s、仿真默认关、config 可显式覆盖；③ 运行期速度指令 `--cmd-topic/--no-cmd-topic/--cmd-timeout`（`:559-588`）+ 三轴限幅 `clip_cmd_vel`（`:121-132`）；④ `safety.damp_kd` 接线；⑤ L3 档位取值策略 `resolve_estop_policy`：real_* 默认 `damp`、仿真 `zero`、config 可覆盖，并打印来源 | 仿真的启动参数/时序**不变**；real_* 多了四个安全默认（拒绝危险开关、q_des 限速、L3 阻尼、cmd_vel 失联回零） | `--help` 有 4 个新参数；`tests/test_failsafe.py` P2 段（含**子进程**验证真机会退出码 2） |
| `signal_io/failsafe.py`（新） | 纯决策内核：WAITING/ARMED/FIRING/EXHAUSTED 四态、gap 判定、恢复即停、单次时长上限、计数、`make_damping_cmd` | 无（新文件，没有任何现有代码 import 它） | `tests/test_failsafe.py` P0 段（7 组用例） |
| `scripts/failsafe_guard.py`（新） | ROS 2 守护进程：订阅 `/lowcmd`，断流就发阻尼；`--enable-publish` **默认关**（只打印 WOULD PUBLISH）；`--selftest` 离线自检；缺 ROS 2 时打印原因并退出码 2 | 无 | `--selftest` 7 项全过；本机无 rclpy 时优雅退出（子进程测试） |
| `tests/test_failsafe.py`（新） | 103 项：内核状态机 / q_des 限速（含实测 q 对齐）/ estop_policy 两档 / damp_kd / 危险开关闸门 / cmd_vel 限幅 / CLI 行为 | — | `.venv/bin/python tests/run_tests.py failsafe` → 103/103 |
| `docs/safety-audit.md`（新） | 本文件 | — | — |
| `signal_io/node.py`（Lead） | 本地时基 `t_ctrl`（`:480-482`）+ publish 异常计数（`:508-516,522-523`） | 仿真不变（t_ctrl ≈ now） | Lead 的 `test_node_loop` 回归 |
| `ros2_controller/controller.py`（Lead） | `PolicyController.set_command`、`LiveCommandController`（cmd_vel 失联回零）；`obs.py` `WalkObsBuilder.set_command` | 不变 | `test_obs` / `test_policy_port` |

本机实测（2026-11-04）：

    .venv/bin/python tests/run_tests.py health safety node policy obs failsafe
    PASS test_failsafe / test_health / test_node_loop / test_obs / test_policy_port / test_safety  6/6
    .venv/bin/python scripts/failsafe_guard.py --selftest          # 7 项全过，退出码 0
    .venv/bin/python ros2_controller/run_controller.py --help      # 有 --cmd-topic/--no-cmd-topic/--cmd-timeout/--force-unsafe
    # 无 ROS 2 时的优雅失败（本机实测）：
    .venv/bin/python scripts/failsafe_guard.py --target real_ros   # 打印原因 + 退出码 2，无 traceback

全量（17 个测试文件）实测 **14/17**，3 个失败**全部是 viz 相关、全部因为可选依赖没装**
（`requirements.txt` 只要求 mujoco + numpy）：

    test_viz_panels / test_viz_scene : matplotlib 缺失（本机 find_spec('matplotlib')=False）
    test_viz_record                  : imageio / imageio_ffmpeg 缺失（视频 0 帧）
    test_viz_live                    : 因为没有 Qt 直接 SKIP（所以"已知的 perf 计时失败"没出现）

这三个测试文件与 `sim_env/viz/` **都不在本任务的 diff 里**（`git status` 也没显示
它们被这次改动碰过），与本任务的修改无关；除它们之外的 14 个测试文件全 PASS。

**没验证的部分（重要）**：`Ros2Transport` 的订阅/发布、CRC、双发布者判定、guard 的
`--enable-publish` 实际发帧、`cmd_vel` 回调在 rclpy 下的行为 —— 本机没有 rclpy，
这些**一行都没跑过**。guard 与 cmd_vel 都必须先在 `sim_ros`（domain 1）验证。

---

## 5. 未修缺口（诚实清单 + 后续改法）

### 5.1 电池/电压无保护（今天只加了"可选检查"，但**拿不到数据**）

* 现状：`transport/ros2.py:250` 没搬 `power_v`；`msg.py:231-247` 的 `LowState` 没有该字段。
  所以 `health.check_battery=True` 也只会一直 SKIP（`stats()["battery"]["n_skipped"]` 会涨）。
* 改法（等 Lead 排期，涉及别人的文件）：

      # signal_io/msg.py：在 LowState 末尾加字段（**不要动 pack/unpack 的 UDP 布局**）
      power_v: float = 0.0        # 只给真机用；本地 UDP 报文里没有它

      # signal_io/transport/ros2.py::_on_state，搬完 bit_flag 之后：
      st.power_v = float(getattr(m, "power_v", 0.0))

  然后 `config` 里 `health.check_battery=true`、`v_warn/v_min` 按明天实测电压曲线标定。
  Go2 是 8S 锂电（满电约 33.6 V，标称 ~29.6 V），但 **26/24 V 这两个值我没有实测依据**，
  只是保守初值；明天请用 go2_real_check 打印的 power_v 与遥控器/App 显示的电量对一下再定。

### 5.2 真机力矩限制仍然只有 q_des 限速 + 固件

q_slew_rate 只能保证"q_des 不跳"，它管不了：

* `kd` 项的冲击（落地瞬间 dq 很大 → kd*dq）；
* 固件实际的电流环/力矩环行为；
* 长时间堵转导致的过热（只有温度 ≥70 °C 才 L1）。

长期方案：真机走"力矩前馈 + 低 PD"（`tau = m.tau` 真的非 0），或在 C++ 层做。
短期：q_slew 20 rad/s + 温度监控 + 别让机器人在坡上长时间顶。

### 5.3 进程级故障没有"保证"兜底

guard 是概率性的：guard 自己死/PC 断电/网线断时无效。**物理兜底**：吊装绳、
场地净空、手放遥控器、电源开关可达。

### 5.4 L3 与站立姿态的语义冲突 —— **已定：real_\* 默认 damp，estop_ms 保持 250**

250 ms 的 DDS 抖动 = 一次 L3。旧行为是发 mode=0 全零（卸力）→ 站着的机器人直接瘫倒。
**决定（Lead 定稿，已实现）**：

* `safety.estop_policy` 默认 `"zero"`（= 旧行为，仿真逐位不变）；
* `run_controller` 在 **real_\* 上默认 `"damp"`**：L3 发 `mode=1/kp=0/kd=8` 的纯关节
  阻尼（与官方 `create_damping_cmd` 同义），走路中失联从"直接瘫倒"变成"阻尼着倒下"；
* `estop_ms` **保持 250**（不再往上调），理由：状态断流本身仍是必须立刻反应的故障，
  调大只是把"失明时间"延长；真正的解是让 L3 的动作更温和 + 外部 guard 兜底；
* config 里写 `"safety": {"estop_policy": "zero"}` 可以显式切回旧行为（例如某些
  场合确实希望立刻卸力）。

**残留**：guard 仍建议开（`--enable-publish` 默认关，先 sim_ros 验证）；固件 kd 上限
可能是 5 → 阻尼会被削弱（**待真机确认**）；L3 锁存仍然意味着"这次运行结束"。

### 5.5 health 的附加通道（cmd_vel / height_scan / linvel_conf）在生产路径没接

`node.py:487` 调 `health.check(...)` 时**没有传 `extra=`**，所以：

* 感知缺失（height_scan 没数据、leg odom 置信度低）**不会**降级到 L1；
* 这些只影响 obs 质量，靠策略自己扛。

改法：在 `ControllerNode.run` 里把 `on_control` 里已经有的 obs meta（`linvel_confidence`、
`height_scan_source`）包成 `Channel` 传给 `health.check(extra=...)`。属于 T2 之外。

### 5.6 丢包率没有监控

建议在 `_on_state` 里用 `tick` 差累计 `n_missed_ticks`（真机 tick 1 ms 递增，
500 Hz 下每帧 +2），写进 `transport.stats()`。丢包率 >1% 就先别落地。

### 5.7 `tau_est` / `bms_state` 没利用

`tau_est` 能反映实际负载（堵转/卡住），现在只搬不用；`bms_state` 有单芯电压与电流，
比 `power_v` 更适合判低电压（能看出某芯异常）。

### 5.8 双发布者没有软件互斥

可以做：guard 与 controller 都检查 `node.count_publishers("/lowcmd")`，>1 就拒绝启动
（`rclpy` 的 `Node.count_publishers` 可用）。今天只在文档里给了人工检查命令。

### 5.9 没有软件急停入口

现在"急停"只能是遥控器（L2+R2 或电源）或 Ctrl-C（会发 100 ms 阻尼）。
可以加一个 `/go2/estop`（`std_msgs/Bool`）订阅，收到就直接切 estop 模式。
注意：**这条本身也有风险**（话题被人误发），需要 Lead 定。

### 5.10 温度只降级不降额

70 °C 只是 L1 下蹲，不会降低输出力矩/KP。真机长时间行走（尤其坡道）应关注
`motor_state.temperature` 的绝对值，别等 alarm。

### 5.11 QoS / 时间戳没有校验

订阅用默认可靠 QoS depth=10（`transport/ros2.py:223`）。消息没有 header 时间戳，
所以"这帧是 5 ms 前还是 100 ms 前"无法判断，只能用收到时刻（`_on_state` 里没有 stamp）。
health 的 age 就是靠 ControllerNode 的墙钟逻辑时钟近似出来的（`node.py:465-467`）。

### 5.12 `yaw` NaN 不判

`health.py:385` 只检查 `rpy[:2]`。改法：`_finite(rpy_arr[:3])`，但会改变现有
`imu_nan` 测试语义（现在只测 roll/pitch），需要同步改 `test_health`。

### 5.13 IMU 全 0 不判

rpy 恒 0 是合法值，health 不会报。真机上如果 `imu_state.rpy` 一直 0，是 IMU 没使能，
只能靠 `go2_real_check` 打印值人工看。

---

## 6. 明天上机：必须做的 5 条

> 前提：机器人**吊起来**、遥控器在手、场地净空 3 m、脚不要站机器人后面。
>
> 现场操作路径以队友交付的 `scripts/real_run.sh`（preflight -> zero -> stand -> policy，
> 每阶段要人输入 yes 确认，policy 阶段有 `--max-duration` 硬上限）和
> `docs/real-bringup.md`（runbook）为主；本节只补它们没覆盖的安全要点。
> **注意：`real_run.sh` 目前不会自动拉起 `failsafe_guard.py`** —— 第 4 条要另开终端。

1. **只读自检 + 确认没有第二个发布者**（不发任何指令）：
       scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
       ros2 topic info /lowcmd          # Publisher count 必须是 0（此时你还没起控制器）
   看三件事：`/lowstate` 频率 ≈500 Hz、`motor_state[0:12].mode` 在进入 debug mode 后
   变 0x01、`tick` 增量 ≈1 ms。**任何一条不对就别往下走。**
2. **只用 zero / stand 起步**，并且 `--duration` 用小值：
       scripts/run_with_ros2.sh ros2_controller/run_controller.py --target real_ros \
           --controller zero --duration 10
       scripts/run_with_ros2.sh ros2_controller/run_controller.py --target real_ros \
           --controller stand --duration 10 --cmd 0 0 0
   看 `summary` 里的 `n_state/n_cmd/pacer.overruns/max_loop_gap_ms/n_publish_err`
   和 `health.level_hist`（应该几乎全是 L0）。**stand 阶段就应该看到关节平滑走到站姿
   （ramp 1 s + start_delay 0.5 s，时基修好了才会生效）**。
3. **policy 只在落地 + 空旷 + `--cmd 0 0 0` 站住之后**，再用 `/go2/cmd_vel` 提速：
       # 控制进程（--target real_ros --controller policy --scan leg_fk --duration 60 --cmd 0 0 0）
       scripts/run_with_ros2.sh scripts/cmd_vel.py 0.2 0 0 --duration 20   # 队友提供的发布器
       # 或者手敲：ros2 topic pub -r 10 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
   三轴被夹在 vx[-0.2,0.8] / vy[-0.3,0.3] / wz[-0.8,0.8]；**3 s 不发消息自动回零**。
   提速顺序：0 → 0.2 → 0.3，每次停 10 s 观察；转向只给 wz ≤ 0.3。
4. **另开一个终端挂着 failsafe_guard（第一次不要 `--enable-publish`）**：
       scripts/run_with_ros2.sh scripts/failsafe_guard.py --target real_ros --duration 60
   观察 30 s：**一次都不要触发**（`n_trips=0`）。如果它以 250 ms 的灵敏度都触发了，
   说明你的指令流本来就不连续 —— 那时候更不能落地。
   验证过 sim_ros 之后，再决定下一次实验是否加 `--enable-publish`。
5. **全程盯这 6 个数**（每轮结束都看 summary）：
   `n_publish_err=0`、`max_loop_gap_ms < 20`、`pacer.overruns` 占比 < 1%、
   `health.level_hist` 里 L1/L2/L3 合计 < 1%、`transport.bad_crc=0`、
   `safety_supervisor.mode_hist` 里 crouch/damping/estop 为 0（除了收腿时刻）。
   出现任何一条：**先遥控器接管，再分析**。

## 明确不要在明天做的事

1. **不要**加 `--force-unsafe` 去关 `--no-health/--no-supervisor/--no-damping`。
2. **不要**同时开 `real.launch.py` 和手动 `run_controller.py`（双 500 Hz 发布者）。
   起之前用 `ros2 topic info /lowcmd` 数 Publisher。
3. **不要**第一次就 `--controller policy` 落地，更不要上台阶/草地（`--scan topic` 未标定）。
4. **不要**用 `--linvel oracle`（独立进程里会被强制退回 estimator，行为与训练不同）。
5. **不要**用 `--no-realtime`（真机必须钉 500 Hz）。
6. **不要**在控制器正常跑的时候给 guard 开 `--enable-publish` 做实验（两个发布者）。
7. **不要**把 L3 当"软停"：real_* 现在默认发**阻尼**（`damp`），它只会让机器人
   "软下来"、不会重新站住，而且锁存。看到 L3 → 先遥控器接管。
8. **不要**指望低电压保护 —— 它还不存在（§5.1）。电池充满、遥控器电量足、带充电器。
9. **不要**用默认 `--duration 20/30` 一把梭：先 10 s，确认没问题再往上加。
10. **不要**在 `/lowstate` 频率不稳（比如 300 Hz 或忽快忽慢）时继续 ——
    先解决网卡/CPU 抢占（`GO2_IFACE` 指对网卡、关掉 Wi-Fi、`taskset` 绑核）。

---

## 7. 附：一分钟自查命令

    # 网卡（config 里写死 eth0，现场用环境变量覆盖，别改文件）
    ip link show enp0s31f6            # 本机有线网卡当前是 DOWN：上机前先 up + 配 192.168.123.x/24
    GO2_IFACE=enp0s31f6 scripts/run_with_ros2.sh ros2_controller/run_controller.py \
        --target real_ros --controller zero --duration 5
    # 谁在发 /lowcmd
    ros2 topic info -v /lowcmd | grep -A3 "Publisher count"
    # 控制器是否"活着但发不出去"（--json 落盘的 summary）
    .venv/bin/python -c "import json,sys;d=json.load(open(sys.argv[1]));print({k:d.get(k) for k in ('n_state','n_cmd','n_publish_err','max_loop_gap_ms')})" logs/run.json
    # guard 能不能自己站住（不需要 ROS 2）
    .venv/bin/python scripts/failsafe_guard.py --selftest
