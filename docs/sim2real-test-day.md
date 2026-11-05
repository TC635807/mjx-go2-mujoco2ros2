# 真机测试日 runbook（Go2 sim2real）

> 面向"明天就要在真机上跑"的场景：按顺序照抄命令即可，每一步都有明确的通过标准。
> 环境前提：控制机 = `wyx-ThinkPad-T14-Gen-1`（Ubuntu 25.10 + Docker + 镜像 `unitree-go2-ros2:humble`），
> 官方栈跑在容器里（Ubuntu 22.04 + ROS 2 Humble + CycloneDDS 0.10.5）。
> 离线验证结论见 `docker/README.md` §6（PASS=14/0）。

---

## 0. 一句话现状

代码链路（DDS + CRC + 观测/动作契约 + 安全层）已经在仿真里逐位验证过，**但没有在真实 Go2 上验证过**。
本 runbook 的目的就是把"第一次上机"拆成可控的小步：先只读，再吊起，再站，最后走。

风险从大到小：①DDS 发现（网卡/domain/组播）②策略 sim2real 差距 ③500 Hz 实时性 ④安全层误触发。

---

## 1. 接线与网络（5 分钟）

**网口不够用的问题**：这台机器只有一个有线口（`enp0s31f6`），它现在连着 Windows 用来 SSH。
狗要用同一个口的话，SSH 就断了。三选一，**推荐 ①**：

1. **USB 千兆网卡**（几十块）接狗，内置口继续连 Windows/路由器 —— SSH 和狗互不干扰；
2. 直接在笔记本上操作（接显示器/键鼠），网口给狗；
3. 笔记本 + 狗都接一个小交换机/路由器。

**给接狗的口配静态 IP（不要默认路由）**：

```bash
NIC=enp0s31f6            # 换成实际接狗的口（USB 网卡通常是 enp0s20f0u2 / enx...）
sudo ip link set $NIC up
sudo ip addr replace 192.168.123.222/24 dev $NIC
ip -br addr show $NIC     # 应看到 192.168.123.222/24
ip route get 192.168.123.161   # 必须 "dev $NIC"，不能走 wlan0/Clash/ZeroTier
ping -c 3 192.168.123.161      # 通 = 网线/网络 OK（狗要上电）
```

> 永久化（可选）：`nmcli con add type ethernet ifname $NIC con-name go2dog ipv4.method manual ipv4.addresses 192.168.123.222/24 ipv4.never-default yes ipv6.method ignore && nmcli con up go2dog`

**上机前把桌面程序关掉**：Firefox / VSCode / 更新器都要关（6 GB 内存、4 核，控制器要稳 500 Hz）。
代理/Clash/ZeroTier 建议退出（它们会抢路由和组播）。

---

## 2. 一键预检

```bash
cd ~/mjx-go2-mujoco2ros2
bash docker/scripts/preflight_real.sh                 # 本机检查
NIC=$NIC bash docker/scripts/preflight_real.sh --with-dog   # 狗已上电接线后再跑一次
```

通过标准：`FAIL=0`，且 `--with-dog` 时能看到
`[PASS] 容器内能看到 /lowstate` + `[PASS] 低层 /lowstate ≈ 500 Hz`。

---

## 3. 只读自检（不发任何指令，最安全）

**机器人准备**：吊起来（四肢离地 5–10 cm，留根绳）、遥控器在手、急停可达。
Go2 进 debug/低层模式：**遥控器 L2 + R2**（关节进入阻尼态，sport mode 被释放）。
确认 `/lowstate` 有数据后，再考虑发指令。

```bash
cd ~/mjx-go2-mujoco2ros2/docker
UNITREE_NIC=$NIC ROS_DOMAIN_ID=0 docker compose run --rm ros2 bash
# ---- 以下在容器里 ----
cd /workspace
ros2 topic list | grep -E 'lowstate|lowcmd|utlidar'
ros2 topic hz /lowstate        # 期望 ≈ 500 Hz
scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5
```

通过标准：
- `/lowstate` ≈ **500 Hz**（不是 500 就是网络/CPU 问题，**别往下走**）；
- `go2_real_check.py` 报 12 个电机 `mode` 正常、没有 NaN、`/lowcmd` 的 CRC 能算出来；
- `ros2 topic list` 里能看到 `/utlidar/cloud`（可选，今天不用）。

> 常见错误：容器里 `ros2 topic list` 空 → `UNITREE_NIC` 写错 / `ROS_DOMAIN_ID` 不是 0 /
> 忘了给网卡配 192.168.123.222。

---

## 3.5 释放运控（二选一）

低层 `rt/lowcmd` 之前必须"夺权"，否则 sport controller 和你的指令打架（表现为指令被覆盖/机器人不理你）。

- **遥控器（推荐，最稳）**：`L2 + R2` → 机器人四腿变软进入 debug 模式。
- **程序化**（容器里 SDK 可用；不加 `--yes` 只读）：

  ```bash
  # 在容器里；iface 用容器看到的那张网卡
  python3 /workspace/docker/scripts/release_sport_mode.py --iface $NIC          # 只读检查
  python3 /workspace/docker/scripts/release_sport_mode.py --iface $NIC --yes    # 真释放
  ```

> 退出测试时记得恢复运控：`--restore mcf`（新固件）或 `--restore sport_mode`（1.1.6 以前）。

---

## 4. 吊起状态：zero → stand

**zero**（确认能发指令、电机进伺服，机器人会软掉，所以必须吊着）：

```bash
ros2 launch ros2_ws/launch/real.launch.py controller:=zero duration:=10
```

看输出：`n_state ≈ 5000`（10 s × 500 Hz）、`n_cmd ≈ 5000`、`bad_crc: 0`。
机器人表现：关节松掉/被重力拉直，**不该有抽搐或异响**。有异响立刻 Ctrl-C。

**stand**（顶到标称站姿，kp 从 0 在 1 s 内升到 60）：

```bash
ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
```

看输出：`kp_alpha` 应从 0 平滑到 1；机器人四条腿平稳收成站姿，**没有猛的一弹**。
> 如果一上来就"猛弹"或"抖"：Ctrl-C 立即停，回来报告（这是增益/时基问题）。

---

## 5. 落地：policy（先慢速）

确认 stand 正常后，**先保持吊着**跑一次 policy，确认它只是小幅度蹬腿、不挣扎：

```bash
ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=15 \
     scan:=leg_fk
```

正常表现：预摆位 1.5 s 把腿摆到训练 home（前腿 0.8/-1.5，后腿 1.0/-1.5），
然后四腿开始按 **trot** 节奏小幅摆动。`height_scan_source` 应为 `leg_fk`，`obs_valid: true`。

**放下地面**（空旷、平地、周围 2 m 无人），速度先给 0.2 m/s：

```bash
ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30 \
     scan:=leg_fk
# 想给速度就用 run_controller 直接跑（launch 里速度是配置默认 0.5）：
scripts/run_with_ros2.sh ros2_controller/run_controller.py --target real_ros \
     --controller policy --policy-profile go2_v33d --scan leg_fk --linvel estimator \
     --cmd 0.2 0 0 --duration 30
```

通过标准：能沿 +x 方向小步走 ≥3 m，不原地打转、不侧翻、`action_abs_max < 2.5`。

---

## 6. 立即中止的条件（出现任一条就 Ctrl-C）

- 机器人发出**齿轮/电机异响**、抽搐、或某一关节明显反向；
- 明显侧倾/前倾超过 45°，或开始绕圈（说明某条腿的关节映射/符号错了）；
- `n_state` 掉到 300 以下、或 summary 里 `bad_crc > 0`；
- `lag_p95_ms > 20`（说明 500 Hz 稳不住）或 `overruns` 占比 > 50%；
- 健康层反复跳 `L1`/`L2`（summary 的 `level_hist`）。

Ctrl-C 后控制器会**自动发 100 ms 阻尼**（kp=0, kd=8）再退出，机器人会软下来 —— 手扶好。

---

## 7. 故障树

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2 topic list` 空 / `n_state=0` | 网卡名错、domain 不是 0、静态 IP 没配、狗没进 debug 模式 | 重跑 `preflight_real.sh --with-dog`；`ip addr` 确认 192.168.123.222；`echo $ROS_DOMAIN_ID` |
| 有 `/lowstate` 但 hz≠500 | 网线/网口协商速率低、CPU 被抢、代理软件抢栈 | `ethtool` 看速率；关 Firefox/VSCode/Clash |
| 机器人一动不动（`n_cmd>0`） | 没释放 sport mode / `mode` 不是 0x01 / CRC 错 | 遥控器 L2+R2 重进 debug；看 summary 的 `bad_crc` |
| 电机进伺服了但腿乱踢 | 关节顺序或符号错 | 见 `docs/go2-sim2real-references.md` §5.5（本项目恒等映射 FR,FL,FR…）；用 `go2_real_check.py` 抓 `motor_state.q` 对照 |
| 一上电就猛弹 | 增益时基/预摆位没生效 | 已修（`signal_io/node.py` 用"接管以来"的时基，见 CHANGELOG 注释）；若仍出现，`--no-supervisor` 对比定位 |
| 走两步就蹲下 | 健康层 L1（`/lowstate` 断流 >40 ms）触发 crouch | `grep level_hist`；抬高 `health_profiles.real_ros.stale_ms` 到 60–80 |
| 走起来但原地打转 | 线速度观测错（腿式里程计） | `--linvel estimator` 已在用；可临时 `--no-health` 排除保护层影响 |

---

## 8. 收尾

1. Ctrl-C 停控制器（自动阻尼）；
2. **遥控器恢复**：再按 L2+R2 退回正常模式，或 App/遥控器切回 sport mode；
3. 把每次运行的 summary 存下来：
   ```bash
   ... run_controller.py ... --json /workspace/logs/real_$(date +%H%M).json
   ```

---

## 9. 本次仍未验证的（诚实的风险清单）

1. **真机 DDS 连通性**：只在环回 + 官方 SDK 互通上验证过，192.168.123.161 上没测过；
2. **策略 sim2real**：v33d 在 CPU MuJoCo 上 31 地形存活 86.7%，真机地面摩擦/电机模型/延迟都不同；
3. **500 Hz 实时性**：这台机器没有 RT 内核、控制器也没设 SCHED_FIFO；
4. **高度扫描**：真机用 `leg_fk`（平地近似），策略训练时是 raycast；
5. **安全层阈值**：40/80/250 ms 是按仿真抖动定的经验值。
   所以顺序必须是 zero → stand → policy，且第一次 policy 一定吊着跑。
6. **没有搬电池电压**（`LowState.power_v`）：低电压时电机掉力矩也不会告警，
   上机前请自己确认电量（App / 遥控器指示灯），别在低电量下试走路。
7. **没有遥控急停**：`wireless_remote` 字段没接，手边唯一急停是物理断电/L2+B 阻尼。
   所以操作时人必须能一手拍到机器人或按遥控器阻尼。

> 关于 summary 里的 `n_action_clamped`：它是"动作被关节限位夹住"的拍数，
> 与训练侧 `clip(ctrl, ctrlrange)` 语义一致，**不是错误**；v33d 常年有 1 个关节贴限位，
> 所以这个数≈100% 属正常。真正要看的是 `action_abs_max`（<2.5）和
> `obs_valid` / `linvel_confidence` / `pacer.overruns`。
