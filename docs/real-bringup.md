# 真机 Go2 上机 runbook（sim2real 首测）

> 适用：主控 PC（本仓 .venv）通过网线直连 Go2，用 ROS 2 + CycloneDDS 发 `/lowcmd`。
> 目标：**明天第一次上机，按阶段走、每一步都知道该看什么、出事知道怎么退。**
> 配套脚本：`scripts/real_run.sh`（启动器）、`scripts/env_check.py`（环境体检）、
> `scripts/real_sensor_check.py`（传感器只读自检）、`docs/ros2-setup.md`（装 ROS 2）。
> 没装 ROS 2 的机器可以走 docker 官方栈容器，见 §7。

---

## 0. 出发前（今天，在本机）

本机现状（已核实）：**没有 ROS 2**（没有 `/opt/ros`、没有 `rclpy`）、
有线网卡 `enp0s31f6` 是 DOWN、`config/go2_sim.json` 里 `real_ros.interface` 写死 `"eth0"`。
所以今天能做的自测都**不需要真机**：

```bash
# 1) 环境体检（不 ping、不发包，本机就能跑完；FAIL 项就是明天要在现场补的）
.venv/bin/python scripts/env_check.py --target real_ros --no-network

# 2) 启动器把命令全部打印一遍，确认参数/路径/网卡探测都对
bash scripts/real_run.sh --dry-run

# 3) 自测
.venv/bin/python tests/run_tests.py env_check
```

本机跑体检**预期会 FAIL 的项**（不是本仓的 bug，是这台机器没装 ROS 2）：

| 检查项 | 本机现状 | 现场要变成 |
|---|---|---|
| `ros2_prefix` / `ros2_cli` / `rclpy` / `rmw_cyclonedds` | 全 FAIL（没装 ROS 2） | 装好后 OK |
| `libddsc` | WARN（没有 CycloneDDS C 库） | OK |
| `unitree_go` | FAIL（消息包不可见） | OK（colcon build 后 source） |
| `iface_exists` | FAIL（网卡叫 enp0s31f6，config 写的是 eth0） | 插网线后自动探测 / --iface 指定 |
| `ping_robot` | FAIL（没插网线） | OK |
| `rtprio` | WARN（ulimit -r = 0） | 一般不用管（本仓不用 SCHED_FIFO） |

出发前要带：网线（千兆、直通/交叉都行，Go2 网口自适）、吊带/吊架、
**已配好静态 IP 的笔记本**（或现场 `sudo ip addr add 192.168.123.99/24 dev <网卡>`），
遥控器（满电）。

---

## 1. 现场物理准备（上电前）

1. 网线：PC 有线口 ↔ Go2 网口。**不要走 Wi-Fi**。
2. PC 静态 IP 设到 `192.168.123.0/24`（例：`192.168.123.99/24`，不设网关）。
3. 机器人上电，等它自己站起来（sport mode 下自带站姿）。
4. 确认链路：`ping 192.168.123.161` 通（Go2 运控固件地址）。
5. 把机器人**吊起**（四足离地），遥控器在手。
6. 遥控器按 **L2+R2** 进 debug mode（释放 sport mode）——**这一步不做，低层指令会被忽略**。
7. 手放在急停/遥控器阻尼键上，从第 2 节开始。

> 网卡名先看一眼：`ip -br addr`。名字不是 `eth0` 很正常（enp3s0 / enp0s31f6 等），
> 启动器会自动探测 192.168.123.0/24 的 UP 网卡并 `export GO2_IFACE`，不用改 config。

---

## 2. 上机时序（一条命令，每步输入 yes）

```bash
bash scripts/real_run.sh              # preflight -> zero -> stand -> policy，逐步确认
```

也可以分阶段跑（**首测强烈建议分阶段**，每步之间留时间看现象）：

| 阶段 | 命令 | 机器人应处于 | 时长 | 期望现象 | 失败现象 / 立刻做什么 |
|---|---|---|---|---|---|
| preflight | `--stage preflight` | 随意，但先别让它动 | ~15 s | env_check 有 FAIL=0；sensor 自检打印 /lowstate 频率、电机 mode、IMU rpy | env_check 有 FAIL → **不要继续**，按它给的「修」处理 |
| zero | `--stage zero` | **吊起**、已进 debug mode | 10 s | 控制器打印 `io = real_ros -> IOROS...`；`/lowstate` 500 Hz；电机 mode 0x01；关节被 kp=kd=0 托住（不动） | `n_state=0` → 网卡/domain/网线；电机 mode 全 0 → sport mode 没释放 |
| stand | `--stage stand` | **吊起**（或有人扶） | 15 s | 12 关节 1 s 内平滑走到标称站姿，然后保持；无异常抖动/异响 | 姿态明显不对/单腿抽 → **遥控器阻尼键**，回查零位与关节顺序 |
| policy | `--stage policy --duration 20 --cmd "0.2 0 0"` | **落地站稳**、周围清空 | 20 s | 先 pre-pose 再 ramp；按 0.2 m/s 缓慢前走；`level_hist` 基本在 L0 | 走偏/抖/前倾 → 急停；`level_hist` 出现 L1/L2 → 看 `health` |

**每阶段之间做什么：**

- zero 之后：确认 `logs/real_<ts>/zero.json` 里 `n_state` 接近 5000（10 s × 500 Hz）、
  `transport.bad_crc = 0`。`bad_crc > 0` 说明 CRC 算错（真机必现"不动"）。
- stand 之后：**看姿态**。Go2 标称站高约 0.28~0.32 m；如果腿明显伸直/蹲死，先别落地。
- policy 之前：必须落地站好、把吊带松开但**人别走远**，手放急停。

---

## 3. 日志在哪、怎么读

每次运行一个目录：`logs/real_<时间戳>/`（gitignore，但别删，排障全靠它）

```text
logs/real_20261105_143000/
  preflight.json      env_check 的完整结果（items 数组 + counts + exit_code）
  sensor.json         real_sensor_check 的报告（有该脚本时）
  zero.json           run_controller --json：n_state / n_cmd / transport / health ...
  stand.json
  policy.json
```

`run_controller --json` 的 summary 关键字段（**只读这几项就够定位大多数问题**）：

| 字段 | 含义 | 健康值 | 异常时看哪 |
|---|---|---|---|
| `n_state` | 收到的 /lowstate 条数 | ≈ duration × 500 | 0 = 链路完全不通（网卡/domain/网线/CYCLONEDDS_URI） |
| `n_cmd` | 发出的 /lowcmd 条数 | ≈ duration × 500 | 为 0 但 n_state>0 = 安全层没放行 |
| `ever_state` | 是否曾经收到过状态 | true | false = 从没连上 |
| `transport.bad_crc` | 发布前 CRC 校验失败次数 | 0 | >0 = go2_crc 布局/字段问题 |
| `n_publish_err` | 发布异常次数 | 0 | >0 = rmw/DDS 异常，看 stderr |
| `level_hist` | 健康等级分布（L0/L1/L2/L3 各占多少拍） | L0 占绝大多数 | 大量 L1 = 姿态/温度；L2/L3 = 断流或翻转 |
| `health` | 温度/电压/姿态/电机 lost 统计 | 见 config health_profiles | 单项异常看具体字段 |
| `controller` | 控制器自报统计（prepose/ramp/动作范围） | 见各自实现 | 动作被 clip = 策略输出异常 |
| `pacer` | 500 Hz 节拍统计（是否追不上） | 平均周期 ≈ 2 ms | 抖动大 → CPU/线程/后台程序 |

---

## 4. 常见报错 -> 排查

| 现象 / 报错 | 最可能原因 | 处理 |
|---|---|---|
| env_check：没找到 `/opt/ros/*` | 没装 ROS 2 | 装（docs/ros2-setup.md）或 docker/ 那套；注意 Humble/Jazzy/Lyrical 与 python 版本要配套 |
| env_check：`rclpy` FAIL 但 /opt/ros 在 | shell 没 source | 用 `scripts/run_with_ros2.sh`（它替你 source）；手工调试再 source |
| env_check：`unitree_go` FAIL | 消息包没构建/没 source | `cd ros2_ws && colcon build --packages-select unitree_go`，再 source `ros2_ws/install/setup.bash` |
| env_check：目标网卡 `eth0` 不存在 | 本机网卡名不同 | `--iface <名字>` 或 `export GO2_IFACE=<名字>`（config 不用改） |
| env_check：网卡 DOWN | 网线没插/对端没上电 | 插线，`sudo ip link set <iface> up` |
| env_check：不在 192.168.123.0/24 | PC 没配静态 IP | `sudo ip addr add 192.168.123.99/24 dev <iface>` |
| env_check：ping 不通 | 网线/上电/IP 冲突 | 先修链路；ICMP 可能被过滤，但上机前必须通 |
| env_check：`ROS_LOCALHOST_ONLY=1` | 只走回环 | `unset ROS_LOCALHOST_ONLY`（apply_io_env 不会替你清） |
| 控制器：整个运行期没收到 /lowstate | domain/网卡/网线 | 回 preflight；确认 `ROS_DOMAIN_ID=0`、CYCLONEDDS_URI 绑的是有线口 |
| 控制器：收到状态但机器人完全不动 | **sport mode 没释放** / CRC 错 / 电机 mode=0 | 遥控器 L2+R2 进 debug mode；看 `bad_crc`；看电机 mode 是否 0x01 |
| 控制器：一上电就抖/软腿 | kp/kd 或标称站姿不对 | 用 stand 阶段先确认；看 `config` 的 `controller.kp` |
| 控制器：`level_hist` 大量 L2/L3 | 断流（tick 停滞）/姿态翻转 | 网线松、Wi-Fi 干扰、别的程序抢 DDS、机器人真的翻了 → **急停** |
| 控制器：`n_publish_err>0` | rmw/DDS 层异常 | 看 stderr；通常重启控制器即可复现并抓日志 |

---

## 5. 出事了怎么办（按严重度）

1. **机器人乱走 / 抽动 / 异响** → 直接松手让遥控器阻尼（或按急停）。物理优先，不要先点终端。
2. **想正常停止控制器** → 终端 `Ctrl-C`。控制器会先发 100 ms 阻尼（kp=0/kd=8）再退出；
   启动器会打印最后阶段与 summary 路径。
3. **`kill -9` / 拔网线 / 掉电** → **不会**有任何阻尼收尾。机器人保持最后一条指令，
   **不会自己卸力** → 必须手按急停或遥控器阻尼键。这是本仓库最需要记住的一条。
4. **要重跑** → 直接再跑 `--stage zero`（不要从 policy 开始）。
5. **怀疑代码问题** → 把整个 `logs/real_<ts>/` 目录留着，里面 summary 就是现场证据。

---

## 6. 与仿真/其它文档的关系

| 文档 | 内容 |
|---|---|
| 本文 | 现场时序与排错（"现在该做什么"） |
| `real_env/README.md` | 真机链路原理、消息/CRC/sport mode（"为什么这样"） |
| `docs/ros2-setup.md` | 装 ROS 2 / CycloneDDS / unitree_go（"怎么装"） |
| `docs/architecture.md` | 节点图与数据流 |
| `docs/safety-audit.md` | 安全层审计（疯车/断链/力矩） |
| `docs/portability.md` | 迁到机载电脑要改什么 |

仿真对照（同一条命令的仿真版，随时可在本机复现）：

```bash
# 仿真 ROS 2 路径：先起仿真，再起控制器
scripts/run_with_ros2.sh sim_env/run_sim.py --target sim_ros --duration 60 --publish-sensors &
scripts/run_with_ros2.sh ros2_controller/run_controller.py --target sim_ros --controller policy --duration 20
```

---

## 7. 容器（docker 官方栈）里怎么跑

实验室主机没装 ROS 2 时，用官方栈容器（镜像 `unitree-go2-ros2:humble`，`network_mode: host`）。
**容器里不要用 `.venv/bin/python`**：那是宿主 python3.13 建的 venv，容器里 import numpy 会失败；
容器里的正确解释器是 PATH 上的 `python3`（= `/opt/venv/bin/python`，py3.10 + numpy 2.2.6 + mujoco 3.17.0）。
`docker/setup_unitree_humble.sh` 已经 `export GO2_PYTHON=/opt/venv/bin/python`，
`scripts/real_run.sh` 与 `scripts/run_with_ros2.sh` 都会优先用它（`real_run.sh` 里是
`PY="${GO2_PYTHON:-$ROOT/.venv/bin/python}"`），所以下面这条能直接跑通。

一条命令（`UNITREE_NIC` 填**宿主**的有线网卡名，例如 `enp0s31f6`）：

```bash
UNITREE_NIC=enp0s31f6 ROS_DOMAIN_ID=0 \
  docker compose -f docker/docker-compose.yml run --rm ros2 \
  bash -lc "python3 scripts/env_check.py --target real_ros && bash scripts/real_run.sh --stage all --yes"
```

要点：

- `network_mode: host`：容器直接用宿主网卡，所以 `GO2_IFACE` 必须填**宿主**网卡名
  （容器里 `ip -br addr` 看到的就是宿主的名字）。
- `GO2_IFACE` / `GO2_DOMAIN_ID` 覆盖在容器里已实测可用（见 `signal_io/config.py` §6）；
  不用改 `config/go2_sim.json`。
- 容器里 `bash scripts/run_with_ros2.sh -c "..."` 正常（它会解析到 `/opt/venv/bin/python`）。
- **交互确认**：`docker compose run` 没有 TTY 时 `real_run.sh` 会拒绝启动（退出码 3），
  所以要显式 `--yes`；人在现场时加 `-it` 保留逐阶段确认更安全：
  `docker compose ... run --rm -it ros2 bash -lc "bash scripts/real_run.sh"`
- 日志落在宿主的 `logs/real_<ts>/`（compose 把仓库目录挂进容器）。

---

## 8. 已知未验证（诚实说明）

- 本仓**从未在真机上跑过**；下面这些在真机上属于"第一次"：
  - ROS 2 ↔ 固件 CycloneDDS 的实际互通（domain 0、rmw、网卡绑定）；
  - `go2_crc` 的 812 字节布局在真固件上的接受度（仿真侧已用官方 C 库 gold value 校验）；
  - 电机 mode 0x01 + `level_flag=0xFF` + head 字段的实际使能时序；
  - 500 Hz 在笔记本（i5-10310U，ulimit -r=0）上的抖动表现；
  - `--scan leg_fk` 平地近似的实际行走质量（地形感知需要先补状态估计/外参标定）。
- 因此首测**只做 zero/stand/policy 小速度**，不要直接上台阶/跑跳。
- 本机没有 ROS 2，所以 `env_check.py` 与 `real_run.sh` 的 ROS 分支只在本机验证到
  "能正确报 FAIL / --dry-run 能打印命令"这一层；装好 ROS 2 的机器上仍需第一次实跑确认。
- 容器（`unitree-go2-ros2:humble`）里 `env_check.py` 与 `real_run.sh --stage all --yes`
  已实测能跑通（含 `GO2_IFACE` / `GO2_DOMAIN_ID` 覆盖）；但容器里**没有真机**，
  也就是"ROS 2 环境这一半通了"，不是"真机能动"。
