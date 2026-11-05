# 宇树官方推荐栈 Docker 环境（Go2 / ROS 2 Humble）

> 目标：把“宇树官方推荐的低层开发生态”做成一个可复现镜像，用来跑本仓
> mjx-go2-mujoco2ros2 的 ROS 2 路径、官方 unitree_sdk2_python 路径，以及真机前的一切离线验证。
>
> 配套审计：[../docs/unitree-official-sdk-conformance.md](../docs/unitree-official-sdk-conformance.md)
>
> 📦 **单文件交付（把代码也打包进镜像，离线搬运）**：见 [`DEPLOY.md`](DEPLOY.md)。
> 一句话：`docker/Dockerfile.deploy` 把仓库 `COPY` 进镜像 → `docker save` 成一个 tar →
> 目标机 `docker load` 就能跑（不需要网络 / pip / colcon / 重建 venv）。

---

## 0. 为什么需要它

本仓宿主环境是 Ubuntu 26.04 (WSL2) + ROS 2 **Lyrical** + CycloneDDS **11.0.1** + Python **3.14**，
而宇树官方的支持范围是：

| 维度 | 宇树官方 | 本仓宿主（原来） | 本镜像 |
|---|---|---|---|
| OS | Ubuntu 20.04 / **22.04**（不支持 WSL/Windows） | Ubuntu 26.04 WSL2 | **Ubuntu 22.04.5** |
| ROS 2 | Foxy（20.04）/ **Humble（22.04，recommend）** | Lyrical Luth | **Humble** |
| CycloneDDS | 机器人固件 **0.10.2** | **11.0.1**（Lyrical 自带） | **0.10.5**（Humble apt，同 0.10.x 系列） |
| Python | 官方 Python SDK 要求 >=3.8 且 **cyclonedds==0.10.2** | **3.14**（SDK 不可用） | **3.10**（SDK 可用） |

所以本镜像 = **官方 README / Quick_start / .devcontainer 里那套组合**，外加本仓需要的 mujoco/numpy。

---

## 1. 镜像里有什么

| 组件 | 版本 / 位置 | 说明 |
|---|---|---|
| 基座 | ros:humble-ros-base（Ubuntu 22.04.5） | 官方 .devcontainer/Dockerfile-humble 同源 |
| ROS 2 | Humble | rmw_cyclonedds_cpp / rosidl-generator-dds-idl / colcon 已装 |
| CycloneDDS | ros-humble-cyclonedds **0.10.5** | 与固件 0.10.2 同系列；官方 README 明确 Humble 不需要重编 |
| 官方 unitree_ros2 | **v0.3.0** → /opt/unitree_ros2/cyclonedds_ws/install | unitree_go / unitree_api / unitree_hg 消息包 |
| 官方 unitree_sdk2（C++） | **2.0.2** → /opt/unitree_robotics | libunitree_sdk2.a + 31 个官方例程（含 go2_low_level / go2_stand_example / go2_sport_client / go2_robot_state_client …） |
| 官方 unitree_sdk2_python | commit 814556d（editable 安装） | 依赖 **cyclonedds==0.10.2**（官方 pin，wheel 自带 libddsc） |
| 本仓 python venv | /opt/venv（Python 3.10） | numpy 2.2.6 + mujoco 3.17.0 + matplotlib；--system-site-packages 能看到 rclpy / unitree_sdk2py |
| 无显示器渲染 | libosmesa6 / libgl1 / EGL / GLFW / X11 库 | 可视化测试用 MUJOCO_GL=osmesa；有 WSLg 时可跑观察器 |

镜像大小：**约 2.7 GB**（docker images）。

---

## 2. 构建

前提：宿主机装好 Docker Engine + compose v2（见 §8 本机安装记录）。

```bash
cd docker

# 方式 A：compose（会读取宿主 HTTP_PROXY/HTTPS_PROXY，构建期走 host 网络）
HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897 \
  docker compose build

# 方式 B：直接 docker build（同一份 Dockerfile）
DOCKER_BUILDKIT=1 docker build --network=host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
  -t unitree-go2-ros2:humble .
```

- 没有代理时，把 --build-arg 代理参数去掉、compose 里 build.network 那行删掉即可（直连构建）。
- 首次构建约 8–12 分钟（取决于代理速度），之后改脚本/文件只重建最后几层。
- 可覆盖的上游版本：UNITREE_ROS2_REF（默认 v0.3.0）、UNITREE_SDK2_REF（2.0.2）、
  UNITREE_SDK2_PY_REF（814556d）、CYCLONEDDS_PY_VER（0.10.2）。

---

## 3. 快速开始（仿真，不需要机器人）

```bash
cd docker

# 进入容器（挂载本仓到 /workspace，host 网络）
docker compose run --rm ros2

# —— 以下都在容器里执行 ——
bash /workspace/docker/scripts/smoke_test.sh          # 环境自检（版本/消息包/SDK/网卡）
bash /workspace/docker/scripts/verify_conformance.sh  # 一键验证（14 项，见 §6）
```

常用命令：

```bash
# 与真机同构的三节点 ROS 2 节点图（sim_sensors + perception + controller）
ros2 launch ros2_ws/launch/sim.launch.py duration:=20

# 双进程验收（ROS 2 路径 / 官方 Python SDK 路径）
python scripts/verify.py --target sim_ros     # 16/16
python scripts/verify.py --target sim_sdk     # 16/16（宿主 py3.14 跑不了，这里可以）

# 单进程观察器 / headless
bash scripts/view_go2_policy.sh --terrain hyb2
bash scripts/view_go2_policy.sh --headless --terrain wide_h050 --duration 15
```

> 容器里 python 已在 PATH 上（/opt/venv），且 ros2/unitree_go 环境由 BASH_ENV 自动 source。

GUI（可选）：compose 已挂载 /tmp/.X11-unix 并透传 DISPLAY。WSLg 下一般可直接弹窗；
若 GL 不可用，用 --headless 或 MUJOCO_GL=osmesa。

---

## 4. 官方 SDK / DDS 互通 / 释放运控

```bash
# A. 官方 SDK 订阅 ROS 2 的 /lowstate（验证 DDS 类型/话题映射）
python sim_env/run_sim.py --target sim_ros --duration 14 &
python docker/scripts/dds_sub_lowstate.py --domain 1 --iface lo --duration 6

# B. 官方 SDK 发带官方 CRC 的 rt/lowcmd，本仓 ROS 2 仿真侧逐条校验
python sim_env/run_sim.py --target sim_ros --duration 16 &
python docker/scripts/dds_pub_lowcmd.py --domain 1 --iface lo --duration 8 --rate 500

# C. 官方 C++ 例程（官方文档：SDK2 与 ROS2 环境二进制冲突，必须清掉 ROS 环境再跑）
bash docker/scripts/run_official_sdk2_example.sh --list
bash docker/scripts/run_official_sdk2_example.sh go2_stand_example <网卡名>

# D. 官方 MotionSwitcher 释放主运控服务（真机；默认只读，--yes 才动手）
python docker/scripts/release_sport_mode.py --iface eth1                # 只读 CheckMode
python docker/scripts/release_sport_mode.py --iface eth1 --yes          # 吊起后释放
python docker/scripts/release_sport_mode.py --iface eth1 --yes --restore mcf

# E. 换机器/换 distro 后先跑「跨版本 DDS 探针」（真机前必做）
#    用官方 SDK 0.10.2 当机器人固件替身，实测本机 ROS 2 能否双向互通；详见 scripts/version_probe/README.md
bash docker/scripts/version_probe/run_version_probe.sh --sdk-src /opt/unitree_sdk2 --domain 6 --iface lo
```

---

## 5. 真机（Go2）

### 5.1 前置（官方 Quick_start）

1. 机型：Go2 **EDU / X**（AIR/PRO 不支持低层二次开发）；记录固件版本（运控服务名 mcf 还是 sport_mode）。
2. 网线连 PC 与机器人；PC 网卡静态 **192.168.123.x/24**（例 192.168.123.222），**不要**设成 .161。
   ping 192.168.123.161 通才算链路 OK。
3. 官方不支持 WSL/Windows：**推荐原生 Ubuntu 22.04**。本机 WSL2 是 mirrored 网络（可以看到 Windows 的网卡），
   理论上可用，但组播发现未验证，见 §9。
4. 机器人吊起 5–10 cm；遥控器 **L2+R2** 进 debug mode（关节阻尼），或用 §4-D 的脚本程序化释放运控。

### 5.2 运行

```bash
# 宿主机：先确认容器里能看到 192.168.123.x 的网卡
UNITREE_NIC=eth1 ROS_DOMAIN_ID=0 docker compose run --rm ros2

# 容器里：网卡名以 ip -4 addr 看到的为准；GO2_INTERFACE 会自动透传给本仓 config
ip -4 addr | grep 192.168.123
python docker/scripts/release_sport_mode.py --iface $UNITREE_NIC          # 先只读
python docker/scripts/release_sport_mode.py --iface $UNITREE_NIC --yes    # 吊起后释放运控

# 只读自检（不发送任何指令）
scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

# 分阶段：zero -> stand -> policy（全程吊起/空旷，手放急停）
ros2 launch ros2_ws/launch/real.launch.py controller:=zero  duration:=10
ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
```

期望自检结果：/lowstate 约 500 Hz、电机 mode=0x01、/lowcmd 无其它发布者（无其它控制器在跑）、
L1 点云 frame=utlidar_lidar。控制器退出会先发 100 ms 阻尼（kp=0/kd=8）再断流。

### 5.3 排障顺序

1. 网线/网段/IP（ping 192.168.123.161）
2. ROS_DOMAIN_ID=0
3. UNITREE_NIC 是否正确（192.168.123.x 那张网卡；容器里名字可能和宿主机不同）
4. ROS_LOCALHOST_ONLY 必须未设置
5. ros2 topic list 能否看到 /lowstate；ros2 topic hz /lowstate 是否 ~500 Hz
6. 电机不动：释放运控（mcf/sport_mode）+ 电机 mode 0x01 + CRC 三件事

---

## 6. 验证结果（2026-10-25 实测，docker compose run + verify_conformance.sh）

结论：**PASS=14 FAIL=0 WARN=0**，日志留在本仓 logs/container/go2_conformance/。

| # | 检查 | 结果 |
|---|---|---|
| 0 | 官方栈自检（Ubuntu 22.04.5 / Humble / CycloneDDS 0.10.5 / unitree_go / sdk2 / sdk2py） | ✅ |
| 0 | 官方 Python SDK CRC 类 | ✅ 对 head=FE EF/level=FF/20×mode=1/q=0 算出 **0x57ECF19D**，与本仓 go2_crc.py gold value 逐位相同 |
| 1 | 本仓单元测试 | ✅ **14/14**（MUJOCO_GL=osmesa 离屏渲染） |
| 2 | ROS 2 闭环 verify.py --target sim_ros | ✅ **16/16**；RTF 1.000，p95 lag 0.92 ms，bad_crc=0 |
| 3 | 官方 SDK 闭环 verify.py --target sim_sdk | ✅ **16/16**；RTF 1.000，p95 lag 1.59 ms（宿主 py3.14 上此前完全不可用） |
| 4 | ROS 2 /lowstate → 官方 SDK rt/lowstate | ✅ 6.02 s 收到 **3008 条 ≈ 499.8 Hz**；tick/关节角/rpy/mode/bit_flag 均正确 |
| 5 | 官方 SDK rt/lowcmd（官方 CRC）→ 本仓 ROS 2 仿真 | ✅ SDK 发 4000 条 @500 Hz；仿真侧 rx_cmd=**3990**、**bad_crc=0** |
| 6 | ros2 launch sim.launch.py duration:=8 三节点图 | ✅ 正常结束：sim 4000 步、bad_crc=0、策略正常跟踪（x≈1.9 m） |
| 7 | 官方 C++ SDK | ✅ libunitree_sdk2.a + 31 个官方例程（含 go2_low_level/go2_stand_example/…） |

互通能成立的关键：官方 C++ SDK 的 DDS 类型名是 unitree_go::msg::dds_::LowCmd_，
ROS 2 生成的名字一致，cyclonedds-python 也会把内部点号名转换成 :: 形式，因此两条官方路径在 DDS 层直接互通。

---

## 7. 目录结构

```text
docker/
├── Dockerfile                 # Ubuntu22.04 + ROS Humble + CycloneDDS + 官方三件套 + 本仓 venv
├── docker-compose.yml         # host 网络 / 挂载本仓 / UNITREE_NIC + ROS_DOMAIN_ID
├── setup_unitree_humble.sh    # 被 entrypoint / bash 自动 source 的环境
├── entrypoint.sh              # source 环境 + 打印一次摘要 + exec
├── README.md                  # 本文件
└── scripts/
    ├── smoke_test.sh          # 环境自检
    ├── verify_conformance.sh  # 一键 14 项验证
    ├── dds_sub_lowstate.py    # 官方 SDK 订阅 rt/lowstate
    ├── dds_pub_lowcmd.py      # 官方 SDK 发 rt/lowcmd（含官方 CRC）
    ├── release_sport_mode.py  # 官方 MotionSwitcher 释放/恢复运控
    ├── run_official_sdk2_example.sh  # 清 ROS 环境跑官方 C++ 例程
    └── version_probe/         # 跨版本 CycloneDDS 互通探针（真机前必做，含实测结果）
        ├── README.md
        ├── run_version_probe.sh
        ├── pub_state.cpp / sub_cmd.cpp
        └── ros2_sub.py / ros2_pub.py
```

---

## 8. 宿主侧一次性安装（本机已做，记录备查）

本机是 WSL2 Ubuntu 26.04（systemd 已启用），用 apt 装了原生 Docker Engine（不是 Docker Desktop）：

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-v2 docker-buildx
sudo systemctl enable --now docker
sudo usermod -aG docker $USER          # 重新登录 / wsl --shutdown 后生效

# 可选：宿主走 Clash 代理时，让 dockerd 能拉镜像
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/http-proxy.conf <<'EOF'
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7897"
Environment="HTTPS_PROXY=http://127.0.0.1:7897"
Environment="NO_PROXY=localhost,127.0.0.1,::1,10.0.0.0/8,172.16.0.0/12,192.168.0.0/16"
EOF
sudo systemctl daemon-reload && sudo systemctl restart docker

# 仅当不想重新登录时，临时让当前会话免 sudo 用 docker（权限上等于 root，介意可删）
sudo tee /etc/systemd/system/docker.service.d/socket-perms.conf <<'EOF'
[Service]
ExecStartPost=/bin/sh -c 'chmod 666 /var/run/docker.sock'
EOF
sudo systemctl daemon-reload && sudo systemctl restart docker

# 宿主 apt 走代理（本机加过；不需要代理可删）
sudo tee /etc/apt/apt.conf.d/99proxy <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7897";
Acquire::https::Proxy "http://127.0.0.1:7897";
EOF
```

镜像构建/运行都**不需要** Docker Desktop；容器用 --network=host 直接复用 WSL 的网络命名空间。

---

## 9. 已知限制 / 注意

1. **官方不支持 WSL2/Windows**（Quick_start 原文）。本机在 WSL2 里跑仿真/离线验证没问题；
   真机 DDS 是否可达取决于 WSL mirrored 网络能否暴露 192.168.123.x 网卡与组播发现，
   **未验证**。真机建议原生 Ubuntu 22.04。
   > 已实测（scripts/version_probe/README.md）：宿主 ROS 2 Lyrical（CycloneDDS **11.0.1**）与官方
   > 0.10.2 **双向都不通**（topic 能发现但收不到，甚至让 0.10.2 侧 segfault）；
   > 容器 Humble（**0.10.5**）双向都通（rx=2026 / received 484）。**所以宿主 Lyrical 栈不要上真机。**
2. **必须 host 网络**：DDS 靠组播发现，bridge/NAT 网络下收不到 rt/lowstate、也发不出 rt/lowcmd。
   compose 已固定 network_mode: host。
3. **CycloneDDS 0.10.5 vs 固件 0.10.2**：同一 0.10.x 系列、官方 Humble 组合即如此；
   若要逐版本对齐，可在 Dockerfile 里按官方 Foxy 方式编译 cyclonedds releases/0.10.x（0.10.2）。
4. **Python SDK 必须 editable 安装**（pip install -e .）：非 editable 的 wheel 不带
   unitree_sdk2py/utils/lib/crc_*.so，CRC 类会直接报 “cannot open shared object file”。
   本镜像用的是官方 README 的 editable 方式。
5. 镜像需要构建期联网（apt / GitHub / PyPI）；本机通过宿主代理构建。
6. 观察器 GUI 需要 WSLg 或 X11；纯 headless 用 --headless 或 MUJOCO_GL=osmesa。
7. **官方 unitree_sdk2 与 ROS 2 环境互斥**（[ROS2_service](https://support.unitree.com/home/zh/developer/ROS2_service)：
   SDK2 要占用 CycloneDDS，不能在初始化了 ROS/ROS2 的环境里启动）。本仓每个进程只用一种 transport，
   不受影响；但跑官方 **C++** 例程要用 §4-C 的脚本清掉 ROS 环境。
8. 本镜像只解决“环境与离线链路”；**真机行为仍未验证**（本仓自述尚未上真机），
   上机请按 §5 和审计文档 §6 的 checklist 逐项确认。


---

## 10. 搬到“控制用 Linux 机器”（真机实际路径）

真机不需要（也不应该）在这台 WSL2/Windows 上跑。官方要求原生 Linux；**控制机的发行版不限**，
因为官方那套 Ubuntu 22.04 + Humble + CycloneDDS 0.10.x 全在镜像里，宿主只提供 Docker + 网卡。

### 10.1 三种搬运方式

| 方式 | 适用 | 命令 |
|---|---|---|
| A. 导镜像过去 | 控制机有 Docker、能收文件 | 本机 `docker save unitree-go2-ros2:humble ｜ pigz -1 > go2-humble-image.tar.gz`（实测 **574 MB**）→ 控制机 `pigz -dc go2-humble-image.tar.gz ｜ sudo docker load` |
| B. 一键脚本 | 控制机是任意 Ubuntu/Debian **x86_64** | `sudo bash linux_host_setup.sh --image go2-humble-image.tar.gz --project mjx-go2-mujoco2ros2.tar.gz --nic <接狗网卡>` |
| C. 现场重建 | 控制机能联网 | `cd docker && docker compose build`（约 10 分钟，见 §2） |

`scripts/linux_host_setup.sh` 会：装 `docker.io` + `docker-compose-v2` → `docker load` 镜像 →
把源码解到 `~/mjx-go2-mujoco2ros2` → （可选）给狗侧网卡配 `192.168.123.222/24`（**不设默认路由**）→
把用户加进 docker 组 → 打印下一步预检命令。全程不要求宿主是 22.04。

### 10.2 接线与 IP

1. 控制机 ↔ Go2 用网线直连（狗在 `192.168.123.161`；**别把 PC 设成 .161**）。
2. PC 侧网卡静态 `192.168.123.222/24`，**不要**设网关/默认路由（否则上网会断）。
3. 控制机需要第二块网口（或 USB 网卡/交换机）来同时连着外网/Windows；只有一块网口时，
   先把环境装好，再把网线换到狗上。
4. 官方不支持在 WSL/Windows 上跑真机；`192.168.123.x` 网卡 + 组播在 WSL2 mirrored 下未验证（§9-1）。

### 10.3 真机预检（只读，不发任何指令）

```bash
cd ~/mjx-go2-mujoco2ros2/docker
ping -c1 192.168.123.161                                  # ① 链路
UNITREE_NIC=<接狗网卡> ROS_DOMAIN_ID=0 docker compose run --rm ros2
# 容器内：
ip -4 addr | grep 192.168.123                             # ② 网卡进容器了
ros2 topic list | grep -E 'lowstate|lowcmd|utlidar'       # ③ DDS 发现
ros2 topic hz /lowstate                                   # ④ 期望 ~500 Hz（关键）
ros2 topic echo --no-arr /lowstate | head -30             # ⑤ 有数据
python docker/scripts/release_sport_mode.py --iface $UNITREE_NIC   # ⑥ 只读 CheckMode
```

判据：③ 有话题但 ④ 是 0 Hz → DDS/QoS/组播问题，**换环境**，不要继续发指令。

### 10.4 真机动作（必须吊起、手放急停）

```bash
# 官方 MotionSwitcher 程序化释放运控（或遥控器 L2+R2 进 debug mode）
python docker/scripts/release_sport_mode.py --iface $UNITREE_NIC --yes

# 只读自检
scripts/run_with_ros2.sh scripts/go2_real_check.py --target real_ros --duration 5

# 分阶段：zero -> stand -> policy
ros2 launch ros2_ws/launch/real.launch.py controller:=zero  duration:=10
ros2 launch ros2_ws/launch/real.launch.py controller:=stand duration:=15
ros2 launch ros2_ws/launch/real.launch.py controller:=policy duration:=30
```

退出时控制器先发 100 ms 阻尼（kp=0/kd=8）再断流；任何异常先遥控器急停/断电。

