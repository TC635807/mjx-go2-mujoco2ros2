# 可迁移性（portability）：把本仓搬到机载电脑 / 实验室另一台机器

> **一句话结论：能搬，但绝不能"整个文件夹 cp -a 过去就能跑"。**
> 源码 + 模型 + 策略 npz + config 可以直接带走；
> `.venv/`、`ros2_ws/{build,install,log}/`、`build/` **必须在新机器上重建**；
> 而 `real_ros`（真机路径）还要求目标机装好 ROS 2 + `rmw_cyclonedds_cpp` + `unitree_go` 消息包。
>
> ⚠️ **如果交付方式是 docker 镜像，本页的「必须重建」两条不适用**：
> 镜像里已经有一套 py3.10 的 `/opt/venv`（numpy/mujoco/matplotlib）和编译好的 `unitree_go`，
> 而 `docker/Dockerfile.deploy` 把仓库 `COPY` 进 `/workspace` —— 宿主的 `.venv` 与
> `ros2_ws/install` 被 `.dockerignore` 排除，**根本不会进容器**。
> 那种情况要搬的是**一个镜像 tar**（`docker save` / `docker load`），不是文件夹；见
> [`../docker/DEPLOY.md`](../docker/DEPLOY.md)。本页剩下的内容是**原生安装**（机载电脑 / 裸机）路径。

**证据口径**：本文所有 `file:line` 都来自**本机工作区实测**（2026-11-04，git `60cc4c1` + 当日上机前改动），
不是猜的。行号会随改动漂移，**同行的锚点注释/函数名才是权威**：

```
grep -n 'GO2_IFACE'        signal_io/config.py
grep -n 'sim = Go2Sim'     ros2_controller/run_controller.py
```

**相关文档**：[ros2-setup.md](ros2-setup.md)（ROS 2 安装）、[real-bringup.md](real-bringup.md)（现场 runbook）、
[../real_env/README.md](../real_env/README.md)（真机通信栈与安全流程）、
[unitree-official-sdk-conformance.md](unitree-official-sdk-conformance.md)（官方口径）、
[../scripts/onboard_setup.sh](../scripts/onboard_setup.sh) 与 [../scripts/package_for_onboard.sh](../scripts/package_for_onboard.sh)（本文配套脚本）。

---

## 0. TL;DR（三行）

1. **带走**：`*.py` / `config/` / `sim_env/models/` / `ros2_controller/policies/*.npz` / `ros2_ws/src/` / `docs/` / `scripts/`
   —— 全仓打包后 **58.4 MiB（zst 压缩后 24.0 MiB，319 个文件）**（本机 2026-11-04 15:46 实测；数字随仓库内容增长）。
2. **重建**：`.venv`（196 MiB，cp313 + x86_64）、`ros2_ws/build`+`install`（22 MiB，容器 `/work` 里 py3.10 构建）、根 `build/`。
3. **前提**：要跑真机（`real_ros`）目标机就必须有 ROS 2（`/opt/ros/<distro>`）；
   纯仿真/离线复现只要 python 3.10+ 和 mujoco/numpy。

---

## 1. 两台机器差在哪（本机实测 vs 目标机预期）

| 项 | 本机（本文写作环境，已验证） | 目标机（实验室 PC / Go2 机载电脑） | 后果 |
|---|---|---|---|
| OS | Ubuntu **25.10**（`/etc/os-release` 实测；`docs/ros2-setup.md:9` 写的 26.04 是旧值） | 22.04（ROS 2 Humble）/ 机载 20.04 | apt 源与 ROS 发行版完全不同 |
| 架构 | `x86_64` | **`aarch64`**（Jetson Orin）或 x86_64 | 二进制产物全部作废 |
| python | **3.13.7**（`.venv/pyvenv.cfg`） | 22.04 → 3.10；20.04 → 3.8（`requirements.txt:1` 要求 ≥3.10） | venv / wheel / `.so` 全不可复用 |
| ROS 2 | **没有**（无 `/opt/ros`；`scripts/env_check.py` 实测 FAIL 6 项） | Humble（官方推荐组合） | `real_ros` 在本机根本跑不了 |
| 网卡 | `enp0s31f6`(**DOWN**)、`wlp0s20f3`(Wi-Fi)、无 `eth0` | 有线口名未知，必须现场 `ip -br addr` | config 里写死 `eth0` 在两台机器上都失败 |
| 资源 | 8 核 / 6.9 GiB | Jetson Orin NX 16 GB，或实验室 PC | 推理算力不是瓶颈（§2.8） |
| 显示 | WSLg/X11（观察器用） | 通常无 GUI | 目标机只能 headless |

> **迁移方向提醒**：目标机（22.04 + Humble）其实比本机更接近官方支持组合
> （官方只覆盖 20.04+Foxy / 22.04+Humble，CycloneDDS 0.10.x，见
> `docs/unitree-official-sdk-conformance.md:35,174-204`）。本机 Lyrical 自带 CycloneDDS 0.11，
> 与固件 0.10.2 的互通性是"未实测"项（`real_env/README.md:213-215`）。
> 所以搬过去很可能**比现在更稳**，不是更差。

---

## 2. 迁移审计（一条一条对着 grep 结果写）

### 2.1 硬编码绝对路径（`/home/...`）

| 在哪 | 影响 | 处理 |
|---|---|---|
| `scripts/view_go2_policy_local.sh:76` —— `exec /home/tc63/mujoco/.venv/bin/python ...` | **最危险的一个**：目标机上直接 `No such file or directory` | 别用它；用 `scripts/view_go2_policy.sh`（走 `run_with_ros2.sh`，python 路径可覆盖）。建议后续改成 `$ROOT/.venv/bin/python` 或 `$GO2_PYTHON` |
| `scripts/fetch_go2_model.sh:4` —— `DST=/home/tc63/...` | 只有"重新下载模型"时才用到；模型已随包带走 | 目标机别跑它；要跑先改 `DST`（建议改成相对 `ROOT`） |
| `scripts/export_policy.py:190` —— 默认 `--pkl /home/tc63/.../go2_walk_v25b.pkl` | 只在训练仓导出策略时用（目标机不导出） | 上机不需要；导出时显式 `--pkl` |
| `scripts/compare_deploy_vs_train_physics.py:37`、`scripts/probe_height_scan_sources.py:28`（`GETUP=/home/tc63/...`） | 训练-部署物理对比工具，目标机没有训练仓 | 不用；要复现就挂载/软链训练仓 |
| `scripts/verify_train_deploy_obs.py:34`（`TRAIN_REPO` 默认 `/home/tc63/...`） | 同上（**可用环境变量覆盖**） | `TRAIN_REPO=/path/to/mjx-go2-training` |
| `scripts/verify_train_deploy_parity.py:47-48`（`GETUP_TRAIN_BODY` / `MENAGERIE_BODY`） | 同上，且依赖 mujoco_menagerie | 不用这两个脚本 |
| `ros2_controller/policies/*.json:3`（`source_pkl`）、`config/domain_gap.json:5`（`go2_train_model`） | 纯元数据：**没有任何代码打开它们**（已 grep 确认） | 原样带走即可，不会报错 |
| `.vscode/settings.json:2`（`cmake.sourceDirectory` 绝对路径） | 未入库（untracked）；只影响 VSCode 的 CMake 插件 | 不打包（打包脚本已排除 `.vscode/`） |

### 2.2 网卡 / DDS 配置

| 项 | 在哪 | 影响 | 处理 |
|---|---|---|---|
| 真机网卡写死 `eth0` | `config/go2_sim.json:88`（`real_ros`）、`:98`（`real_sdk`）；代码默认值 `signal_io/config.py:186,199` | 本机根本没有 `eth0` → `CYCLONEDDS_URI` 绑到不存在的网卡，**收不到 `/lowstate`** | **不要改 config**，用环境变量覆盖（下一行） |
| `GO2_IFACE` / `GO2_INTERFACE` 覆盖 | `signal_io/config.py:308`（`GO2_IFACE` 优先，旧名 `GO2_INTERFACE` 兼容） | 不改文件就能换网卡 | `export GO2_IFACE=enp3s0`（现场 `ip -br addr` 查真名） |
| 自动探测网卡 | `signal_io/config.py:548` `detect_iface()`；只被 `scripts/real_run.sh` / `scripts/env_check.py` 调用 | `load_config()` **不会**自动调用它（避免行为不可预测） | 手工跑 `run_controller.py` 时必须自己 `export GO2_IFACE` |
| CycloneDDS 绑定写法 | `signal_io/config.py` 的 `apply_io_env()`（`<Interfaces><NetworkInterface .../>`） | 多网卡机器不绑网卡会走到 Wi-Fi | 已在代码里处理；`ROS_LOCALHOST_ONLY` 必须未设置 |
| 网段 / 静态 IP | `real_env/README.md:51-57`（PC 需静态 `192.168.123.x/24`） | DHCP 或网段错 → DDS 发现不到机器人 | 现场配静态 IP，`ping 192.168.123.161` 通才算物理链路 OK |

### 2.3 ROS 2 发行版

| 项 | 在哪 | 影响 | 处理 |
|---|---|---|---|
| 自动选发行版 | `scripts/run_with_ros2.sh:39-49`（`ROS_SETUP` > `ROS_DISTRO` > `/opt/ros/lyrical` > `/opt/ros/humble`） | 目标机 Humble 时**能正常工作**（命中 humble 分支，或显式 `ROS_DISTRO=humble`） | 推荐 `export ROS_DISTRO=humble` |
| 硬编码 `/opt/ros/lyrical` | `scripts/demo_ros2_policy.sh:18`、`scripts/view_go2_policy.sh:75`、注释 `scripts/bench_io.py:4` | 目标机没 lyrical → `source` 失败，脚本直接死 | 用 `run_with_ros2.sh` / `real_run.sh`；这两个 demo 脚本建议后续改成探测（§6） |
| `unitree_go` 需要 DDS IDL 生成器 | `docs/ros2-setup.md:26,30` | 少装 `ros-<distro>-rosidl-generator-dds-idl` → colcon 在 `rosidl_generator_dds_idl` 步失败 | 目标机 apt 装 `ros-humble-rosidl-generator-dds-idl`（`onboard_setup.sh --with-ros` 会把命令打出来） |
| 官方 SDK 与 ROS 2 环境互斥 | `docs/unitree-official-sdk-conformance.md:50`、`docker/scripts/run_official_sdk2_example.sh:31` | `real_sdk`（unitree_sdk2py）与 rclpy 同进程互斥；且 Python 3.14 不可用 | 真机一律走 `real_ros`（`real_env/README.md:199-202`） |

### 2.4 Python 版本 / ABI（为什么不能拷 `.venv`）

实测：

```
.venv/pyvenv.cfg                            version = 3.13.7    home = /usr/bin
.venv/bin/python3 -> /usr/bin/python3       （**绝对**符号链接）
.venv/.../mujoco-3.17.0.dist-info/WHEEL:4   Tag: cp313-cp313-manylinux_2_27_x86_64
.venv/.../numpy-2.5.4.dist-info/WHEEL       Tag: cp313-cp313-manylinux_2_27_x86_64
.venv/.../mujoco/_constants.cpython-313-x86_64-linux-gnu.so
```

拷到目标机的三条具体后果（按目标机形态）：

| 目标机 | 现象 |
|---|---|
| 同 x86_64 但 python 3.10 | `.venv/bin/python3 -> /usr/bin/python3` 变成 3.10，解释器按 `cpython-310-*.so` 找扩展，目录里只有 `cpython-313-*` → `ImportError: ... cannot open shared object file`（numpy/mujoco 都是） |
| 同 3.13 但 arm64 | `.so` 是 x86-64 ELF → `ImportError: ... wrong ELF class` / `invalid ELF header` |
| 目标机 `/usr/bin/python3` 不是 3.13 | venv 的解释器符号链接断掉 → `bad interpreter: No such file or directory` |

结论：`.venv` 是**机器专属产物**。`package_for_onboard.sh` 默认排除它，
`onboard_setup.sh` 在目标机上重建。

### 2.5 构建产物（必须重建）

| 项 | 在哪 | 证据 | 处理 |
|---|---|---|---|
| `ros2_ws/build/` | `.gitignore:8` | `ros2_ws/build/unitree_go/CMakeCache.txt:499` `CMAKE_CACHEFILE_DIR=/work/ros2_ws/build/unitree_go`、`:574` `CMAKE_HOME_DIRECTORY=/work/ros2_ws/src/unitree_go` —— 是**容器 `/work` 里**构建的 | 目标机删掉重 build（`onboard_setup.sh --clean-before-ros --with-ros`） |
| `ros2_ws/install/` | `.gitignore:9` | `install/unitree_go/local/lib/python3.10/dist-packages/.../unitree_go_s__*.cpython-310-x86_64-linux-gnu.so`；`lib/*.so` 是 `ELF 64-bit LSB ... x86-64` | 同上；且 py3.10 的 install 与 3.13 解释器不匹配 |
| `ros2_ws/log/` | `.gitignore:10` | colcon 日志 | 不带 |
| 根 `build/` | **未 gitignore**（untracked） | VSCode CMake Tools 产物（`.vscode/settings.json:2` 指向 `ros2_ws/src/unitree_go`） | 不带（打包脚本排除） |
| `third_party/` | `.gitignore:13` | 外部 SDK（unitree_ros2 / unitree_sdk2_python / cyclonedds，体积大） | 不带；目标机按 `docs/ros2-setup.md` §3 或官方仓库获取 |
| `logs/`、`*.csv` | `.gitignore:16,22` | 真机/仿真录制（本机 588 KB） | 不带（要复盘就单独拷那几次的 `logs/`） |

### 2.6 MuJoCo 依赖：**真机路径也要**

这条最容易被忽略：

```
sim_env/robot.py:15                     import mujoco                       （模块级）
ros2_controller/run_controller.py:107   from sim_env.robot import ...         （模块级）
ros2_controller/run_controller.py:174   def joint_ranges(...)                 （函数体 :183 才 import mujoco，读 ctrlrange）
ros2_controller/run_controller.py:527   sim = Go2Sim(model_xml(cfg), ...)      （**任何 target 都执行**）
```

> 行号一定会随改动漂移；定位锚点：`grep -n 'sim = Go2Sim' ros2_controller/run_controller.py`。

即 `--target real_ros` 也会加载 `sim_env/models/go2/scene.xml`（注释写明"只借模型拿标称站姿/关节范围，不参与仿真"）。
所以目标机上：

* **必须**装 mujoco + numpy（`requirements.txt:2-3`），**必须**带 `sim_env/models/go2/`（38 MiB，已在 git 里）；
* **不需要** OpenGL/X11：`MjModel.from_xml_path` 不建渲染上下文，只有 `mujoco.viewer` 才要 GUI；
* 反例：`scripts/go2_real_check.py`（只读自检）不 import mujoco，所以在没装 mujoco 的机器上也能跑。

### 2.7 arm64 / Jetson 注意

| 项 | 在哪 | 影响 | 处理 |
|---|---|---|---|
| 官方 `crc_amd64.so` | `tests/test_go2_crc.py:87`（文件不在就 SKIP）、`signal_io/go2_crc.py:78-79,178`（只作为出处注释） | arm64 上这个 `.so` 不存在，该测试**自动 SKIP**；真机 CRC 用的是纯 python 实现（`signal_io/msg.py` 的 `crc32_core`），不受影响 | 无需处理 |
| mujoco / numpy 的 aarch64 wheel | — | PyPI 是否提供与目标机 python 版本匹配的 aarch64 wheel **取决于版本**；装不上就得源码编译（要 cmake/编译器，很慢） | `onboard_setup.sh` 第 3 步会真装一遍并打印版本；装不上按 pip 报错换版本，别硬来 |
| BLAS 线程数 | 已在 `ros2_controller/run_controller.py:93-95` 用 `os.environ.setdefault(...,"1")` 钉死单线程（1536-768 前向 12 ms → 0.21 ms，`docs/go2-sim-fidelity.md:257-258`） | 不钉死会在多核机器（Jetson 也是）上吃掉 50 Hz 的 20 ms 预算 | 已处理；**别**改回多线程 |
| 实时性 | `real_env/README.md:222-224`（`ros2 launch` 三节点下 p95 1–9 ms） | Jetson 降频/散热差时抖动更大 | 首测在吊起状态先看 `env_check` 的 CPU/核数项，抖动大再考虑绑核 |

### 2.8 体积与算力

| 目录 | 本机实测 | 结论 |
|---|---|---|
| `.venv/` | **196 MiB** | 不带 |
| `ros2_ws/`（build 16 M + install 6.4 M + log 0.5 M） | 23 MiB | 不带 build/install/log，只带 `src/`（28 个文件） |
| `sim_env/models/` | 38 MiB | **带**（真机也要建模） |
| `ros2_controller/policies/` | 16 MiB（28 个 npz） | **带**（含 `*.equiv.npz` 回归样本，测试要用） |
| `docs/assets/` | 4.2 MiB | 带（图片，可选） |
| 整树 | 272 MiB（`du -sb`） | 打包后 58.4 MiB → zst 24.0 MiB |

推理算力：策略是纯 numpy 前向（单次 ~3.1 MFLOPs，单线程 0.21–0.7 ms），
Jetson Orin Nano/NX 余量上千倍（`docs/go2-sim-fidelity.md:257-259`）；MuJoCo 只用来建模型，不步进物理。

---

## 3. 必须重建 vs 可以直接带走

| | 清单 | 为什么 |
|---|---|---|
| **必须重建** | `.venv/` | cp313 + x86_64 的 `.so`，且解释器是 `/usr/bin/python3` 的绝对符号链接（§2.4） |
| **必须重建** | `ros2_ws/build/`、`ros2_ws/install/`、`ros2_ws/log/` | CMakeCache 写死 `/work` 绝对路径；`.so` 是 x86-64 + py3.10（§2.5） |
| **必须重建** | 根 `build/` | VSCode CMake 缓存 |
| **必须重建** | `third_party/`（要用 dds 后端/官方 SDK 时） | 体积大、与平台绑定 |
| **直接带走** | 全部 `.py`（80 个）、`config/*.json`、`requirements.txt` | 纯文本；路径由 `BASH_SOURCE` / `__file__` 推导，无绝对路径 |
| **直接带走** | `sim_env/models/go2/`（MJCF + 16 个 .obj + 地形 png） | MuJoCo 模型跨平台，真机路径也要 |
| **直接带走** | `ros2_controller/policies/*.npz` + `*.json` | numpy 数组，跨架构/跨 numpy 1.x–2.x 可读 |
| **直接带走** | `ros2_ws/src/unitree_go/`（24 个 .msg + package.xml + CMakeLists.txt） | 目标机重新 colcon build 的输入 |
| **直接带走** | `docs/`、`scripts/`、`tests/`、`docker/` | 纯文本 / 脚本 |
| **看情况** | `.git/`（27 MiB） | 打包默认排除；要保留历史用 `--with-git` |
| **看情况** | `logs/` | 只在复盘时拷那几次的目录 |

---

## 4. 搬到机载电脑 / 实验室机器：6 步

> 前 2 步在**本机**做，后 4 步在**目标机**做。每步都有"期望 / 失败怎么办"。
> 全程**不需要 sudo**（除了你自己决定装 ROS 2 时）。

### 第 1 步（本机）打包 + 离线自检

```bash
bash scripts/package_for_onboard.sh --verify
```

期望（本机实测输出）：

```
文件数        : 319
仓库整树      : 272.3 MiB（含 .venv/build/ros2_ws 产物）
未压缩 tar    : 58.4 MiB
压缩包        : 24.0 MiB  (/tmp/go2-deploy-20261104-154653.tar.zst)
[verify] 结果: 7 项通过, 0 项失败
```

> 数字会随仓库内容变化，**关键看最后一行：7 项自检全 PASS**。

失败怎么办：

* `py_compile` FAIL → 仓库里有语法错（或同事正在改文件，等改完重打）；
* `策略 npz` FAIL → `config/go2_sim.json` 里的默认策略档 npz 不在包里；
* `没有混入 .venv...` FAIL → 排除规则被破坏，**别带上机**。

### 第 2 步（本机 → 目标机）拷贝 + 校验

```bash
scp /tmp/go2-deploy-*.tar.zst* wyx@<目标机>:/home/wyx/
# 目标机上：
cd /home/wyx && sha256sum -c go2-deploy-*.tar.zst.sha256   # 必须 OK
mkdir -p ~/go2 && tar -I zstd -xf go2-deploy-*.tar.zst -C ~/go2
cd ~/go2/mjx-go2-mujoco2ros2 && cat MANIFEST.txt
```

期望：`sha256sum -c` 输出 `OK`；解包后**只有** `mjx-go2-mujoco2ros2/`（含 `MANIFEST.txt`），**没有** `.venv` / `build` / `ros2_ws/install`。

失败怎么办：`sha256` 不对 = 传输坏了，重拷（别 --force）。若解包后出现 `.venv/`，说明包打错了。

### 第 3 步（目标机）python 侧装机

```bash
bash scripts/onboard_setup.sh --dry-run     # 先看它要做什么
bash scripts/onboard_setup.sh               # 真装：venv + requirements + 自检
```

期望：`依赖 OK：numpy x / mujoco y`，且 python 版本行与目标机一致（22.04 → 3.10）。

失败怎么办：

* python < 3.10（20.04 的 3.8）→ 装 3.10 后用 `--python /usr/bin/python3.10`（脚本会告诉你，不会自己 sudo）；
* `mujoco` 装不上（aarch64 wheel 缺失）→ 看 pip 报错，换版本或源码编译；
* `.venv` 版本不匹配 → `rm -rf .venv` 后重跑（脚本只提示、不删）。

### 第 4 步（目标机）ROS 2 + `unitree_go` 消息包

```bash
# 若还没装 ROS 2（22.04 + Humble 为例；这条要 sudo，由你决定）：
#   sudo apt install ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
#                    ros-humble-rosidl-generator-dds-idl ros-dev-tools
bash scripts/onboard_setup.sh --with-ros     # source /opt/ros/<distro> 再 colcon build
# 跨机器拷来旧 install 时：
bash scripts/onboard_setup.sh --clean-before-ros --with-ros
```

期望：`colcon build 完成：.../ros2_ws/install`，并且第 6 步自检里 `unitree_go` 由 FAIL 变 OK。

失败怎么办：

* `rosidl_generator_dds_idl` 步失败 → 缺 `ros-<distro>-rosidl-generator-dds-idl`（`docs/ros2-setup.md:30`）；
* 没 `colcon` → `sudo apt install python3-colcon-common-extensions`；
* `ros2_ws/install` 是别的 python 版本建的 → `--clean-before-ros`。

### 第 5 步（目标机）网卡 + DDS

```bash
ip -br addr                                  # 找出连机器人的有线口（例 enp3s0 / eth0）
export GO2_IFACE=enp3s0                      # ← 只有这一步是现场必做
ping -c 3 192.168.123.161                    # 通 = 物理链路 OK
```

期望：`ping` 通；`env_check` 里"目标网卡"项 OK（本机实测是 FAIL，因为 config 写的是 `eth0`，
而本机只有 `enp0s31f6`(DOWN)/`wlp0s20f3`）。

失败怎么办：网卡名错 → 换 `GO2_IFACE`；`ping` 不通 → 先修网线/静态 IP（`real_env/README.md:51-57`），
**不要**在这一步之前启动控制器。

### 第 6 步（目标机）验收

```bash
.venv/bin/python scripts/env_check.py --target real_ros     # 期望退出码 0（允许 WARN）
bash scripts/onboard_setup.sh --with-ros                    # 也可，它把上面这步当收尾
bash scripts/real_run.sh --stage preflight                  # 只读阶段（不发 LowCmd），到实验室第一步
```

期望：`env_check` 退出码 0，没有 FAIL 项。**仿真侧**另有一条独立验收（不需要 ROS）：

```bash
.venv/bin/python tests/run_tests.py obs safety policy
```

---

## 5. `real_run.sh` / `env_check.py` 在目标机能不能继续用

> 两者都已在仓库里（T3 交付）：`scripts/env_check.py`、`scripts/real_run.sh`（另有 `docs/real-bringup.md` 现场 runbook）。
> 本节行号同样是**快照**（这两个文件还在演进），锚点：查 `--stage` 那一行。
> 若哪天它们被删掉，等价入口是 `scripts/run_with_ros2.sh` + `ros2_controller/run_controller.py`，
> 下列条件除"自动探测网卡"一条外完全相同。

### 5.1 `scripts/env_check.py`：**可以继续用**，逐条条件

| 条件 | 为什么 | 本机实测 |
|---|---|---|
| 用 `.venv/bin/python` 跑它（不是系统 python） | 它要 import numpy/mujoco 检查版本与模型 | ✅ 本机可跑（`--no-network`，退出码 1 因为没 ROS） |
| 目标机装好 ROS 2 + `rmw_cyclonedds_cpp` + `unitree_go` | 这半边检查项会 FAIL | ❌ 本机无 `/opt/ros`，6 项 FAIL |
| 传 `--iface <网卡>` 或先 `export GO2_IFACE=<网卡>` | 否则读 config 的 `eth0`，目标机同样不存在 | ❌ 本机实测 `目标网卡 eth0 ... 不存在（本机网卡: lo, enp0s31f6, ...）` |
| 退出码当闸门：0=可上机，1=先修 | `scripts/real_run.sh` 的 preflight 就按这个约定 | ✅ 本机 `--with-ros` 时 `onboard_setup.sh` 会以退出码 1 结束（已对齐） |
| `--no-network` 时不 ping 机器人 | 没上电/没插网线也要能跑完 | ✅ |

**唯一不成立的情况**：目标机的 python < 3.10，或 `.venv` 没重建 —— 那时脚本本身 import 就失败。

### 5.2 `scripts/real_run.sh`：**可以继续用**，逐条条件

| 条件 | 说明 |
|---|---|
| 目标机有 `/opt/ros/<distro>`，且**该 ROS 的解释器版本 = venv 的 python 版本** | `run_with_ros2.sh` 是把 ROS 的 `site-packages` 塞进 venv 的 `sys.path` 用的；ABI 不一致 → `rclpy` 的 `.so` import 失败。22.04 + Humble = py3.10，所以目标机 `.venv` 也必须 3.10 |
| `ros2_ws/install`（或官方 `unitree_ws/install`）已 source | 否则 `unitree_go` / `std_msgs` 导入失败。`real_run.sh` **自己不 source `/opt/ros`**，一律复用 `scripts/run_with_ros2.sh`（`scripts/real_run.sh:29`），后者依次尝试 `~/unitree_ws`、`$UNITREE_ROS2_SRC`、本仓 `ros2_ws/install`（`scripts/run_with_ros2.sh:60-67`） |
| 网卡能探测到，或显式 `--iface` | T3 的实现会探测 `192.168.123.0/24` 的 UP 网卡并 `export GO2_IFACE`；探测不到就必须手写 |
| `env_check` 通过（或显式 `--force`） | 安全闸门：ROS 侧没齐时**不允许**起控制器 |
| 脚本有可执行位 | `chmod +x scripts/*.sh`（`env_check` 会检查这一项） |
| 别加需要 GUI 的参数 | `--viewer` 之类在无头机载电脑上会失败；观察器本来也只在仿真侧用 |
| 阶段化调用：先 `--stage preflight` | `scripts/real_run.sh:149-167` 提供 all/preflight/zero/stand/policy；首测只跑 preflight（只读、不发 LowCmd） |
| `--dry-run` 可以在**没有 ROS 2 的机器**上先看它要做什么 | `scripts/real_run.sh:13`；适合上机前先在家里把命令过一遍 |

### 5.3 两者都**不能**照搬的部分

* `.venv` 和 `ros2_ws/install` 里的二进制（§2.4 / §2.5）；
* `scripts/view_go2_policy.sh` / `demo_ros2_policy.sh` 里硬编码的 `/opt/ros/lyrical`（目标机没这个发行版）；
* 任何假设"有 WSLg / DISPLAY"的路径（观察器）。

---

## 6. 已知风险 / 未验证项（上机前必读）

| 风险 | 状态 | 怎么处理 |
|---|---|---|
| mujoco/numpy 在目标机（arm64 或 py3.10）有无匹配 wheel | **未验证**（本机只有 cp313/x86_64 的） | `onboard_setup.sh` 第 3 步实装一遍就知道；装不上按 pip 报错换版本 |
| 目标机 Humble（CycloneDDS 0.10.x）与固件 0.10.2 的互通 | 官方口径覆盖，但**本仓未实测** | 先 `ros2 topic list` 能看到 `/lowstate` 再进控制 |
| 机载电脑（Go2 内置）能否开发 | 官方明确"不支持在 Go2 内置电脑上开发"（`docs/unitree-official-sdk-conformance.md:35`） | 首选实验室 PC 跑，机器人只当被控端；非要机载，先按本文 6 步验一遍 |
| `scripts/demo_ros2_policy.sh:18`、`scripts/view_go2_policy.sh:75` 硬编码 lyrical | **未改**（不在本任务 write scope） | 上机别用这两个；要修就改成 `ROS_SETUP` / `ROS_DISTRO` 探测（同 `scripts/run_with_ros2.sh:39-49`） |
| `scripts/view_go2_policy_local.sh:76` 硬编码 `/home/tc63/.venv/bin/python` | **未改** | 同上：不要用；建议改成 `$GO2_PYTHON` 或 `$ROOT/.venv/bin/python` |
| `scripts/fetch_go2_model.sh:4` 硬编码 `DST` | **未改** | 目标机不需要下载模型 |
| `requirements.txt:1` 注释写"仓库在 3.14 上验证"，本机实测 `.venv` 是 **3.13.7**；`docs/ros2-setup.md:9` 写 26.04，实测 **25.10** | 文档与实测不一致（不影响运行） | 以 `onboard_setup.sh` / `env_check.py` 打印的实测值为准；建议 Lead 顺手订正这两处 |
| 打包脚本 `--verify` 只验"本机解释器 + 离线内容" | 设计如此 | 目标机依赖靠 `onboard_setup.sh`；真机链路靠 `env_check.py` + `go2_real_check.py` |

---

## 7. 两个新脚本的用法（备忘）

### `scripts/package_for_onboard.sh`（本机打包）

| 参数 | 作用 |
|---|---|
| （无） | 打 `/tmp/go2-deploy-<时间>.tar.zst` + `.sha256`，包内带 `MANIFEST.txt` |
| `--verify` | 打完解到临时目录做 7 项离线自检（MANIFEST / 无混入产物 / py_compile / bash -n / import numpy+mujoco / npz 可加载 / 模型+消息源码齐） |
| `--verify-only FILE` | 只验已有的包（转发给别人前再验一次） |
| `--out FILE` / `--format zst\\|gz\\|none` | 指定输出与压缩方式（默认自动挑 zstd，没有就 gzip） |
| `--dest user@host:/path` | 改用 rsync 直接推源码（**不做 `--delete`**，只增不删），并推一份 `MANIFEST.txt` |
| `--with-git` | 连 `.git` 一起带（默认排除） |
| `--with-third-party` | 连 `third_party/` 一起带（默认排除） |
| `--dry-run` | 只统计"会打进去什么"（文件数/体积），不写压缩包 |
| `--python PATH` | 自检用哪个解释器（默认仓库 `.venv/bin/python`） |

实现要点：先打**未压缩 tar**（这样才能把 `MANIFEST.txt` 追加进包里），再压缩；
`MANIFEST.txt` 里的"文件数"不含它自己，压缩包里比它多 1 个文件。

### `scripts/onboard_setup.sh`（目标机装机）

| 参数 | 作用 |
|---|---|
| （无） | python 检查 → 建/校验 venv → `pip install -r requirements.txt` → `env_check` |
| `--with-ros` | 额外：探测 `/opt/ros/<distro>` → source → `colcon build --packages-select unitree_go` → 自检（自检 FAIL 时退出码 1） |
| `--dry-run` | 只打印每一步（任何机器都能跑，不改任何东西） |
| `--python PATH` | 用哪个解释器建 venv |
| `--ros-distro NAME` | 显式指定发行版（否则按 `ROS_DISTRO` → humble/jazzy/lyrical/iron/foxy 顺序探测） |
| `--skip-pip` | 跳过 pip（离线装机、依赖已就绪） |
| `--clean-before-ros` | 先删 `ros2_ws/{build,install,log}` 再 build（跨机器拷来的旧产物必用） |
| `--force-colcon` | 即使 install 比源码新也强制重建 |
| `--allow-old-python` | python < 3.10 时强行继续（只用于探路；numpy/mujoco 大概率装不上） |

幂等性：venv 版本一致就跳过创建；pip 装过会说 `already satisfied`；install 比源码新就跳过 colcon。
**全程不 sudo、不自动删任何东西。**

---

## 8. 相关文件

| 文件 | 作用 |
|---|---|
| [scripts/package_for_onboard.sh](../scripts/package_for_onboard.sh) | 本机打包 + 离线自检（§4 第 1 步） |
| [scripts/onboard_setup.sh](../scripts/onboard_setup.sh) | 目标机装机（§4 第 3/4 步） |
| [scripts/env_check.py](../scripts/env_check.py) | 目标机环境体检（§5.1） |
| `scripts/real_run.sh` | 上机启动器（§5.2，T3 交付） |
| [scripts/run_with_ros2.sh](../scripts/run_with_ros2.sh) | 把 ROS 2 + venv python 拼起来的通用 wrapper（§2.3） |
| [signal_io/config.py](../signal_io/config.py) | `GO2_IFACE` 覆盖（:308）、`apply_io_env()`、`detect_iface()`（:548） |
| [config/go2_sim.json](../config/go2_sim.json) | `io_profiles.real_ros.interface`（:88）—— **别改**，用环境变量 |
| [docs/ros2-setup.md](ros2-setup.md) | 本机（Lyrical）装法；目标机把发行版名换成 humble 即可 |
| [real_env/README.md](../real_env/README.md) | 真机通信栈 / 网段 / 安全流程 |
