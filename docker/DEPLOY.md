# 单文件交付：把「工具链 + 代码」全打包进一个 docker 镜像

> 回答两个问题：**能不能全打包？** 能。**运行时还必须用 docker 吗？** 看你在哪台机器上跑（见 §4）。

## 0. 一页结论

| 问题 | 答案 |
|---|---|
| 镜像里已经有什么？ | Ubuntu 22.04 + ROS 2 Humble + CycloneDDS **0.10.2**（= 固件同版本）+ 官方 `unitree_ros2` 消息包（已 colcon build）+ 官方 C++/Python SDK + `/opt/venv`（py3.10 + numpy + mujoco + matplotlib） |
| 那"必须重建 .venv / ros2_ws/install"是什么？ | **只针对原生安装路径**（机载电脑 / 实验室裸机）。docker 路径里这两样**根本不用宿主提供** —— 镜像里已经重建好了 |
| 能不能连代码一起打包？ | 能：`docker/Dockerfile.deploy` 把仓库 `COPY` 进 `/workspace`，得到 2.8 GB 的单镜像；`docker save` 成 tar / `zstd` 压一下就是单文件交付物 |
| 目标机要联网/装东西吗？ | 不要。只要目标机有 docker，`docker load` 后直接 `docker run`（不需要 pip / colcon / apt / 网络） |
| 为什么现在默认是挂载而不是烘进去？ | 现场改代码要立刻生效 → bind-mount（`docker compose`）；交付/离线搬运要单文件 → 烘进去。两种都在，按需选 |

## 1. 两种跑法（别混着理解）

| | A. bind-mount（默认，`docker-compose.yml`） | B. 自包含镜像（`Dockerfile.deploy`） |
|---|---|---|
| 代码从哪来 | 宿主目录挂到 `/workspace` | 构建时 `COPY` 进镜像 |
| 宿主 `.venv` / `ros2_ws/install` | 容器能看见但**不能用**（py3.13 vs py3.10）→ 必须 `GO2_SKIP_PROJECT_WS=1` | **不存在**（`.dockerignore` 已排除），不会被误用 |
| 改代码 | 保存即生效，不用重建 | 要重新 `--build`（COPY 层失效） |
| 交付/离线搬运 | 要同时带镜像 + 仓库 tar | **一个镜像 tar 就够** |
| 适合 | 今天在现场调参、看日志 | 拷到实验室机器 / 机载电脑 / 备用机 |

> 关键点：A 方案里"必须重建 .venv"不是缺陷，而是"宿主的 venv 不属于容器"；B 方案里这件事已经由镜像解决。

## 2. 构建（本机 22 秒，只在基础镜像之上加一层 COPY）

```bash
# 在仓库根目录（上下文必须是根，因为要 COPY 整个仓）
docker build -f docker/Dockerfile.deploy -t go2-deploy:day0 .

# 或者用脚本（带自检 / 保存）
bash docker/build_deploy_image.sh --build --verify
```

构建期就会跑一次 `import numpy, mujoco, rclpy, unitree_go.msg`，依赖不对当场失败。
`.dockerignore` 排除了 `.venv/ build/ ros2_ws/{build,install,log}/ logs/ __pycache__/ third_party/ .git/`，
所以镜像里只有源码 + 模型 + 策略 npz + config + docs（约 90 MB，压缩后更小）。

## 3. 离线搬运（目标机不需要网络）

```bash
# 源机：导出（2.8 GB 未压缩；zstd -19 通常 ~1.1 GB）
docker save -o /tmp/go2-deploy.tar go2-deploy:day0
zstd -T0 -19 /tmp/go2-deploy.tar -o /tmp/go2-deploy.tar.zst
sha256sum /tmp/go2-deploy.tar.zst > /tmp/go2-deploy.tar.zst.sha256

# 目标机：导入并直接跑（没有任何 -v 挂载）
sha256sum -c go2-deploy.tar.zst.sha256
zstd -d go2-deploy.tar.zst -c | docker load
docker run --rm -it --network=host \
    -e UNITREE_NIC=<接狗的有线网卡> -e ROS_DOMAIN_ID=0 go2-deploy:day0 \
    bash -lc "python3 scripts/env_check.py --target real_ros --iface $UNITREE_NIC"
```

> 真机 DDS 靠组播发现，**必须 `--network=host`**（bridge/NAT 下收不到 `/lowstate`，也发不出去 `/lowcmd`）。
> 网卡名在容器里看 `ip -br addr`（host 网络下通常与宿主一致，但要确认它是 192.168.123.x 那张）。

## 4. 运行时到底需不需要 docker？

| 运行环境 | 需要 docker 吗 | 说明 |
|---|---|---|
| **本机（Ubuntu 25.10，没装 ROS 2）** | **需要**（推荐） | 原生装 ROS 2 不在官方支持范围，且本机没有；镜像已构建好，容器内实测 controller 收帧 493 Hz、`health` 全 L0 |
| 实验室 PC（Ubuntu 22.04 + Humble 原生） | 不需要 | 先用 `bash scripts/onboard_setup.sh --with-ros` 建 venv + colcon build `unitree_go`，之后可直接 `bash scripts/real_run.sh` |
| Go2 机载电脑（Jetson，22.04 + Humble） | 不需要（更推荐原生） | 延迟最低；arm64 上没有现成镜像，要自己 `docker build`（基础镜像需 arm64 版） |
| 想"一份文件拷过去就能跑" | 需要 | 就是 §3 的自包含镜像 |

**docker 会不会影响 500 Hz 实时性？** 不会成为瓶颈但有前提：

* DDS 走 `--network=host`（无 NAT/端口映射开销）；
* 容器本身实测：controller 收帧 **493 Hz**、`pacer.lag_p95 = 0.94 ms`、`health` 2999/2999 全 L0；
* 但**别在同一个容器里同时开 perception / viewer**：实测会把 `state_age` 顶到 30 ms → 触发 L1 下蹲。真机默认 `--scan leg_fk`，不需要 perception。
* 需要更稳就 `docker run --cpuset-cpus=2,3 --cap-add=SYS_NICE` 再配合 `taskset`。

## 5. 与其它文档的关系

* 现场流程与命令：[`docs/sim2real-day0.md`](../docs/sim2real-day0.md)
* 原生安装 / 换机器的重建清单：[`docs/portability.md`](../docs/portability.md)
* 工具链镜像本体（不含代码）：[`docker/README.md`](README.md)
* 安全前提：[`docs/safety-audit.md`](../docs/safety-audit.md)
