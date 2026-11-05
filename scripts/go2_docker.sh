#!/usr/bin/env bash
# =============================================================================
# scripts/go2_docker.sh —— 在「宇树官方栈」容器里跑一条命令
# =============================================================================
# 为什么需要它：本机（Ubuntu 25.10）没装 ROS 2，rclpy / unitree_go 全在镜像里。
# 所以 scripts/*.py 里凡是 import rclpy 的（真机自检、探针、控制器），
# 在宿主直接 python3 跑必然 ModuleNotFoundError。用这个包装器一行搞定：
#
#   bash scripts/go2_docker.sh python3 scripts/probe_remote.py --iface enp0s31f6 --duration 30
#   bash scripts/go2_docker.sh python3 scripts/real_sensor_check.py --target real_ros --light
#   bash scripts/go2_docker.sh python3 ros2_controller/run_controller.py --target real_ros \
#        --controller policy --cmd 0 0 0 --duration 30
#
# 想进交互式容器自己敲命令（推荐现场用）：
#
#   bash scripts/go2_docker.sh bash
#   # 进去以后：python3 scripts/probe_remote.py --iface enp0s31f6 --duration 30
#
# 环境变量：
#   UNITREE_NIC  接狗的有线网卡（默认 enp0s31f6）
#   GO2_IFACE    同上（覆盖 config 里的 eth0），默认跟随 UNITREE_NIC
#   GO2_IMAGE    镜像名（默认 unitree-go2-ros2:humble；自包含版用 go2-deploy:day0）
#   GO2_DOMAIN   DDS domain（真机固定 0）
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NIC="${UNITREE_NIC:-enp0s31f6}"
IFACE="${GO2_IFACE:-$NIC}"
IMAGE="${GO2_IMAGE:-unitree-go2-ros2:humble}"
DOMAIN="${GO2_DOMAIN:-0}"

if [ "$#" -eq 0 ]; then
  echo "用法: bash scripts/go2_docker.sh <命令> [参数...]" >&2
  echo "  例: bash scripts/go2_docker.sh bash                # 进交互式容器" >&2
  echo "  例: bash scripts/go2_docker.sh python3 -V" >&2
  exit 2
fi

# 已经在容器里？容器内没有 docker 命令，而 /workspace 下正好有个 docker/ 目录，
# 直接 exec docker 会报 "cannot execute: Is a directory"。这里给个人话提示。
if ! command -v docker >/dev/null 2>&1; then
  echo "[go2_docker] 没有 docker 命令 —— 你可能已经在容器里了。" >&2
  echo "[go2_docker] 容器内直接跑，不要再套这层：" >&2
  echo "    $*" >&2
  exit 2
fi

# 只在真有 tty 时加 -t，否则管道/重定向场景会报 "the input device is not a TTY"
TTY_FLAG=""
if [ -t 0 ] && [ -t 1 ]; then TTY_FLAG="-t"; fi

exec docker run --rm -i $TTY_FLAG \
  --network=host \
  -e UNITREE_NIC="$NIC" \
  -e GO2_IFACE="$IFACE" \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -v "$ROOT:/workspace" \
  -w /workspace \
  "$IMAGE" \
  "$@"
