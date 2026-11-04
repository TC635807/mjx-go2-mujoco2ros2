#!/usr/bin/env bash
# =============================================================================
# 在「控制 Go2 的 Linux 机器」上执行：装 Docker + 导入官方栈镜像 + 解出本仓。
# 适用于 Ubuntu/Debian x86_64（20.04/22.04/24.04/25.10/26.04 均可）。
# 官方栈（Ubuntu 22.04 + ROS 2 Humble + CycloneDDS 0.10.x）在容器里，
# 不要求宿主机是 22.04 —— 宿主机只负责提供 Docker 和网卡。
#
# 典型用法（tarball 与脚本放同一目录）：
#   sudo bash linux_host_setup.sh --image go2-humble-image.tar.gz \
#        --project mjx-go2-mujoco2ros2.tar.gz --nic enp3s0 --dog-ip 192.168.123.222
#
# 参数：
#   --image PATH     docker save 出来的镜像包（必填，.tar/.tar.gz/.tar.zst 均可）
#   --project PATH   本仓源码包（必填）
#   --dest DIR       解压目标（默认 /home/<你>/mjx-go2-mujoco2ros2）
#   --nic NAME       接 Go2 的网卡名（可选；给了才配静态 IP）
#   --dog-ip IP      该网卡静态地址（默认 192.168.123.222，不能是 .161）
#   --skip-apt       跳过 apt（要求机器上已有 docker；离线场景用）
# =============================================================================
set -euo pipefail

IMAGE=""; PROJECT=""; DEST=""; NIC=""; DOG_IP="192.168.123.222"; SKIP_APT=0

usage() { sed -n '2,20p' "$0"; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --image)   IMAGE="$2"; shift 2 ;;
    --project) PROJECT="$2"; shift 2 ;;
    --dest)    DEST="$2"; shift 2 ;;
    --nic)     NIC="$2"; shift 2 ;;
    --dog-ip)  DOG_IP="$2"; shift 2 ;;
    --skip-apt) SKIP_APT=1; shift ;;
    -h|--help) usage ;;
    *) echo "未知参数: $1"; usage ;;
  esac
done
[ -n "$IMAGE" ] && [ -n "$PROJECT" ] || usage
[ "$(id -u)" -eq 0 ] || { echo "请用 sudo 运行"; exit 1; }

TARGET_USER="${SUDO_USER:-$USER}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
DEST="${DEST:-$TARGET_HOME/mjx-go2-mujoco2ros2}"

echo "== 0. 基本信息 =="
uname -m
. /etc/os-release && echo "$PRETTY_NAME"
[ "$(uname -m)" = "x86_64" ] || echo "!! 警告: 本镜像只支持 x86_64；当前 $(uname -m) 需要换 arm64 镜像"

echo "== 1. Docker =="
if ! command -v docker >/dev/null 2>&1; then
  if [ "$SKIP_APT" -eq 1 ]; then
    echo "!! 没有 docker 且指定了 --skip-apt，退出"; exit 1
  fi
  apt-get update
  apt-get install -y docker.io docker-compose-v2 || apt-get install -y docker.io docker-compose
fi
systemctl enable --now docker
docker version --format '{{.Server.Version}}' || true
usermod -aG docker "$TARGET_USER" || true

echo "== 2. 导入官方栈镜像 =="
case "$IMAGE" in
  *.gz)  pigz -dc "$IMAGE" | docker load ;;
  *.zst) zstd -dc "$IMAGE" | docker load ;;
  *)     docker load -i "$IMAGE" ;;
esac
docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}' | grep unitree || true

echo "== 3. 解出本仓源码 -> $DEST =="
if [ -d "$DEST" ] && [ -n "$(ls -A "$DEST" 2>/dev/null)" ]; then
  echo "目录已存在，跳过解压（如需覆盖请先手动删掉 $DEST）"
else
  mkdir -p "$DEST"
  tar -xzf "$PROJECT" -C "$DEST"
fi
chown -R "$TARGET_USER:$TARGET_USER" "$DEST"

echo "== 4. 狗侧网卡（可选） =="
if [ -n "$NIC" ]; then
  ip link show "$NIC" >/dev/null 2>&1 || { echo "!! 网卡 $NIC 不存在，跳过"; NIC=""; }
fi
if [ -n "$NIC" ]; then
  ip link set "$NIC" up
  ip addr replace "$DOG_IP/24" dev "$NIC"
  if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager; then
    nmcli con delete go2dog >/dev/null 2>&1 || true
    nmcli con add type ethernet ifname "$NIC" con-name go2dog \
      ipv4.method manual ipv4.addresses "$DOG_IP/24" \
      ipv4.never-default yes ipv6.method ignore >/dev/null || true
    nmcli con up go2dog >/dev/null 2>&1 || true
  fi
  echo "已给 $NIC 配置 $DOG_IP/24（不设默认路由）"
fi

echo
echo "== 完成。下一步 =="
echo "  cd $DEST/docker"
if [ -n "$NIC" ]; then
  echo "  ping -c1 192.168.123.161        # 通说明到狗链路 OK"
  echo "  UNITREE_NIC=$NIC ROS_DOMAIN_ID=0 docker compose run --rm ros2"
else
  echo "  ip -br addr                     # 先找到接狗那张网卡名"
  echo "  UNITREE_NIC=<网卡名> ROS_DOMAIN_ID=0 docker compose run --rm ros2"
fi
echo "  容器内预检："
echo "    ip -4 addr | grep 192.168.123"
echo "    ros2 topic list | grep -E 'lowstate|lowcmd'"
echo "    ros2 topic hz /lowstate       # 期望 ~500 Hz"
