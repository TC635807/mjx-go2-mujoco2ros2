#!/usr/bin/env bash
# 把「工具链 + 本仓代码」打成单个自包含镜像，并支持离线搬运。
#
#   bash docker/build_deploy_image.sh --build                 # 构建 go2-deploy:day0（22 s）
#   bash docker/build_deploy_image.sh --build --save         # 再 docker save 成 tar（离线搬运用）
#   bash docker/build_deploy_image.sh --load /path/img.tar.zst   # 目标机：load 回来
#   bash docker/build_deploy_image.sh --verify                # 在镜像里跑 env_check（无挂载）
#
# 为什么需要它：docker/Dockerfile 只造工具链，代码靠 bind-mount；
# 这个脚本把仓库 COPY 进镜像（docker/Dockerfile.deploy），得到一个
# 不依赖宿主文件的交付物 —— 目标机只要 docker load 就能跑，
# 不需要网络 / pip / colcon / 重建 venv。
#
# 注意：代码改了要重新 --build（COPY 层失效）。现场改代码请用 bind-mount 的 compose 用法。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
IMG="${IMG:-go2-deploy:day0}"
BASE="${BASE_IMAGE:-unitree-go2-ros2:humble}"
OUT="${OUT:-/tmp/go2-deploy-image-$(date +%Y%m%d-%H%M).tar}"
DO_BUILD=0; DO_SAVE=0; DO_VERIFY=0; LOAD=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1; shift ;;
    --save) DO_SAVE=1; shift ;;
    --verify) DO_VERIFY=1; shift ;;
    --load) LOAD="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --img) IMG="$2"; shift 2 ;;
    -h|--help) sed -n "2,22p" "$0" | sed "s/^# \{0,1\}//"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

command -v docker >/dev/null || { echo "没有 docker" >&2; exit 1; }

if [[ "$DO_BUILD" == 1 ]]; then
  echo "[img] 构建 $IMG（FROM $BASE + COPY 仓库）"
  docker build -f docker/Dockerfile.deploy --build-arg "BASE_IMAGE=$BASE" -t "$IMG" .
  docker images | grep -E "^$IMG " || true
fi

if [[ "$DO_VERIFY" == 1 ]]; then
  echo "[img] 镜像内自检（无挂载、无宿主文件）"
  docker run --rm --network=host -e GO2_BANNER=0 -e UNITREE_NIC=lo -e ROS_DOMAIN_ID=1 \
      "$IMG" bash -lc "python3 scripts/env_check.py --target sim_ros --iface lo | tail -6; echo ---; python3 -c \"import numpy,mujoco,rclpy,unitree_go.msg as m;print(chr(79)+chr(75), numpy.__version__, mujoco.__version__)\""
fi

if [[ "$DO_SAVE" == 1 ]]; then
  echo "[img] docker save -> $OUT"
  docker save -o "$OUT" "$IMG"
  ls -lh "$OUT"
  case "$OUT" in
    *.zst) zstd -T0 -19 -f "$OUT" >/dev/null 2>&1 || true ;;
  esac
  echo "[img] 目标机执行： docker load -i $OUT"
fi

if [[ -n "$LOAD" ]]; then
  echo "[img] docker load <- $LOAD"
  docker load -i "$LOAD"
  docker images | grep -E "go2-deploy|unitree-go2" || true
fi