#!/usr/bin/env bash
# =============================================================================
# 在“不含 ROS 2 环境”的进程里跑官方 unitree_sdk2 C++ 例程。
#
# 为什么需要它（官方 ROS2_service 文档原文）：
#   “Unitree SDK2 与 ROS2 环境存在二进制冲突：Unitree SDK2 需要占用 CycloneDDS，
#    无法在初始化了 ROS/ROS2 环境中启动。”
# 所以官方 C++ 例程必须用 SDK2 自带的 libddsc，且同一进程里不能有 rclpy/rmw 的
# DDS 参与者在跑。容器里 BASH_ENV 会自动 source ROS 环境，本脚本负责把它清掉。
#
# 用法（容器内）：
#   bash docker/scripts/run_official_sdk2_example.sh go2_stand_example <网卡名>
#   bash docker/scripts/run_official_sdk2_example.sh --list
# =============================================================================
set -u

BIN_DIR="${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/bin"

if [ "${1:-}" = "--list" ]; then
  ls "$BIN_DIR"
  exit 0
fi

if [ "$#" -lt 1 ]; then
  echo "用法: $0 <例程名> [参数...]   （--list 查看全部例程）" >&2
  exit 2
fi

# 清掉 ROS 2 / colcon 环境（BASH_ENV 会在脚本启动前先 source 一次，这里覆盖回来）
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY
unset RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_DOMAIN_ID
unset PYTHONPATH

# 官方 SDK2 自带的 CycloneDDS（libddsc / libddscxx）优先
export LD_LIBRARY_PATH="${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/lib:${LD_LIBRARY_PATH:-}"

echo "[sdk2-cxx] env: ROS_DISTRO=${ROS_DISTRO:-<unset>} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>} LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
exec "$BIN_DIR/$1" "${@:2}"
