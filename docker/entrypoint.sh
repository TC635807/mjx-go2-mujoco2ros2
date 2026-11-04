#!/usr/bin/env bash
# 容器入口：source 官方 Humble 环境 -> 打印一次环境摘要 -> 执行命令
set -e

# shellcheck disable=SC1091
source /opt/setup_unitree_humble.sh

if [ "${GO2_BANNER:-1}" = "1" ]; then
  echo "=================================================================="
  echo " Unitree Go2 dev container  (official-recommended stack)"
  echo "   ROS_DISTRO      : ${ROS_DISTRO}"
  echo "   RMW             : ${RMW_IMPLEMENTATION}"
  echo "   ROS_DOMAIN_ID   : ${ROS_DOMAIN_ID}"
  echo "   UNITREE_NIC     : ${UNITREE_NIC:-<auto>}  (CYCLONEDDS_URI set: ${CYCLONEDDS_URI:+yes})"
  echo "   unitree_ros2    : ${UNITREE_ROS2_DIR:-/opt/unitree_ros2} (unitree_go/unitree_api/unitree_hg)"
  echo "   unitree_sdk2    : ${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics} (C++)"
  echo "   unitree_sdk2py  : $(python3 -c 'import unitree_sdk2py, cyclonedds; print("cyclonedds", cyclonedds.__version__ if hasattr(cyclonedds,"__version__") else "0.10.2")' 2>/dev/null || echo 'missing')"
  echo "   python (venv)   : $(python3 -c 'import sys,mujoco,numpy; print(sys.version.split()[0], "mujoco", mujoco.__version__, "numpy", numpy.__version__)' 2>/dev/null || echo missing)"
  echo "   project         : ${GO2_PROJECT:-/workspace}"
  echo "   网络接口        : $(ip -o -4 addr show 2>/dev/null | awk '{print $2"="$4}' | tr '\n' ' ')"
  echo "=================================================================="
fi

exec "$@"
