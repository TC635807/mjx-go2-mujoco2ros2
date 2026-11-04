#!/usr/bin/env bash
# =============================================================================
# 宇树官方 ROS 2 Humble 环境变量（被 entrypoint / bash 自动 source）
#   - ROS 2 Humble
#   - 官方 unitree_ros2 消息包（unitree_go / unitree_api / unitree_hg）
#   - RMW = rmw_cyclonedds_cpp（与机器人固件 CycloneDDS 0.10.2 对齐）
#   - 可选：UNITREE_NIC 把 DDS 绑定到指定网卡（真机默认 eth0 或实际网卡名）
#   - 可选：ROS_DOMAIN_ID（真机 0 / 仿真 1）
# 注意：本仓 signal_io/config.py::apply_io_env() 会在 rclpy.init() 之前再按
#       --target 覆盖这几个变量；这里的值只影响手工 ros2 CLI / 官方例程。
# =============================================================================

# 不要被外部环境打乱
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "${UNITREE_ROS2_DIR:-/opt/unitree_ros2}/cyclonedds_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${UNITREE_ROS2_DIR:-/opt/unitree_ros2}/cyclonedds_ws/install/setup.bash"
fi

# 官方要求：必须用 CycloneDDS 才能和 Go2 固件的 DDS 互通
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 官方要求：domain 0 = 真机（固件固定）；本项目仿真用 1，避免串台
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

# 官方 setup.sh / setup_local.sh 同款：把 DDS 绑定到指定网卡。
# 真机：UNITREE_NIC=<有线网卡名，例如 eth1>；仿真/回环：lo（默认）。
UNITREE_NIC="${UNITREE_NIC:-lo}"
if [ -n "${UNITREE_NIC}" ]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_NIC}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
fi
export UNITREE_NIC

# 本仓 python（numpy/mujoco）放到 PATH 前面；ROS 的 python 依赖仍在系统里
if [ -d "${VENV:-/opt/venv}/bin" ]; then
  case ":${PATH}:" in
    *":${VENV:-/opt/venv}/bin:"*) ;;
    *) export PATH="${VENV:-/opt/venv}/bin:${PATH}" ;;
  esac
fi

# WARN: 不要把 /usr/lib/python3/dist-packages 塞进 PYTHONPATH。
#   本镜像的 venv 是 --system-site-packages，本来就能看到它（sys.path 末尾）；
#   而 PYTHONPATH 会排在 venv/系统 site-packages 之前，把 Ubuntu 自带的
#   numpy 1.21.5 顶到前面，盖掉 /usr/local 里的 numpy 2.x（本仓要求 >=1.26）。
#   项目 wrapper scripts/run_with_ros2.sh 里同样的追加用 GO2_SKIP_PY_DIST 跳过。
export GO2_SKIP_PY_DIST=1

# 本仓 python（numpy/mujoco + 能看到系统 rclpy/unitree_sdk2py）
export GO2_PYTHON="${GO2_PYTHON:-${VENV:-/opt/venv}/bin/python}"

export GO2_PROJECT="${GO2_PROJECT:-/workspace}"
export UNITREE_ROS2_SRC="${UNITREE_ROS2_DIR:-/opt/unitree_ros2}"
# 容器内用官方工作区提供 unitree_go，不 source 宿主 /workspace/ros2_ws/install
export GO2_SKIP_PROJECT_WS=1

# 真机时把“容器里看到的网卡名”透传给本仓 config（不改配置文件）：
# 只有显式指定了非 lo 的 UNITREE_NIC 才生效，避免把仿真也绑到有线网卡。
if [ -n "${UNITREE_NIC:-}" ] && [ "${UNITREE_NIC}" != "lo" ]; then
  export GO2_INTERFACE="${GO2_INTERFACE:-${UNITREE_NIC}}"
fi
