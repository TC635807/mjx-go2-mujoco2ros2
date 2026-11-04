#!/usr/bin/env bash
# 用 ROS 2 环境跑本仓库的 python 脚本（rclpy 在 /opt/ros 下，venv 里没有）。
#
# 为什么要这层 wrapper（按 source 顺序）：
#   1) ROS 2 本体：默认按 ROS_DISTRO 找 /opt/ros/<distro>/setup.bash
#      （Ubuntu 26.04 装的是 Lyrical；宇树官方推荐的是 Ubuntu 22.04 + Humble），
#      也可用 ROS_SETUP=/opt/ros/<distro>/setup.bash 显式指定。提供 rclpy、
#      rosidl、launch、rmw。
#   2) /usr/lib/python3/dist-packages 补进 PYTHONPATH：
#      venv 只装 mujoco/numpy，看不到 apt 装的 python3-empy/lark/yaml；
#      rosidl 的 Python 生成器运行时要 import 它们。
#   3) unitree_go 消息包（colcon 构建产物）：
#      依次尝试 $HOME/unitree_ws/install（unitree_ros2 官方工作区）、
#      $UNITREE_ROS2_SRC/cyclonedds_ws/install（容器/官方推荐布局），
#      最后是本仓 ros2_ws/install（后 source 的覆盖前面的）。
#      容器里用 GO2_SKIP_PROJECT_WS=1 跳过“宿主 python 3.14 构建的 install”，
#      避免和容器 python 3.10 的 unitree_go 串台。
#   4) 最后用 venv 的 python 执行目标脚本（GO2_PYTHON 可覆盖；容器里指向
#      /opt/venv/bin/python），保证 mujoco/numpy 版本一致。
#
# 可用环境变量：
#   ROS_SETUP=/opt/ros/humble/setup.bash   指定 ROS 2 环境脚本
#   ROS_DISTRO=humble                      不指定 ROS_SETUP 时按它找 /opt/ros/<distro>
#   UNITREE_ROS2_SRC=/opt/unitree_ros2     官方 unitree_ros2 工作区（源码根）
#   GO2_SKIP_PROJECT_WS=1                  不 source 本仓 ros2_ws/install
#   GO2_PYTHON=/opt/venv/bin/python        用哪个 python 执行
#
# 注意：ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI 不在这里设，
#       由各脚本 load_config + apply_io_env 在 rclpy.init 之前设置（按 --target 区分
#       仿真 domain 1 / 真机 domain 0），这样同一个 wrapper 能跑两种环境。
#
#   scripts/run_with_ros2.sh sim_env/run_sim.py --target sim_ros --duration 30 --viewer
#   scripts/run_with_ros2.sh ros2_controller/run_controller.py --target real_ros --duration 10
#
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 1) ROS 2 基础环境（rclpy / rosidl / launch / rmw_cyclonedds_cpp）
if [ -n "${ROS_SETUP:-}" ] && [ -f "${ROS_SETUP}" ]; then
  source "${ROS_SETUP}"
elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/lyrical/setup.bash ]; then
  source /opt/ros/lyrical/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "[warn] 没找到 ROS 2 的 setup.bash（用 ROS_SETUP=/opt/ros/<distro>/setup.bash 指定）" >&2
fi

# 2) venv 看不到 apt 装的 python3-*（rosidl 要用 empy/lark），补一下。
#    容器里的 venv 是 --system-site-packages，本来就能看到它，而且 PYTHONPATH 会
#    把 Ubuntu 自带的 numpy 1.21.5 顶到 numpy 2.x 前面（本仓要求 numpy>=1.26），
#    所以容器里用 GO2_SKIP_PY_DIST=1 跳过这条。
if [ -z "${GO2_SKIP_PY_DIST:-}" ]; then
  export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
fi

# 3) unitree_go 消息（colcon 构建产物；后 source 的优先）
for f in "$HOME"/unitree_ws/install/setup.bash \
         "${UNITREE_ROS2_SRC:-/nonexistent}"/cyclonedds_ws/install/setup.bash; do
  [ -f "$f" ] && source "$f"
done
# 本仓 ros2_ws/install 是“当前机器 python 版本”构建的；容器里必须跳过
if [ -z "${GO2_SKIP_PROJECT_WS:-}" ] && [ -f "$ROOT"/ros2_ws/install/setup.bash ]; then
  source "$ROOT"/ros2_ws/install/setup.bash
fi

# 4) 统一用 venv python（mujoco/numpy 在 venv 里）
exec "${GO2_PYTHON:-$ROOT/.venv/bin/python}" "$@"
