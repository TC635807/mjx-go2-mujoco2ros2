#!/usr/bin/env bash
# 用 ROS 2 环境跑本仓库的脚本（rclpy 在 /opt/ros 下，venv 里没有）
#
#   scripts/run_with_ros2.sh sim_env/run_sim.py --transport ros2 --duration 30 --viewer
#
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f /opt/ros/lyrical/setup.bash ]; then
  source /opt/ros/lyrical/setup.bash
else
  echo "[warn] 没找到 /opt/ros/lyrical/setup.bash，先装 ROS 2 Lyrical" >&2
fi
# venv 看不到 apt 装的 python3-*（rosidl 要用 empy/lark），补一下
export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
# unitree_ros2 的 unitree_go 消息（colcon 构建产物）
for f in "$HOME"/unitree_ws/install/setup.bash "$ROOT"/ros2_ws/install/setup.bash; do
  [ -f "$f" ] && source "$f"
done
exec "$ROOT/.venv/bin/python" "$@"
