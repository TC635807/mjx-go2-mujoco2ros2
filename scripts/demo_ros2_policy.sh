#!/usr/bin/env bash
# 端到端演示：MuJoCo 仿真侧 + 策略控制器，两个**独立进程**，通过 ROS 2 (/lowstate, /lowcmd) 通信。
#
#   ./scripts/demo_ros2_policy.sh [时长秒] [机型 go2] [档位 sim_clean|real_mild|real_severe]
#
# 例:
#   ./scripts/demo_ros2_policy.sh 15 go2
#   ./scripts/demo_ros2_policy.sh 15 go2 real_severe   # 注入丢帧/延迟，看健康层降级
# 注意：不要开 set -u —— ROS 的 setup.bash 里有未定义变量（AMENT_TRACE_SETUP_FILES）
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DUR="${1:-15}"
ROBOT="${2:-go2}"
PROFILE="${3:-sim_clean}"
SCAN="${4:-}"   # nominal | leg_fk（空 = 用 config 里的值）
mkdir -p "$ROOT/logs"

source /opt/ros/lyrical/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "=== [1/3] 单独起仿真侧（$ROBOT, profile=$PROFILE, ${DUR}s）==="
"$ROOT/.venv/bin/python" "$ROOT/sim_env/run_sim.py" --target sim_ros --robot "$ROBOT" \
    --profile "$PROFILE" --duration "$((DUR + 3))" --json "$ROOT/logs/demo_sim.json" &
SIM_PID=$!
sleep 3

echo "=== [2/3] 起策略控制器（同 domain，独立进程）==="
"$ROOT/.venv/bin/python" "$ROOT/ros2_controller/run_controller.py" --target sim_ros --robot "$ROBOT" \
    --controller policy ${SCAN:+--scan "$SCAN"} --duration "$DUR" --json "$ROOT/logs/demo_ctrl.json"
CTRL_RC=$?

echo "=== [3/3] 等仿真侧收尾 ==="
wait "$SIM_PID"
SIM_RC=$?
echo "controller rc=$CTRL_RC  sim rc=$SIM_RC"
exit $CTRL_RC
