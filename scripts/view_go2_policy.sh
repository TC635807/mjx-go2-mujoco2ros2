#!/usr/bin/env bash
# Go2 观测入口：**自动拉起 ROS 2 节点图**，再用观察器节点订阅真实话题。
#
#   启动:  sim_sensors（MuJoCo） + perception + controller
#   观察:  go2_viewer（只订阅 /lowstate、/lowcmd、/go2/odom、
#          /perception/height_scan、/go2/controller/{obs,action,info}）
#
# 用法:
#   bash scripts/view_go2_policy.sh                          # 平地，一直跑到关窗口/Ctrl-C
#   bash scripts/view_go2_policy.sh --terrain wide_h050
#   bash scripts/view_go2_policy.sh --duration 20
#   bash scripts/view_go2_policy.sh --headless --duration 10
#   bash scripts/view_go2_policy.sh --record logs/go2_view
#   PROFILE=go2_v29c bash scripts/view_go2_policy.sh
#   bash scripts/view_go2_policy.sh --local                  # 旧的单进程观察器
#
# 注意：本脚本会自动 source ROS 2 与 ros2_ws/install。没有 ROS 2 时用 --local。
set -eo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

DUR=0
TERRAIN=flat
PROFILE="${PROFILE:-go2_v33d}"
CMD_STR=""
HEADLESS=0
REC=""
HAS_MODEL=0
VIEW_ARGS=()

usage() {
  sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local|--legacy)
      exec bash "$ROOT/scripts/view_go2_policy_local.sh" "${@:2}"
      ;;
    --duration) DUR="$2"; shift 2 ;;
    --terrain) TERRAIN="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --cmd) CMD_STR="$2 $3 $4"; shift 4 ;;
    --headless) HEADLESS=1; shift ;;
    --record) REC="$2"; shift 2 ;;
    --follow) VIEW_ARGS+=(--follow); shift ;;
    --trail) VIEW_ARGS+=(--trail "$2"); shift 2 ;;
    --sync-hz) VIEW_ARGS+=(--sync-hz "$2"); shift 2 ;;
    --plot-hz) VIEW_ARGS+=(--plot-hz "$2"); shift 2 ;;
    --plots) VIEW_ARGS+=(--plots "$2"); shift 2 ;;
    --model) VIEW_ARGS+=(--model "$2"); HAS_MODEL=1; shift 2 ;;
    --scene) VIEW_ARGS+=(--scene "$2"); shift 2 ;;
    --robot-style) VIEW_ARGS+=(--robot-style "$2"); shift 2 ;;
    --lite) VIEW_ARGS+=(--lite "$2"); shift 2 ;;
    --list-terrains)
      "$ROOT/.venv/bin/python" - <<'PY'
import json
p = "sim_env/models/go2/terrains.json"
d = json.load(open(p, encoding="utf-8"))
for e in d.get("terrains", []):
    print("%-14s %s" % (e.get("name", "?"), e.get("desc", "")))
PY
      exit 0
      ;;
    -h|--help) usage ;;
    *)
      echo "[view] 未知参数: $1" >&2
      echo "[view] 旧版单进程参数请加 --local，例如: bash scripts/view_go2_policy.sh --local --obstacles" >&2
      exit 2
      ;;
  esac
done

source /opt/ros/lyrical/setup.bash
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
fi
export PYTHONPATH="${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1

mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/view_ros2_launch.log"
echo "[view] 启动 ROS 2 节点图: sim_sensors + perception + controller"
LAUNCH_ARGS=(duration:="$DUR" terrain:="$TERRAIN" profile:=sim_clean
             viewer:=false policy_profile:="$PROFILE")
if [[ -n "$CMD_STR" ]]; then
  LAUNCH_ARGS+=(cmd:="$CMD_STR")
fi
ros2 launch ros2_ws/launch/sim.launch.py "${LAUNCH_ARGS[@]}" >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# 等节点图把话题挂起来（最多 10 s）
for _ in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -qx "/lowstate"; then
    break
  fi
  sleep 0.2
done
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "[view] ROS 2 launch 提前退出，日志尾部:" >&2
  tail -30 "$LOG" >&2 || true
  exit 1
fi

# 地形模式：观察器要加载同一份地形 scene，否则 3D 里的地面/障碍物对不上
if [[ "$TERRAIN" != "flat" && "$HAS_MODEL" == "0" ]]; then
  TERRAIN_SCENE=$("$ROOT/.venv/bin/python" - "$TERRAIN" <<'PY'
import json, os, sys
root = os.getcwd()
name = sys.argv[1]
d = json.load(open("sim_env/models/go2/terrains.json", encoding="utf-8"))
for e in d.get("terrains", []):
    if e.get("name") == name:
        p = e["scene"]
        print(p if os.path.isabs(p) else os.path.join(root, p))
        break
else:
    sys.exit(1)
PY
) || { echo "[view] 找不到地形场景: $TERRAIN" >&2; exit 1; }
  VIEW_ARGS+=(--model "$TERRAIN_SCENE")
fi

VIEW_CMD=("$ROOT/.venv/bin/python" "$ROOT/scripts/view_ros2.py"
          --target sim_ros --truth --duration "$DUR" "${VIEW_ARGS[@]}")
if [[ "$HEADLESS" == "1" ]]; then
  VIEW_CMD+=(--headless)
fi
if [[ -n "$REC" ]]; then
  VIEW_CMD+=(--record "$REC")
fi

echo "[view] 观察器节点 -> /lowstate + /perception/height_scan + /go2/controller/*"
echo "[view] launch 日志: $LOG"
"${VIEW_CMD[@]}"
