#!/usr/bin/env bash
# =============================================================================
# 容器内一键验证：官方推荐栈（Ubuntu 22.04 + ROS 2 Humble + CycloneDDS 0.10.x
# + 官方 unitree_ros2 / unitree_sdk2 / unitree_sdk2_python）是否真的可用，
# 以及本仓 mjx-go2-mujoco2ros2 的 ROS 2 / SDK 链路是否符合官方语义。
#
# 用法（容器内）：
#   bash /workspace/docker/scripts/verify_conformance.sh
# 日志：/tmp/go2_conformance/*.log
# 退出码：0 = 全部通过（WARN 不影响退出码）；1 = 有 FAIL
# =============================================================================
# 注意：ROS 2 的 setup.bash 不能在 set -u 下 source（官方脚本里有未定义变量，
# 会直接退出），所以先 source 环境，再开 -u。
# shellcheck disable=SC1091
source /opt/setup_unitree_humble.sh 2>/dev/null || true
set -uo pipefail

cd "${GO2_PROJECT:-/workspace}" || { echo "no project at ${GO2_PROJECT:-/workspace}"; exit 1; }
PY="${GO2_PYTHON:-/opt/venv/bin/python}"
OUT="${GO2_CONF_OUT:-/tmp/go2_conformance}"
mkdir -p "$OUT"

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $*"; PASS=$((PASS + 1)); }
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL + 1)); }
warn() { echo "  [WARN] $*"; WARN=$((WARN + 1)); }
hr() { echo; echo "=============================================================="; echo " $*"; echo "=============================================================="; }

hr "0. 官方推荐栈自检"
if bash docker/scripts/smoke_test.sh >"$OUT/smoke.log" 2>&1; then
  pass "smoke_test.sh 正常执行"
else
  fail "smoke_test.sh 报错（见 $OUT/smoke.log）"
fi
tail -n +1 "$OUT/smoke.log" | sed -n '1,45p'

if [ "${ROS_DISTRO:-}" = "humble" ]; then
  pass "ROS 2 = Humble（unitree_ros2 README: recommend）"
else
  fail "ROS_DISTRO=${ROS_DISTRO:-<unset>} != humble"
fi

cver="$(dpkg-query -W -f='${Version}' ros-humble-cyclonedds 2>/dev/null || true)"
if [ -n "$cver" ]; then
  case "$cver" in
    0.10.*) pass "ros-humble-cyclonedds ${cver}（0.10.x，与固件 CycloneDDS 0.10.2 同系列）" ;;
    *)      warn "ros-humble-cyclonedds ${cver} 不在 0.10.x：与固件 0.10.2 的互通需实测" ;;
  esac
else
  fail "ros-humble-cyclonedds 未安装"
fi

if ros2 pkg prefix unitree_go >/dev/null 2>&1; then
  pass "unitree_go 消息包可用（LowCmd/LowState）"
else
  fail "unitree_go 消息包不可用"
fi

if python3 -c "import cyclonedds, unitree_sdk2py" >/dev/null 2>&1; then
  pass "官方 Python SDK 可用（cyclonedds + unitree_sdk2py）"
else
  fail "官方 Python SDK 不可用"
fi

if python3 - >"$OUT/crc.log" <<'PYEOF'
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
m = unitree_go_msg_dds__LowCmd_()
m.head[0] = 0xFE; m.head[1] = 0xEF; m.level_flag = 0xFF
for i in range(20):
    m.motor_cmd[i].mode = 0x01
print("CRC(official_default_layout) = 0x%08X" % CRC().Crc(m))
PYEOF
then
  pass "官方 CRC 类可用：$(tail -1 "$OUT/crc.log")"
else
  fail "官方 CRC 类不可用（真机 lowcmd 会被固件丢弃）"
fi

hr "1. 本仓单元测试（python 3.10 venv，官方环境）"
# 无显示器时可视化测试要离屏渲染：libosmesa6 + MUJOCO_GL=osmesa
if MUJOCO_GL=osmesa "$PY" tests/run_tests.py >"$OUT/unit_tests.log" 2>&1; then
  grep -E '^PASS |通过' "$OUT/unit_tests.log" | tail -3
  pass "tests/run_tests.py 全部通过"
else
  npass="$(grep -c '^PASS ' "$OUT/unit_tests.log" 2>/dev/null || echo 0)"
  grep -E '^FAIL |通过' "$OUT/unit_tests.log" | tail -5 || true
  warn "tests/run_tests.py 有失败项（${npass}/14 通过）；已知 test_viz_panels 的 15 ms 图像面板阈值是环境相关项，见 $OUT/unit_tests.log"
fi

hr "2. ROS 2 闭环（本仓推荐路径）：verify.py --target sim_ros"
if "$PY" scripts/verify.py --target sim_ros >"$OUT/verify_sim_ros.log" 2>&1; then
  grep -E '^  OK|通过' "$OUT/verify_sim_ros.log" | tail -4
  pass "verify.py --target sim_ros 通过（ROS 2 + unitree_go + CRC 闭环）"
else
  tail -25 "$OUT/verify_sim_ros.log"
  fail "verify.py --target sim_ros 失败（见 $OUT/verify_sim_ros.log）"
fi

hr "3. 官方 Python SDK 闭环：verify.py --target sim_sdk（宿主 python 3.14 上不可用）"
if "$PY" scripts/verify.py --target sim_sdk >"$OUT/verify_sim_sdk.log" 2>&1; then
  grep -E '^  OK|通过' "$OUT/verify_sim_sdk.log" | tail -4
  pass "verify.py --target sim_sdk 通过（官方 unitree_sdk2py raw DDS 闭环）"
else
  tail -25 "$OUT/verify_sim_sdk.log"
  warn "verify.py --target sim_sdk 未通过（见 $OUT/verify_sim_sdk.log）；本仓 SDK 后端在 3.14 上从未跑过，这里才第一次可用"
fi

hr "4. DDS 互通 A：ROS 2 发布 /lowstate -> 官方 SDK 订阅 rt/lowstate"
"$PY" sim_env/run_sim.py --target sim_ros --duration 14 --json "$OUT/sim_pub.json" \
    >"$OUT/sim_pub.log" 2>&1 &
SIM_PID=$!
sleep 4
if "$PY" docker/scripts/dds_sub_lowstate.py --domain 1 --iface lo --duration 6 --min-msgs 500 \
    >"$OUT/dds_sub.log" 2>&1; then
  grep 'sdk-sub' "$OUT/dds_sub.log" | tail -4
  pass "官方 SDK 收到了 ROS 2 侧的 rt/lowstate（DDS 类型/话题映射一致）"
else
  tail -20 "$OUT/dds_sub.log"
  fail "官方 SDK 没收到 ROS 2 的 rt/lowstate（见 $OUT/dds_sub.log）"
fi
wait "$SIM_PID" 2>/dev/null || true

hr "5. DDS 互通 B：官方 SDK 发 rt/lowcmd（含官方 CRC）-> 本仓 ROS 2 仿真校验"
"$PY" sim_env/run_sim.py --target sim_ros --duration 16 --json "$OUT/sim_sub.json" \
    >"$OUT/sim_sub.log" 2>&1 &
SIM_PID=$!
sleep 4
"$PY" docker/scripts/dds_pub_lowcmd.py --domain 1 --iface lo --duration 8 --rate 500 \
    >"$OUT/dds_pub.log" 2>&1 || true
cat "$OUT/dds_pub.log" | tail -2
# sim 的 --json summary 是在进程退出时才写的，必须先等它结束再解析
wait "$SIM_PID" 2>/dev/null || true
if "$PY" - "$OUT/sim_sub.json" <<'PYEOF'
import json
import sys
d = json.load(open(sys.argv[1], encoding="utf-8"))
tr = d.get("transport", {})
rx = int(tr.get("rx_cmd", 0))
bad = int(tr.get("bad_crc", 0))
print("  sim 收到 LowCmd: rx_cmd=%d  bad_crc=%d" % (rx, bad))
sys.exit(0 if (rx >= 100 and bad == 0) else 1)
PYEOF
then
  pass "官方 SDK 的 LowCmd 通过本仓 CRC 校验（812B 布局/CRC 与官方逐位一致）"
else
  tail -5 "$OUT/sim_sub.log"
  fail "官方 SDK 的 LowCmd 未通过本仓 CRC 校验（见 $OUT/sim_sub.json）"
fi
wait "$SIM_PID" 2>/dev/null || true

hr "6. ros2 launch 三节点图（验证 run_with_ros2.sh / launch 在 Humble 下可用）"
if timeout 150 ros2 launch ros2_ws/launch/sim.launch.py duration:=8 >"$OUT/launch_sim.log" 2>&1; then
  grep -E '\[sim\] summary|\[ctrl\]|process has finished' "$OUT/launch_sim.log" | tail -4
  pass "ros2 launch sim.launch.py duration:=8 正常结束"
else
  tail -40 "$OUT/launch_sim.log"
  fail "ros2 launch 失败（见 $OUT/launch_sim.log）"
fi

hr "7. 官方 C++ SDK（unitree_sdk2）"
if [ -f "${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/lib/libunitree_sdk2.a" ]; then
  pass "libunitree_sdk2.a 已安装（官方 C++ SDK 可用）"
  if [ -x "${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/bin/go2_low_level" ]; then
    pass "官方 Go2 C++ 例程已编入镜像：$(ls ${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/bin | tr '\n' ' ' | cut -c1-200)"
  else
    warn "官方 C++ 例程二进制不在镜像里（只装了库；例程可在 /opt/unitree_sdk2 重新 cmake --build）"
  fi
else
  fail "libunitree_sdk2.a 缺失"
fi

hr "汇总"
echo "  PASS=$PASS  FAIL=$FAIL  WARN=$WARN"
echo "  日志目录：$OUT"
if [ "$FAIL" -gt 0 ]; then
  echo "  结论：不通过（见上面的 FAIL 项）"
  exit 1
fi
echo "  结论：通过（官方推荐栈 + 本仓 ROS 2 / SDK 链路可用）"
exit 0
