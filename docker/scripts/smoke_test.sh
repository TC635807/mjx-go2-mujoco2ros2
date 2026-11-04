#!/usr/bin/env bash
# 环境自检：确认镜像就是“宇树官方推荐栈”，并且各 SDK 可用。
# 用法（容器内任意目录）： bash /workspace/docker/scripts/smoke_test.sh
set -u

echo "== 系统 =="
. /etc/os-release
echo "  ${PRETTY_NAME}  kernel=$(uname -r)  python=$(python3 -V 2>&1)"

echo
echo "== ROS 2（官方推荐 Humble）=="
echo "  ROS_DISTRO=${ROS_DISTRO:-<unset>}  RMW=${RMW_IMPLEMENTATION:-<unset>}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
ros2 --help >/dev/null 2>&1 && echo "  ros2 CLI: OK" || echo "  ros2 CLI: MISSING"

echo
echo "== CycloneDDS（机器人固件 = 0.10.2）=="
dpkg -l 2>/dev/null | awk '/cyclonedds/ {printf "  %-42s %s\n", $2, $3}'
python3 - <<'PY'
try:
    import cyclonedds
    print("  python cyclonedds:", getattr(cyclonedds, "__version__", "?"))
except Exception as e:
    print("  python cyclonedds: MISSING", e)
PY

echo
echo "== 官方 unitree_ros2 消息包 =="
for p in unitree_go unitree_api unitree_hg; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    echo "  $p: OK ($(ros2 pkg prefix "$p"))"
  else
    echo "  $p: MISSING"
  fi
done
python3 - <<'PY'
try:
    from unitree_go.msg import LowCmd, LowState, MotorCmd
    m = LowCmd()
    print("  python unitree_go: OK  motor_cmd slots =", len(m.motor_cmd))
except Exception as e:
    print("  python unitree_go: MISSING", e)
PY

echo
echo "== 官方 unitree_sdk2（C++）=="
if [ -f "${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/lib/libunitree_sdk2.a" ]; then
  echo "  libunitree_sdk2.a: OK (${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics})"
else
  echo "  libunitree_sdk2.a: MISSING"
fi
ls "${UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}/bin" 2>/dev/null | head -5 | sed 's/^/  example: /'

echo
echo "== 官方 unitree_sdk2_python =="
python3 - <<'PY'
try:
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    print("  unitree_sdk2py: OK  CRC 类与 unitree_go IDL 可用")
except Exception as e:
    print("  unitree_sdk2py: FAIL", e)
PY

echo
echo "== 本仓仿真依赖（venv）=="
python3 - <<'PY'
try:
    import numpy, mujoco, sys
    print("  python:", sys.version.split()[0], "numpy:", numpy.__version__, "mujoco:", mujoco.__version__)
except Exception as e:
    print("  venv 依赖: FAIL", e)
PY

echo
echo "== 网络接口（DDS 用它发现机器人）=="
ip -o -4 addr show 2>/dev/null | awk '{printf "  %-8s %s\n", $2, $4}'
echo
echo "（真机：上表里 192.168.123.x/24 的网卡名就是 UNITREE_NIC）"
