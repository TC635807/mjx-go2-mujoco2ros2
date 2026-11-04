#!/usr/bin/env bash
# =============================================================================
# 跨版本 CycloneDDS 互通探针
#
# 回答一个问题：**当前这台机器上的 ROS 2（以及它自带的 CycloneDDS）能不能和宇树
# 机器人固件的 CycloneDDS 0.10.2 互通？**
#
# 做法：用官方 unitree_sdk2（C++，自带 libddsc = 0.10.2，和机器人固件同版本）写两个
# 最小程序，和本机 ROS 2 的 rclpy 互相收发 unitree_go 的 LowState / LowCmd：
#   A) SDK(0.10.2) 发布 rt/lowstate  ->  ROS 2 订阅 /lowstate    （等价 机器人 -> PC）
#   B) ROS 2 发布 /lowcmd            ->  SDK(0.10.2) 订阅 rt/lowcmd（等价 PC -> 机器人）
# 两边都能收到 = DDS 层兼容；A 收不到（或 topic 可见但 0 条）= 本机 ROS 2 栈不能上真机。
#
# 用法：
#   bash run_version_probe.sh --sdk-src /opt/unitree_sdk2
#   # 容器里（Humble + CycloneDDS 0.10.5）跑 = 正对照，应该 PASS
#   # 宿主 Lyrical（CycloneDDS 11.0.1）跑 = 2026-10-25 实测 FAIL（见 README）
#
# 参数：
#   --sdk-src DIR     官方 unitree_sdk2 源码根（含 include/ lib/ thirdparty/）
#   --ros-setup FILE  ROS 2 setup.bash（默认按 ROS_DISTRO 找 /opt/ros/<distro>）
#   --ws-setup FILE   额外 workspace setup.bash（必须能提供 unitree_go，可重复）
#   --domain N        DDS domain（默认取 ROS_DOMAIN_ID，未设置则 1）
#   --iface NAME      网卡名（默认 lo；传空字符串表示不绑定，交给 autodetermine）
#   --seconds N       每段观察时长（默认 6）
# =============================================================================
set -u

SDK_SRC=""
ROS_SETUP="${ROS_SETUP:-}"
WS_SETUPS=()
DOMAIN="${ROS_DOMAIN_ID:-1}"
IFACE="lo"
SECS=6
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${TMPDIR:-/tmp}/go2_version_probe"
mkdir -p "$WORK"

while [ $# -gt 0 ]; do
  case "$1" in
    --sdk-src) SDK_SRC="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    --ws-setup) WS_SETUPS+=("$2"); shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --iface) IFACE="$2"; shift 2 ;;
    --seconds) SECS="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$SDK_SRC" ] || [ ! -d "$SDK_SRC/include" ]; then
  echo "需要 --sdk-src 指向官方 unitree_sdk2 源码根（含 include/ lib/ thirdparty/）" >&2
  exit 2
fi

if [ -z "$ROS_SETUP" ]; then
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_SETUP=/opt/ros/humble/setup.bash
  fi
fi

echo "===== 探针环境 ====="
if [ -n "$ROS_SETUP" ]; then
  # ROS 2 setup 脚本不能和 set -u 一起 source
  set +u
  source "$ROS_SETUP"
  for ws in "${WS_SETUPS[@]:-}"; do
    [ -n "$ws" ] && [ -f "$ws" ] && source "$ws"
  done
  set -u
fi
echo "  ROS_DISTRO      = ${ROS_DISTRO:-<unset>}"
echo "  RMW             = ${RMW_IMPLEMENTATION:-<unset>}"
echo "  ROS_DOMAIN_ID   = $DOMAIN"
echo "  iface           = ${IFACE:-<autodetermine>}"
echo "  CycloneDDS(ROS) = $(dpkg-query -W -f='${Version}' ros-${ROS_DISTRO:-humble}-cyclonedds 2>/dev/null || echo '?')"
echo "  SDK_SRC         = $SDK_SRC"

# ---- 编译两个最小程序（链接 SDK 自带的 libddsc/libddscxx）-------------------
INC=(-I"$SDK_SRC/include" -I"$SDK_SRC/thirdparty/include" -I"$SDK_SRC/thirdparty/include/ddscxx")
LIB=(-L"$SDK_SRC/lib/x86_64" -lunitree_sdk2 -L"$SDK_SRC/thirdparty/lib/x86_64" -lddscxx -lddsc -lpthread
     -Wl,-rpath,"$SDK_SRC/thirdparty/lib/x86_64")
echo "===== 编译探针 ====="
g++ -O2 -std=c++17 "${INC[@]}" "$HERE/pub_state.cpp" -o "$WORK/pub_state" "${LIB[@]}" || exit 1
g++ -O2 -std=c++17 "${INC[@]}" "$HERE/sub_cmd.cpp"   -o "$WORK/sub_cmd"   "${LIB[@]}" || exit 1
echo "  OK: $WORK/pub_state / $WORK/sub_cmd"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="$DOMAIN"
if [ -n "$IFACE" ]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
fi
export LD_LIBRARY_PATH="$SDK_SRC/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}"

PASS=0; FAIL=0
echo
echo "===== A) SDK(0.10.2) 发布 rt/lowstate -> ROS 2 订阅 /lowstate（机器人 -> PC）====="
"$WORK/pub_state" "$DOMAIN" "$IFACE" > "$WORK/pubA.log" 2>&1 &
PUB=$!
sleep 2
python3 "$HERE/ros2_sub.py" "$SECS" 2>&1 | tee "$WORK/subA.log" | grep -vE '^\[WARN' | tail -3
A_OK=$(grep -c 'rx=[1-9]' "$WORK/subA.log" 2>/dev/null || true)
if [ "${A_OK:-0}" -gt 0 ]; then echo "  => A PASS"; PASS=$((PASS+1)); else echo "  => A FAIL（ROS 2 侧收不到 0.10.2 的 rt/lowstate）"; FAIL=$((FAIL+1)); fi
wait $PUB 2>/dev/null || true

echo
echo "===== B) ROS 2 发布 /lowcmd -> SDK(0.10.2) 订阅 rt/lowcmd（PC -> 机器人）====="
"$WORK/sub_cmd" "$DOMAIN" "$IFACE" "$SECS" > "$WORK/subB.log" 2>&1 &
SUB=$!
sleep 2
python3 "$HERE/ros2_pub.py" "$SECS" 2>&1 | tail -1
wait $SUB 2>/dev/null || true
tail -1 "$WORK/subB.log"
if grep -q 'received [1-9]' "$WORK/subB.log"; then echo "  => B PASS"; PASS=$((PASS+1)); else echo "  => B FAIL（SDK 侧收不到 ROS 2 的 /lowcmd）"; FAIL=$((FAIL+1)); fi

echo
echo "===== 结论 ====="
echo "  PASS=$PASS FAIL=$FAIL"
if [ "$FAIL" -gt 0 ]; then
  echo "  当前 ROS 2 栈的 CycloneDDS 版本与宇树 0.10.2 DDS 不互通："
  echo "  不要用这套环境直接上真机；请用官方栈（Ubuntu 22.04 + Humble + CycloneDDS 0.10.x，见 docker/）。"
  exit 1
fi
echo "  双向互通：这套 ROS 2 + CycloneDDS 组合可以和机器人固件 0.10.2 通信。"
exit 0
