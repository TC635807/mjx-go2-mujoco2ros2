#!/usr/bin/env bash
# =============================================================================
# 上机前一天/当天：在「控制用 Linux 机器」上跑这一条，把真机前能离线查的全查完。
#
#   bash docker/scripts/preflight_real.sh              # 只做本机检查
#   bash docker/scripts/preflight_real.sh --with-dog   # 额外进容器读 /lowstate（狗已上电接线）
#   NIC=enp0s31f6 bash docker/scripts/preflight_real.sh --with-dog   # 指定网卡
#
# 退出码 0 = 可以上机；非 0 = 有 FAIL（先修完再看 --with-dog）
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$HERE")"
PROJECT_DIR="$(dirname "$DOCKER_DIR")"
IMAGE="unitree-go2-ros2:humble"
NIC="${NIC:-}"
WITH_DOG=0
[ "${1:-}" = "--with-dog" ] && WITH_DOG=1

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $*"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $*"; WARN=$((WARN+1)); }
hr()   { echo; echo "== $* =="; }

hr "1. Docker 与镜像"
if command -v docker >/dev/null 2>&1; then
  pass "docker $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo '?')"
else
  fail "没有 docker"; fi
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  pass "镜像 $IMAGE 已导入（$(docker image inspect -f '{{.Size}}' "$IMAGE" | awk '{printf "%.1f GB", $1/1e9}')）"
else
  fail "镜像 $IMAGE 不存在（docker load -i go2-humble-image.tar.gz）"; fi
if command -v docker compose >/dev/null 2>&1; then
  pass "docker compose v$(docker compose version --short 2>/dev/null)"
else
  warn "没有 docker compose（可用纯 docker run 起容器）"; fi

hr "2. 接狗网卡"
# 自动找：名字给了就用给的；否则找唯一一张有 192.168.123.x 的网卡
if [ -z "$NIC" ]; then
  NIC="$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2}' | head -1)"
fi
if [ -n "$NIC" ]; then
  pass "网卡 $NIC 已配置 $(ip -o -4 addr show dev "$NIC" | awk '{print $4}' | head -1)"
else
  fail "没有任何网卡配了 192.168.123.x/24（狗没接线时正常）。手动配：
        sudo ip link set <接狗网卡名> up
        sudo ip addr replace 192.168.123.222/24 dev <接狗网卡名>"
fi
if [ -n "$NIC" ]; then
  spd="$(ethtool "$NIC" 2>/dev/null | awk -F': ' '/Speed/{print $2}')"
  if [ -n "$spd" ]; then
    case "$spd" in
      1000Mb/s|2500Mb/s|10000Mb/s) pass "链路速率 $spd" ;;
      100Mb/s) warn "链路速率只有 100Mb/s（够用，但建议换 CAT5e 以上网线）" ;;
      *)       warn "链路速率 $spd：500Hz 低层可能丢包，建议换网线/换口" ;;
    esac
  fi
  # 路由必须走这张网卡，不能被 Clash/TUN/ZeroTier 抢走
  rt="$(ip route get 192.168.123.161 2>/dev/null | head -1)"
  case "$rt" in
    *"dev $NIC"*) pass "到 192.168.123.161 的路由走 $NIC" ;;
    "")           warn "还 ping 不到 192.168.123.161（狗没上电/没插线时正常）" ;;
    *)            fail "到 192.168.123.161 的路由没走 $NIC：$rt（Clash/ZeroTier/TUN 抢了路由？）" ;;
  esac
fi

hr "3. 到机器人的链路"
if ping -c 2 -W 1 192.168.123.161 >/dev/null 2>&1; then
  pass "ping 192.168.123.161 通"
else
  if [ "$WITH_DOG" -eq 1 ]; then fail "ping 192.168.123.161 不通（网线/静态IP/狗是否上电）"
  else warn "ping 192.168.123.161 不通（没接线时正常）"; fi
fi

hr "4. 本机资源与干扰"
mem_avail="$(awk '/MemAvailable/{printf "%.1f", $2/1048576}' /proc/meminfo)"
awk -v m="$mem_avail" 'BEGIN{exit !(m>1.5)}' && pass "可用内存 ${mem_avail} GB" \
  || warn "可用内存只有 ${mem_avail} GB：关掉 Firefox/VSCode 再上机（控制器要稳定 500 Hz）"
nproc_="$(nproc)"
pass "CPU $nproc_ 线程，负载 $(cut -d' ' -f1-3 /proc/loadavg)"
if pgrep -c -f 'firefox|code' >/dev/null 2>&1; then
  warn "检测到 Firefox/VSCode 在跑：上机前关掉，避免 500 Hz 抖动"
fi
tun="$(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | grep -E '^(Meta|mihomo|Clash|utun|tun)' | head -3 | tr '\n' ' ')"
[ -n "$tun" ] && warn "发现 TUN 设备：$tun（会干扰组播/路由；已用 CYCLONEDDS_URI 绑网卡兜底，但建议上机时退出代理）"
grep -q '192.168.123' /etc/hosts 2>/dev/null && warn "/etc/hosts 里有 192.168.123 的条目，确认没把 .161 指错"

hr "5. 容器内只读自检（不发送任何指令）"
if [ -z "$NIC" ]; then
  warn "跳过（没有可用网卡）"
else
  cd "$DOCKER_DIR" || exit 1
  out="$(UNITREE_NIC="$NIC" ROS_DOMAIN_ID=0 docker compose run --rm --no-TTY ros2 bash -lc '
      ip -o -4 addr show | grep -c 192.168.123 || true
      echo ---TOPICS---
      timeout 6 ros2 topic list 2>/dev/null | grep -E "lowstate|lowcmd|utlidar" || echo NONE
    ' 2>&1)"
  if echo "$out" | grep -q '/lowstate'; then pass "容器内能看到 /lowstate 话题"
  elif [ "$WITH_DOG" -eq 1 ]; then fail "容器内看不到 /lowstate：domain/网卡/CYCLONEDDS_URI 有问题"
  else warn "容器内没看到 /lowstate（狗没接时正常）"; fi
  if [ "$WITH_DOG" -eq 1 ]; then
    hz="$(UNITREE_NIC="$NIC" ROS_DOMAIN_ID=0 docker compose run --rm --no-TTY ros2 bash -lc \
          'timeout 8 ros2 topic hz /lowstate 2>&1 | head -6' 2>&1)"
    echo "$hz" | sed 's/^/      /'
    echo "$hz" | grep -qE 'average rate: *[4-6][0-9][0-9]' && pass "低层 /lowstate ≈ 500 Hz" \
      || fail "低层 /lowstate 不是 500 Hz（看上面的输出）"
  fi
fi

hr "汇总"
echo "  PASS=$PASS FAIL=$FAIL WARN=$WARN"
if [ "$FAIL" -eq 0 ]; then
  echo "  ✅ 可以上机。下一步：scripts/go2_real_check.py（只读），再 zero -> stand -> policy"
  echo "     详细顺序见 docs/sim2real-test-day.md"
  exit 0
fi
echo "  ❌ 先修 FAIL 项再看 docs/sim2real-test-day.md 的故障树"
exit 1
