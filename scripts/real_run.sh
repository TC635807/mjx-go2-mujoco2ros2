#!/usr/bin/env bash
# 真机 Go2 上机启动器：一条命令按「体检 -> 零力矩 -> 站姿 -> 策略」四个阶段上机。
#
# =============================================================================
# 职责
# =============================================================================
#   把「明天到实验室要按顺序敲的几条命令 + 每步该看什么 + 出事怎么退」收进一个
#   脚本，并且把安全闸门写死在流程里：
#     * preflight 阶段先跑 scripts/env_check.py，环境不通过就**拒绝**启动控制器
#       （除非显式 --force）；
#     * 每个阶段前打印「机器人此刻应该处于什么状态」，并要人输入 yes 确认；
#     * policy 阶段时长有硬上限（--max-duration，默认 60 s），防止一次跑飞；
#     * --dry-run 在本机（没有 ROS 2、没有真机）也能把每条命令完整打印出来。
#
# =============================================================================
# 数据流
# =============================================================================
#   本脚本（bash，主控 PC 侧）
#     |- 阶段 preflight:  .venv/bin/python scripts/env_check.py --target real_ros
#     |                   [--iface $GO2_IFACE] --json logs/real_<ts>/preflight.json
#     |                   传感器只读自检（scripts/real_sensor_check.py 或
#     |                   scripts/go2_real_check.py）—— 只订阅，不发 LowCmd
#     |- 阶段 zero:       scripts/run_with_ros2.sh ros2_controller/run_controller.py
#     |                     --target real_ros --controller zero  --duration 10
#     |- 阶段 stand:      同上 --controller stand --duration 15
#     '- 阶段 policy:     同上 --controller policy --duration N --scan leg_fk
#                             [--cmd VX VY WZ] [--policy-profile NAME]
#   每个阶段的 summary 落在 logs/real_<ts>/<stage>.json（run_controller --json）。
#   **本脚本不自己 source /opt/ros** —— 一律复用 scripts/run_with_ros2.sh，
#   保证「ROS 2 + venv python + unitree_go」的 source 顺序只有一处维护。
#
# =============================================================================
# 真机要点（现场按这个顺序想）
# =============================================================================
#   1) 机器人先【吊起】，遥控器在手，L2+R2 进 debug mode（释放 sport mode）。
#   2) preflight 通过不等于能发指令：还要在 zero 阶段看到 /lowstate 有数据、
#      电机 mode 变 0x01（go2_real_check.py 会打印）。
#   3) stand 阶段把关节顶到标称站姿：吊着先看姿态对不对，再落地。
#   4) policy 阶段必须落地、周围清空、手放急停。首测用小速度（--cmd "0.2 0 0"）。
#   5) 控制器被正常 Ctrl-C 时会自己发 100 ms 阻尼（kp=0/kd=8）；
#      被 kill -9 / 掉电 / 拔网线时【不会】——机器人保持最后一条指令，
#      不会自己卸力，必须手按急停或遥控器阻尼键。
#   6) domain：真机 0，仿真 1（config 的 io_profiles 已分开，别串台）。
#   7) 容器（docker 官方栈）：解释器优先用 GO2_PYTHON（= /opt/venv/bin/python）；
#      容器里 .venv/bin/python 是宿主 3.13 建的，不能用。见 docs/real-bringup.md §7。
#   8) 网卡：本脚本自动探测 192.168.123.0/24 的 UP 网卡并 export GO2_IFACE，
#      不用改 config/go2_sim.json（那里写的是 eth0）。探测不到就沿用配置并告警，
#      此时用 --iface 指定（ip -br addr 看名字）。
#
# =============================================================================
# 常见失败模式
# =============================================================================
#   "env_check 未通过"           -> 按它打印的「修」逐条处理；现场硬上才 --force。
#   "没有可用的终端（非交互）"   -> 想免确认必须显式 --yes（不许静默自动跑）。
#   "duration 超过上限"          -> 改 --max-duration，或分两次跑；默认 60 s。
#   "run_controller 起不来"       -> 通常是 ROS 2 没装/未 source：先单独跑
#                                   scripts/run_with_ros2.sh ros2_controller/run_controller.py --help
#   "整个运行期没收到 /lowstate"  -> 网卡/domain/CYCLONEDDS_URI/网线；回 preflight 看。
#   "机器人不动"                  -> sport mode 没释放 / CRC / 电机 mode，见 real_env/README.md。
#
# =============================================================================
# 相关文件
# =============================================================================
#   scripts/env_check.py               环境体检（preflight 的闸门）
#   scripts/run_with_ros2.sh           唯一的 ROS 2 环境入口（本脚本只调它）
#   scripts/go2_real_check.py          传感器只读自检（fallback）
#   ros2_controller/run_controller.py  控制器（--stage -> --controller 的映射）
#   docs/real-bringup.md               现场 runbook（时序/期望现象/排错）
#   logs/real_<ts>/                    本次上机的全部日志（gitignore）
#
# 用法: scripts/real_run.sh --help
#
set -u -o pipefail   # 故意不用 -e：阶段失败也要走到 trap 打印收尾信息

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || { echo "找不到仓库根目录: $ROOT" >&2; exit 1; }

# 解释器优先级（顺序有讲究，容器里踩过；这里只用来跑 env_check + 数值校验，不跑控制器）：
#   1) GO2_PYTHON —— docker/setup_unitree_humble.sh 会 export /opt/venv/bin/python
#      （容器里的 python3.10 + numpy/mujoco；宿主 .venv 是 python3.13 建的，容器里 import
#       numpy 会失败，所以容器里绝不能用 .venv/bin/python）
#   2) 本仓 .venv/bin/python —— 本机开发默认
#   3) PATH 上的 python3 —— 万一 venv 坏了/没建
PY="${GO2_PYTHON:-$ROOT/.venv/bin/python}"
if [ ! -x "$PY" ]; then
  printf '[warn] GO2_PYTHON / venv 解释器不可执行: %s，往下回退\n' "$PY" >&2
  PY="$ROOT/.venv/bin/python"
fi
if [ ! -x "$PY" ]; then
  PY="$(command -v python3 || true)"
fi
if [ -z "$PY" ]; then
  echo "GO2_PYTHON / $ROOT/.venv/bin/python / python3 都不可用" >&2
  exit 1
fi

# ------------------------------------------------------------------ 默认参数
STAGE="all"
DURATION=30            # policy 阶段时长（秒）
MAX_DURATION=60        # policy 阶段时长硬上限
ZERO_S=10              # zero 阶段固定 10 s（够看清 /lowstate 与电机 mode）
STAND_S=15             # stand 阶段固定 15 s（够关节从趴到站平滑过渡）
CMD=""                 # policy 速度指令 "vx vy wz"；空 = 用配置默认
PROFILE=""             # policy 档 -> run_controller --policy-profile
SCAN="leg_fk"          # height scan 来源（真机安全档；topic 需要感知节点在跑）
IFACE=""               # 有线网卡名；空 = 自动探测
IFACE_SRC=""           # 网卡名来源：--iface / auto / 空
ASSUME_YES=0
DRY=0
DO_PREFLIGHT=1
FORCE=0
NO_NETWORK=0

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT/logs/real_$STAMP"
LAST_STAGE="（未开始）"
SUMMARY_LIST=""        # 每个阶段的 json 路径，换行分隔（trap 里打印）

# ------------------------------------------------------------------ 输出工具
say()  { printf '%s\n' "$*"; }
warn() { printf '[warn] %s\n' "$*" >&2; }
err()  { printf '[error] %s\n' "$*" >&2; }
die()  { err "$*"; exit 2; }

# 把一条命令按 shell 可复制粘贴的形式打印（参数里有空格会自动加引号）
q() {
  local out="" a
  for a in "$@"; do out="$out$(printf '%q ' "$a")"; done
  printf '%s\n' "$out" | sed -e 's/[[:space:]]*$//'
}

# 执行（dry-run 时只打印）。返回值由调用方判断。
run() {
  if [ "$DRY" = 1 ]; then
    printf '[dry-run] %s\n' "$(q "$@")"
    return 0
  fi
  printf '[run] %s\n' "$(q "$@")"
  "$@"
}

usage() {
  cat <<'EOF'
真机 Go2 上机启动器（主控 PC 侧）

用法: scripts/real_run.sh [选项]

阶段（--stage）:
  all          preflight -> zero -> stand -> policy（默认；首测推荐）
  preflight    只做环境体检 + 传感器只读自检，不发任何指令（最安全）
  zero         零力矩 10 s：确认 /lowstate 有数据、电机能进伺服
  stand        站姿 PD 15 s：看关节平滑走到标称站姿
  policy       策略控制 --duration 秒（默认 30，硬上限 --max-duration）

选项:
  --stage NAME         all|preflight|zero|stand|policy（默认 all）
  --duration SEC       policy 阶段时长（默认 30）
  --max-duration SEC   时长上限（默认 60；超过直接拒绝，除非 --force）
  --cmd "VX VY WZ"     policy 的速度指令，例如 --cmd "0.2 0 0"（默认取配置）
  --profile NAME       policy 档（传给 --policy-profile，例如 go2_v33d）
  --scan SRC           height scan 来源（默认 leg_fk；真机安全档）
  --iface NAME         有线网卡名（默认自动探测 192.168.123.0/24 的 UP 网卡）
  --yes                跳过每个阶段的交互确认（危险：确认得你自己按）
  --dry-run            只打印将执行的命令，不执行（本机随时可跑）
  --no-preflight       跳过 preflight（不推荐；env_check 的闸门会一起跳过）
  --no-network         传给 env_check：不 ping 机器人（没插网线时用）
  --force              env_check 不通过 / 时长超限 也继续（现场自担风险）
  -h, --help           显示本帮助

机器人状态要求（每个阶段前也会再打印一次）:
  preflight  只读，机器人可以吊起也可以落地，但先别让它动
  zero       必须【吊起】，遥控器在手并已进 debug mode（释放 sport mode）
  stand      仍应【吊起或有人扶着】，看姿态没问题再落地
  policy     必须【落地站稳】，周围 1 m 清空，手放急停/阻尼键

日志:
  logs/real_<时间戳>/preflight.json  zero.json  stand.json  policy.json

例:
  scripts/real_run.sh --dry-run                      # 本机先把命令看一遍
  scripts/real_run.sh --stage preflight              # 到实验室第一步
  scripts/real_run.sh --stage zero                   # 吊起，确认链路
  scripts/real_run.sh --stage stand                  # 吊起，看站姿
  scripts/real_run.sh --stage policy --duration 20 --cmd "0.2 0 0"
  scripts/real_run.sh                                # 全流程（每步要 yes 确认）
EOF
}

# ------------------------------------------------------------------ 参数解析
while [ $# -gt 0 ]; do
  case "$1" in
    --stage)        STAGE="$2"; shift 2 ;;
    --duration)     DURATION="$2"; shift 2 ;;
    --max-duration) MAX_DURATION="$2"; shift 2 ;;
    --cmd)          CMD="$2"; shift 2 ;;
    --profile)      PROFILE="$2"; shift 2 ;;
    --scan)         SCAN="$2"; shift 2 ;;
    --iface)        IFACE="$2"; shift 2 ;;
    --yes|-y)       ASSUME_YES=1; shift ;;
    --dry-run|-n)   DRY=1; shift ;;
    --no-preflight) DO_PREFLIGHT=0; shift ;;
    --no-network)   NO_NETWORK=1; shift ;;
    --force)        FORCE=1; shift ;;
    -h|--help)      usage; exit 0 ;;
    *)              err "未知参数: $1"; usage; exit 2 ;;
  esac
done

case "$STAGE" in
  all|preflight|zero|stand|policy) ;;
  *) die "--stage 只能是 all|preflight|zero|stand|policy（收到 $STAGE）" ;;
esac

is_num() { printf '%s' "$1" | grep -Eq '^[0-9]+([.][0-9]+)?$'; }

is_num "$DURATION"     || die "--duration 必须是数字（收到 $DURATION）"
is_num "$MAX_DURATION" || die "--max-duration 必须是数字（收到 $MAX_DURATION）"

# 时长上限：policy 阶段的硬护栏（--force 可越过，但会大声告警）
if ! "$PY" -c 'import sys; sys.exit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)' \
        "$DURATION" "$MAX_DURATION"; then
  if [ "$FORCE" = 1 ]; then
    warn "policy 时长 $DURATION s 超过上限 $MAX_DURATION s，--force 继续（请确认真有必要）"
  else
    die "policy 时长 $DURATION s 超过上限 $MAX_DURATION s；改 --duration 或显式 --max-duration（或 --force）"
  fi
fi

# --cmd 必须是 3 个数字（vx vy wz）：set -f 关掉通配，避免 * 被展开
VX=""; VY=""; WZ=""
if [ -n "$CMD" ]; then
  set -f
  set -- $CMD
  if [ $# -ne 3 ]; then
    set +f
    die "--cmd 需要正好 3 个数字 (vx vy wz)，收到: $CMD"
  fi
  VX="$1"; VY="$2"; WZ="$3"
  set +f
  set --
  for _a in "$VX" "$VY" "$WZ"; do
    is_num "$_a" || die "--cmd 里的 $_a 不是数字（收到: $CMD）"
  done
fi

# ------------------------------------------------------------------ 网卡探测
# 目的：config/go2_sim.json 里写死 "eth0"，但现场机器往往是 enp3s0/enp0s31f6。
# 这里用 signal_io.config.detect_iface()（只读）挑「192.168.123.0/24 的 UP 网卡」，
# 探测不到就不猜：沿用配置文件默认值，并提示用 --iface 手动指定。
if [ -n "$IFACE" ]; then
  IFACE_SRC="--iface"
else
  IFACE="$(env -u GO2_IFACE -u GO2_INTERFACE "$PY" - "$ROOT" <<'PYEOF' 2>/dev/null
import sys
sys.path.insert(0, sys.argv[1])
try:
    from signal_io.config import detect_iface
    print(detect_iface() or "")
except Exception:
    print("")
PYEOF
)"
  if [ -n "$IFACE" ]; then IFACE_SRC="auto"; fi
fi

if [ -n "$IFACE" ]; then
  export GO2_IFACE="$IFACE"
  export GO2_INTERFACE="$IFACE"     # 兼容旧名（docker/ 里在用）
fi

# ------------------------------------------------------------------ 收尾（trap）
on_exit() {
  local rc=$?
  say ""
  say "================ real_run.sh 收尾 ================"
  say "最后执行的阶段 : $LAST_STAGE"
  if [ -n "$SUMMARY_LIST" ]; then
    say "summary 文件   :"
    printf '%s\n' "$SUMMARY_LIST" | while IFS= read -r f; do
      if [ -n "$f" ]; then say "  - $f"; fi
    done
  fi
  say "日志目录       : $LOG_DIR"
  say "⚠ 安全提醒：若本进程被 kill -9 / 掉电 / 拔网线，退出阻尼【不会】执行——"
  say "  机器人不会自己卸力，必须手按急停或遥控器阻尼键！"
  if [ "$rc" -ne 0 ]; then
    say "退出码         : $rc（0 = 全部阶段正常结束；1 = 阶段失败；2 = 参数/前置检查拒绝；3 = 人工中止）"
  fi
}
trap on_exit EXIT
trap 'say ""; say "[real_run] 收到 SIGINT：控制器会自己发 100 ms 阻尼后退出"; exit 130' INT
trap 'say ""; say "[real_run] 收到 SIGTERM：控制器会自己发 100 ms 阻尼后退出"; exit 143' TERM

# ------------------------------------------------------------------ 阶段确认
# msg: 机器人此刻应处于什么状态。非 --yes 时必须人肉输入 yes；没有终端则拒绝。
confirm_stage() {
  local stage="$1" msg="$2" ans=""
  say ""
  say "------------------------------------------------------------------"
  say "阶段: $stage"
  say "机器人应处于: $msg"
  if [ "$DRY" = 1 ]; then
    say "[dry-run] 跳过交互确认"
    return 0
  fi
  if [ "$ASSUME_YES" = 1 ]; then
    say "[--yes] 已跳过确认（确认得你自己按）"
    return 0
  fi
  if [ ! -r /dev/tty ]; then
    err "没有可用的终端（非交互环境）：想免确认必须显式加 --yes"
    exit 3
  fi
  printf '确认继续请输入 yes（其它任意输入中止）: ' >&2
  read -r ans < /dev/tty || ans=""
  case "$ans" in
    yes|YES|Yes) return 0 ;;
    *) err "已中止（输入: $ans）"; exit 3 ;;
  esac
}

json_path() { printf '%s/%s.json' "$LOG_DIR" "$1"; }

# 传感器只读自检脚本：优先用 T1 的 real_sensor_check.py，没有就退到 go2_real_check.py
sensor_script() {
  if [ -f "$ROOT/scripts/real_sensor_check.py" ]; then
    printf '%s\n' "$ROOT/scripts/real_sensor_check.py"
  elif [ -f "$ROOT/scripts/go2_real_check.py" ]; then
    printf '%s\n' "$ROOT/scripts/go2_real_check.py"
  fi
}

stage_preflight() {
  LAST_STAGE="preflight"
  confirm_stage preflight "只读阶段：不发任何 LowCmd。机器人吊起或落地都可以，但先别让它动。"
  if [ "$DRY" != 1 ]; then mkdir -p "$LOG_DIR"; fi
  local jp rc ss
  jp="$(json_path preflight)"
  say ""
  say "-- 1/2 环境体检（env_check，只读）--"
  if [ -n "$IFACE" ]; then
    if [ "$NO_NETWORK" = 1 ]; then
      run "$PY" scripts/env_check.py --target real_ros --iface "$IFACE" --no-network --json "$jp"
    else
      run "$PY" scripts/env_check.py --target real_ros --iface "$IFACE" --json "$jp"
    fi
  else
    if [ "$NO_NETWORK" = 1 ]; then
      run "$PY" scripts/env_check.py --target real_ros --no-network --json "$jp"
    else
      run "$PY" scripts/env_check.py --target real_ros --json "$jp"
    fi
  fi
  rc=$?
  SUMMARY_LIST="$SUMMARY_LIST
$jp"
  if [ "$DRY" = 1 ]; then
    say "[dry-run] 未实际运行 env_check（真实执行时它的退出码决定是否继续）"
  elif [ "$rc" -ne 0 ]; then
    if [ "$FORCE" = 1 ]; then
      warn "env_check 未通过（rc=$rc），但 --force：继续（风险自负）"
    else
      die "env_check 未通过（rc=$rc）。按它打印的「修」逐条处理；现场确需硬上请加 --force"
    fi
  else
    say "-- 环境体检通过 --"
  fi

  say ""
  say "-- 2/2 传感器只读自检（只订阅，不发 LowCmd）--"
  ss="$(sensor_script)"
  if [ -z "$ss" ]; then
    warn "没找到 scripts/real_sensor_check.py 或 scripts/go2_real_check.py，跳过"
    return 0
  fi
  # 两个脚本的 CLI 不同：real_sensor_check.py 支持 --iface/--json（把报告也收进本次日志），
  # go2_real_check.py 只认 --target（老脚本，保持最小调用面）。
  case "$ss" in
    */real_sensor_check.py)
      # --light：本步只判低层链路健康。全量订阅（点云/odom/pose）会把自己的
      # /lowstate 回调饿死，实测把 500 Hz 报成 190~275 Hz（假 FAIL）；轻量模式 496 Hz。
      # 点云/雷达单独跑一次不带 --light 的完整自检（见 docs/sim2real-day0.md §3.2）。
      if [ -n "$IFACE" ]; then
        run "$ROOT/scripts/run_with_ros2.sh" "$ss" --target real_ros --iface "$IFACE" --light --json "$(json_path sensor)"
      else
        run "$ROOT/scripts/run_with_ros2.sh" "$ss" --target real_ros --light --json "$(json_path sensor)"
      fi
      ;;
    *)
      run "$ROOT/scripts/run_with_ros2.sh" "$ss" --target real_ros
      ;;
  esac
  rc=$?
  if [ "$DRY" = 1 ]; then
    say "[dry-run] 未实际运行传感器自检"
  elif [ "$rc" -ne 0 ]; then
    # 只读自检失败不阻断 preflight（它要求真机在线）；但 policy 前必须人工确认
    warn "传感器自检 rc=$rc（只读，不阻断）。进入 policy 前必须先确认 /lowstate 有数据"
  fi
  return 0
}

stage_zero() {
  LAST_STAGE="zero"
  confirm_stage zero "必须【吊起/四足离地】，遥控器在手。正确顺序：先让本阶段跑起来（开始以 500 Hz 发零力矩）-> 再【短按一次】L2+R2 释放 sport mode。绝对不要先按 L2+R2 再起程序：中间那段没有任何指令源的窗口会让四肢疯狂抽搐（2026-11-05 实测踩过）。"
  if [ "$DRY" != 1 ]; then mkdir -p "$LOG_DIR"; fi
  local jp rc
  jp="$(json_path zero)"
  SUMMARY_LIST="$SUMMARY_LIST
$jp"
  run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
      --target real_ros --controller zero --duration "$ZERO_S" --json "$jp"
  rc=$?
  if [ "$rc" -ne 0 ]; then
    warn "zero 阶段 rc=$rc：回看 $jp（summary.n_state=0 说明根本没收到 /lowstate）"
  fi
  return 0
}

stage_stand() {
  LAST_STAGE="stand"
  confirm_stage stand "仍应【吊起或有人扶着】。本阶段把 12 个关节平滑顶到标称站姿并保持；姿态诡异立刻按遥控器阻尼键。"
  if [ "$DRY" != 1 ]; then mkdir -p "$LOG_DIR"; fi
  local jp rc
  jp="$(json_path stand)"
  SUMMARY_LIST="$SUMMARY_LIST
$jp"
  run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
      --target real_ros --controller stand --duration "$STAND_S" --json "$jp"
  rc=$?
  if [ "$rc" -ne 0 ]; then
    warn "stand 阶段 rc=$rc：回看 $jp 与控制器 stdout"
  fi
  return 0
}

stage_policy() {
  LAST_STAGE="policy"
  confirm_stage policy "必须【落地站稳】，周围 1 m 内无人/无障碍，手放在急停或遥控器阻尼键上。首测先用小速度：--cmd \"0.2 0 0\"。"
  if [ "$DRY" != 1 ]; then mkdir -p "$LOG_DIR"; fi
  local jp rc
  jp="$(json_path policy)"
  SUMMARY_LIST="$SUMMARY_LIST
$jp"
  if [ -n "$VX" ]; then
    if [ -n "$PROFILE" ]; then
      run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
          --target real_ros --controller policy --scan "$SCAN" --duration "$DURATION" \
          --cmd "$VX" "$VY" "$WZ" --policy-profile "$PROFILE" --json "$jp"
    else
      run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
          --target real_ros --controller policy --scan "$SCAN" --duration "$DURATION" \
          --cmd "$VX" "$VY" "$WZ" --json "$jp"
    fi
  else
    if [ -n "$PROFILE" ]; then
      run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
          --target real_ros --controller policy --scan "$SCAN" --duration "$DURATION" \
          --policy-profile "$PROFILE" --json "$jp"
    else
      run "$ROOT/scripts/run_with_ros2.sh" ros2_controller/run_controller.py \
          --target real_ros --controller policy --scan "$SCAN" --duration "$DURATION" \
          --json "$jp"
    fi
  fi
  rc=$?
  if [ "$rc" -ne 0 ]; then
    warn "policy 阶段 rc=$rc：回看 $jp（summary 里有 n_state / health 等级）"
  fi
  return 0
}

# ------------------------------------------------------------------ 主流程
say "=========================================================="
say "Go2 真机上机启动器  stage=$STAGE"
say "仓库      : $ROOT"
say "python    : $PY"
if [ -n "$IFACE" ]; then
  say "网卡      : $IFACE（来源: $IFACE_SRC，已 export GO2_IFACE）"
else
  say "网卡      : 未探测到，沿用 config 里的 interface（建议 --iface 指定）"
fi
if [ "$DRY" = 1 ]; then
  say "模式      : DRY-RUN（只打印，不执行）"
else
  say "模式      : 真实执行"
fi
if [ -n "$PROFILE" ]; then
  say "policy    : duration=$DURATION s（上限 $MAX_DURATION s）scan=$SCAN profile=$PROFILE"
else
  say "policy    : duration=$DURATION s（上限 $MAX_DURATION s）scan=$SCAN"
fi
if [ -n "$VX" ]; then say "cmd       : ($VX, $VY, $WZ)"; fi
say "日志目录  : $LOG_DIR"
say "=========================================================="

if [ "$STAGE" = "preflight" ]; then
  if [ "$DO_PREFLIGHT" = 1 ]; then
    stage_preflight
  else
    warn "--stage preflight 与 --no-preflight 同时给，按 --no-preflight 跳过"
  fi
else
  if [ "$DO_PREFLIGHT" = 1 ]; then
    stage_preflight
  else
    warn "--no-preflight：跳过环境体检与传感器自检（风险自负）"
  fi
  case "$STAGE" in
    zero)   stage_zero ;;
    stand)  stage_stand ;;
    policy) stage_policy ;;
    all)
      stage_zero
      stage_stand
      stage_policy
      ;;
  esac
fi

if [ "$DRY" = 1 ]; then
  say ""
  say "[dry-run] 以上为将要执行的命令，未做任何实际操作。"
  say "[dry-run] 去掉 --dry-run 即真实执行（每个阶段仍需输入 yes）。"
else
  say ""
  say "全部阶段结束。日志目录: $LOG_DIR"
fi
exit 0
