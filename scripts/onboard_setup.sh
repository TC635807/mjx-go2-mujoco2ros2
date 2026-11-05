#!/usr/bin/env bash
# 目标机（实验室 PC / Go2 机载电脑）一次性装机：python 检查 -> venv -> 依赖 -> (可选) ROS 2 + colcon -> 自检。
#
# 用法:
#   bash scripts/onboard_setup.sh                 # 只装 python 侧（venv + requirements），再跑自检
#   bash scripts/onboard_setup.sh --with-ros      # 额外 source ROS 2 并 colcon build 本仓 unitree_go
#   bash scripts/onboard_setup.sh --dry-run       # 只打印每一步要做什么（任何机器都能跑，不改任何东西）
#   ROS_DISTRO=humble bash scripts/onboard_setup.sh --with-ros    # 显式指定发行版
#   bash scripts/onboard_setup.sh --python /usr/bin/python3.10 --with-ros
#   bash scripts/onboard_setup.sh --clean-before-ros --with-ros   # 先删 ros2_ws/{build,install,log} 再 build
#
# 为什么需要它（而不是照着 README 手打几条命令）:
#   1) **不要**把源机器（本机）的 .venv 拷过来：它是 cp313 + x86_64 的二进制，
#      到了 arm64/py3.10 上 import numpy/mujoco 直接崩。必须在目标机上重建。
#   2) **不要**把 ros2_ws/build、ros2_ws/install 拷过来：那是容器（/work）里用
#      py3.10 构建的，CMakeCache 里写死绝对路径，install 里的 .so 是 x86-64。
#      必须在目标机 source /opt/ros/<distro> 后重新 colcon build。
#   3) venv 的 python 版本**必须与 ROS 2 的解释器一致**（22.04 + Humble = py3.10）：
#      scripts/run_with_ros2.sh 是把系统 ROS 塞进 venv 的 sys.path 用的，ABI 不一致时
#      rclpy 的 .so import 会失败。脚本会先对比并提醒。
#
# 设计约定:
#   * 幂等：venv 已在且版本对 -> 跳过创建；依赖已满足 -> pip 会说 already satisfied；
#     unitree_go 已 build 且源码没变 -> 跳过 colcon（--force-colcon 可强制）。
#   * **绝不 sudo**：需要 apt 装东西时只把命令行打出来，由你自己决定。
#   * **绝不自动删**：python 版本不匹配的旧 .venv / 旧 install 只提示、不删；
#     要删请显式 --clean-before-ros（会先打印要删什么）。
#   * --dry-run 不执行任何写操作（除了 mktemp 之外不碰磁盘）。
#
# 退出码:
#   0 = 装机完成（允许 WARN；没开 --with-ros 时，ROS 2 那一半的 FAIL 只提示不阻断 ——
#       仿真/离线机器本来就可以没有 ROS 2）；
#   1 = 硬失败（python 版本不够 / pip 失败 / colcon 失败），或开了 --with-ros 而自检 FAIL
#       （既然你要求装 ROS 2，那 ROS 2 这半就必须真的能用）。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

usage() {
  sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
}

log()  { printf '\n[setup] %s\n' "$*"; }
info() { printf '  %s\n' "$*"; }
warn() { printf '  [warn] %s\n' "$*" >&2; }
die()  { printf '  [fail] %s\n' "$*" >&2; exit 1; }
note() { printf '[setup] %s\n' "$*"; }

WITH_ROS=0
DRY=0
PYBIN=""
ROS_DISTRO_ARG=""
SKIP_PIP=0
FORCE_COLCON=0
CLEAN_ROS=0
ALLOW_OLD_PY=0
CHECK_FAIL=0          # env_check 有没有 FAIL；--with-ros 时它决定退出码

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)             usage ;;
    --with-ros)            WITH_ROS=1; shift ;;
    --dry-run)             DRY=1; shift ;;
    --python)              PYBIN="${2:?--python 需要解释器路径}"; shift 2 ;;
    --ros-distro)          ROS_DISTRO_ARG="${2:?--ros-distro 需要发行版名}"; shift 2 ;;
    --skip-pip)            SKIP_PIP=1; shift ;;
    --force-colcon)        FORCE_COLCON=1; shift ;;
    --clean-before-ros)    CLEAN_ROS=1; shift ;;
    --allow-old-python)    ALLOW_OLD_PY=1; shift ;;
    *)                     die "未知参数: $1（--help 看用法）" ;;
  esac
done

# run: 真正执行命令；--dry-run 时只打印（返回 0）
run() {
  if [ "$DRY" = 1 ]; then
    printf '  [dry]'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  printf '  [run]'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

VENV="$ROOT/.venv"
VENV_PY="$VENV/bin/python"

# ================================================================== 0) 机器画像
log "0/5 目标机画像"
info "仓库         : $ROOT"
info "uname        : $(uname -srm)"
if [ -r /etc/os-release ]; then info "发行版       : $(. /etc/os-release; echo "${PRETTY_NAME:-unknown}")"; fi
if [ -z "$PYBIN" ]; then PYBIN="python3"; fi
if command -v "$PYBIN" >/dev/null 2>&1; then
  info "python3      : $("$PYBIN" -V 2>&1) @ $(command -v "$PYBIN")"
else
  die "找不到 $PYBIN（用 --python /usr/bin/python3.x 指定）"
fi
info "CPU 核数     : $(nproc 2>/dev/null || echo '?')"
info "内存         : $(awk '/MemTotal/{printf "%.1f GiB", $2/1048576}' /proc/meminfo 2>/dev/null || echo '?')"
info "磁盘(仓库盘) : 可用 $(df -h "$ROOT" 2>/dev/null | awk 'NR==2{print $4}' || echo '?')"
info "ROS 2        : $(
  if [ -d /opt/ros ]; then ls -d /opt/ros/*/ 2>/dev/null | sed 's,/opt/ros/,,' | tr '\n' ' ' | sed 's/ $//'; else echo '未安装（没有 /opt/ros，纯仿真/离线场景可以没有）'; fi
)"

# ================================================================== 1) python 版本
log "1/5 python 版本（requirements.txt 要求 >= 3.10）"
PY_MM="$("$PYBIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
PY_OK="$("$PYBIN" -c 'import sys; print(1 if sys.version_info >= (3, 10) else 0)')"
if [ "$PY_OK" = "1" ]; then
  info "$PYBIN = $PY_MM ✓"
else
  warn "$PYBIN = $PY_MM 低于 requirements.txt 要求的 3.10"
  info "  修法（二选一，脚本不会替你 sudo）:"
  info "    * Ubuntu 22.04：sudo apt install python3.10 python3.10-venv  然后 --python /usr/bin/python3.10"
  info "    * Ubuntu 20.04（部分机载电脑）：默认 python3 是 3.8，需要 deadsnakes/conda 装 3.10+，或升级系统"
  if [ "$ALLOW_OLD_PY" = 1 ]; then
    warn "--allow-old-python：继续装（numpy/mujoco 可能没有对应 wheel，会失败）"
  elif [ "$DRY" = 1 ]; then
    warn "--dry-run：继续演示，不中止"
  else
    die "python 版本不够，先解决再跑（或 --allow-old-python 强行试）"
  fi
fi

# ================================================================== 2) venv
log "2/5 venv（$VENV）"
if [ -f "$VENV/pyvenv.cfg" ]; then
  OLD_MM="$(awk -F' = ' '/^version/{print $2; exit}' "$VENV/pyvenv.cfg" | cut -d. -f1-2)"
  if [ "$OLD_MM" = "$PY_MM" ]; then
    info "已存在且版本一致（python$OLD_MM），跳过创建"
  else
    warn "已存在的 .venv 是 python$OLD_MM，当前解释器是 python$PY_MM —— 两者 ABI 不兼容"
    info "  修法（脚本不替你删）：rm -rf .venv 后重跑本脚本"
    die ".venv 版本不匹配（也可以 --python /usr/bin/python$OLD_MM 用它）"
  fi
else
  if [ ! -e "$VENV" ]; then
    info "创建 venv（--system-site-packages 不开：numpy/mujoco 只认 venv 里的版本）"
    run "$PYBIN" -m venv "$VENV"
  else
    die "$VENV 存在但不是 venv（没有 pyvenv.cfg）；先移走它"
  fi
fi

# ================================================================== 3) 依赖
log "3/5 python 依赖（requirements.txt）"
if [ "$SKIP_PIP" = 1 ]; then
  info "--skip-pip：跳过（离线装机时用，前提是依赖已经装好）"
else
  run "$VENV_PY" -m pip install --upgrade pip || warn "升级 pip 失败（离线/镜像问题），继续用现有 pip"
  run "$VENV_PY" -m pip install -r "$ROOT/requirements.txt"
fi
if [ "$DRY" != 1 ]; then
  DEPS="$("$VENV_PY" -c 'import numpy, mujoco; print("numpy %s / mujoco %s" % (numpy.__version__, mujoco.__version__))' 2>&1)" \
    || die "装完还是 import 不了 numpy/mujoco：$DEPS"
  info "依赖 OK：$DEPS"
  info "架构检查：$(uname -m) + python$PY_MM（arm64 上 mujoco/numpy 必须装到 aarch64 轮子；若这里报错见 docs/portability.md §6）"
fi

# ================================================================== 4) ROS 2 + colcon（可选）
log "4/5 ROS 2 + unitree_go 消息包（--with-ros 才做）"
ROS_SETUP_FILE=""
if [ "$WITH_ROS" = 0 ]; then
  info "未加 --with-ros：跳过 ROS 2。注意 real_ros 真机路径需要它（docs/portability.md §4 第 3 步）"
else
  ROS_DISTRO_EFF=""
  if [ -n "$ROS_DISTRO_ARG" ]; then
    ROS_DISTRO_EFF="$ROS_DISTRO_ARG"
  elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_DISTRO_EFF="$ROS_DISTRO"
  else
    for cand in humble jazzy lyrical iron foxy; do
      if [ -f "/opt/ros/$cand/setup.bash" ]; then ROS_DISTRO_EFF="$cand"; break; fi
    done
  fi
  if [ -z "$ROS_DISTRO_EFF" ] || [ ! -f "/opt/ros/$ROS_DISTRO_EFF/setup.bash" ]; then
    warn "没找到 ROS 2（/opt/ros/*/setup.bash）"
    info "  修法（脚本不会替你 sudo；22.04 + Humble 为例）:"
    info "    sudo apt install ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \\"
    info "                     ros-humble-rosidl-generator-dds-idl ros-dev-tools"
    info "  注意 ros-<distro>-rosidl-generator-dds-idl 必须装：unitree_go 要生成 CycloneDDS IDL，"
    info "  少了它 colcon build 会在 rosidl_generator_dds_idl 步失败（docs/ros2-setup.md:30）。"
    if [ "$DRY" = 1 ]; then
      warn "--dry-run：继续演示，不中止"
    else
      die "--with-ros 但本机没有 ROS 2"
    fi
  else
    ROS_SETUP_FILE="/opt/ros/$ROS_DISTRO_EFF/setup.bash"
    info "使用 ROS 2：$ROS_DISTRO_EFF（$ROS_SETUP_FILE）"

    # venv python 必须与 ROS 2 的解释器同版本（ABI 一致），否则 rclpy 的 .so 加载不了
    ROS_PY_MM="$(ls -d "/opt/ros/$ROS_DISTRO_EFF/lib/python3"*/site-packages 2>/dev/null | head -1 | sed 's,.*/python3\([0-9]*\)\.\([0-9]*\)/.*,\1.\2,')"
    if [ -n "$ROS_PY_MM" ] && [ "$ROS_PY_MM" != "python3*" ]; then
      if [ "$ROS_PY_MM" != "$PY_MM" ]; then
        warn "ROS 2 的解释器是 python$ROS_PY_MM，而 venv 是 python$PY_MM —— ABI 不一致"
        info "  scripts/run_with_ros2.sh 是把 ROS 的 site-packages 塞进 venv 的 sys.path，"
        info "  版本不一致时 rclpy / unitree_go 的 .so 会 import 失败（典型报错 undefined symbol / cannot open）。"
        info "  修法：用与 ROS 同版本的解释器重建 venv（--python /usr/bin/python$ROS_PY_MM）"
      else
        info "venv 与 ROS 2 同为 python$ROS_PY_MM ✓"
      fi
    fi

    if [ "$CLEAN_ROS" = 1 ]; then
      info "--clean-before-ros：将删除以下构建产物（源码不删）:"
      for d in ros2_ws/build ros2_ws/install ros2_ws/log; do
        [ -e "$ROOT/$d" ] && info "    $d（$(du -sh "$ROOT/$d" 2>/dev/null | cut -f1)）"
      done
      if [ "$DRY" = 1 ]; then
        info "  [dry] rm -rf ros2_ws/build ros2_ws/install ros2_ws/log"
      else
        rm -rf "$ROOT/ros2_ws/build" "$ROOT/ros2_ws/install" "$ROOT/ros2_ws/log"
      fi
    fi

    # 旧 install 的 python 版本对不对得上（跨机器拷贝来的 install 基本都对不上）
    OLD_INST_PY="$(ls -d "$ROOT"/ros2_ws/install/unitree_go/local/lib/python3.* 2>/dev/null | head -1 | sed 's,.*/python3\([0-9]*\)\.\([0-9]*\),\1.\2,')"
    if [ -n "$OLD_INST_PY" ] && [ "$OLD_INST_PY" != "$PY_MM" ]; then
      warn "ros2_ws/install 是 python$OLD_INST_PY 构建的，当前是 python$PY_MM —— 必须重建"
      info "  建议：bash scripts/onboard_setup.sh --clean-before-ros --with-ros"
    fi

    # colcon：只有在 source 之后才在 PATH 里
    if [ "$DRY" = 1 ]; then
      info "  [dry] source $ROS_SETUP_FILE"
    else
      # ROS 的 setup.bash 里有未定义变量，source 期间别让 -e/-u 误伤
      set +e
      source "$ROS_SETUP_FILE"
      set -e
      info "ROS 环境已 source（ROS_DISTRO=${ROS_DISTRO:-?} RMW=${RMW_IMPLEMENTATION:-<未设>}）"
    fi
    if [ "$DRY" = 1 ] || command -v colcon >/dev/null 2>&1; then
      : # dry-run 假装有 colcon
    else
      warn "没有 colcon（ROS 2 的构建工具）"
      info "  修法：sudo apt install python3-colcon-common-extensions"
      die "缺 colcon"
    fi

    # 幂等：install 已存在且比源码新 -> 跳过（--force-colcon 强制重建）
    MARK="$ROOT/ros2_ws/install/unitree_go/share/unitree_go/package.xml"
    NEED_BUILD=1
    if [ "$FORCE_COLCON" = 0 ] && [ -f "$MARK" ] && [ "$OLD_INST_PY" = "$PY_MM" ]; then
      NEWER="$(find "$ROOT/ros2_ws/src/unitree_go" -newer "$MARK" -print -quit 2>/dev/null || true)"
      if [ -z "$NEWER" ]; then
        NEED_BUILD=0
        info "unitree_go 已 build 且源码没变（$MARK），跳过 colcon（--force-colcon 可强制）"
      else
        info "源码比 install 新（$NEWER），需要重新 build"
      fi
    fi
    if [ "$NEED_BUILD" = 1 ]; then
      if [ "$DRY" = 1 ]; then
        printf '  [dry] ( cd %q && colcon build --packages-select unitree_go )\n' "$ROOT/ros2_ws"
      else
        ( cd "$ROOT/ros2_ws" && colcon build --packages-select unitree_go ) \
          || die "colcon build 失败：先看上面的 rosidl/dds-idl 报错（常见原因：没装 ros-<distro>-rosidl-generator-dds-idl；或 install/ 是别的 python 版本建的，用 --clean-before-ros 再试）"
        info "colcon build 完成：$ROOT/ros2_ws/install"
      fi
    fi
  fi
fi

# ================================================================== 5) 自检
log "5/5 上机前自检"
if [ -f "$ROOT/scripts/env_check.py" ]; then
  info "调 scripts/env_check.py --target real_ros --no-network（不 ping、不发 DDS；只读）"
  if [ "$DRY" = 1 ]; then
    printf '  [dry] %q %q --target real_ros --no-network\n' "$VENV_PY" "$ROOT/scripts/env_check.py"
    info "  想连机器人一起查：$VENV_PY scripts/env_check.py --target real_ros --iface <网卡>"
  else
    RC=0
    "$VENV_PY" "$ROOT/scripts/env_check.py" --target real_ros --no-network || RC=$?
    if [ "$RC" = "0" ]; then
      info "env_check 通过（WARN 项见上面输出，逐条按'修法'处理）"
    else
      CHECK_FAIL=1
      warn "env_check 有 FAIL（退出码 $RC）：ros2 侧没齐的话真机 real_ros 一定不通，先补 ROS 2"
      info "  仿真/离线场景可以先不管 ROS 2 那一半，但别在没修的情况下上真机。"
    fi
  fi
else
  warn "没有 scripts/env_check.py，退回内置最小自检"
  if [ "$DRY" = 1 ]; then
    printf '  [dry] %q -c "import numpy, mujoco, json; ..."\n' "$VENV_PY"
  else
    "$VENV_PY" -c 'import json, os, numpy, mujoco
root = os.getcwd()
cfg = json.load(open(os.path.join(root, "config", "go2_sim.json"), encoding="utf-8"))
npz = ((cfg.get("policy_profiles") or {}).get("go2") or {}).get("npz")
print("  numpy %s / mujoco %s" % (numpy.__version__, mujoco.__version__))
print("  默认策略:", npz, "存在" if npz and os.path.isfile(os.path.join(root, npz)) else "缺失")
m = mujoco.MjModel.from_xml_path(os.path.join(root, "sim_env", "models", "go2", "scene.xml"))
print("  MuJoCo 场景可加载:", m.nbody, "body /", m.ngeom, "geom")' \
      || die "内置自检失败"
  fi
fi

# ================================================================== 收尾
log "完成。下一步（真机）:"
if [ "$WITH_ROS" = 1 ] && [ -n "$ROS_SETUP_FILE" ]; then
  info "1) 每次新开终端都要先:  source $ROS_SETUP_FILE && source ros2_ws/install/setup.bash"
else
  info "1) 还要装 ROS 2 那一半:  bash scripts/onboard_setup.sh --with-ros"
fi
info "2) 网卡（config 里默认 eth0，现场用环境变量覆盖，别改 config）:"
info "     export GO2_IFACE=<有线网卡名>     # 例: enp3s0 / eth0；ip -br addr 查"
info "     # 旧名 GO2_INTERFACE 仍兼容；两者都给以 GO2_IFACE 为准"
info "3) 只读自检:  GO2_IFACE=<网卡> .venv/bin/python scripts/env_check.py --target real_ros"
info "4) 上机跑:    GO2_IFACE=<网卡> bash scripts/real_run.sh --help"
info "5) 不要拷 .venv / ros2_ws/install 到别的机器（py 版本 + 架构不兼容，见 docs/portability.md §3）"

# 退出码：开了 --with-ros 就要求 ROS 2 那半真的能用；没开则只提示
if [ "$CHECK_FAIL" = 1 ] && [ "$WITH_ROS" = 1 ]; then
  note "自检有 FAIL 且开了 --with-ros：ROS 2 这半没齐，退出码 1"
  exit 1
fi
exit 0
