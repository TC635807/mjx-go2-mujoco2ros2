#!/usr/bin/env bash
# 上机前打包：把本仓打成"能带走"的包，并可选做离线自检（解包 -> 编译 -> 依赖 -> 策略）。
#
# 用法:
#   bash scripts/package_for_onboard.sh                       # 打到 /tmp/go2-deploy-<时间>.tar.zst
#   bash scripts/package_for_onboard.sh --verify              # 打包 + 解到临时目录做离线自检
#   bash scripts/package_for_onboard.sh --out /tmp/go2.tgz --format gz
#   bash scripts/package_for_onboard.sh --dest wyx@192.168.123.99:/home/wyx   # 直接 rsync 推过去
#   bash scripts/package_for_onboard.sh --with-git            # 连 .git 一起带（要保留提交历史时）
#   bash scripts/package_for_onboard.sh --verify-only /tmp/go2-deploy-xxx.tar.zst
#   bash scripts/package_for_onboard.sh --dry-run             # 只统计"会打进去什么"，不写压缩包
#
# 为什么必须有这一层（不要手写 tar czf .）:
#   1) .venv/ 是 **cp313 + x86_64** 的二进制：mujoco/numpy 的扩展模块文件名形如
#      _constants.cpython-313-x86_64-linux-gnu.so，wheel tag = cp313-cp313-manylinux_2_27_x86_64。
#      搬到 arm64 或 Python 3.10 的机载电脑上 import 直接报 "cannot open shared object file"。
#   2) ros2_ws/{build,install,log}/ 是**另一台机器/容器**里构建的：build/unitree_go/CMakeCache.txt
#      里写死 CMAKE_HOME_DIRECTORY=/work/ros2_ws/src/unitree_go，install 里的 .so 也是
#      cpython-310-x86_64。目标机必须重新 colcon build，带过去只会让 CMake 找错路径。
#   3) build/ 与 .vscode/ 是 VSCode 的 CMake 产物（untracked、内含绝对路径）。
#   4) logs/、*.csv 是跑出来的数据（几十 MB），对复现没价值。
#   所以默认只带「源码 + 模型 + 策略 npz + config + docs + 脚本」。
#
# 产物:
#   <out>                    包本体（默认 /tmp/go2-deploy-<时间>.tar.zst）
#   <out>.sha256             包的 sha256（拷到目标机后先 sha256sum -c 验一遍）
#   包内 <repo>/MANIFEST.txt 文件数 / 未压缩体积 / 顶层条目分布 / 关键产物 sha256 / 排除清单
#
# 设计约定:
#   * 一律不往仓库里写东西（只写 --out 指定的位置和 $TMPDIR）；大文件不进 git。
#   * rsync 模式（--dest）不做 --delete：只增不删，避免把目标机上已有的东西删掉。
#   * 自检是**离线**的：只验证"包本身完整 + 本机解释器能编译 + 本机依赖可用"；
#     目标机的依赖必须靠 scripts/onboard_setup.sh 装（见 docs/portability.md）。
#
# 退出码: 0 = 成功；1 = 参数错误或自检失败。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$(basename "$ROOT")"
PARENT="$(dirname "$ROOT")"

usage() {
  sed -n '2,31p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
}

log()  { printf '[pack] %s\n' "$*"; }
warn() { printf '[pack][warn] %s\n' "$*" >&2; }
die()  { printf '[pack][fail] %s\n' "$*" >&2; exit 1; }

# ------------------------------------------------------------------ 参数
OUT=""
DEST=""
FORMAT=""
WITH_GIT=0
WITH_TP=0
VERIFY=0
VERIFY_ONLY=""
DRY=0
PYBIN=""

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)          usage ;;
    --out)              OUT="${2:?--out 需要路径}"; shift 2 ;;
    --dest)             DEST="${2:?--dest 需要 user@host:/path}"; shift 2 ;;
    --format)           FORMAT="${2:?--format 需要 zst|gz|none}"; shift 2 ;;
    --with-git)         WITH_GIT=1; shift ;;
    --with-third-party) WITH_TP=1; shift ;;
    --verify)           VERIFY=1; shift ;;
    --verify-only)      VERIFY_ONLY="${2:?--verify-only 需要包路径}"; shift 2 ;;
    --python)           PYBIN="${2:?--python 需要解释器路径}"; shift 2 ;;
    --dry-run)          DRY=1; shift ;;
    *)                  die "未知参数: $1（--help 看用法）" ;;
  esac
done

# 校验用的解释器：优先仓库 venv（只有它才有 mujoco/numpy），否则退回 python3
if [ -z "$PYBIN" ]; then
  if [ -x "$ROOT/.venv/bin/python" ]; then PYBIN="$ROOT/.venv/bin/python"; else PYBIN="python3"; fi
fi
if [ -x "$PYBIN" ]; then :; else command -v "$PYBIN" >/dev/null 2>&1 || die "找不到 python: $PYBIN"; fi

TMP=""
cleanup() { if [ -n "$TMP" ]; then rm -rf "$TMP"; fi; }
trap cleanup EXIT
TMP="$(mktemp -d "${TMPDIR:-/tmp}/go2pack.XXXXXX")"

# ------------------------------------------------------------------ 排除清单
# 注意：tar 的 --exclude 匹配的是 -C 之后的工作目录里的路径（也就是 $BASE/...），
# 写成 "$BASE/.venv" 这种带前缀的形式才不会误伤别处的同名文件。
EXCLUDES=(
  "--exclude=$BASE/.venv"
  "--exclude=$BASE/build"
  "--exclude=$BASE/.vscode"
  "--exclude=$BASE/.pytest_cache"
  "--exclude=$BASE/ros2_ws/build"
  "--exclude=$BASE/ros2_ws/install"
  "--exclude=$BASE/ros2_ws/log"
  "--exclude=$BASE/logs"
  "--exclude=__pycache__"
  "--exclude=*.pyc"
  "--exclude=*.pyo"
  "--exclude=*.csv"
  "--exclude=.DS_Store"
)
if [ "$WITH_GIT" != 1 ]; then EXCLUDES+=("--exclude=$BASE/.git"); fi
if [ "$WITH_TP"  != 1 ]; then EXCLUDES+=("--exclude=$BASE/third_party"); fi

# 关键产物：包里必须能对上号的东西（缺了在 MANIFEST 里标 MISSING，肉眼可见）
KEY_FILES=(
  "requirements.txt"
  "config/go2_sim.json"
  "ros2_controller/policies/go2_walk_v33d.npz"
  "ros2_controller/policies/go2_walk_v33d.json"
  "ros2_controller/policies/go2_walk_v25b.equiv.npz"
  "sim_env/models/go2/go2.xml"
  "sim_env/models/go2/scene.xml"
  "ros2_ws/src/unitree_go/package.xml"
  "ros2_ws/src/unitree_go/CMakeLists.txt"
  "scripts/run_with_ros2.sh"
  "scripts/real_run.sh"
  "scripts/env_check.py"
  "scripts/package_for_onboard.sh"
  "scripts/onboard_setup.sh"
  "docs/portability.md"
)

# ------------------------------------------------------------------ MANIFEST
# 参数: $1=未压缩 tar 路径（用来数文件），$2=输出文件
write_manifest() {
  local tarf="$1" mf="$2" nfiles usize pyver deps osname gitinfo

  pyver="$("$PYBIN" -V 2>&1)"
  deps="$("$PYBIN" -c 'import numpy, mujoco; print("numpy %s / mujoco %s" % (numpy.__version__, mujoco.__version__))' 2>/dev/null || echo "本机不可用（不影响包里内容）")"
  if [ -r /etc/os-release ]; then osname="$(. /etc/os-release; echo "${PRETTY_NAME:-unknown}")"; else osname="unknown"; fi
  if [ -d "$ROOT/.git" ]; then
    gitinfo="$(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')@$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo '?')"
    if ! git -C "$ROOT" diff --quiet 2>/dev/null; then gitinfo="$gitinfo (dirty)"; fi
  else
    gitinfo="未打包（--with-git 可带）"
  fi

  nfiles="$(tar -tf "$tarf" | grep -v '/$' | wc -l | tr -d ' ')"
  usize="$(stat -c %s "$tarf")"

  {
    echo "# MANIFEST.txt"
    echo "# 由 scripts/package_for_onboard.sh 生成；同时打进包里，目标机可据此核对。"
    echo ""
    echo "生成时间      : $(date '+%Y-%m-%d %H:%M:%S %z')"
    echo "源仓库        : $ROOT"
    echo "主机          : $(uname -srm)"
    echo "发行版        : $osname"
    echo "python(校验)  : $pyver @ $(command -v "$PYBIN" 2>/dev/null || echo "$PYBIN")"
    echo "numpy/mujoco  : $deps"
    echo "git           : $gitinfo"
    echo "打包方式      : $MODE_TAG"
    echo "文件数        : $nfiles（不含本 MANIFEST.txt；压缩包里多这一个文件）"
    echo "未压缩字节    : $usize ($(awk -v b="$usize" 'BEGIN{printf "%.1f MiB", b/1048576}'))"
    echo ""
    echo "## 排除清单（这些**故意不带**，目标机必须重新生成）"
    local e
    for e in "${EXCLUDES[@]}"; do echo "  - ${e#--exclude=}"; done
    echo ""
    echo "## 顶层条目文件数"
    tar -tf "$tarf" | grep -v '/$' | sed "s,^$BASE/,," \
      | awk -F/ 'NF>1{print $1} NF==1{print "(根文件)"}' \
      | sort | uniq -c | sort -rn | awk '{printf "  %6d  %s\n", $1, $2}'
    echo ""
    echo "## 关键产物 sha256"
    local f
    for f in "${KEY_FILES[@]}"; do
      if [ -f "$ROOT/$f" ]; then
        printf '  %s  %s\n' "$(sha256sum "$ROOT/$f" | cut -d' ' -f1)" "$f"
      else
        printf '  %-64s  %s\n' 'MISSING' "$f"
      fi
    done
    echo ""
    echo "## 目标机下一步"
    echo "  1) tar -I zstd -xf <包> -C <目标目录>   # .tar.gz 用 tar -xzf"
    echo "  2) bash scripts/onboard_setup.sh --dry-run    # 先看它要做什么"
    echo "  3) bash scripts/onboard_setup.sh --with-ros   # 真装（.venv + 依赖 + colcon build unitree_go）"
    echo "  4) GO2_IFACE=<有线网卡名> bash scripts/env_check.py"
    echo "  详见 docs/portability.md"
  } > "$mf"
}

# ------------------------------------------------------------------ 压缩
pick_format() {
  if [ -n "$FORMAT" ]; then
    case "$FORMAT" in
      zst|gz|none) echo "$FORMAT" ;;
      *) die "--format 只支持 zst|gz|none" ;;
    esac
  elif command -v zstd >/dev/null 2>&1; then
    echo zst
  else
    echo gz
  fi
}

compress_tar() {   # $1=未压缩 tar, $2=输出, $3=zst|gz|none
  local tarf="$1" out="$2" fmt="$3"
  case "$fmt" in
    zst)  command -v zstd >/dev/null 2>&1 || die "选了 zst 但没装 zstd（用 --format gz 退回 gzip）"
          zstd -q -T0 -9 -f -o "$out" "$tarf" ;;
    gz)   gzip -9 -c "$tarf" > "$out" ;;
    none) cp "$tarf" "$out" ;;
  esac
}

# ------------------------------------------------------------------ 自检
# 在解包目录（或源树）上做离线检查。$1=仓库根目录；$2=1 表示"对源树原地验"
verify_tree() {
  local src="$1" inplace="${2:-0}"
  local pass=0 fail=0
  _ok()   { printf '  PASS  %s\n' "$*"; pass=$((pass + 1)); }
  _bad()  { printf '  FAIL  %s\n' "$*"; fail=$((fail + 1)); }
  _skip() { printf '  SKIP  %s\n' "$*"; }

  [ -d "$src" ] || die "自检目录不存在: $src"
  echo "[verify] 检查目录: $src"

  # 1) MANIFEST 在不在（源树原地验时本来就没有，跳过）
  if [ "$inplace" = 1 ]; then
    _skip "MANIFEST.txt（对源树原地验，不检查）"
  elif [ -f "$src/MANIFEST.txt" ]; then
    _ok "MANIFEST.txt 存在"
  else
    _bad "MANIFEST.txt 缺失"
  fi

  # 2) 不该带的目录有没有混进来（混进来=目标机上必踩 ABI/绝对路径坑）
  if [ "$inplace" = 1 ]; then
    _skip "排除目录检查（源树本来就含 .venv 等）"
  else
    local bad_dirs="" d
    for d in .venv build .vscode logs ros2_ws/build ros2_ws/install ros2_ws/log third_party; do
      if [ -e "$src/$d" ]; then bad_dirs="$bad_dirs $d"; fi
    done
    if [ -z "$bad_dirs" ]; then
      _ok "没有混入 .venv / build / ros2_ws{/build,/install,/log} / logs / third_party"
    else
      _bad "包里混入了不该带的目录:$bad_dirs"
    fi
  fi

  # 3) 全部 .py 过一遍 py_compile（用本地解释器：只能证明语法与本地版本兼容）
  local pyout rc=0 npy
  npy="$(find "$src" -name '*.py' | wc -l | tr -d ' ')"
  pyout="$("$PYBIN" -m compileall -q "$src" 2>&1)" || rc=$?
  if [ "$rc" -eq 0 ]; then
    _ok "py_compile: $npy 个 .py 全部通过（$("$PYBIN" -V 2>&1)）"
  else
    _bad "py_compile 失败（前 15 行）:"
    printf '%s\n' "$pyout" | head -15 | sed 's/^/        /'
  fi

  # 4) 全部 .sh 过一遍 bash -n
  local nsh shbad
  nsh="$(find "$src" -name '*.sh' | wc -l | tr -d ' ')"
  shbad="$(
    find "$src" -name '*.sh' -print0 \
      | while IFS= read -r -d '' f; do bash -n "$f" >/dev/null 2>&1 || echo "$f"; done \
      | wc -l | tr -d ' '
  )"
  if [ "$shbad" = "0" ]; then
    _ok "bash -n: $nsh 个 .sh 全部通过"
  else
    _bad "bash -n: $shbad/$nsh 个 .sh 有语法错误"
  fi

  # 5) 依赖（验的是**本机**解释器；目标机依赖靠 onboard_setup.sh 装）
  local deps
  if deps="$("$PYBIN" -c 'import numpy, mujoco; print("numpy %s / mujoco %s" % (numpy.__version__, mujoco.__version__))' 2>&1)"; then
    _ok "import numpy/mujoco OK: $deps"
  else
    _bad "import numpy/mujoco 失败（本机缺依赖，不是包的问题）: $(printf '%s' "$deps" | tail -1)"
  fi

  # 6) 策略 npz 能否加载 + config 里的默认档是否在包里
  local npzout rc2=0
  npzout="$("$PYBIN" - "$src" <<'PY' 2>&1
import glob, json, os, sys
import numpy as np

root = sys.argv[1]
pol_dir = os.path.join(root, "ros2_controller", "policies")
files = sorted(glob.glob(os.path.join(pol_dir, "*.npz")))
if not files:
    print("policies 目录里没有 npz")
    raise SystemExit(1)

bad = []
for f in files:
    try:
        with np.load(f) as z:
            keys = list(z.keys())
            if not keys:
                bad.append("%s: 空 npz" % os.path.basename(f))
                continue
            for k in keys:
                if z[k].dtype == object:
                    bad.append("%s: %s 是 object dtype" % (os.path.basename(f), k))
    except Exception as exc:
        bad.append("%s: %r" % (os.path.basename(f), exc))

# 默认策略档的 npz 必须在包里，否则真机 --controller policy 起不来
with open(os.path.join(root, "config", "go2_sim.json"), encoding="utf-8") as fh:
    cfg = json.load(fh)
prof = (cfg.get("policy_profiles") or {}).get(cfg.get("robot", "go2")) or {}
default_npz = prof.get("npz") or (cfg.get("policy") or {}).get("npz")
if default_npz and not os.path.isfile(os.path.join(root, default_npz)):
    bad.append("config 的默认策略 %s 不在包里" % default_npz)

if bad:
    print("问题 %d 条:" % len(bad))
    for b in bad[:10]:
        print("  -", b)
    raise SystemExit(1)
print("%d 个 npz 全部可加载；默认策略档 = %s" % (len(files), default_npz))
PY
)" || rc2=$?
  if [ "$rc2" -eq 0 ]; then
    _ok "策略 npz: $npzout"
  else
    _bad "策略 npz 检查失败: $(printf '%s' "$npzout" | tail -3 | tr '\n' ' ')"
  fi

  # 7) 模型 + 消息源码在不在（真机 real_ros 也要 mujoco 建模型，见 docs/portability.md §2.6）
  local nmsg
  nmsg="$(find "$src/ros2_ws/src/unitree_go/msg" -name '*.msg' 2>/dev/null | wc -l | tr -d ' ')"
  if [ -f "$src/sim_env/models/go2/scene.xml" ] && [ -f "$src/sim_env/models/go2/go2.xml" ] && [ "$nmsg" -ge 10 ]; then
    _ok "模型 + 消息源码齐全（go2.xml/scene.xml + $nmsg 个 .msg）"
  else
    _bad "模型或消息源码缺失（sim_env/models/go2/{go2,scene}.xml 或 ros2_ws/src/unitree_go/msg/*.msg）"
  fi

  echo ""
  echo "[verify] 结果: $pass 项通过, $fail 项失败"
  if [ "$fail" -ne 0 ]; then return 1; fi
  return 0
}

verify_archive() {   # $1=包路径
  local pkg="$1" exdir src c
  [ -f "$pkg" ] || die "包不存在: $pkg"
  exdir="$TMP/extract"
  rm -rf "$exdir"; mkdir -p "$exdir"
  echo "[verify] 解包 -> $exdir"
  case "$pkg" in
    *.tar.zst|*.tzst) command -v zstd >/dev/null 2>&1 || die "解 .zst 需要 zstd"
                      tar -I zstd -xf "$pkg" -C "$exdir" ;;
    *.tar.gz|*.tgz)   tar -xzf "$pkg" -C "$exdir" ;;
    *.tar)            tar -xf  "$pkg" -C "$exdir" ;;
    *)                die "不认识的包扩展名: $pkg（支持 .tar.zst/.tar.gz/.tgz/.tar）" ;;
  esac
  # 顶层可能只有一个仓库目录（MANIFEST 在它里面）：找出含 config/go2_sim.json 的那层
  src=""
  if [ -f "$exdir/config/go2_sim.json" ]; then
    src="$exdir"
  else
    for c in "$exdir"/*/; do
      if [ -f "$c/config/go2_sim.json" ]; then src="$(cd "$c" && pwd)"; break; fi
    done
  fi
  [ -n "$src" ] || die "解包后找不到仓库根（没有 config/go2_sim.json）"
  verify_tree "$src" 0
}

# ------------------------------------------------------------------ 只校验已有包
if [ -n "$VERIFY_ONLY" ]; then
  verify_archive "$VERIFY_ONLY"
  log "自检通过（离线检查，不含 ROS/真机）"
  exit 0
fi

# ------------------------------------------------------------------ rsync 模式
if [ -n "$DEST" ]; then
  command -v rsync >/dev/null 2>&1 || die "rsync 模式需要 rsync（去掉 --dest 就用 tar）"
  RS=(-rlpt --info=stats1)
  local_pat=""
  for local_pat in "${EXCLUDES[@]}"; do
    local_pat="${local_pat#--exclude=}"
    RS+=("--exclude=${local_pat#"$BASE"/}")
  done
  log "rsync 模式 -> $DEST（不做 --delete）"
  if [ "$DRY" = 1 ]; then
    printf '  [dry] rsync'
    printf ' %q' "${RS[@]}" "$ROOT/" "$DEST"
    printf '\n'
    exit 0
  fi
  rsync "${RS[@]}" "$ROOT/" "$DEST"

  TARF="$TMP/pack.tar"
  log "生成 MANIFEST（要先把文件清单过一遍）"
  tar -cf "$TARF" "${EXCLUDES[@]}" -C "$PARENT" "$BASE"
  MODE_TAG="rsync -> $DEST（未做 --delete）"
  write_manifest "$TARF" "$TMP/MANIFEST.txt"
  rsync -a "$TMP/MANIFEST.txt" "$DEST/MANIFEST.txt"

  log "已推送源码 + MANIFEST.txt"
  log "未压缩体积: $(awk -v b="$(stat -c %s "$TARF")" 'BEGIN{printf "%.1f MiB", b/1048576}')（不含被排除项）"
  if [ "$VERIFY" = 1 ]; then
    log "rsync 模式的自检在**本机源树**上做（远端离线验不了，见 docs/portability.md）"
    verify_tree "$ROOT" 1 || die "自检失败：先在本机修好再推"
  fi
  exit 0
fi

# ------------------------------------------------------------------ tar 模式
FMT="$(pick_format)"
case "$FMT" in
  zst)  EXT="tar.zst" ;;
  gz)   EXT="tar.gz" ;;
  none) EXT="tar" ;;
esac
if [ -z "$OUT" ]; then OUT="/tmp/go2-deploy-$(date '+%Y%m%d-%H%M%S').$EXT"; fi
mkdir -p "$(dirname "$OUT")"

TARF="$TMP/pack.tar"
log "打包源: $ROOT"
log "排除   : ${#EXCLUDES[@]} 条规则（.venv / build / .vscode / ros2_ws{/build,/install,/log} / logs / __pycache__ / *.pyc / *.csv 等，--help 看全量）"

t0=${SECONDS:-0}
tar -cf "$TARF" "${EXCLUDES[@]}" -C "$PARENT" "$BASE"

# MANIFEST 打进包内 $BASE/MANIFEST.txt（用 --transform 改成员名，避免解包后根目录多一个文件）
MODE_TAG="tar ($FMT)"
write_manifest "$TARF" "$TMP/MANIFEST.txt"
tar -rf "$TARF" -C "$TMP" --transform "s,^MANIFEST.txt\$,${BASE}/MANIFEST.txt," MANIFEST.txt

NFILES="$(tar -tf "$TARF" | grep -v '/$' | wc -l | tr -d ' ')"
TAR_BYTES="$(stat -c %s "$TARF")"
SRC_BYTES="$(du -sb "$ROOT" 2>/dev/null | cut -f1)"

if [ "$DRY" = 1 ]; then
  log "dry-run：会打 $NFILES 个文件，未压缩 $(awk -v b="$TAR_BYTES" 'BEGIN{printf "%.1f MiB", b/1048576}')；仓库整树 $(awk -v b="$SRC_BYTES" 'BEGIN{printf "%.1f MiB", b/1048576}')"
  log "dry-run：不写压缩包（去掉 --dry-run 才落到 $OUT）"
  exit 0
fi

log "压缩 ($FMT) -> $OUT"
compress_tar "$TARF" "$OUT" "$FMT"
OUT_BYTES="$(stat -c %s "$OUT")"
( cd "$(dirname "$OUT")" && sha256sum "$(basename "$OUT")" > "$(basename "$OUT").sha256" )

echo ""
log "===== 打包完成 ====="
printf '  文件数        : %s\n' "$NFILES"
printf '  仓库整树      : %s（含 .venv/build/ros2_ws 产物）\n' "$(awk -v b="$SRC_BYTES" 'BEGIN{printf "%.1f MiB", b/1048576}')"
printf '  未压缩 tar    : %s\n' "$(awk -v b="$TAR_BYTES" 'BEGIN{printf "%.1f MiB", b/1048576}')"
printf '  压缩包        : %s  (%s)\n' "$(awk -v b="$OUT_BYTES" 'BEGIN{printf "%.1f MiB", b/1048576}')" "$OUT"
printf '  相对整树体积  : %s\n' "$(awk -v a="$SRC_BYTES" -v b="$OUT_BYTES" 'BEGIN{printf "%.0f%%", b*100/a}')"
printf '  耗时          : %ss\n' "$((${SECONDS:-0} - t0))"
printf '  sha256        : %s\n' "$(cat "$OUT.sha256")"
printf '  MANIFEST      : 包内 %s/MANIFEST.txt\n' "$BASE"

if [ "$VERIFY" = 1 ]; then
  echo ""
  verify_archive "$OUT" || die "自检失败：这个包别带上机"
  log "自检通过（离线：编译/脚本语法/依赖/策略 npz；不含 ROS 与真机）"
fi

log "下一步: tar -I zstd -xf $(basename "$OUT") -C <目标目录> && bash scripts/onboard_setup.sh --dry-run"
