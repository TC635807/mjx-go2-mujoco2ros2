#!/usr/bin/env bash
# v6 部署侧闭环验收: 17x11 扫描 (243 obs) + 真机 L1 几何遮挡 (SimLidar+HeightMapper)
#
# 做什么:
#   1) 从 best 检查点导出 pkl -> npz (best 常优于 final, v5 实测);
#   2) 构建 "部署 body + 训练 mix 地形" 的单文件场景 (地形逐点相同);
#   3) 闭环 run 矩阵: 平地/地形 x (raycast 真感知 | nominal 盲走) x (oracle | estimator 速度);
#   4) 汇总 report.txt (前进距离/姿态/速度/tilt/健康等级)。
#
# 用法:
#   bash scripts/eval_v6_deploy.sh [ckpt]              # 默认 v6a@7.37M (交付检查点)
#   PROF=go2_v4 bash scripts/eval_v6_deploy.sh         # 用旧档验证脚本链路
set -uo pipefail
cd "$(dirname "$0")/.."
HERE="$(pwd)"
R1=/home/tc63/mujoco/mjx-go2-training
PY=/home/tc63/mujoco/.venv/bin/python
OUT="$HERE/logs/go2_v6deploy"
PROF="${PROF:-go2_v6}"
SUF="${PROF#go2_}"
export PROF
CKPT="${1:-$R1/models/go2_v6a_logs/checkpoints/000007372800}"
CKPT="$(readlink -f "$CKPT" 2>/dev/null || echo "$CKPT")"
mkdir -p "$OUT"

say() { echo "[$(date +%H:%M:%S)] $*"; }

say "===== 1/3 导出策略 (ckpt=$CKPT, profile=$PROF) ====="
PKL="$R1/policies/${SUF}_best.pkl"
NPZ="$HERE/models/policies/go2_walk_${SUF}.npz"
if [ ! -f "$NPZ" ]; then
  if [ ! -f "$PKL" ]; then
    JAX_PLATFORMS=cpu "$PY" "$R1/sim/ckpt_to_pkl.py" --ckpt "$CKPT" --out "$PKL" \
      || { say "!! ckpt->pkl 失败"; exit 1; }
  fi
  JAX_PLATFORMS=cpu "$PY" scripts/export_policy.py --pkl "$PKL" \
    --out "$NPZ" --name "go2_walk_$SUF" || { say "!! npz 导出失败"; exit 1; }
fi
say "npz = $NPZ"

say "===== 2/3 构建部署地形场景 (训练 mix 地形 + 部署 body) ====="
PYTHONPATH=src "$PY" - <<PYEOF
import sys, os
sys.path.insert(0, "scripts")
from compare_deploy_vs_train_physics import build_deploy_terrain
p = build_deploy_terrain("/tmp/go2_deploy_mix.xml",
                         train_terrain=os.path.join("$R1", "models/go2/scene_mjx_terrain_mix.xml"))
print("terrain:", p, os.path.getsize(p), "bytes")
PYEOF

run() {
  local tag="$1" scene="$2" model="$3" scan="$4" linvel="$5" dur="$6"
  local dir="$OUT/$tag"
  rm -rf "$dir"; mkdir -p "$dir"
  local margs=()
  [ -n "$model" ] && margs=(--model "$model")
  "$PY" scripts/view_policy.py --robot go2 --controller policy --scene "$scene" \
    "${margs[@]}" --policy-profile "$PROF" --duration "$dur" \
    --linvel "$linvel" --scan "$scan" --cmd 0.5 0 0 \
    --headless --plots off --record "$dir" --json "$dir/summary.json" \
    > "$dir/run.log" 2>&1
  echo "  $tag: exit=$? $(grep -c . "$dir/run.log") log lines"
}

say "===== 3/3 闭环 run 矩阵 ====="
run flat_ray_oracle   flat      ""                       raycast   oracle    15
run flat_ray_est      flat      ""                       raycast   estimator 15
run flat_nominal_est  flat      ""                       nominal   estimator 15
run mix_ray_oracle    obstacles /tmp/go2_deploy_mix.xml  raycast   oracle    8
run mix_ray_est       obstacles /tmp/go2_deploy_mix.xml  raycast   estimator 8
run mix_nominal_est   obstacles /tmp/go2_deploy_mix.xml  nominal   estimator 8

say "===== 汇总 ====="
"$PY" - "$OUT" <<'PYEOF'
import csv, json, os, sys
out = sys.argv[1]
rows = []
for tag in sorted(os.listdir(out)):
    d = os.path.join(out, tag)
    sp = os.path.join(d, "summary.json")
    fp = os.path.join(d, "frames.csv")
    if not os.path.isfile(sp):
        rows.append((tag, "NO_SUMMARY", "", "", "", "", ""))
        continue
    s = json.load(open(sp))
    fin = s.get("final", {})
    n_cmd = (s.get("controller") or {}).get("n_calls", 0)
    try:
        n = 0; sx = 0.0; vy = 0.0; vz = 0.0; tilt = 0.0
        with open(fp, newline="") as fh:
            for r in csv.DictReader(fh):
                n += 1
                sx += float(r.get("vx_body") or 0.0)
                vy += float(r.get("vy_body") or 0.0)
                vz += float(r.get("vz_body") or 0.0)
                tilt += float(r.get("tilt") or 0.0)
        n = max(n, 1)
        stat = ("%.3f/%.3f/%.3f" % (sx / n, vy / n, vz / n),
                "%.2f" % (tilt / n))
    except Exception as e:
        stat = ("NA", "NA")
    rows.append((
        tag,
        "%.2fs" % float(s.get("duration_s", 0.0)),
        "x=%.2f y=%.2f" % (float(fin.get("x", 0.0)), float(fin.get("y", 0.0))),
        "upright=%s tilt=%.1f" % (bool(fin.get("upright", False)),
                                  float(fin.get("tilt_deg", 0.0))),
        "vxyz_body(mean)=%s" % stat[0],
        "tilt_mean=%s" % stat[1],
        "n_ctrl=%d" % n_cmd,
    ))

hdr = ("tag", "dur", "final_xy", "final_pose", "body_vel", "tilt", "ctl")
w = [max(len(str(r[i])) for r in rows + [hdr]) for i in range(len(hdr))]
lines = ["  ".join(str(hdr[i]).ljust(w[i]) for i in range(len(hdr)))]
lines += ["  ".join(str(r[i]).ljust(w[i]) for i in range(len(hdr))) for r in rows]
txt = "\n".join(lines)
print(txt)
note = (
    "\n注: mix 地形行跑在**部署 CPU MuJoCo 的 hfield 接触**上, 该接触与训练 warp 不等价\n"
    "(docs/go2-delivery.md R1), 因此 mix 行只作接口链路验证, **地形通过率以 warp 口径为准**\n"
    "(384 env 评测见 mjx-go2-training/logs/go2_v6_final_matrix.tsv)。\n")
open(os.path.join(out, "report.txt"), "w", encoding="utf-8").write(
    "v6 部署侧闭环 (profile=%s)\n\n%s\n%s" % (os.environ.get("PROF", ""), txt, note))
PYEOF
say "report -> $OUT/report.txt"
