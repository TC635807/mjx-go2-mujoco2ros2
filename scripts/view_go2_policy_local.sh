#!/usr/bin/env bash
# Go2 交付策略的**可视化检查**入口。包装 scripts/view_policy.py, 把该开的开关都开对。
#
# 默认交付模型 = v33d (20.97M 步: 混合地形课程 + 70/30 混合命令 + 4 段 lr 衰减;
# 1536-768, 243 维 obs, 默认档见 config/go2_sim.json 的 policy_profiles.go2).
# 回退档: v29c (hyb2 全命令最稳) / v25b (旧交付 + 部署 port 回归参考).
# 旧模型可回滚: PROFILE=go2_v29c bash scripts/view_go2_policy.sh
#              PROFILE=go2_v25b bash scripts/view_go2_policy.sh
#
# 用法:
#   bash scripts/view_go2_policy.sh                       # 平地, 开 3D 窗口
#   bash scripts/view_go2_policy.sh --terrain hyb2        # 地形 (3x3 混合: 竖面+rough)
#   bash scripts/view_go2_policy.sh --terrain rubble      # 碎石
#   bash scripts/view_go2_policy.sh --terrain big1        # 5x5 大混合 (v23/v25 训练地形)
#   bash scripts/view_go2_policy.sh --list-terrains       # 列出全部地形
#   bash scripts/view_go2_policy.sh --duration 30
#   bash scripts/view_go2_policy.sh --headless            # 无 GUI: 终端 ANSI 仪表盘
#   bash scripts/view_go2_policy.sh --obstacles           # 旧障碍场景
#   bash scripts/view_go2_policy.sh --record logs/go2_view   # 落盘 CSV + summary.png
#   PROFILE=go2_v20 bash scripts/view_go2_policy.sh       # 回滚模型
#
# 窗口里键盘: w/s=前进速度 ±0.1 (0~1.0)  a/d=转向 ±0.2 rad/s (-1~1)
#             r=重置(地形上会重新在外圈平地出生)  空格=暂停  h=3D叠加  f=曲线  i=仪表盘  q=退出
# 3D 窗口里能看到: 187 个高度采样点(按特征着色) / 足端接触 / 速度与指令箭头 / 轨迹;
# 另外三块是 tau·高度·速度曲线、17x11 高度热图、243 维 obs 分段条、健康等级 HUD。
set -euo pipefail
cd "$(dirname "$0")/.."

# 单线程 BLAS 必须钉死: 1024-512 策略前向在默认 OpenBLAS 线程数下实测 12 ms/次(并占满 12~16 核),
# 单线程 0.21 ms/次。50 Hz 控制预算 20 ms —— 多线程小矩阵纯属浪费, 实测把观察器 RTF 从 1.0 压到 0.3。
# (python 入口里也 setdefault 了一层, 这里导出是为了兜住任何子进程。)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1

# 默认一直跑(按 Q / 关窗口退出); --duration N 指定自动退出
DUR=0
SCENE=flat
EXTRA=()
REC=""
PROFILE="${PROFILE:-go2_v33d}"
SCAN="${SCAN:-raycast}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DUR="$2"; shift 2 ;;
    --headless) EXTRA+=(--headless); shift ;;
    --obstacles) SCENE=obstacles; shift ;;
    --record) REC="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done

ARGS=(--robot go2 --controller policy --scene "$SCENE" --duration "$DUR"
      --policy-profile "$PROFILE"
      --linvel oracle          # 仿真用真值速度: 训练 obs 0:3 就是 qvel[0:3]
      --scan "$SCAN"           # v6 默认 raycast: SimLidar + HeightMapper 真感知链路
      --cmd 0.5 0 0            # 前进 0.5 m/s (训练里 vx∈[0,1.0])
      --follow)
if [[ -n "$REC" ]]; then
  ARGS+=(--record "$REC" --json "$REC/summary.json")
fi
echo "== Go2 策略闭环检查 =="
echo "  策略 : profile=$PROFILE  (档位见 config/go2_sim.json 的 policy_profiles; 默认 v33d; 回退: PROFILE=go2_v29c / go2_v25b)"
echo "  物理 : CPU MuJoCo 500Hz / 策略 50Hz / 健康+安全 100Hz(分频) / motor+PD kp=20 kd=1 / 足端硬接触(与训练一致)"
echo "  速度 : oracle(真值)   高程: $SCAN (raycast = 真机 L1 几何 + 机体自遮挡 + 高程图记忆; 900射线@15Hz)"
echo "  地形 : 默认平地 (--terrain <名字> 切地形, --list-terrains 看全部)"
echo "  指令 : 初始 vx=0.5 m/s (窗口里 W/S 调速, A/D 转向, Q 退出)"
echo "  时长 : 默认一直跑直到 Q；--duration N 自动退出"
echo "  画面 : 60 Hz 刷新(仿真时间; --sync-hz 可调) | 默认关仪表盘图像(--dashboard 开)"
if command -v xdpyinfo >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  echo "  显示 : DISPLAY=$DISPLAY (WSLg) -> 开 3D 窗口"
else
  echo "  显示 : 无 DISPLAY -> 建议加 --headless"
fi
echo

exec /home/tc63/mujoco/.venv/bin/python scripts/view_policy.py "${ARGS[@]}" "${EXTRA[@]}"
