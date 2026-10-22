#!/usr/bin/env python3
"""生成改进版系统架构图（可在 Drawnix 里继续编辑）。

产物:
    系统架构图.drawnix           项目根目录，原图同款格式，直接拖进 Drawnix 打开
    docs/系统架构图.drawnix      同一份（保留历史路径）
    docs/assets/系统架构图.png   预览（上色便于阅读；drawnix 本身不上色）
    /mnt/d/Downloads/系统架构图.drawnix    （Windows 侧可直接打开）

改布局只需动下面的 BOXES / ARROWS。
"""

from __future__ import annotations

import json
import math
import os
import shutil
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WIN_DIR = "/mnt/d/Downloads"

# ---------------------------------------------------------------- 布局常量
SHARED_X, SHARED_W = 200, 800
LEFT_X, LEFT_W = 80, 470
RIGHT_X, RIGHT_W = 590, 470
PANEL_X, PANEL_W = 1120, 440
ROW_Y = [40, 170, 350, 530, 720, 910, 1090]
ROW_H = [100, 150, 160, 130, 170, 150, 150]
W, H = 1620, 1580

# (id, 文本, x, y, w, h, 字号)
BOXES: List[Tuple[str, str, int, int, int, int, int]] = [
    ("r0_task",
     "① 目标 / 指令层\n"
     "速度指令 (vx, vy, ωz)：config 注入；/cmd_vel 订阅待接\n"
     "目标高度 / 姿态 / 步态参数",
     SHARED_X, ROW_Y[0], 440, ROW_H[0], 14),
    ("r0_train",
     "⑧ 训练与工具链（离线，与实时回路解耦）\n"
     "mjx-go2-training：MJX + Brax PPO 批量训练（GPU）\n"
     "export_policy.py → 纯 numpy npz（与 jax 逐位一致 2.7e-15）\n"
     "（训练仓）sim/warmstart_bignet.py：网络宽度精确等价扩展",
     680, ROW_Y[0], 380, ROW_H[0], 12),

    ("r1_input",
     "② 统一信息输入层（obs.py · estimator.py · health.py · perception.py）\n"
     "obs.py：91 维观测 = 48 base + 35 height scan + 8 gait phase；与训练 _get_obs 逐段对齐 9/9（max diff = 0）\n"
     "estimator.py：腿式里程计，补真机没有的基座线速度（FK 与 MuJoCo 对齐 3.9e-16 m，8 ms 收敛）\n"
     "health.py：每路 {value, stamp, valid, σ} → L0 正常 / L1 降级 / L2 保护 / L3 急停（迟滞 + L3 锁存）\n"
     "perception.py：点云 → 高程图 → 机体系 35 点高度采样（仿真与真机同一份代码）",
     SHARED_X, ROW_Y[1], 800, ROW_H[1], 13),

    ("r2_ctrl",
     "③ 控制层（50 Hz）—— 只认 LowState，不认环境（controllers.py）\n"
     "policy：导出的 brax PPO，纯 numpy 推理（12 µs/次）；动作 = 12 维关节绝对位置目标\n"
     "        → 填 LowCmd 的 q，kp = 20 / kd = 1 / τ = 0；先按训练机型的关节范围夹紧\n"
     "        交接前用 prepose 把关节摆到训练 home（Go2 的 home 与 Go2 不同，直接交接会摔）\n"
     "stand / zero / lqr_stub：同一 update(state, t) → LowCmd 接口，换一个字符串即切换\n"
     "probe_policy_controller.py：单进程闭环，用来把「控制器实现问题」和「传输层问题」分开",
     SHARED_X, ROW_Y[2], 860, ROW_H[2], 12),

    ("r3_out",
     "④ 统一输出层\n"
     "LowCmd：12 × {mode, q, dq, τ, kp, kd}\n"
     "话题 rt/lowcmd（rmw_cyclonedds_cpp）+ unitree_go/msg/LowCmd\n"
     "与真机同一对话题、同一消息类型",
     LEFT_X, ROW_Y[3], LEFT_W, ROW_H[3], 12),

    ("r3_safe",
     "⑦ 安全层（safety.py，独立于控制层，不依赖 ROS2）\n"
     "SafetySupervisor：track → crouch → damping → estop\n"
     "限幅 · 斜率限制 · 看门狗（仿真 60 ms / 真机 20 ms）\n"
     "L3 卸力并锁存，需显式 reset 才解除\n"
     "任何节点失联 → 确定性降级，绝不静默；真机建议下沉到 MCU 兜底",
     RIGHT_X, ROW_Y[3], RIGHT_W, ROW_H[3], 12),

    ("r4_sim",
     "⑤ 仿真侧（domain 1 + lo，robot.py + node.py）\n"
     "MuJoCo 单环境（CPU）500 Hz；实测 0.023 ms/步\n"
     "机型：sim_env/models/go2（真机同款）\n"
     "\n"
     "与训练物理逐步等价（600 步 max|Δq| = 3.1e-15）\n"
     "电机语义：τ = kp(q_des−q) + kd(dq_des−dq) + τ_ff → 限幅 → 关节动力学",
     LEFT_X, ROW_Y[4], LEFT_W, ROW_H[4], 11),

    ("r4_real",
     "⑤ 真机侧（domain 0 + eth0）\n"
     "Go2（带激光雷达）\n"
     "宇树电机 MCU：500 Hz 电流环，\n"
     "关节状态与力矩限幅由 MCU 负责\n"
     "上电前必须释放自带运控服务（MotionSwitcher）\n"
     "真机 rt/lowcmd 的 CRC32 待补（需 CDR 序列化字节）",
     RIGHT_X, ROW_Y[4], RIGHT_W, ROW_H[4], 12),

    ("r5_sim_state",
     "⑥ 仿真回传\n"
     "关节 / IMU / 足端力 → LowState（684 B，CRC32 与宇树参考实现逐位一致）\n"
     "模拟雷达 → 点云",
     LEFT_X, ROW_Y[5], LEFT_W, ROW_H[5], 12),
    ("r5_real_state",
     "⑥ 真机回传\n"
     "关节 / IMU → LowState\n"
     "真雷达 → 点云（PointCloud2 订阅待接）\n"
     "实测收包率 49% → 98.6%（spin_once 抽干队列）",
     RIGHT_X, ROW_Y[5], RIGHT_W, ROW_H[5], 12),

    ("r5_obs",
     "⑨ 可视化观察器（scripts/view_policy.py + sim_env/viz/，只读旁路，不参与回路）\n"
     "两个窗口：MuJoCo 窗口只放机器人 + 3D 叠加（35 个高度采样点按特征值着色 / 足端接触 / 速度估计与真值箭头 / 轨迹 / 小 HUD）；\n"
     "数据窗口 = matplotlib/QtAgg 13×7.5 英寸 6 宫格（力矩 · 高度倾斜 · 速度估计vs真值 · 高度扫描 · 步态相位 · 动作）\n"
     "无图形界面时退化成终端 ANSI 仪表盘；--snapshot 离屏出带叠加的 PNG。实测 RTF 1.00 / sync 中位 2.4 ms",
     LEFT_X, ROW_Y[6], 980, ROW_H[6], 12),

    ("s1", "域差清单（config/domain_gap.json，17 条，每条带 文件:行 证据）\n"
     "质量 trunk 5.204 → 6.921 kg（+33%）· 膝力矩 ±35.55 → ±45.43 N·m\n"
     "关节 damping 1.0 → 0.1 · home 姿态 · 接触参数 · 传感器噪声与延迟\n"
     "height_scan 源：hfield 真值 → 深度相机（有盲区/噪声）\n"
     "每项标注 closed / mitigated / open + 缓解措施",
     PANEL_X, 40, PANEL_W, 250, 12),
    ("s2", "实时性分工\n"
     "开发机 = Win + WSL2，非实时（偶发 5 ms 毛刺）\n"
     "ROS2 控制层（50 Hz）= 软实时，实测 p95 2~5 ms\n"
     "500 Hz 硬实时由电机 MCU 负责\n"
     "500 Hz 通信那一拍全套开销 ≈ 204 µs（占 2 ms 的 10%）\n"
     "真机若要在 PC 发低层指令：绑核 / RT 内核 / C++ 节点",
     PANEL_X, 320, PANEL_W, 230, 12),
    ("s3", "参考开源\n"
     "unitree_mujoco（官方仿真器，DDS 直连）\n"
     "quadruped_ros2_control（ros2-control + 真机 Go2）\n"
     "mujoco_ros2_control（ros2_control 硬件接口 + 传感器）\n"
     "legged_control（ROS2 + MPC/WBC）\n"
     "walk-these-ways（Go2 RL sim2real）\n"
     "legged_gym / IsaacLab（height scan 与域随机化配方）",
     PANEL_X, 580, PANEL_W, 230, 12),
    ("s4", "关键约定\n"
     "① 回路里只保留一种 DDS 客户端（只用 ROS2），\n"
     "    避免 ROS2 RMW 与 unitree_sdk2 冲突\n"
     "② 仿真电机节点与真机 MCU 同一接口语义\n"
     "③ 输入层缺信息 → 显式降级，禁止静默补零\n"
     "④ 安全层不放进控制层（控制器崩了仍要能卸力）\n"
     "⑤ obs 构造与训练共用同一份定义（逐段对账过）\n"
     "⑥ spin_once 必须抽干队列，否则延迟被 QoS 深度顶死",
     PANEL_X, 840, PANEL_W, 280, 12),
    ("s5", "实测结果（WSL2 / Python 3.14 / MuJoCo 3.12 / ROS 2 Lyrical）\n"
     "策略前向 numpy vs jax：2.7e-15　·　obs vs 训练环境：9/9 段 max diff = 0\n"
     "Go2 力矩模型 vs 位置模型：600 步 max|Δq| = 3.1e-15\n"
     "ROS2 两进程跑策略（Go2）：5.8 m / 12 s（指令 0.5 m/s），倾斜 3.1°，健康层全程 L0\n"
     "Go2（未重训）：1.2 m / 12 s，能站能走但明显退化 → 真机必须用 Go2 重训的策略\n"
     "自测 12/12 测试文件通过　·　verify.py sim_udp / sim_ros 各 16/16",
     PANEL_X, 1150, PANEL_W, 380, 12),
]

# (源 box, 源连接, 目标 box, 目标连接, 预览用的折线航点[可选])
ARROWS: List[Tuple[str, Tuple[float, float], str, Tuple[float, float], Optional[List[Tuple[float, float]]]]] = [
    ("r0_task", (0.5, 1.0), "r1_input", (0.25, 0.0), None),
    ("r0_train", (1.0, 0.5), "r2_ctrl", (1.0, 0.5), None),          # 模型部署
    ("r1_input", (0.5, 1.0), "r2_ctrl", (0.5, 0.0), None),
    ("r2_ctrl", (0.25, 1.0), "r3_out", (0.5, 0.0), None),
    ("r2_ctrl", (0.75, 1.0), "r3_safe", (0.5, 0.0), None),
    ("r3_safe", (0.0, 0.5), "r3_out", (1.0, 0.5), None),            # 安全层可改写 LowCmd
    ("r3_out", (0.5, 1.0), "r4_sim", (0.5, 0.0), None),
    ("r3_out", (1.0, 0.5), "r4_real", (0.0, 0.5), None),
    ("r4_sim", (0.5, 1.0), "r5_sim_state", (0.5, 0.0), None),
    ("r4_real", (0.5, 1.0), "r5_real_state", (0.5, 0.0), None),
    ("r5_sim_state", (0.0, 0.5), "r1_input", (0.0, 0.5),            # 闭环：仿真状态回流
     [(0.0, 0.5), (-0.10, 0.5), (-0.10, 0.0), (0.0, 0.0)]),
    ("r5_real_state", (1.0, 0.5), "r1_input", (1.0, 0.5),           # 闭环：真机状态回流
     [(1.0, 0.5), (1.10, 0.5), (1.10, 0.0), (1.0, 0.0)]),
    ("r5_sim_state", (0.7, 1.0), "r5_obs", (0.35, 0.0), None),      # 观察器只读旁路
]


PALETTE = {
    "r0": ("#e8f0fe", "#4a76c8"), "r1": ("#e6f4ea", "#2e7d4f"), "r2": ("#fff4e5", "#c77700"),
    "r3": ("#f3e8fd", "#7b3fbf"), "r4": ("#fde8e8", "#c0392b"), "r5": ("#e8f7fd", "#1f7a9c"),
    "s": ("#f1f1f1", "#666666")
}


def _by_id(bid: str):
    for b in BOXES:
        if b[0] == bid:
            return b
    raise KeyError(bid)


def _pt(b, conn) -> List[float]:
    return [b[2] + conn[0] * b[4], b[3] + conn[1] * b[5]]


def make_drawnix() -> Dict:
    els: List[Dict] = []
    for bid, text, x, y, w, h, _fs in BOXES:
        els.append({
            "id": bid, "type": "geometry", "shape": "rectangle", "angle": 0, "opacity": 1,
            "text": {"children": [{"text": text}], "type": "paragraph", "align": "center"},
            "points": [[x, y], [x + w, y + h]], "strokeWidth": 2,
        })
    for i, (sid, sconn, tid, tconn, _route) in enumerate(ARROWS):
        p0, p1 = _pt(_by_id(sid), sconn), _pt(_by_id(tid), tconn)
        pts = [p0, p1] if (abs(p0[0] - p1[0]) < 2 or abs(p0[1] - p1[1]) < 2) else [p0, [p0[0], p1[1]], p1]
        els.append({
            "id": "a%02d" % i, "type": "arrow-line",
            "shape": "elbow",     # 原图里 arrow-line 只有 elbow，保持一致避免 Drawnix 不认
            "source": {"marker": "none", "connection": list(sconn), "boundId": sid},
            "texts": [],
            "target": {"marker": "arrow", "connection": list(tconn), "boundId": tid},
            "opacity": 1, "points": pts, "strokeWidth": 2,
        })
    return {"type": "drawnix", "version": 1, "source": "web", "elements": els,
            "viewport": {"zoom": 0.72, "origination": [20, 20]},
            "theme": {"themeColorMode": "default"}}


def render_png(path: str) -> None:
    from PIL import Image, ImageDraw, ImageFont

    img = Image.new("RGB", (W, H), "#ffffff")
    d = ImageDraw.Draw(img)
    cache: Dict[int, object] = {}

    def font(size: int):
        if size not in cache:
            cache[size] = _load_font(size)
        return cache[size]

    for bid, text, x, y, w, h, fs in BOXES:
        key = "s" if bid.startswith("s") else bid[:2]
        fill, stroke = PALETTE[key]
        d.rounded_rectangle([x, y, x + w, y + h], radius=9, fill=fill, outline=stroke, width=2)
        d.rounded_rectangle([x + 3, y + 3, x + w - 3, y + h - 3], radius=9, outline="#ffffff", width=1)
        _center_text(d, text, x, y, w, h, font(fs), "#1b1b1b")
    for sid, sconn, tid, tconn, route in ARROWS:
        sb, tb = _by_id(sid), _by_id(tid)
        if route:
            p0, p1 = _pt(sb, sconn), _pt(tb, tconn)
            pts = []
            for u, v in route:
                pts.append([sb[2] + u * sb[4] if v in (0.5,) else 0, 0])
            # route 是相对源框的比例；这里直接换成绝对坐标
            pts = [[sb[2] + u * sb[4], sb[3] + v * sb[5]] for u, v in route]
            x0, y0 = _pt(sb, sconn)
            pts[0] = [x0, y0]
            x1, y1 = _pt(tb, tconn)
            pts[-1] = [x1, y1]
        else:
            p0, p1 = _pt(sb, sconn), _pt(tb, tconn)
            pts = [p0, p1] if (abs(p0[0] - p1[0]) < 2 or abs(p0[1] - p1[1]) < 2) else [p0, [p0[0], p1[1]], p1]
        _draw_arrow(d, pts, "#7a7a7a")
    img.save(path)
    print("预览:", path, img.size)


def _load_font(size: int):
    from PIL import ImageFont

    for p in ["/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def _tokenize(line: str) -> List[str]:
    """把一行拆成"不可再分的块"：ASCII 连续片段整体保留，中文按字，空格单独成块。

    这样 unitree_go/msg/LowCmd、2~5 ms、ros2-control 这类词就不会被从中间劈开。
    """
    toks: List[str] = []
    buf = ""
    for ch in line:
        if ch == " ":
            if buf:
                toks.append(buf); buf = ""
            toks.append(" ")
        elif ord(ch) < 128:
            buf += ch
        else:
            if buf:
                toks.append(buf); buf = ""
            toks.append(ch)
    if buf:
        toks.append(buf)
    return toks


def _wrap(d, text: str, font, max_px: float) -> List[str]:
    lines: List[str] = []
    for raw in text.split("\n"):
        cur = ""
        for tk in _tokenize(raw):
            cand = cur + tk
            if cur and d.textlength(cand, font=font) > max_px:
                lines.append(cur.rstrip())
                cur = "" if tk == " " else tk
            elif not cur and d.textlength(tk, font=font) > max_px:
                # 单个块就超宽（超长英文/路径）：只能按字符切
                piece = ""
                for ch in tk:
                    if piece and d.textlength(piece + ch, font=font) > max_px:
                        lines.append(piece); piece = ch
                    else:
                        piece += ch
                cur = piece
            else:
                cur = cand
        lines.append(cur.rstrip())
    return lines


def _center_text(d, text, x, y, w, h, font, color) -> None:
    lines: List[str] = _wrap(d, text, font, w - 26)
    lh = font.size + 5
    cy = y + (h - lh * len(lines)) / 2
    for i, ln in enumerate(lines):
        wpx = d.textlength(ln, font=font)
        d.text((x + (w - wpx) / 2, cy + i * lh), ln, font=font, fill=color)


def _draw_arrow(d, pts, color) -> None:
    for i in range(len(pts) - 1):
        d.line([tuple(pts[i]), tuple(pts[i + 1])], fill=color, width=2)
    (x0, y0), (x1, y1) = pts[-2], pts[-1]
    ang = math.atan2(y1 - y0, x1 - x0)
    L, Wd = 13, 5.5
    d.polygon([(x1, y1),
               (x1 - L * math.cos(ang) + Wd * math.sin(ang), y1 - L * math.sin(ang) - Wd * math.cos(ang)),
               (x1 - L * math.cos(ang) - Wd * math.sin(ang), y1 - L * math.sin(ang) + Wd * math.cos(ang))],
              fill=color)


def main() -> int:
    doc = make_drawnix()
    text = json.dumps(doc, ensure_ascii=False, indent=1)
    outs = [os.path.join(ROOT, "系统架构图.drawnix"),          # 项目根目录（用户要的位置）
            os.path.join(ROOT, "docs", "系统架构图.drawnix")]  # docs 里保留一份，历史链接不失效
    for out in outs:
        with open(out, "w", encoding="utf-8") as f:
            f.write(text)
        print("drawnix:", out, "元素数:", len(doc["elements"]), "%d B" % len(text.encode("utf-8")))
    os.makedirs(os.path.join(ROOT, "docs", "assets"), exist_ok=True)
    render_png(os.path.join(ROOT, "docs", "assets", "系统架构图.png"))
    if os.path.isdir("/mnt/d/Downloads"):
        for name in ("系统架构图.drawnix", "系统架构图_改进版.drawnix"):
            dst = os.path.join("/mnt/d/Downloads", name)
            shutil.copyfile(outs[0], dst)
            print("Windows 副本:", dst)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
