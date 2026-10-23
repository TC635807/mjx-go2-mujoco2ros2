#!/usr/bin/env python3
"""生成项目架构总览图（PNG 预览 + Drawnix 可编辑源文件）。

产物（都在 docs/ 下，README 只引用这两份）:
    docs/assets/系统架构图.png   —— 便于阅读的上色预览
    docs/系统架构图.drawnix      —— 可拖进 Drawnix 继续编辑

图的内容分左右两栏：
    左栏 = 运行时闭环（配置/离线产物 → 控制层 50 Hz → 信号与 IO 层 → 仿真/真机 500 Hz → LowState 回流）
    右栏 = 仓库目录结构、target/IO 矩阵、关键契约、验证数据

维护方式：只改下面的 GROUPS / BOXES / EDGES，然后重新运行
    .venv/bin/python scripts/make_architecture_diagram.py
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 2100, 1500

# ---------------------------------------------------------------- 配色
PALETTE: Dict[str, Tuple[str, str]] = {
    "offline": ("#eaf2fe", "#4a76c8"),
    "tool": ("#eef3f8", "#5b6b7f"),
    "ctrl": ("#fff4e5", "#c77700"),
    "io": ("#e6f4ea", "#2e7d4f"),
    "sim": ("#f3e8fd", "#7b3fbf"),
    "real": ("#fde8e8", "#c0392b"),
    "loop": ("#e8f7fd", "#1f7a9c"),
    "panel": ("#f7f8fa", "#8a949e"),
}
ARROW_COLOR = "#6b7280"

# ---------------------------------------------------------------- 版面
TITLE_Y = 18
LEFT_X, LEFT_W = 30, 1482
RIGHT_X, RIGHT_W = 1548, 522

# 分组框：只画背景 + 标题，子框单独画在上面
GROUPS: List[dict] = [
    dict(id="g_ctrl", x=LEFT_X, y=232, w=LEFT_W, h=268,
         title="③ 控制层 ros2_controller/ · go2_controller 节点 · 50 Hz（仿真 / 真机同一份代码）", color="ctrl"),
    dict(id="g_io", x=LEFT_X, y=532, w=LEFT_W, h=238,
         title="④ 信号 / 感知 / IO 层 signal_io/（go2_perception + transport；控制器只认话题，不认环境）", color="io"),
    dict(id="g_loop", x=LEFT_X, y=1056, w=LEFT_W, h=152,
         title="⑦ LowState 回流（闭环）", color="loop"),
    dict(id="g_run", x=LEFT_X, y=1225, w=LEFT_W, h=225,
         title="⑧ 运行入口与验收（常用命令）", color="tool"),
]

# 普通框
BOXES: List[dict] = [
    # ---- 顶部：配置 / 离线产物 / 工具
    dict(id="a1", x=30, y=80, w=770, h=132, color="offline",
         title="① 配置 & 训练产物（离线）",
         lines=[
             "config/go2_sim.json：robot · io_profiles · 26 档 policy_profiles",
             "config/domain_gap.json：8 条训练↔真机域差 + SignalDegrader",
             "mjx-go2-training → ros2_controller/policies/：npz + json + .equiv.npz",
             "mjx-go2-training → sim_env/models/go2/：MJCF + 31 个地形 + terrains.json",
         ]),
    dict(id="a2", x=820, y=80, w=692, h=132, color="tool",
         title="② 入口 · 验收 · 观察（scripts/ + tests/）",
         lines=[
             "ros2 launch ros2_ws/launch/{sim,real}.launch.py（ROS 2 节点图）",
             "sim_env/run_sim.py / ros2_controller/run_controller.py（--target 切仿真/真机）",
             "scripts/verify.py = 16 项验收；tests/run_tests.py = 14 个纯 Python 测试",
             "view_go2_policy.sh → view_policy.py；eval_terrain_matrix.py；export_policy.py",
         ]),

    # ---- 控制层
    dict(id="obs", x=44, y=282, w=270, h=198, color="ctrl",
         title="obs.py",
         lines=["243 维行走观测", "48 本体 + 187 高度扫描", "+ 8 trot 相位",
                "与训练 _get_obs 逐段对账"]),
    dict(id="policy", x=338, y=282, w=248, h=198, color="ctrl",
         title="policy.py",
         lines=["纯 numpy MLP", "默认 1536-768", "推理 0.70 ms（p99 1.45）",
                "输出 12 维关节位置目标"]),
    dict(id="controller", x=610, y=282, w=252, h=198, color="ctrl",
         title="controller.py",
         lines=["Zero / Stand / Policy", "WbcLqrStub", "统一接口",
                "update(state,t) → LowCmd"]),
    dict(id="health", x=886, y=282, w=232, h=198, color="ctrl",
         title="health.py",
         lines=["L0 正常", "L1 降级 / L2 保护", "L3 急停（锁存）",
                "迟滞 + 逐级回落"]),
    dict(id="safety", x=1142, y=282, w=356, h=198, color="ctrl",
         title="safety.py",
         lines=["SafetyLimiter：限幅 · 斜率 · 看门狗",
                "SafetySupervisor：", "track → crouch → damping → estop",
                "真机 20 ms / 仿真 60 ms · L3 卸力"]),

    # ---- 信号与 IO 层
    dict(id="msg", x=44, y=582, w=315, h=82, color="io",
         title="msg.py",
         lines=["LowCmd 524 B / LowState 684 B", "CRC32 与宇树 crc32_core 逐位一致",
                "20 个电机槽位（Go2 用前 12）"]),
    dict(id="node", x=371, y=582, w=360, h=82, color="io",
         title="node.py",
         lines=["SimNode / ControllerNode / Pacer", "SimNode：500 Hz 物理 + 发 LowState",
                "ControllerNode：50 Hz 控制 + 500 Hz 保持 + 健康/安全"]),
    dict(id="cfg", x=743, y=582, w=280, h=82, color="io",
         title="config.py",
         lines=["target / io_profiles 切换", "apply_io_env()：domain / 网卡 / RMW",
                "load_config()：robot + policy profile"]),
    dict(id="sense", x=1035, y=582, w=462, h=82, color="io",
         title="estimator.py / perception.py / domain_gap.py",
         lines=["腿式里程计补真机没有的基座线速度",
                "点云 → 高程图 → 187 点高度扫描",
                "延迟 / 丢包 / 噪声 / 偏置注入"]),
    dict(id="transport", x=44, y=676, w=1453, h=78, color="io",
         title="transport/ — 同一 Transport 接口，4 个可互换后端",
         lines=["loopback（同进程） · udp（双进程零依赖） · ros2（rclpy + rmw_cyclonedds_cpp） · dds（unitree_sdk2py；Python 3.14 不可用）",
                "仿真数据源 go2_sim_sensors → /go2/odom + /go2/lidar/points；真机固件 DDS 直接给 /lowstate、L1 给 /utlidar/cloud"]),

    # ---- 端点
    dict(id="sim", x=30, y=790, w=730, h=250, color="sim",
         title="⑤ 仿真端 sim_env/ · 500 Hz",
         lines=["run_sim.py --publish-sensors → SimNode → robot.py:Go2Sim（MuJoCo 单环境）",
                "sim_dt = 2 ms · PD：τ = kp(q_des−q) + kd(0−dq) · 力矩限幅",
                "发布 /lowstate（500 Hz）+ /go2/odom（50 Hz）+ /go2/lidar/points（15 Hz）",
                "models/go2/：MJCF + 31 地形；viz/ 只读旁路"]),
    dict(id="real", x=782, y=790, w=730, h=250, color="real",
         title="⑥ 真机端 real_env/ + ros2_ws/ · 500 Hz",
         lines=["Go2 低层电机通道 dt = 2 ms（/lowstate / /lowcmd）",
                "真机固件 DDS → /lowstate（无需自写 bridge）；L1 → /utlidar/cloud（frame=utlidar_lidar）",
                "target = real_ros（ROS 2）；domain 0 + 有线网卡；real_sdk 当前 Python 3.14 不可用",
                "上电前 L2+R2 / ReleaseMode 释放 sport mode；控制器与仿真同一份代码"]),
    dict(id="loop", x=44, y=1096, w=1453, h=98, color="loop",
         title="",
         lines=["IMU + 12 关节 + 足端力 + tick → obs / health / safety；仿真侧另有 qvel / geom_xpos 真值",
                "仿真回环：MuJoCo → LowState → 控制层；真机回环：Go2 MCU → /lowstate → 控制层",
                "仿真侧看门狗 60 ms；真机侧 HealthMonitor 断流分级 40/80/250 ms → 阻尼/急停；SignalDegrader 注入域差压测降级路径"]),

    # ---- 运行入口与验收（4 列）
    dict(id="run_view", x=44, y=1276, w=350, h=155, color="tool",
         title="观察器 go2_viewer（ROS 2 节点，可选）",
         lines=["scripts/view_go2_policy.sh → 自动拉起节点图 + viewer",
                "--terrain wide_h050 / mixed1 / rubble",
                "--headless / --record / --local（旧单进程）"]),
    dict(id="run_verify", x=404, y=1276, w=340, h=155, color="tool",
         title="消息 / 站立 / 实时性",
         lines=["python scripts/verify.py --target sim_udp",
                "16/16；--quick 只跑单进程",
                "python tests/run_tests.py → 14/14"]),
    dict(id="run_parity", x=754, y=1276, w=390, h=155, color="tool",
         title="训练 ↔ 部署对账",
         lines=["verify_train_deploy_obs.py：obs 逐段 max|diff| = 0",
                "verify_train_deploy_parity.py：PD 严格等价",
                "动作 jax vs numpy ≤ 1.15e-6"]),
    dict(id="run_ros2", x=1154, y=1276, w=344, h=155, color="tool",
         title="ROS 2 双进程 / 真机",
         lines=["ros2 launch ros2_ws/launch/sim.launch.py",
                "ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py",
                "详见 docs/ros2-setup.md / real_env/README.md"]),

    # ---- 右栏
    dict(id="tree", x=RIGHT_X, y=80, w=RIGHT_W, h=660, color="panel", align="left",
         title="仓库结构",
         lines=[
             "mjx-go2-mujoco2ros2/",
             "├─ ros2_controller/      50 Hz 控制",
             "│  ├─ run_controller.py  入口",
             "│  ├─ controller.py      Zero/Stand/Policy",
             "│  ├─ obs.py             243 维观测",
             "│  ├─ policy.py          纯 numpy 前向",
             "│  ├─ health.py          L0–L3",
             "│  ├─ safety.py          限幅/斜率/看门狗",
             "│  └─ policies/          26 档 npz",
             "├─ signal_io/            信号 & IO",
             "│  ├─ msg.py             LowCmd/LowState",
             "│  ├─ config.py          target/io_profiles",
             "│  ├─ node.py            SimNode/ControllerNode",
             "│  ├─ transport/         local/udp/ros2/dds",
             "│  ├─ estimator.py       腿式里程计",
             "│  ├─ perception.py      高程图/扫描",
             "│  ├─ perception_node.py ROS 2 感知节点",
             "│  ├─ ros2_sensors.py    仿真 odom/点云",
             "│  └─ domain_gap.py      域差注入",
             "├─ sim_env/              仿真 500 Hz",
             "│  ├─ run_sim.py         仿真入口",
             "│  ├─ robot.py           Go2Sim",
             "│  ├─ models/go2/        MJCF + 31 地形",
             "│  └─ viz/               3D/曲线/面板/录制",
             "├─ real_env/             真机部署指南",
             "├─ config/               go2_sim.json",
             "│                        domain_gap.json",
             "├─ scripts/              观察/验收/对账/评测",
             "├─ tests/                12 个测试",
             "├─ ros2_ws/launch/       sim / real launch",
             "├─ ros2_ws/src/          unitree_go 消息包",
             "├─ third_party/          外部 SDK（未入库）",
             "└─ docs/                 架构 + 专项文档 + 图",
         ]),
    dict(id="io_matrix", x=RIGHT_X, y=760, w=RIGHT_W, h=235, color="panel", align="left",
         title="target / IO / 环境",
         lines=[
             "sim_udp    udp      仿真（零依赖）",
             "sim_ros    ros2     仿真（ROS 2）",
             "sim_sdk    dds      仿真（SDK）",
             "real_ros   ros2     真机 Go2",
             "real_sdk   dds      真机 Go2",
             "",
             "loopback 仅用于单进程验证 / 观察器；",
             "dds 后端在 Python 3.14 下不可用（见 docs/ros2-setup.md）",
         ]),
    dict(id="contracts", x=RIGHT_X, y=1015, w=RIGHT_W, h=190, color="panel", align="left",
         title="关键契约",
         lines=[
             "obs = 243 = 48 本体 + 187 扫描 + 8 相位",
             "action = 12 个绝对关节位置目标",
             "PD：kp = 20 / kd = 1",
             "sim 500 Hz / 控制 50 Hz",
             "默认策略 go2_v33d（1536-768）",
             "26 档 profile；回退 go2_v29c",
         ]),
    dict(id="evidence", x=RIGHT_X, y=1225, w=RIGHT_W, h=225, color="panel", align="left",
         title="验证与数据",
         lines=[
             "tests/run_tests.py         14/14",
             "scripts/verify.py          16/16",
             "obs 对账                   逐段 max|diff| = 0",
             "动作 jax vs numpy          ≤ 1.15e-6",
             "PD vs <position>           1.1e-15 rad",
             "31 个地形 scene 契约测试   通过",
             "策略推理 0.70 ms（p99 1.45 ms）",
         ]),
]

# 连线：src/dst = (box_id, side)，side ∈ T/B/L/R；via 为可选折线点
EDGES: List[dict] = [
    dict(src_xy=(771, 500), dst_xy=(771, 532), label="LowCmd"),
    dict(src_xy=(392, 754), dst_xy=(392, 790)),
    dict(src_xy=(1147, 754), dst_xy=(1147, 790)),
    dict(src_xy=(392, 1040), dst_xy=(392, 1056), label="LowState"),
    dict(src_xy=(1147, 1040), dst_xy=(1147, 1056), label="LowState"),
    dict(src_xy=(30, 1120), dst_xy=(44, 381),
         via=[(14, 1120), (14, 381)]),
]

SIDE_POINT = {"T": (0.5, 0.0), "B": (0.5, 1.0), "L": (0.0, 0.5), "R": (1.0, 0.5)}


def _by_id(bid: str) -> dict:
    for b in BOXES:
        if b["id"] == bid:
            return b
    for g in GROUPS:
        if g["id"] == bid:
            return g
    raise KeyError(bid)


def _pt(b: dict, side: str) -> List[float]:
    u, v = SIDE_POINT[side]
    return [b["x"] + u * b["w"], b["y"] + v * b["h"]]


def _edge_points(e: dict) -> List[List[float]]:
    if "src_xy" in e:
        p0 = [float(e["src_xy"][0]), float(e["src_xy"][1])]
    else:
        p0 = _pt(_by_id(e["src"][0]), e["src"][1])
    if "dst_xy" in e:
        p1 = [float(e["dst_xy"][0]), float(e["dst_xy"][1])]
    else:
        p1 = _pt(_by_id(e["dst"][0]), e["dst"][1])
    if e.get("via"):
        return [p0] + [[float(x), float(y)] for x, y in e["via"]] + [p1]
    if abs(p0[0] - p1[0]) < 1.0 or abs(p0[1] - p1[1]) < 1.0:
        return [p0, p1]
    return [p0, [p0[0], p1[1]], p1]


# ---------------------------------------------------------------- 字体与文字
def _font_path(bold: bool) -> str:
    names = (
        ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
         "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc"]
        if bold else
        ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
         "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"]
    )
    for p in names:
        if os.path.exists(p):
            return p
    return "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


_FONT_CACHE: Dict[Tuple[bool, int], object] = {}


def _font(bold: bool, size: int):
    from PIL import ImageFont
    key = (bold, int(size))
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = ImageFont.truetype(_font_path(bold), int(size))
    return _FONT_CACHE[key]


def _tokenize(line: str) -> List[str]:
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


def _wrap(draw, text: str, font, max_px: float) -> List[str]:
    lines: List[str] = []
    for raw in str(text).split("\n"):
        cur = ""
        for tk in _tokenize(raw):
            cand = cur + tk
            if cur and draw.textlength(cand, font=font) > max_px:
                lines.append(cur.rstrip())
                cur = "" if tk == " " else tk
            elif not cur and draw.textlength(tk, font=font) > max_px:
                piece = ""
                for ch in tk:
                    if piece and draw.textlength(piece + ch, font=font) > max_px:
                        lines.append(piece); piece = ch
                    else:
                        piece += ch
                cur = piece
            else:
                cur = cand
        lines.append(cur.rstrip())
    return lines


# ---------------------------------------------------------------- PNG 渲染
def _draw_box(draw, b: dict) -> None:
    fill, stroke = PALETTE[b["color"]]
    x, y, w, h = b["x"], b["y"], b["w"], b["h"]
    draw.rounded_rectangle([x + 2, y + 3, x + w + 2, y + h + 3], radius=10, fill="#00000010")
    draw.rounded_rectangle([x, y, x + w, y + h], radius=10, fill=fill, outline=stroke, width=2)
    draw.rounded_rectangle([x + 4, y + 4, x + w - 4, y + h - 4], radius=8,
                           outline="#ffffff", width=1)

    is_group = b.get("id", "").startswith("g_")
    title = b.get("title", "")
    if is_group:
        # 分组标题固定画在顶部，避免子框把标题盖住
        tf = _font(True, 16)
        tw = draw.textlength(title, font=tf)
        draw.text((x + (w - tw) / 2, y + 9), title, font=tf, fill="#111827")
        return
    body = b.get("lines") or []
    tf = _font(True, 16 if is_group else 15)
    bf = _font(False, 13 if b["id"] in ("tree", "io_matrix") else 12.5)
    align = b.get("align", "center")
    pad = 16

    title_lines = _wrap(draw, title, tf, w - pad * 2) if title else []
    body_lines: List[str] = []
    for ln in body:
        body_lines.extend(_wrap(draw, ln, bf, w - pad * 2))

    title_lh = tf.size + 4
    body_lh = bf.size + 5
    gap = 8 if title_lines and body_lines else 0
    total = len(title_lines) * title_lh + gap + len(body_lines) * body_lh
    if align == "left" and b["id"] == "tree":
        # 目录树从顶部排；其余左对齐面板保持垂直居中
        ty = y + 10 if total <= h - 20 else y + (h - total) / 2
    else:
        ty = y + (h - total) / 2

    for ln in title_lines:
        tw = draw.textlength(ln, font=tf)
        draw.text((x + (w - tw) / 2, ty), ln, font=tf, fill="#111827")
        ty += title_lh
    ty += gap
    for ln in body_lines:
        if align == "left":
            draw.text((x + pad, ty), ln, font=bf, fill="#1f2937")
        else:
            tw = draw.textlength(ln, font=bf)
            draw.text((x + (w - tw) / 2, ty), ln, font=bf, fill="#1f2937")
        ty += body_lh


def _arrowhead(draw, p_prev: Sequence[float], p_end: Sequence[float]) -> None:
    ang = math.atan2(p_end[1] - p_prev[1], p_end[0] - p_prev[0])
    L, Wd = 13, 6
    draw.polygon(
        [(p_end[0], p_end[1]),
         (p_end[0] - L * math.cos(ang) + Wd * math.sin(ang),
          p_end[1] - L * math.sin(ang) - Wd * math.cos(ang)),
         (p_end[0] - L * math.cos(ang) - Wd * math.sin(ang),
          p_end[1] - L * math.sin(ang) + Wd * math.cos(ang))],
        fill=ARROW_COLOR)


def _draw_edge(draw, e: dict) -> None:
    pts = _edge_points(e)
    for i in range(len(pts) - 1):
        draw.line([tuple(pts[i]), tuple(pts[i + 1])], fill=ARROW_COLOR, width=2)
    _arrowhead(draw, pts[-2], pts[-1])
    label = e.get("label") or ""
    if label and len(pts) >= 2:
        p0, p1 = pts[0], pts[-1]
        lx, ly = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        f = _font(False, 11)
        tw = draw.textlength(label, font=f)
        draw.rounded_rectangle([lx - tw / 2 - 4, ly - 9, lx + tw / 2 + 4, ly + 9],
                               radius=3, fill="#ffffff")
        draw.text((lx - tw / 2, ly - 7), label, font=f, fill=ARROW_COLOR)


def render_png(path: str) -> None:
    from PIL import Image, ImageDraw

    img = Image.new("RGB", (W, H), "#ffffff")
    draw = ImageDraw.Draw(img)
    draw.text((LEFT_X + 4, TITLE_Y),
              "mjx-go2-mujoco2ros2 · 项目架构总览",
              font=_font(True, 27), fill="#0f172a")
    draw.text((LEFT_X + 510, TITLE_Y + 12),
              "Go2 · MuJoCo 500 Hz 仿真 → ROS 2 / DDS 真机 · 50 Hz 控制闭环",
              font=_font(False, 14), fill="#475569")

    for g in GROUPS:
        _draw_box(draw, g)
    for b in BOXES:
        _draw_box(draw, b)
    for e in EDGES:
        _draw_edge(draw, e)
    img.save(path)
    print("PNG:", path, img.size)


# ---------------------------------------------------------------- Drawnix
def _drawnix_text(b: dict) -> str:
    parts = []
    if b.get("title"):
        parts.append(b["title"])
    parts.extend(b.get("lines") or [])
    return "\n".join(parts)


def _drawnix_connection(b: dict, side: str) -> List[float]:
    return list(SIDE_POINT[side])


def _box_for_point(pt: Sequence[float]) -> dict:
    candidates = BOXES + GROUPS
    inside = [b for b in candidates
              if b["x"] <= pt[0] <= b["x"] + b["w"] and b["y"] <= pt[1] <= b["y"] + b["h"]]
    if inside:
        # 优先具体模块；分组框只作为兜底
        inside.sort(key=lambda b: b["id"].startswith("g_"))
        return inside[0]
    return min(candidates, key=lambda b: (b["x"] + b["w"] / 2 - pt[0]) ** 2
               + (b["y"] + b["h"] / 2 - pt[1]) ** 2)


def _rel_conn(b: dict, pt: Sequence[float]) -> List[float]:
    u = min(1.0, max(0.0, (pt[0] - b["x"]) / b["w"]))
    v = min(1.0, max(0.0, (pt[1] - b["y"]) / b["h"]))
    return [round(u, 4), round(v, 4)]


def make_drawnix() -> dict:
    els: List[dict] = []
    for b in GROUPS + BOXES:
        els.append({
            "id": b["id"], "type": "geometry", "shape": "rectangle", "angle": 0, "opacity": 1,
            "text": {"children": [{"text": _drawnix_text(b)}], "type": "paragraph",
                     "align": "left" if b.get("align") == "left" else "center"},
            "points": [[b["x"], b["y"]], [b["x"] + b["w"], b["y"] + b["h"]]],
            "strokeWidth": 2,
        })
    for i, e in enumerate(EDGES):
        pts = _edge_points(e)
        if "src" in e:
            src_id, src_conn = e["src"][0], _drawnix_connection(_by_id(e["src"][0]), e["src"][1])
        else:
            sb = _box_for_point(pts[0])
            src_id, src_conn = sb["id"], _rel_conn(sb, pts[0])
        if "dst" in e:
            dst_id, dst_conn = e["dst"][0], _drawnix_connection(_by_id(e["dst"][0]), e["dst"][1])
        else:
            tb = _box_for_point(pts[-1])
            dst_id, dst_conn = tb["id"], _rel_conn(tb, pts[-1])
        els.append({
            "id": "a%02d" % i,
            "type": "arrow-line",
            "shape": "elbow",
            "source": {"marker": "none", "connection": src_conn,
                       "boundId": src_id},
            "texts": [],
            "target": {"marker": "arrow", "connection": dst_conn,
                       "boundId": dst_id},
            "opacity": 1,
            "points": pts,
            "strokeWidth": 2,
        })
    return {"type": "drawnix", "version": 1, "source": "web", "elements": els,
            "viewport": {"zoom": 0.62, "origination": [10, 10]},
            "theme": {"themeColorMode": "default"}}


def main() -> int:
    os.makedirs(os.path.join(ROOT, "docs", "assets"), exist_ok=True)
    png = os.path.join(ROOT, "docs", "assets", "系统架构图.png")
    drawnix = os.path.join(ROOT, "docs", "系统架构图.drawnix")
    render_png(png)
    doc = make_drawnix()
    with open(drawnix, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    print("Drawnix:", drawnix, "元素数:", len(doc["elements"]),
          "%d B" % os.path.getsize(drawnix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
