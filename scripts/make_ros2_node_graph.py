#!/usr/bin/env python3
"""生成 ROS 2 节点图（仿真 / 真机同构，标注话题/类型/频率与实现文件）。

产物:
    docs/assets/ros2_node_graph.png   —— 便于阅读的上色预览
    docs/ros2节点图.drawnix            —— 可拖进 Drawnix 继续编辑（富文本）

两栏 = 两种运行配置：
    左栏 仿真 target=sim_ros，数据源 go2_sim_sensors（MuJoCo）
    右栏 真机 target=real_ros，数据源 Go2 固件 DDS + Unitree L1
两栏共用同一份 go2_perception / go2_controller —— 只换最上面的数据源节点。

每条边都标了 话题名 + 消息类型 + 频率；每个节点都标了实现文件与发布/订阅清单。
维护方式：只改下面的 COLUMNS / VIEWER / TABLE_ROWS，然后重新运行
    python3 scripts/make_ros2_node_graph.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from make_architecture_diagram import _font  # noqa: E402

# ---------------------------------------------------------------- 版面
W, H = 1980, 1800
LEFT_X, COL_W, GAP = 40, 880, 140
RIGHT_X = LEFT_X + COL_W + GAP          # 1060
GROUP_Y, GROUP_H = 110, 1090            # 110 .. 1200
INSET = 150                             # 卡片左右留白（放 /lowstate /lowcmd 标签）
CARD_W = COL_W - 2 * INSET              # 600
CARD_H = 210
SRC_Y, PER_Y, CTRL_Y = 220, 570, 920

# 底部「话题载荷」表
TABLE_Y, TABLE_H = 1320, 430
TABLE_HEAD_Y, TABLE_HEAD_H = 1370, 28
TABLE_ROW_Y, TABLE_ROW_H = 1402, 32
TABLE_BG_X, TABLE_BG_W = 60, 1860
T_TOPIC = (72, 292)
T_TYPE = (380, 400)
T_PAYLOAD = (800, 1110)
ZEBRA = ("#ffffff", "#f5f8fc")

PALETTE: Dict[str, Tuple[str, str]] = {
    "sim": ("#f3e8fd", "#7b3fbf"),
    "real": ("#fde8e8", "#c0392b"),
    "io": ("#e6f4ea", "#2e7d4f"),
    "ctrl": ("#fff4e5", "#c77700"),
    "tool": ("#eef3f8", "#5b6b7f"),
    "frame": ("#f8fafc", "#cbd5e1"),
    "band": ("#f7f8fa", "#8a949e"),
}
EDGE = {"sense": "#2e7d4f", "ctrl": "#c77700", "dim": "#94a3b8"}

# 字号与行高（PNG 用 size/lh，Drawnix 用 dw 里的字号档位）
STYLES: Dict[str, dict] = {
    "title": dict(size=20, bold=True, color="#0f172a", lh=27, dw="20"),
    "rate": dict(size=12.5, bold=False, color="#64748b", lh=27, dw="13"),
    "role": dict(size=13.5, bold=False, color="#334155", lh=20, dw="14"),
    "file": dict(size=12.5, bold=False, color="#2e7d4f", lh=20, dw="13"),
    "body": dict(size=13.5, bold=False, color="#1f2937", lh=20, dw="14"),
    "meta": dict(size=11.5, bold=False, color="#475569", lh=16, dw="12"),
    "topic": dict(size=14.5, bold=True, color="#2e7d4f", lh=19, dw="15"),
    "col_title": dict(size=18, bold=True, color="#0f172a", lh=24, dw="18"),
    "col_sub": dict(size=12, bold=False, color="#64748b", lh=18, dw="12"),
    "h1": dict(size=30, bold=True, color="#0f172a", lh=38, dw="28"),
    "h2": dict(size=14, bold=False, color="#475569", lh=20, dw="14"),
    "note": dict(size=14, bold=False, color="#64748b", lh=22, dw="14"),
    "legend": dict(size=13, bold=False, color="#475569", lh=20, dw="13"),
    "table_title": dict(size=16, bold=True, color="#0f172a", lh=22, dw="16"),
    "table_hint": dict(size=12, bold=False, color="#64748b", lh=18, dw="12"),
    "table_head": dict(size=12.5, bold=True, color="#475569", lh=18, dw="13"),
    "table_dot": dict(size=11, bold=False, color="#2e7d4f", lh=20, dw="12"),
    "table_topic": dict(size=13, bold=True, color="#2e7d4f", lh=20, dw="13"),
    "table_type": dict(size=12, bold=False, color="#475569", lh=18, dw="12"),
    "table_payload": dict(size=12.5, bold=False, color="#1f2937", lh=18, dw="13"),
}

Seg = Tuple[str, str]          # (文本, 样式名)
Line = List[Seg]

# ---------------------------------------------------------------- 内容
COLUMNS = [
    dict(
        key="sim", x=LEFT_X, accent="sim",
        title="仿真 · target=sim_ros · domain 1",
        sub="ros2 launch ros2_ws/launch/sim.launch.py duration:=30",
        footer="仿真链路：MuJoCo 500 Hz 物理 + 15 Hz raycast 点云；controller 用 --scan topic 消费 height_scan",
        src_per=[
            [("/go2/lidar/points", "topic")],
            [("sensor_msgs/PointCloud2 · 15 Hz", "meta")],
            [("/go2/odom", "topic")],
            [("nav_msgs/Odometry · 50 Hz", "meta")],
        ],
        nodes=[
            dict(id="src_sim", y=SRC_Y, accent="sim", lines=[
                [("go2_sim_sensors", "title"), ("   ① 数据源 · MuJoCo 500 Hz", "rate")],
                [("物理步进 + raycast 点云 / 真值里程计", "role")],
                [("文件  sim_env/run_sim.py --publish-sensors", "file")],
                [("发布  /lowstate 500 Hz · /go2/odom 50 Hz · /go2/lidar/points 15 Hz", "body")],
                [("订阅  /lowcmd（unitree_go/LowCmd）", "body")],
            ]),
            dict(id="per_sim", y=PER_Y, accent="io", lines=[
                [("go2_perception", "title"), ("   ② 感知 · 15 Hz", "rate")],
                [("点云 + 位姿 → 高程图 → 187 点高度扫描", "role")],
                [("文件  signal_io/perception_node.py", "file")],
                [("订阅  /go2/lidar/points · /go2/odom", "body")],
                [("发布  /perception/height_scan（Float32MultiArray 15 Hz）", "body")],
            ]),
            dict(id="ctrl_sim", y=CTRL_Y, accent="ctrl", lines=[
                [("go2_controller", "title"), ("   ③ 控制 · 50 Hz 控制 / 500 Hz 发送", "rate")],
                [("obs(243) → policy → health → safety → LowCmd", "role")],
                [("文件  ros2_controller/run_controller.py --scan topic", "file")],
                [("订阅  /lowstate（unitree_go/LowState）· /perception/height_scan", "body")],
                [("发布  /lowcmd（unitree_go/LowCmd，12 维关节目标）", "body")],
                [("调试  /go2/controller/{obs,action,info}", "body")],
            ]),
        ],
    ),
    dict(
        key="real", x=RIGHT_X, accent="real",
        title="真机 · target=real_ros · domain 0",
        sub="ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py",
        footer="真机默认：controller=stand / scan=leg_fk；确认 /utlidar/robot_pose 可靠后再开 perception + scan:=topic",
        src_per=[
            [("/utlidar/cloud", "topic")],
            [("sensor_msgs/PointCloud2 · ~11 Hz", "meta")],
            [("/utlidar/robot_pose", "topic")],
            [("geometry_msgs/PoseStamped · 可选", "meta")],
        ],
        nodes=[
            dict(id="src_real", y=SRC_Y, accent="real", lines=[
                [("Go2 固件（DDS）", "title"), ("   ① 数据源 · 500 Hz", "rate")],
                [("真机固件直出 LowState / LowCmd，无需自写 bridge", "role")],
                [("文件  real_env/README.md（真机部署说明，无自写节点）", "file")],
                [("发布  /lowstate · /utlidar/cloud（PointCloud2 ~11 Hz）", "body")],
                [("可选  /utlidar/robot_pose（PoseStamped，很多固件无数据）", "body")],
                [("订阅  /lowcmd（unitree_go/LowCmd）", "body")],
            ]),
            dict(id="per_real", y=PER_Y, accent="io", lines=[
                [("go2_perception", "title"), ("   ② 感知 · 15 Hz", "rate")],
                [("点云 + 位姿 → 高程图 → 187 点高度扫描", "role")],
                [("文件  signal_io/perception_node.py", "file")],
                [("订阅  /utlidar/cloud · /utlidar/robot_pose（可缺）", "body")],
                [("发布  /perception/height_scan（Float32MultiArray 15 Hz）", "body")],
            ]),
            dict(id="ctrl_real", y=CTRL_Y, accent="ctrl", lines=[
                [("go2_controller", "title"), ("   ③ 控制 · 50 Hz 控制 / 500 Hz 发送", "rate")],
                [("obs(243) → policy → health → safety → LowCmd", "role")],
                [("文件  ros2_controller/run_controller.py --scan topic", "file")],
                [("订阅  /lowstate（unitree_go/LowState）· /perception/height_scan", "body")],
                [("发布  /lowcmd（unitree_go/LowCmd，12 维关节目标）", "body")],
                [("调试  /go2/controller/{obs,action,info}", "body")],
            ]),
        ],
    ),
]

VIEWER = dict(
    id="viewer", y=1225, h=80, accent="tool",
    lines=[
        [("go2_viewer（可选，两列共用）", "title"), ("   只读订阅，不参与控制闭环", "rate")],
        [("文件  scripts/view_ros2.py", "file")],
        [("订阅  /lowstate · /lowcmd · /go2/odom · /perception/height_scan · /go2/controller/{obs,action,info}", "body")],
    ],
)

TABLE_TITLE = "话题载荷 · 每个话题里装的是什么"
TABLE_HINT = "箭头 = 数据流方向 · 边标签 = 话题 + 类型 + 频率"
TABLE_HEAD = [("话题", T_TOPIC), ("类型 · 频率", T_TYPE), ("数据内容（payload）", T_PAYLOAD)]

TABLE_ROWS = [
    dict(kind="ctrl", topic="/lowstate", type="unitree_go/LowState · 500 Hz",
         payload="tick · IMU 四元数/角速度/加速度/rpy · 12×(q, dq, τ_est, 温度) · 4 足端力 · bit_flag"),
    dict(kind="ctrl", topic="/lowcmd", type="unitree_go/LowCmd · 500 Hz",
         payload="mode_pr / mode_machine · 12×(q_des, dq_des, kp, kd, τ_ff) · CRC32"),
    dict(kind="sense", topic="/go2/odom", type="nav_msgs/Odometry · 50 Hz",
         payload="pose(position + orientation) · twist.linear(vx, vy, vz) · frame=world → base_link"),
    dict(kind="sense", topic="/go2/lidar/points", type="sensor_msgs/PointCloud2 · 15 Hz",
         payload="xyz 点云（raycast 采样，frame=world）"),
    dict(kind="sense", topic="/utlidar/cloud", type="sensor_msgs/PointCloud2 · ~11 Hz",
         payload="xyz 点云（L1 雷达，frame=utlidar_lidar）"),
    dict(kind="sense", topic="/utlidar/robot_pose", type="geometry_msgs/PoseStamped · 可选",
         payload="position + orientation（很多固件无数据）"),
    dict(kind="sense", topic="/perception/height_scan", type="std_msgs/Float32MultiArray · 15 Hz",
         payload="187 = 17×11 · clip(z_base − h_ground − 0.268, ±0.35) × 5（负 = 前方更高）"),
    dict(kind="dim", topic="/go2/controller/obs", type="std_msgs/Float32MultiArray · 50 Hz",
         payload="243 = 48 本体（v/ω/重力/指令/q/dq/上一动作）+ 187 扫描 + 8 相位"),
    dict(kind="dim", topic="/go2/controller/action", type="std_msgs/Float32MultiArray · 50 Hz",
         payload="12 维关节位置目标（策略原始输出，送 safety 前）"),
    dict(kind="dim", topic="/go2/controller/info", type="std_msgs/String · 10 Hz",
         payload="JSON：level / mode / 速度来源 / 策略统计 / timestamp"),
]


# ---------------------------------------------------------------- 计算
def _style(name: str, accent: str | None = None) -> dict:
    st = dict(STYLES[name])
    if accent and name in ("file", "topic", "table_dot", "table_topic"):
        st["color"] = accent
    return st


def _font_of(st: dict):
    return _font(bool(st["bold"]), int(round(st["size"])))


def _line_width(line: Line, accent: str | None = None) -> float:
    return sum(_font_of(_style(name, accent)).getlength(text) for text, name in line)


def _line_height(line: Line) -> float:
    return max(STYLES[name]["lh"] for _, name in line)


def _block_size(lines: Sequence[Line], accent: str | None = None) -> Tuple[float, float]:
    return (max((_line_width(ln, accent) for ln in lines), default=0.0),
            sum(_line_height(ln) for ln in lines))


def _card_rect(col: dict, node: dict) -> dict:
    return dict(x=col["x"] + INSET, y=node["y"], w=CARD_W, h=CARD_H)


def _edges(col: dict) -> List[dict]:
    x, key = col["x"], col["key"]
    card_x, card_w = x + INSET, CARD_W
    right = card_x + card_w
    cx = x + COL_W / 2
    cy_src, cy_ctrl = SRC_Y + CARD_H / 2, CTRL_Y + CARD_H / 2
    lx, rx = x + INSET / 2, x + COL_W - INSET / 2
    return [
        dict(id="e_%s_src_per" % key, color=EDGE["sense"],
             src=("src_" + key, (0.5, 1.0)), dst=("per_" + key, (0.5, 0.0)),
             points=[[cx, SRC_Y + CARD_H], [cx, PER_Y]],
             chip=(cx, (SRC_Y + CARD_H + PER_Y) / 2), lines=col["src_per"]),
        dict(id="e_%s_per_ctrl" % key, color=EDGE["sense"],
             src=("per_" + key, (0.5, 1.0)), dst=("ctrl_" + key, (0.5, 0.0)),
             points=[[cx, PER_Y + CARD_H], [cx, CTRL_Y]],
             chip=(cx, (PER_Y + CARD_H + CTRL_Y) / 2),
             lines=[[("/perception/height_scan", "topic")],
                    [("std_msgs/Float32MultiArray · 15 Hz", "meta")]]),
        dict(id="e_%s_src_ctrl" % key, color=EDGE["ctrl"],
             src=("src_" + key, (0.0, 0.5)), dst=("ctrl_" + key, (0.0, 0.5)),
             points=[[card_x, cy_src], [lx, cy_src], [lx, cy_ctrl], [card_x, cy_ctrl]],
             chip=(lx, (cy_src + cy_ctrl) / 2),
             lines=[[("/lowstate", "topic")], [("LowState · 500 Hz", "meta")]]),
        dict(id="e_%s_ctrl_src" % key, color=EDGE["ctrl"],
             src=("ctrl_" + key, (1.0, 0.5)), dst=("src_" + key, (1.0, 0.5)),
             points=[[right, cy_ctrl], [rx, cy_ctrl], [rx, cy_src], [right, cy_src]],
             chip=(rx, (cy_src + cy_ctrl) / 2),
             lines=[[("/lowcmd", "topic")], [("LowCmd · 500 Hz", "meta")]]),
    ]


# ---------------------------------------------------------------- PNG
def _arrowhead(draw, p0, p1, color: str) -> None:
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    length, width = 12, 5.5
    draw.polygon([
        (p1[0], p1[1]),
        (p1[0] - length * math.cos(ang) + width * math.sin(ang),
         p1[1] - length * math.sin(ang) - width * math.cos(ang)),
        (p1[0] - length * math.cos(ang) - width * math.sin(ang),
         p1[1] - length * math.sin(ang) + width * math.cos(ang)),
    ], fill=color)


def _draw_text_block(draw, lines: Sequence[Line], cx: float, top: float,
                     accent: str | None = None) -> float:
    y = top
    for line in lines:
        styled = [(_style(name, accent), text) for text, name in line]
        total = sum(_font_of(st).getlength(text) for st, text in styled)
        lh = max(st["lh"] for st, _ in styled)
        base = y + max(st["size"] for st, _ in styled) * 0.82
        x = cx - total / 2
        for st, text in styled:
            f = _font_of(st)
            draw.text((x, base), text, font=f, fill=st["color"], anchor="ls")
            x += f.getlength(text)
        y += lh
    return y


def _draw_card(draw, rect: dict, lines: Sequence[Line], accent: str) -> None:
    fill, stroke = PALETTE[accent]
    x, y, w, h = rect["x"], rect["y"], rect["w"], rect["h"]
    draw.rounded_rectangle([x + 3, y + 4, x + w + 3, y + h + 4], radius=16,
                           fill="#0f172a12")
    draw.rounded_rectangle([x, y, x + w, y + h], radius=16, fill=fill,
                           outline=stroke, width=2)
    draw.rounded_rectangle([x + 10, y + 16, x + 16, y + h - 16], radius=3,
                           fill=stroke)
    _, th = _block_size(lines, stroke)
    _draw_text_block(draw, lines, x + w / 2, y + (h - th) / 2, accent=stroke)


def _draw_chip(draw, lines: Sequence[Line], cx: float, cy: float, color: str) -> None:
    w, h = _block_size(lines, color)
    w += 24
    h += 14
    draw.rounded_rectangle([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2],
                           radius=7, fill="#ffffff", outline=color, width=1)
    _draw_text_block(draw, lines, cx, cy - h / 2 + 7, accent=color)


def _draw_band(draw, band: dict) -> None:
    fill, stroke = PALETTE[band["accent"]]
    x, y, w, h = LEFT_X, band["y"], W - 2 * LEFT_X, band["h"]
    draw.rounded_rectangle([x + 3, y + 4, x + w + 3, y + h + 4], radius=16,
                           fill="#0f172a10")
    draw.rounded_rectangle([x, y, x + w, y + h], radius=16, fill=fill,
                           outline=stroke, width=2)
    _, th = _block_size(band["lines"], stroke)
    _draw_text_block(draw, band["lines"], x + w / 2, y + (h - th) / 2,
                     accent=stroke)


def _draw_table(draw) -> None:
    fill, stroke = PALETTE["band"]
    x, y, w, h = LEFT_X, TABLE_Y, W - 2 * LEFT_X, TABLE_H
    draw.rounded_rectangle([x + 3, y + 4, x + w + 3, y + h + 4], radius=16,
                           fill="#0f172a10")
    draw.rounded_rectangle([x, y, x + w, y + h], radius=16, fill=fill,
                           outline=stroke, width=2)
    draw.text((x + 24, y + 12), TABLE_TITLE, font=_font(True, 16), fill="#0f172a")
    f = _font(False, 12)
    tw = draw.textlength(TABLE_HINT, font=f)
    draw.text((x + w - 24 - tw, y + 17), TABLE_HINT, font=f, fill="#64748b")

    draw.rectangle([TABLE_BG_X, TABLE_HEAD_Y, TABLE_BG_X + TABLE_BG_W,
                    TABLE_HEAD_Y + TABLE_HEAD_H], fill="#e2e8f0")
    for text, (cx, cw) in TABLE_HEAD:
        draw.text((cx + 10, TABLE_HEAD_Y + 6), text, font=_font(True, 12.5),
                  fill="#475569")

    for i, row in enumerate(TABLE_ROWS):
        ry = TABLE_ROW_Y + i * TABLE_ROW_H
        bg = ZEBRA[i % 2]
        draw.rectangle([TABLE_BG_X, ry, TABLE_BG_X + TABLE_BG_W, ry + TABLE_ROW_H],
                       fill=bg)
        draw.line([(TABLE_BG_X, ry + TABLE_ROW_H),
                   (TABLE_BG_X + TABLE_BG_W, ry + TABLE_ROW_H)],
                  fill="#e8edf4", width=1)
        color = EDGE[row["kind"]]
        draw.ellipse([T_TOPIC[0] + 3, ry + 12, T_TOPIC[0] + 9, ry + 18], fill=color)
        draw.text((T_TOPIC[0] + 16, ry + 8), row["topic"], font=_font(True, 13),
                  fill=color)
        draw.text((T_TYPE[0] + 10, ry + 9), row["type"], font=_font(False, 12),
                  fill="#475569")
        draw.text((T_PAYLOAD[0] + 10, ry + 8), row["payload"], font=_font(False, 12.5),
                  fill="#1f2937")


def render(path: str) -> None:
    from PIL import Image, ImageDraw

    img = Image.new("RGB", (W, H), "#ffffff")
    draw = ImageDraw.Draw(img)

    # 标题
    draw.text((LEFT_X + 4, 16), "ROS 2 节点图：仿真 / 真机同构",
              font=_font(True, 30), fill="#0f172a")
    draw.text((LEFT_X + 4, 62),
              "同一组话题名、同一份 perception / controller；每条边标话题/类型/频率，底部表列出每个话题的载荷",
              font=_font(False, 14), fill="#475569")

    # 图例
    lx0 = 1370
    for i, (label, color) in enumerate([("感知数据", EDGE["sense"]),
                                        ("控制数据", EDGE["ctrl"])]):
        x = lx0 + i * 150
        draw.rounded_rectangle([x, 45, x + 40, 49], radius=2, fill=color)
        draw.text((x + 48, 38), label, font=_font(False, 13), fill="#475569")

    mid_x = LEFT_X + COL_W + GAP / 2
    for col in COLUMNS:
        x, w = col["x"], COL_W
        draw.rounded_rectangle([x, GROUP_Y, x + w, GROUP_Y + GROUP_H],
                               radius=20, fill=PALETTE["frame"][0],
                               outline=PALETTE["frame"][1], width=2)
        draw.text((x + w / 2, GROUP_Y + 22), col["title"], font=_font(True, 18),
                  fill="#0f172a", anchor="ma")
        draw.text((x + w / 2, GROUP_Y + 58), col["sub"], font=_font(False, 12),
                  fill="#64748b", anchor="ma")
        draw.line([(x + 24, GROUP_Y + 90), (x + w - 24, GROUP_Y + 90)],
                  fill="#e2e8f0", width=1)

        for node in col["nodes"]:
            _draw_card(draw, _card_rect(col, node), node["lines"], node["accent"])

        for edge in _edges(col):
            pts = [tuple(p) for p in edge["points"]]
            for i in range(len(pts) - 1):
                draw.line([pts[i], pts[i + 1]], fill=edge["color"], width=3)
            _arrowhead(draw, pts[-2], pts[-1], edge["color"])
            _draw_chip(draw, edge["lines"], edge["chip"][0], edge["chip"][1],
                       edge["color"])

        draw.text((x + w / 2, 1154), col["footer"], font=_font(False, 12),
                  fill="#64748b", anchor="ma")

    # 中间说明
    draw.line([(mid_x, GROUP_Y + 20), (mid_x, GROUP_Y + GROUP_H - 20)],
              fill="#dbe2ea", width=1)
    f = _font(True, 14)
    for i, text in enumerate(["同一组话题", "同一份代码", "只换数据源"]):
        tw = draw.textlength(text, font=f)
        draw.rounded_rectangle([mid_x - tw / 2 - 8, 548 + i * 30 - 4,
                                mid_x + tw / 2 + 8, 548 + i * 30 + 20],
                               radius=4, fill="#ffffff")
        draw.text((mid_x - tw / 2, 548 + i * 30), text, font=f, fill="#64748b")

    _draw_band(draw, VIEWER)
    _draw_table(draw)

    img.save(path)
    print("PNG:", path, img.size)


# ---------------------------------------------------------------- Drawnix
def _drawnix_lines(lines: Sequence[Line], accent: str | None) -> List[dict]:
    children: List[dict] = []
    for i, line in enumerate(lines):
        for j, (text, name) in enumerate(line):
            st = _style(name, accent)
            leaf: dict = {"text": text + ("\n" if i < len(lines) - 1
                                          and j == len(line) - 1 else "")}
            if st["bold"]:
                leaf["bold"] = True
            leaf["font-size"] = st["dw"]
            leaf["color"] = st["color"]
            children.append(leaf)
    return children


def _geom(eid: str, x: float, y: float, w: float, h: float,
          lines: Sequence[Line] | None, fill: str, stroke: str,
          accent: str | None = None, align: str = "center",
          stroke_width: int = 2) -> dict:
    el: dict = {
        "id": eid, "type": "geometry", "shape": "rectangle", "angle": 0,
        "opacity": 1, "points": [[x, y], [x + w, y + h]],
        "fill": fill, "strokeColor": stroke, "strokeWidth": stroke_width,
    }
    if lines is not None:
        el["text"] = {"children": _drawnix_lines(lines, accent),
                      "type": "paragraph", "align": align}
    return el


def _chip_rect(lines: Sequence[Line], cx: float, cy: float,
               accent: str) -> dict:
    w, h = _block_size(lines, accent)
    w += 24
    h += 14
    return dict(x=cx - w / 2, y=cy - h / 2, w=w, h=h)


def _table_elements() -> List[dict]:
    els: List[dict] = []
    fill, stroke = PALETTE["band"]
    els.append(_geom("payloads", LEFT_X, TABLE_Y, W - 2 * LEFT_X, TABLE_H,
                     None, fill, stroke))
    els.append(_geom("payloads_title", LEFT_X + 24, TABLE_Y + 8, 760, 30,
                     [[(TABLE_TITLE, "table_title")]], fill, fill, align="left"))
    els.append(_geom("payloads_hint", LEFT_X + 560, TABLE_Y + 10,
                     W - 2 * LEFT_X - 584, 26,
                     [[(TABLE_HINT, "table_hint")]], fill, fill, align="right"))
    els.append(_geom("table_head_bg", TABLE_BG_X, TABLE_HEAD_Y, TABLE_BG_W,
                     TABLE_HEAD_H, None, "#e2e8f0", "#e2e8f0", stroke_width=1))
    for j, (text, (cx, cw)) in enumerate(TABLE_HEAD):
        els.append(_geom("table_head_%d" % j, cx + 8, TABLE_HEAD_Y, cw - 16,
                         TABLE_HEAD_H, [[(text, "table_head")]],
                         "#e2e8f0", "#e2e8f0", align="left", stroke_width=1))
    for i, row in enumerate(TABLE_ROWS):
        ry = TABLE_ROW_Y + i * TABLE_ROW_H
        bg = ZEBRA[i % 2]
        color = EDGE[row["kind"]]
        els.append(_geom("trow_%d" % i, TABLE_BG_X, ry, TABLE_BG_W,
                         TABLE_ROW_H, None, bg, bg, stroke_width=1))
        els.append(_geom("ttopic_%d" % i, T_TOPIC[0] + 8, ry, T_TOPIC[1] - 16,
                         TABLE_ROW_H,
                         [[("●  ", "table_dot"), (row["topic"], "table_topic")]],
                         bg, bg, accent=color, align="left", stroke_width=1))
        els.append(_geom("ttype_%d" % i, T_TYPE[0] + 8, ry, T_TYPE[1] - 16,
                         TABLE_ROW_H, [[(row["type"], "table_type")]],
                         bg, bg, align="left", stroke_width=1))
        els.append(_geom("tpay_%d" % i, T_PAYLOAD[0] + 8, ry, T_PAYLOAD[1] - 16,
                         TABLE_ROW_H, [[(row["payload"], "table_payload")]],
                         bg, bg, align="left", stroke_width=1))
    return els


def make_drawnix() -> dict:
    els: List[dict] = []

    # 栏底 + 栏标题
    for col in COLUMNS:
        fx, fy = col["x"], GROUP_Y
        fill, stroke = PALETTE["frame"]
        els.append(_geom("g_" + col["key"], fx, fy, COL_W, GROUP_H, None,
                         fill, stroke))
        els.append(_geom("h_" + col["key"], fx + 24, fy + 12, COL_W - 48, 72,
                         [[(col["title"], "col_title")],
                          [(col["sub"], "col_sub")]],
                         fill, fill))
        els.append(_geom("hline_" + col["key"], fx + 24, fy + 90,
                         COL_W - 48, 1, None, "#e2e8f0", "#e2e8f0",
                         stroke_width=1))
        els.append(_geom("foot_" + col["key"], fx + 30, fy + 1040,
                         COL_W - 60, 36, [[(col["footer"], "col_sub")]],
                         fill, fill))

    # 节点卡片 + 左侧色条
    for col in COLUMNS:
        for node in col["nodes"]:
            rect = _card_rect(col, node)
            fill, stroke = PALETTE[node["accent"]]
            els.append(_geom(node["id"], rect["x"], rect["y"], rect["w"],
                             rect["h"], node["lines"], fill, stroke,
                             accent=stroke))
            els.append(_geom("bar_" + node["id"], rect["x"] + 10,
                             rect["y"] + 16, 6, rect["h"] - 32, None,
                             stroke, stroke, stroke_width=1))

    # 边 + 边标签
    for col in COLUMNS:
        for edge in _edges(col):
            src_id, src_conn = edge["src"]
            dst_id, dst_conn = edge["dst"]
            els.append({
                "id": edge["id"], "type": "arrow-line", "shape": "elbow",
                "source": {"marker": "none",
                           "connection": [src_conn[0], src_conn[1]],
                           "boundId": src_id},
                "texts": [],
                "target": {"marker": "arrow",
                           "connection": [dst_conn[0], dst_conn[1]],
                           "boundId": dst_id},
                "opacity": 1,
                "points": [[float(x), float(y)] for x, y in edge["points"]],
                "strokeColor": edge["color"], "strokeWidth": 3,
            })
            rect = _chip_rect(edge["lines"], edge["chip"][0],
                              edge["chip"][1], edge["color"])
            els.append(_geom("c_" + edge["id"], rect["x"], rect["y"],
                             rect["w"], rect["h"], edge["lines"],
                             "#ffffff", edge["color"],
                             accent=edge["color"], stroke_width=1))

    # 中间说明
    els.append(_geom("divider", mid_x_ := LEFT_X + COL_W + GAP / 2,
                     GROUP_Y + 20, 1, GROUP_H - 40, None, "#dbe2ea",
                     "#dbe2ea", stroke_width=1))
    els.append(_geom("mid_note", mid_x_ - 65, 536, 130, 100,
                     [[("同一组话题", "note")], [("同一份代码", "note")],
                      [("只换数据源", "note")]],
                     "#ffffff", "#ffffff"))

    # 观察器 + 话题载荷表
    fill, stroke = PALETTE[VIEWER["accent"]]
    els.append(_geom(VIEWER["id"], LEFT_X, VIEWER["y"], W - 2 * LEFT_X,
                     VIEWER["h"], VIEWER["lines"], fill, stroke,
                     accent=stroke))
    els.extend(_table_elements())

    # 标题 / 副标题 / 图例
    els.append(_geom("title", LEFT_X + 4, 10, 760, 42,
                     [[("ROS 2 节点图：仿真 / 真机同构", "h1")]],
                     "#ffffff", "#ffffff", align="left"))
    els.append(_geom("subtitle", LEFT_X + 4, 56, 1300, 30,
                     [[("同一组话题名、同一份 perception / controller；每条边标话题/类型/频率，底部表列出每个话题的载荷", "h2")]],
                     "#ffffff", "#ffffff", align="left"))
    legend_x = 1370
    for i, (label, color) in enumerate([("感知数据", EDGE["sense"]),
                                        ("控制数据", EDGE["ctrl"])]):
        x = legend_x + i * 150
        els.append(_geom("leg_line_%d" % i, x, 44, 40, 4, None, color, color,
                         stroke_width=1))
        els.append(_geom("leg_text_%d" % i, x + 48, 36, 72, 24,
                         [[(label, "legend")]], "#ffffff", "#ffffff",
                         align="left"))

    return {"type": "drawnix", "version": 1, "source": "web", "elements": els,
            "viewport": {"zoom": 0.45, "origination": [10, 10]},
            "theme": {"themeColorMode": "default"}}


# ---------------------------------------------------------------- 自检
def audit() -> None:
    problems = 0
    for col in COLUMNS:
        for node in col["nodes"]:
            for line in node["lines"]:
                w = _line_width(line, PALETTE[node["accent"]][1])
                if w > CARD_W - 28:
                    problems += 1
                    print("  ! 卡片文字过宽 %.0f>%d: %s" %
                          (w, CARD_W - 28, line[0][0][:40]))
        for edge in _edges(col):
            if not edge["id"].endswith(("src_ctrl", "ctrl_src")):
                continue
            w, _ = _block_size(edge["lines"], edge["color"])
            if w + 24 > INSET:
                problems += 1
                print("  ! 边标签过宽 %.0f>%d: %s" %
                      (w + 24, INSET, edge["lines"][0][0][0]))
    for line in VIEWER["lines"]:
        if _line_width(line, PALETTE[VIEWER["accent"]][1]) > W - 2 * LEFT_X - 40:
            problems += 1
            print("  ! 横带文字过宽:", line[0][0][:40])
    for row in TABLE_ROWS:
        checks = ((row["topic"], "table_topic", T_TOPIC[1]),
                  (row["type"], "table_type", T_TYPE[1]),
                  (row["payload"], "table_payload", T_PAYLOAD[1]))
        for text, style_name, cw in checks:
            if _font_of(_style(style_name, EDGE[row["kind"]])).getlength(text) > cw - 24:
                problems += 1
                print("  ! 表行过宽:", text[:40])
    print("自检:", "OK" if not problems else "%d 处需调整" % problems)


def main() -> int:
    os.makedirs(os.path.join(ROOT, "docs", "assets"), exist_ok=True)
    png = os.path.join(ROOT, "docs", "assets", "ros2_node_graph.png")
    drawnix = os.path.join(ROOT, "docs", "ros2节点图.drawnix")
    audit()
    render(png)
    doc = make_drawnix()
    with open(drawnix, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    print("Drawnix:", drawnix, "元素数:", len(doc["elements"]),
          "%d B" % os.path.getsize(drawnix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
