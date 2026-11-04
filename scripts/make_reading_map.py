#!/usr/bin/env python3
"""生成「项目阅读地图」PNG：三条阅读路线 + 速查入口。

产物:
    docs/assets/reading_map.png

维护方式：只改下面的 A_STEPS / B_STEPS / C_STEPS / BOTTOM，然后重新运行：
    python3 scripts/make_reading_map.py

设计目的：和 docs/reading-map.md 配套，一眼看清"先读什么、后读什么、
每条路线大概多久、每个文件解决什么问题"，不用在 60+ 文件里乱翻。
"""

from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from make_architecture_diagram import _font  # noqa: E402

W, H = 2060, 1380
MARGIN = 36

# 每条路线一种颜色：(填充, 边框, 主文字色)
ROUTES = {
    "A": ("#eefaf3", "#2e7d4f", "#1f5c3a"),
    "B": ("#f3e8fd", "#7b3fbf", "#5b2b8f"),
    "C": ("#fdecec", "#c0392b", "#8e2418"),
}
INK = "#0f172a"
SUB = "#475569"
NOTE = "#64748b"
CARD_BORDER = "#dbe3ec"

# ------------------------------------------------------------------ 内容
# 每个 step: (编号, 文件/命令, 一句话重点)
A_STEPS = [
    ("A1", "README.md", "项目是什么 / 默认策略 / 结果"),
    ("A2", "系统架构图.png + architecture.md", "分层、数据流、IO matrix"),
    ("A3", "ros2_node_graph.png", "仿真/真机同构的节点图"),
    ("A4", "bash scripts/view_go2_policy.sh --headless --duration 5", "观察器输出 / obs / scan"),
    ("A5", "docs/results.md", "验证到什么程度、真机还差什么"),
]

B_STEPS = [
    ("B1", "config/go2_sim.json", "io / policy / health / safety / perception"),
    ("B2", "signal_io/config.py", "load_config 7 步装配 + apply_io_env"),
    ("B3", "signal_io/msg.py + go2_crc.py", "两种报文格式 / 812B CRC"),
    ("B4", "signal_io/node.py", "2ms 循环 / 逻辑时钟 / 断流"),
    ("B5", "ros2_controller/obs.py + policy.py", "243 维观测 / npz 前向"),
    ("B6", "ros2_controller/controller.py", "策略输出 → LowCmd / pre-pose"),
    ("B7", "ros2_controller/health.py + safety.py", "L0–L3 / track→crouch→damping→estop"),
    ("B8", "sim_env/robot.py + run_sim.py", "Go2Sim / PD / 500Hz 主循环"),
    ("B9", "signal_io/perception.py + estimator.py", "高度扫描 / 腿式里程计"),
    ("B10", "tests/ + scripts/verify.py", "14 个测试 / 9 项验收"),
]

C_STEPS = [
    ("C1", "real_env/README.md §0–§5", "话题 / 网络 / 上电顺序 / safety"),
    ("C2", "signal_io/transport/ros2.py", "ROS2↔DDS / CRC / 队列抽干"),
    ("C3", "signal_io/go2_crc.py", "812B 偏移表 / 为什么不用 CDR"),
    ("C4", "config.py: io_profiles + apply_io_env", "domain 0 / eth0 / tick_dt"),
    ("C5", "ros2_controller/run_controller.py", "装配顺序 / 阻尼收尾"),
    ("C6", "ros2_controller/health.py", "lost / 温度 / bit_flag / 阈值"),
    ("C7", "signal_io/perception_node.py", "L1 坐标系 / 外参 / 无 odom 回退"),
    ("C8", "ros2_ws/launch/real.launch.py", "真机节点图 / 默认安全参数"),
    ("C9", "scripts/go2_real_check.py", "只读自检 / 输出怎么读"),
    ("C10", "docs/go2-sim2real-references.md", "已跑通项目 / 雷达 / 踩坑"),
]

BOTTOM = [
    ("配置", "config/go2_sim.json", "一次运行读到的全部参数"),
    ("测试", "tests/（14 个）", "改代码后先用它自证没坏"),
    ("真机", "scripts/go2_real_check.py", "上真机前先跑这个只读探针"),
    ("速查", "docs/reading-map.md §7", "命令 / 配置 / 文档 / 自检 20 问"),
]


def _round(draw, box, radius=10, fill="#ffffff", outline=CARD_BORDER, width=2):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def _text(draw, xy, text, size, color=INK, bold=False):
    draw.text(xy, text, font=_font(bold, size), fill=color)


def _fit(text: str, size: int, max_w: float) -> str:
    """按像素宽度截断文本（中文按等宽估算），超长加省略号。"""
    f = _font(False, size)
    if f.getlength(text) <= max_w:
        return text
    out = text
    while out and f.getlength(out + "…") > max_w:
        out = out[:-1]
    return out + "…"


def _chip(draw, x, y, w, h, route, step, path, note):
    """一个阅读步骤卡片：左侧色块编号 + 文件路径 + 重点。"""
    fill, stroke, dark = ROUTES[route]
    _round(draw, [x, y, x + w, y + h], radius=8, fill="#ffffff", outline=CARD_BORDER)
    # 左侧编号色块
    draw.rounded_rectangle([x, y, x + 56, y + h], radius=8, fill=fill, outline=fill)
    f_num = _font(True, 17)
    tw = draw.textlength(step, font=f_num)
    draw.text((x + 28 - tw / 2, y + h / 2 - 11), step, font=f_num, fill=dark)
    # 文件/命令
    _text(draw, (x + 70, y + 12), _fit(path, 16, w - 84), 16, INK, bold=True)
    # 重点
    _text(draw, (x + 70, y + 38), _fit(note, 13, w - 84), 13, SUB)


def _route_column(draw, x, y, w, title, subtitle, route, steps, minutes, note, col_h):
    fill, stroke, dark = ROUTES[route]
    chip_h, gap = 66, 12
    head_h = 86
    total_h = col_h
    checkpoint = note          # 循环里的 note 会被覆盖，先保存路线检查点
    # 列背景
    _round(draw, [x, y, x + w, y + total_h], radius=14, fill="#fbfcfe", outline=stroke, width=2)
    # 列标题栏
    draw.rounded_rectangle([x, y, x + w, y + head_h], radius=14, fill=fill, outline=fill)
    draw.rectangle([x, y + head_h - 14, x + w, y + head_h], fill=fill)
    _text(draw, (x + 18, y + 14), title, 21, dark, bold=True)
    _text(draw, (x + 18, y + 46), subtitle, 13, SUB)
    # 预计时长（右上）
    f = _font(True, 14)
    tw = draw.textlength(minutes, font=f)
    draw.rounded_rectangle([x + w - tw - 34, y + 16, x + w - 16, y + 44],
                           radius=12, fill="#ffffff", outline=stroke)
    draw.text((x + w - tw - 25, y + 22), minutes, font=f, fill=dark)
    # 步骤卡片
    cy = y + head_h + 14
    for i, (step, path, chip_note) in enumerate(steps):
        _chip(draw, x + 14, cy, w - 28, chip_h, route, step, path, chip_note)
        if i < len(steps) - 1:
            # 竖向箭头
            ax = x + 14 + 28
            draw.line([ax, cy + chip_h + 1, ax, cy + chip_h + gap - 1], fill=stroke, width=2)
            draw.polygon([(ax, cy + chip_h + gap + 1), (ax - 4, cy + chip_h + gap - 5),
                          (ax + 4, cy + chip_h + gap - 5)], fill=stroke)
        cy += chip_h + gap
    # 底部「读完后能回答」提示，填满 A 列空白并给 B/C 做收尾
    ny = y + total_h - 84
    draw.rounded_rectangle([x + 14, ny, x + w - 14, y + total_h - 14],
                           radius=8, fill=fill, outline=fill)
    _text(draw, (x + 26, ny + 9), "读完后能回答", 12, dark, bold=True)
    _text(draw, (x + 26, ny + 30), _fit(checkpoint, 12, w - 52), 12, SUB)
    return total_h


def render(path: str) -> None:
    from PIL import Image, ImageDraw

    img = Image.new("RGB", (W, H), "#ffffff")
    draw = ImageDraw.Draw(img)

    # ---- 标题
    draw.rounded_rectangle([MARGIN, 22, W - MARGIN, 118], radius=16, fill="#f8fafc",
                           outline="#e2e8f0", width=2)
    _text(draw, (MARGIN + 26, 34), "mjx-go2-mujoco2ros2 · 项目阅读地图", 30, INK, bold=True)
    _text(draw, (MARGIN + 26, 80),
          "从入口到细节：三条路线按目标选一条；配套文档 docs/reading-map.md（含代码级调用链、"
          "目录/配置/测试速查、读完自检 20 问）", 14, SUB)

    # ---- 顶部入口条
    entries = [
        ("入口 1", "README.md", "总览 / 快速开始 / 策略库 / 结果"),
        ("入口 2", "系统架构图.png + ros2_node_graph.png", "先建立全局图景"),
        ("入口 3", "docs/reading-map.md", "本文档：路线 + 调用链 + 速查"),
    ]
    ew = (W - 2 * MARGIN - 2 * 24) / 3
    ey = 136
    for i, (tag, name, note) in enumerate(entries):
        ex = MARGIN + i * (ew + 24)
        _round(draw, [ex, ey, ex + ew, ey + 84], radius=12, fill="#ffffff")
        _text(draw, (ex + 16, ey + 11), tag, 12, NOTE)
        _text(draw, (ex + 16, ey + 30), _fit(name, 16, ew - 32), 16, INK, bold=True)
        _text(draw, (ex + 16, ey + 56), _fit(note, 13, ew - 32), 13, SUB)

    # ---- 三条路线
    col_y = 246
    gap = 24
    col_w = (W - 2 * MARGIN - 2 * gap) / 3
    # 三列等长：以步数最多的 B/C 为准，A 底部用提示卡补齐
    max_steps = max(len(A_STEPS), len(B_STEPS), len(C_STEPS))
    col_h = 86 + max_steps * 66 + (max_steps - 1) * 12 + 14 + 96
    _route_column(draw, MARGIN, col_y, col_w,
                  "路线 A · 快速上手", "先跑起来，看懂项目在做什么", "A", A_STEPS, "≈ 1.5 h",
                  "项目做什么？50/500 Hz 谁负责？真机为何不用自写低速桥？", col_h)
    _route_column(draw, MARGIN + col_w + gap, col_y, col_w,
                  "路线 B · 彻底理解实现", "配置 → 消息 → 循环 → 策略 → 仿真 → 测试", "B",
                  B_STEPS, "≈ 1 天",
                  "换 target 为何能换环境？两种 CRC 区别？断流后多久 L2/L3？", col_h)
    _route_column(draw, MARGIN + 2 * (col_w + gap), col_y, col_w,
                  "路线 C · 真机 ROS 2 专项", "通信 / CRC / 网络 / 安全 / 感知", "C",
                  C_STEPS, "≈ 3 h",
                  "/rt/lowcmd 为何错？CRC 覆盖哪些字节？上机前 4 件事？", col_h)
    bottom_y = col_y + col_h + 22

    # ---- 底部速查
    bw = (W - 2 * MARGIN - 3 * 18) / 4
    for i, (tag, name, note) in enumerate(BOTTOM):
        bx = MARGIN + i * (bw + 18)
        _round(draw, [bx, bottom_y, bx + bw, bottom_y + 78], radius=12,
               fill="#f8fafc", outline="#e2e8f0")
        _text(draw, (bx + 14, bottom_y + 9), tag, 12, NOTE)
        _text(draw, (bx + 14, bottom_y + 27), _fit(name, 14, bw - 28), 14, INK, bold=True)
        _text(draw, (bx + 14, bottom_y + 50), _fit(note, 12, bw - 28), 12, SUB)

    # ---- 页脚
    _text(draw, (MARGIN + 4, bottom_y + 96),
          "维护：新增/移动文件时同步更新 docs/reading-map.md §4 与本文；"
          "真机待验证项用 TODO(real)、已知坑用 WARN: 标注。", 12, NOTE)
    _text(draw, (W - MARGIN - 210, bottom_y + 96), "scripts/make_reading_map.py", 12, "#94a3b8")

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img.save(path)
    print("PNG:", path, img.size)


if __name__ == "__main__":
    render(os.path.join(ROOT, "docs", "assets", "reading_map.png"))
