#!/usr/bin/env python3
"""生成 ROS 2 节点图（仿真 / 真机同构）。

产物:
    docs/assets/ros2_node_graph.png

设计目标：让人一眼看出“仿真和真机是同一张 ROS 2 节点图，只换最上面的数据源节点”。
复用 scripts/make_architecture_diagram.py 的字体、换行与配色，保持两张图风格一致。
"""

from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from make_architecture_diagram import (  # noqa: E402
    ARROW_COLOR, PALETTE, _arrowhead, _draw_box, _font, _wrap,
)

W, H = 1900, 1250
LEFT_X, COL_W, GAP = 40, 850, 120
RIGHT_X = LEFT_X + COL_W + GAP
TOP_Y = 96


def _box(bid: str, x: int, y: int, w: int, h: int, color: str,
         title: str, lines=None) -> dict:
    return dict(id=bid, x=x, y=y, w=w, h=h, color=color, title=title,
                lines=lines or [])


def _arrow(draw, pts, label: str = "", label_at=(0.5, 0.5)) -> None:
    for i in range(len(pts) - 1):
        draw.line([tuple(pts[i]), tuple(pts[i + 1])], fill=ARROW_COLOR, width=3)
    _arrowhead(draw, pts[-2], pts[-1])
    if label:
        # 在指定线段上取比例位置放标签
        seg = max(0, min(len(pts) - 2, int(label_at[0])))
        a, b = pts[seg], pts[seg + 1]
        t = label_at[1]
        lx, ly = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        f = _font(False, 12)
        tw = draw.textlength(label, font=f)
        draw.rounded_rectangle([lx - tw / 2 - 5, ly - 10, lx + tw / 2 + 5, ly + 10],
                               radius=4, fill="#ffffff", outline="#d1d5db", width=1)
        draw.text((lx - tw / 2, ly - 8), label, font=f, fill=ARROW_COLOR)


def _column(draw, x: int, title: str, color_src: str, source_title: str,
            source_lines, launch_lines, real: bool) -> None:
    panel = _box(("g_sim" if not real else "g_real"), x, 78, COL_W, 940,
                 "panel", title)
    _draw_box(draw, panel)

    box_x, box_w = x + 100, COL_W - 200
    src_y, src_h = 145, 205
    src = _box("src_%d" % x, box_x, src_y, box_w, src_h, color_src,
               source_title, source_lines)
    _draw_box(draw, src)

    per_y, per_h = 440, 160
    per = _box("per_%d" % x, box_x, per_y, box_w, per_h, "io",
               "go2_perception · 15 Hz",
               ["signal_io/perception_node.py（仿真/真机同一份）",
                "PointCloud2 + Odometry → HeightMapper",
                "发布 /perception/height_scan（Float32MultiArray）"])
    _draw_box(draw, per)

    ctrl_y, ctrl_h = 690, 190
    ctrl = _box("ctrl_%d" % x, box_x, ctrl_y, box_w, ctrl_h, "ctrl",
                "go2_controller · 50 Hz 控制 / 500 Hz 发送",
                ["ros2_controller/run_controller.py --scan topic",
                 "LowState + height_scan → obs → policy → health → safety",
                 "发布 /lowcmd（12 维关节位置目标，kp=20 / kd=1）"])
    _draw_box(draw, ctrl)

    launch_y, launch_h = 930, 70
    launch = _box("launch_%d" % x, box_x, launch_y, box_w, launch_h, "tool",
                  "", launch_lines)
    _draw_box(draw, launch)

    cx = x + COL_W // 2
    # source -> perception（传感器数据）
    _arrow(draw, [(cx, src_y + src_h), (cx, per_y)],
           "/go2/odom + /go2/lidar/points")
    # perception -> controller（高度扫描）
    _arrow(draw, [(cx, per_y + per_h), (cx, ctrl_y)],
           "/perception/height_scan")
    # controller -> source（lowcmd，走右侧）
    rx = x + COL_W - 50
    _arrow(draw, [(box_x + box_w, ctrl_y + ctrl_h // 2), (rx, ctrl_y + ctrl_h // 2),
                  (rx, src_y + src_h // 2), (box_x + box_w, src_y + src_h // 2)],
           "/lowcmd", label_at=(1, 0.5))
    # source -> controller（lowstate，走左侧）
    lx = x + 50
    _arrow(draw, [(box_x, src_y + src_h // 2), (lx, src_y + src_h // 2),
                  (lx, ctrl_y + ctrl_h // 2), (box_x, ctrl_y + ctrl_h // 2)],
           "/lowstate", label_at=(1, 0.5))


def render(path: str) -> None:
    from PIL import Image, ImageDraw

    img = Image.new("RGB", (W, H), "#ffffff")
    draw = ImageDraw.Draw(img)
    draw.text((LEFT_X + 4, 18), "ROS 2 节点图：仿真 / 真机同构",
              font=_font(True, 27), fill="#0f172a")
    draw.text((LEFT_X + 520, 30),
              "同一组话题名、同一份 perception / controller；仿真只替换最上面的数据源节点",
              font=_font(False, 14), fill="#475569")

    _column(draw, LEFT_X, "仿真 · domain 1 · target=sim_ros", "sim",
            "go2_sim_sensors · MuJoCo 500 Hz",
            ["sim_env/run_sim.py --publish-sensors",
             "/lowstate（500 Hz）· /go2/odom（50 Hz）",
             "/go2/lidar/points（PointCloud2，15 Hz）",
             "数据源 = MuJoCo 物理 + raycast"],
            ["ros2 launch ros2_ws/launch/sim.launch.py duration:=30"],
            real=False)

    _column(draw, RIGHT_X, "真机 · domain 0 · target=real_ros", "real",
            "Go2 固件 DDS + Unitree L1",
            ["lowstate/lowcmd 直接可见（无需自写 bridge）",
             "真机话题：/lowstate · /lowcmd",
             "L1：/utlidar/cloud（frame=utlidar_lidar）",
             "odom/pose 很多固件没有数据 → 默认 scan=leg_fk"],
            ["ROS_DOMAIN_ID=0 ros2 launch ros2_ws/launch/real.launch.py"],
            real=True)

    # 中间说明
    mid_x = LEFT_X + COL_W + GAP // 2
    draw.line([(mid_x, 120), (mid_x, 1000)], fill="#cbd5e1", width=2)
    f = _font(True, 15)
    for i, line in enumerate(["同一组话题", "同一份代码", "只换数据源"]):
        tw = draw.textlength(line, font=f)
        draw.text((mid_x - tw / 2, 470 + i * 30), line, font=f, fill="#64748b")

    # 观察器节点（两列共用）
    _draw_box(draw, _box("viewer", LEFT_X, 1035, W - 2 * LEFT_X, 75, "tool",
                         "go2_viewer（可选，两列共用）",
                         ["scripts/view_ros2.py：订阅 /lowstate + /lowcmd + /go2/odom + /perception/height_scan + /go2/controller/{obs,action,info}",
                          "scripts/view_go2_policy.sh 会自动拉起上面的节点图，再启动本节点；关窗口即整体退出"]))

    # 底部话题契约
    _draw_box(draw, _box("topics", LEFT_X, 1130, W - 2 * LEFT_X, 90, "panel",
                         "话题契约（仿真 / 真机一致）",
                         ["/lowstate  unitree_go/LowState  500 Hz    "
                          "/lowcmd  unitree_go/LowCmd  500 Hz",
                          "/go2/odom  nav_msgs/Odometry   /go2/lidar/points  sensor_msgs/PointCloud2   "
                          "/perception/height_scan  std_msgs/Float32MultiArray"]))
    img.save(path)
    print("PNG:", path, img.size)


def main() -> int:
    out = os.path.join(ROOT, "docs", "assets", "ros2_node_graph.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    render(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
