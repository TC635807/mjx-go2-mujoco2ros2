#!/usr/bin/env python3
"""viz/record.py 的自测：CSV / 关键帧 / 汇总图 + 无界面 ANSI 终端仪表盘。

    .venv/bin/python tests/test_viz_record.py

对应任务验收：
  1) 写 30 帧到临时目录 -> CSV 数据行数 30、表头含 t/z/tilt/level、每行列数与表头一致；
  2) close() 后 summary.png 与 report.md 存在且非空；
  3) TerminalDashboard.render() 返回 str、行数固定、含等级字样、且按显示列宽不折行。

顺带验证：save_frames=False 不调 render_fn；render_fn 返回 None / 抛异常都不打断记录；
close() 幂等；零帧 close() 也出产物；record.py 的导入链不拉 tkinter/Qt。
"""

from __future__ import annotations

import csv
import importlib.util
import io
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OK = 0


def check(name, cond, detail=""):
    global OK
    if cond:
        OK += 1
        print("  [OK]   %s %s" % (name, detail))
    else:
        print("  [FAIL] %s %s" % (name, detail))
        raise SystemExit(1)


def _raises(exc, fn):
    try:
        fn()
        return False
    except exc:
        return True
    except Exception:
        return False


def _load_record():
    """优先按包路径导入（脚本真实用法）；包还没就绪时退化为直接加载文件。"""
    try:
        from sim_env.viz.record import (Recorder, TerminalDashboard, CSV_COLUMNS,
                                           display_width)
        return Recorder, TerminalDashboard, CSV_COLUMNS, display_width, "sim_env.viz.record"
    except Exception as exc:
        print("  [info] 包导入失败 (%r)，改为直接从文件加载 record.py" % (exc,))
        path = os.path.join(ROOT, "ros2_controller", "viz", "record.py")
        spec = importlib.util.spec_from_file_location("_viz_record_standalone", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return (mod.Recorder, mod.TerminalDashboard, mod.CSV_COLUMNS,
                mod.display_width, path)


def make_frame(i):
    """造一帧会踩到所有分支的假 VizFrame（奇偶帧分别有/无真值）。"""
    from sim_env.viz.frame import VizFrame
    f = VizFrame()
    f.t = i * 0.1
    f.wall = 100.0 + i * 0.1
    f.base_pos = np.array([0.05 * i, 0.01 * i, 0.30 + 0.01 * np.sin(i)])
    f.rpy = np.array([0.02 * np.sin(0.3 * i), 0.03 * np.cos(0.2 * i), 0.1 * i])
    f.base_lin_vel_world = np.array([0.5 + 0.01 * i, 0.02 * np.sin(i), 0.001 * i])
    f.base_lin_vel_truth = np.array([0.5, 0.0, 0.0]) if i % 2 == 0 else None
    f.joint_tau = np.linspace(-5.0, 5.0, 12) + 0.1 * i
    f.action = np.linspace(-1.0, 1.0, 12) * (0.5 + 0.01 * i)
    f.height_scan = np.zeros(35)
    f.height_scan[0] = -0.5
    f.height_scan[-1] = 0.5
    f.foot_force = np.array([10.0, 0.0, 10.0, 0.0])
    f.phase = np.array([0.5, 2.0, -2.0, -0.5])
    f.obs = np.zeros(91)
    f.level = i % 4
    f.level_name = "L%d" % (i % 4)
    f.mode = ["track", "crouch", "damping", "estop"][i % 4]
    f.obs_valid = (i % 2 == 0)
    f.sources = {"linvel": "estimator", "scan": "lidar", "foot": "oracle"}
    f.rtf = 0.98 - 0.001 * i
    f.overruns = i // 10
    f.n_cmd = 100 + i
    f.n_state = 100 + i
    return f


def test_recorder(tmp):
    print("--- 1. Recorder: CSV / 关键帧 / 汇总 ---")
    Recorder, _, CSV_COLUMNS, _, _ = _load_record()
    out = os.path.join(tmp, "run1")
    rec = Recorder(out, fps=10.0, save_frames=True)
    check("CSV 在写出前就已创建", os.path.isfile(rec.csv_path), os.path.basename(rec.csv_path))

    frames = [make_frame(i) for i in range(30)]

    def render_fn():
        return np.full((16, 24, 3), 128, dtype=np.uint8)

    for f in frames:
        rec.write(f, render_fn=render_fn)
    check("30 帧全部写入", rec.rows == 30, "rows = %d" % rec.rows)

    with open(rec.csv_path, newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    header, data = rows[0], rows[1:]
    check("CSV 数据行数 = 30", len(data) == 30, "数据行 = %d" % len(data))
    for need in ("t", "z", "tilt", "level"):
        check("表头含 %s" % need, need in header)
    check("tau 列正好 12 条", sum(1 for h in header if h.startswith("tau_")
                                     and h != "tau_max_abs") == 12)
    bad = [i for i, r in enumerate(data) if len(r) != len(header)]
    check("每行列数 == 表头列数", not bad, "列数 = %d，异常行 = %s" % (len(header), bad[:3]))
    check("列数与模块常量 CSV_COLUMNS 一致", len(header) == len(CSV_COLUMNS), "%d 列" % len(header))
    check("第 0 行 t == 0.0", abs(float(data[0][header.index("t")]) - 0.0) < 1e-9)
    check("第 29 行 t == 2.9", abs(float(data[29][header.index("t")]) - 2.9) < 1e-9)
    vxt = data[1][header.index("vx_truth")]
    check("无真值帧的 vx_truth 写成 nan", vxt.lower() in ("nan", ""), "value = %r" % vxt)
    check("关键帧数受 fps 限制且 > 0", 0 < rec.frames_saved <= 30, "frames = %d" % rec.frames_saved)
    kf = os.listdir(rec.frames_dir) if os.path.isdir(rec.frames_dir) else []
    check("关键帧 PNG 真的落盘", len(kf) == rec.frames_saved and all(n.endswith(".png") for n in kf),
          "%d 个文件" % len(kf))

    res = rec.close(summary={"duration_s": 3.0, "note": "unit-test"})
    check("close() 返回 dict", isinstance(res, dict))
    check("返回 rows == 30", res["rows"] == 30)
    for key in ("summary_png", "report_md"):
        p = res[key]
        check("%s 存在且非空" % os.path.basename(p),
              os.path.isfile(p) and os.path.getsize(p) > 0,
              "%d bytes" % (os.path.getsize(p) if os.path.isfile(p) else -1))
    check("summary.png 是合法 PNG（魔数）",
          open(res["summary_png"], "rb").read(8) == b"\x89PNG\r\n\x1a\n")
    txt = open(res["report_md"], encoding="utf-8").read()
    check("report.md 含统计表 / CSV 列 / summary", "## 统计" in txt and "tau_max_abs" in txt
          and "unit-test" in txt)
    check("close() 幂等", rec.close()["rows"] == 30)
    check("close() 后再 write() 抛 RuntimeError",
          _raises(RuntimeError, lambda: rec.write(frames[0])), "RuntimeError")
    return res


def test_recorder_variants(tmp):
    print("--- 2. Recorder 的降级 / 边界 ---")
    Recorder, _, _, _, _ = _load_record()
    frames = [make_frame(i) for i in range(6)]

    rec_off = Recorder(os.path.join(tmp, "run_off"), save_frames=False)
    calls = []
    for f in frames:
        rec_off.write(f, render_fn=lambda: calls.append(1))
    check("save_frames=False 不调用 render_fn", not calls)
    res_off = rec_off.close()
    check("save_frames=False 仍写 summary/report",
          os.path.getsize(res_off["summary_png"]) > 0
          and os.path.getsize(res_off["report_md"]) > 0)
    check("save_frames=False 时不建 frames 目录", res_off["frames_dir"] is None)

    rec_none = Recorder(os.path.join(tmp, "run_none"))

    def boom():
        raise RuntimeError("模拟渲染崩溃")

    rec_none.write(make_frame(0), render_fn=lambda: None)
    rec_none.write(make_frame(1), render_fn=boom)
    check("render_fn 返回 None / 抛异常都不打断记录", rec_none.rows == 2,
          "rows = %d" % rec_none.rows)
    res_none = rec_none.close()
    check("渲染异常被记进 warnings", any("render_fn" in w for w in res_none["warnings"]),
          "%d 条 warning" % len(res_none["warnings"]))

    rec_bare = Recorder(os.path.join(tmp, "run_bare"))
    rec_bare.write(object())          # 完全裸对象：所有字段走 getattr 默认值
    check("裸对象也能写一行 CSV（防御性）", rec_bare.rows == 1 and rec_bare.close()["rows"] == 1)

    rec_empty = Recorder(os.path.join(tmp, "run_empty"))
    res_empty = rec_empty.close()
    check("零帧 close() 也产出两个文件",
          res_empty["rows"] == 0 and os.path.getsize(res_empty["summary_png"]) > 0
          and os.path.getsize(res_empty["report_md"]) > 0,
          "rows = %d" % res_empty["rows"])


def test_dashboard():
    print("--- 3. TerminalDashboard: 无 GUI、固定行数、行不折 ---")
    _, TerminalDashboard, _, display_width, _ = _load_record()
    guis = [m for m in ("tkinter", "PySide6", "PyQt5", "PyQt6") if m in sys.modules]
    check("record.py 的导入链不含 GUI 工具包", not guis, "已加载: %s" % guis)

    dash = TerminalDashboard(width=110)
    bare = dash.render(object())
    check("裸对象 render() 也不崩且行数固定",
          isinstance(bare, str) and len(bare.split("\n")) == dash.line_count)
    f0 = make_frame(0)
    s0 = dash.render(f0)
    check("render() 返回 str", isinstance(s0, str))
    check("以 ANSI 清屏序列开头", s0.startswith("\033[H\033[J"), repr(s0[:8]))
    lines0 = s0.split("\n")
    check("行数固定 == line_count", len(lines0) == dash.line_count,
          "%d vs %d" % (len(lines0), dash.line_count))
    check("含等级字样 L0", "L0" in s0)
    check("含模式字样 track", "track" in s0)
    check("tau 正负用不同字符（= 与 -）", "=" in s0 and "-" in s0)
    check("含四条腿名", all(leg in s0 for leg in ("FR", "FL", "RR", "RL")))
    w0 = [display_width(l) for l in lines0]
    check("每行显示宽度 == width（不折行）", all(w == dash.width for w in w0),
          "min=%d max=%d width=%d" % (min(w0), max(w0), dash.width))
    check("无空行、无尾随换行", "\n\n" not in s0 and not s0.endswith("\n"))

    f3 = make_frame(3)  # level 3 + estop + 无真值
    s3 = dash.render(f3)
    check("换一帧后行数仍固定", len(s3.split("\n")) == dash.line_count)
    check("含等级字样 L3", "L3" in s3)
    check("L3 帧含 estop 模式", "estop" in s3)
    check("无真值时显示 n/a", "n/a" in s3)
    w3 = [display_width(l) for l in s3.split("\n")]
    check("L3 帧行宽也 == width", all(w == dash.width for w in w3))

    narrow = TerminalDashboard(width=40)
    sn = narrow.render(f0)
    wn = [display_width(l) for l in sn.split("\n")]
    check("width=40 仍固定行数且不超宽",
          len(sn.split("\n")) == narrow.line_count and all(w <= 40 for w in wn),
          "max = %d" % max(wn))

    plain = TerminalDashboard(width=80, use_color=False, clear_screen=False)
    sp = plain.render(f0)
    check("use_color=False 时无 ANSI 颜色码", "\033[" not in sp)

    buf = io.StringIO()
    rate = TerminalDashboard(width=90)
    ok1 = rate.write(f0, stream=buf, now=1000.0)
    ok2 = rate.write(f0, stream=buf, now=1000.01)
    ok3 = rate.write(f0, stream=buf, now=1000.20)
    check("write() 按 10 Hz 限频", (ok1, ok2, ok3) == (True, False, True),
          "writes = %s" % [ok1, ok2, ok3])
    check("write() 真的写了两个整屏", buf.getvalue().count("\033[H\033[J") == 2)

    s_extra = rate.render(f0, extra={"tau_scale": 40.0, "note": "extra-note"})
    check("extra 不改变行数且能带注释",
          len(s_extra.split("\n")) == rate.line_count and "extra-note" in s_extra)


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="viz_record_test_") as tmp:
        test_recorder(tmp)
        test_recorder_variants(tmp)
        test_dashboard()
    print("OK test_viz_record: 全部通过（共 %d 项）" % OK)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
