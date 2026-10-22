"""可视化观察器：一个 MuJoCo 窗口里看闭环效果 + 策略/观测内部状态。

    frame.py     VizFrame（所有组件共用的唯一数据结构）+ 地面高度采集
    scene.py     3D 叠加：35 个高度采样点 / 足端接触 / 速度与指令箭头 / 轨迹
    panels.py    MjvFigure 实时曲线 + matplotlib 仪表盘图像 + 文字 HUD
    record.py    CSV / 关键帧 / 汇总图 + 无界面终端仪表盘

入口是 scripts/view_policy.py。契约见 docs/viz-interfaces.md。
"""

from .frame import (FrameCollector, FlatGround, RaycastGround, VizFrame,  # noqa: F401
                    make_ground)

# scene / panels 由队友并行开发，允许暂时缺失（driver 也是这么兜的）
try:
    from .scene import SceneOverlay  # noqa: F401
except Exception:  # pragma: no cover
    SceneOverlay = None  # type: ignore
try:
    from .panels import FigurePanel, ImagePanel, TextHud  # noqa: F401
except Exception:  # pragma: no cover
    FigurePanel = ImagePanel = TextHud = None  # type: ignore
try:
    from .record import Recorder, TerminalDashboard  # noqa: F401
except Exception:  # pragma: no cover
    Recorder = TerminalDashboard = None  # type: ignore

__all__ = ["VizFrame", "FrameCollector", "FlatGround", "RaycastGround", "make_ground",
           "SceneOverlay", "FigurePanel", "ImagePanel", "TextHud",
           "Recorder", "TerminalDashboard"]
