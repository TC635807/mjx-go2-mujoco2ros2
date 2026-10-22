# 可视化观察器 · 接口契约 v1（冻结）

> 目标：一个脚本，在**仿真环境里实时看闭环保洁效果**，并且能看到策略/观测内部状态。
> 一个 MuJoCo 窗口里看全部：3D 叠加 + 原生曲线 + 仪表盘图像 + 文字 HUD；无界面时退化成终端仪表盘。

## 0. 为什么能做到「单窗口、零额外依赖」

mujoco 3.12 的 `launch_passive` 返回的 `Handle` 暴露了四件套（已实测）：

    handle.user_scn            # MjvScene：加 3D geom 标记（点/箭头/接触球/轨迹）
    handle.set_figures([(MjrRect, MjvFigure)])   # 原生 strip chart（实时曲线）
    handle.set_images([(MjrRect, rgb_uint8)])    # 叠任意 RGB 图像 -> 可放 matplotlib 渲染的仪表盘
    handle.set_texts([(font, gridpos, text1, text2)])  # 文字 HUD

所以曲线用 MjvFigure、复杂面板用 matplotlib(agg) 渲染成 RGB 再 set_images，**不需要 tkinter/Qt**。
本机 venv 已装 matplotlib 3.11.2（agg 后端，够用；没有交互后端也没关系）。

## 1. 全局约定

- 项目根 `/home/tc63/mujoco/mjx-go2-mujoco2ros2`，用 `.venv/bin/python`。
- **测试不用 pytest**：`tests/test_xxx.py` 要能直接 `python` 跑，成功打印 `OK ...` 并 exit 0。
- 运行时模块禁止 import jax/brax/rclpy；`viz/` 里可以用 numpy + mujoco + matplotlib(仅 figures 的 image panel)。
- 关节顺序 FR/FL/RR/RL x hip/thigh/calf；四元数 wxyz；单位 SI，角度 rad。
- **只改自己 write_scopes 里的文件**。

## 2. `src/go2_bridge/viz/frame.py`（Lead 提供，不要改）

所有可视化组件都只消费这一个数据类，互相之间不直接依赖。

    @dataclass
    class VizFrame:
        t: float; wall: float
        base_pos: np.ndarray          # (3,) 世界系
        base_quat: np.ndarray         # (4,) wxyz
        base_lin_vel_world: np.ndarray # (3,) 观测里用的（估计值）
        base_lin_vel_truth: Optional[np.ndarray]  # (3,) 仿真真值，真机为 None
        rpy: np.ndarray               # (3,)
        joint_q: np.ndarray           # (12,)
        joint_dq: np.ndarray          # (12,)
        joint_q_des: np.ndarray       # (12,) 指令目标
        joint_tau: np.ndarray         # (12,) 实际施加力矩
        foot_pos_world: np.ndarray    # (4,3)
        foot_force: np.ndarray        # (4,)
        scan_xy_body: np.ndarray      # (35,2) 机体系采样点（不含 yaw 旋转）
        scan_xy_world: np.ndarray     # (35,3) 世界系采样点（z = 地面高度）
        height_scan: np.ndarray       # (35,) 特征值（clip 后 *5）
        phase: np.ndarray             # (4,) 每条腿的相位 phi_i (rad)
        action: np.ndarray            # (12,) 策略原始输出
        action_applied: np.ndarray    # (12,) 夹紧后真正下发的目标
        obs: np.ndarray               # (91,)
        level: int; level_name: str   # 0..3 / "L0".."L3"
        mode: str                     # track/crouch/damping/estop
        obs_valid: bool
        sources: Dict[str, str]       # 每一路来源：oracle/estimator/leg_fk/nominal/lidar/zero
        rtf: float; overruns: int; n_cmd: int; n_state: int

## 3. `src/go2_bridge/viz/scene.py`（owner: viz-scene）

    class SceneOverlay:
        def __init__(self, model, data, cfg: Optional[dict] = None): ...
        def reset(self) -> None: ...
        def draw(self, scn, frame) -> None:
            """往 scn 里追加 geom。必须自己管 ngeom，别越界；每帧重画。"""
        def set_camera(self, cam, frame) -> None:   # 可选：跟随躯干 / 俯视
        def stats(self) -> dict: ...

必须画出来的东西（这是这个脚本最有价值的部分）：
1. **35 个高度采样点**：位置取 frame.scan_xy_world，半径按 |height_scan| 缩放，
   颜色映射 height_scan（**负=地面更高/上坡，正=下陷**，用冷暖色区分并在图例里说明）。
2. **4 个足端接触标记**：foot_force > 阈值 用绿色球，否则灰色空心/小点。
3. **基座速度箭头**：从 base_pos 出发指向 base_pos + v*scale，估计用蓝色；
   若有真值，真值用橙色画在旁边对比。
4. **指令箭头**：世界系 +x 方向的指令速度（0.5 m/s），单独颜色。
5. **轨迹拖尾**：基座最近 N 个位置的小球（N 可由 cfg 给，默认 400）。

性能要求：draw() 单帧 < 1 ms（不要每帧 new 大数组）；scn.maxgeom 不够时要能优雅退化。

验收 `tests/test_viz_scene.py`：
- 造一个假 VizFrame + 真 `mujoco.MjvScene`，调用 draw()，断言 `scn.ngeom` 增加了预期数量且 <= maxgeom；
- 断言高度点颜色随 height_scan 符号变化（取两个极值点比较 rgba）；
- 连续调用 3 次 draw()，断言**不会**累积（每次重画后 ngeom 相等）。

## 4. `src/go2_bridge/viz/panels.py`（owner: viz-panels）

    class FigurePanel:
        """原生 MjvFigure 实时曲线。"""
        def __init__(self, which: Sequence[str] = ("tau",), window: float = 5.0): ...
        def update(self, frame) -> None: ...            # 推一个样本
        def viewports(self, width: int, height: int) -> List[Tuple[MjrRect, MjvFigure]]: ...

    class ImagePanel:
        """matplotlib(agg) 渲染成 RGB，交给 handle.set_images 叠加。"""
        def __init__(self, dpi: int = 80, size=(6.4, 3.6)): ...
        def update(self, frame) -> None: ...
        def viewport_image(self, width: int, height: int) -> Optional[Tuple[MjrRect, np.ndarray]]: ...

    class TextHud:
        def update(self, frame, extra: Optional[dict] = None) -> None: ...
        def texts(self) -> List[tuple]: ...             # (font, gridpos, text1, text2)

FigurePanel 必须支持的图（`which` 里选）：
- "tau"      12 个关节力矩（6 条曲线，左右腿各一组或按腿分组）
- "height"   基座高度 z 与倾斜角（两条 y 轴含义不同也没关系，同一图上归一化画）
- "vel"      基座线速度：估计 vs 真值（3 分量各一条 + 范数）
- "action"   动作幅值 mean/max + 夹紧计数
- "err"      估计误差 |v_est - v_true|（有真值才有意义）

ImagePanel 里画一个「仪表盘」：
- 左上：height scan 7x5 热图（机体系，x 向右为前），带 colorbar 与符号说明
- 右上：4 条腿的步态相位极坐标（或圆形进度）
- 下：obs 91 维的分段条形（10 段，标注段名与当前值范围）
- 左下角文字：当前高度特征均值、动作 mean/max、健康等级

性能：`ImagePanel.update()` 到 20 Hz 即可，单帧 < 15 ms；渲染失败（无 matplotlib）要能降级为 None 而不是抛异常。

验收 `tests/test_viz_panels.py`：
- 造 100 帧假 VizFrame 推进，断言 FigurePanel 内部缓冲长度受限（滚动窗口）；
- 断言 `viewports()` 返回的 MjrRect 在给定宽高内、MjvFigure 的 linename/linewidth 已设置；
- ImagePanel：断言 `viewport_image()` 返回 (MjrRect, uint8 数组) 且 shape 与 rect 一致；
- TextHud：断言 texts() 返回的每条都是 4 元组且 text1/text2 是 str。

## 5. `src/go2_bridge/viz/record.py`（owner: viz-record）

    class Recorder:
        """落盘：CSV + 关键帧 PNG + 结尾汇总图。"""
        def __init__(self, out_dir: str, fps: float = 10.0, save_frames: bool = True): ...
        def write(self, frame, render_fn=None) -> None: ...   # render_fn() 返回 HxWx3 uint8 时存帧
        def close(self, summary: Optional[dict] = None) -> dict: ...   # 写 summary.png + report.md

    class TerminalDashboard:
        """无界面时的替代：ANSI 原地刷新（\033[H\033[J），10 Hz，不需要任何 GUI。"""
        def __init__(self, width: int = 100): ...
        def render(self, frame, extra: Optional[dict] = None) -> str: ...

TerminalDashboard 至少要有：健康等级（L0-L3 带颜色）、模式、tilt、z、vx(估计/真值/误差)、
每腿接触、tau 的 12 条横向条（用 ▁▂▃▅▇ 或 = 与 - 表示正负）、动作 mean/max、
height scan 均值、RTF、overrun、n_cmd/n_state、每一路来源。

验收 `tests/test_viz_record.py`：
- 用假 VizFrame 写 30 帧到临时目录，断言 CSV 行数=30、表头含 t/z/tilt/level、列数与表头一致；
- 断言 close() 之后 summary.png 与 report.md 存在且非空；
- TerminalDashboard.render() 的返回是 str、不含换行残留问题（行数固定），且包含 "L0" 之类的等级字样。

## 6. Lead 负责（不要碰）

`src/go2_bridge/viz/__init__.py`、`src/go2_bridge/viz/frame.py`、`scripts/view_policy.py`、
`docs/viz.md`、`README.md`、其余 tests。

## 7. 驾驶脚本 `scripts/view_policy.py` 的形态（Lead 写，供你理解上下文）

    .venv/bin/python scripts/view_policy.py --robot go2 --controller policy --duration 60
    .venv/bin/python scripts/view_policy.py --robot go2 --controller policy --scan nominal
    .venv/bin/python scripts/view_policy.py --headless --duration 20   # 终端仪表盘 + CSV

单进程：Go2Sim + PolicyController + HealthMonitor + SafetySupervisor 全跑起来，
500 Hz 物理，观测每拍收集，3D 同步 ~60 Hz，曲线 ~20 Hz，仪表盘 ~15 Hz，终端 ~10 Hz。
