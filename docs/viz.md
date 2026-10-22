# 可视化观察器（view_policy.py）

> 一个脚本，在**仿真环境里实时看闭环效果**，并且把策略/观测的内部状态摊开。

    .venv/bin/python scripts/view_policy.py --robot go2 --controller policy --duration 60

一个 MuJoCo 窗口里同时看到四层信息，**不需要 tkinter / Qt**（用的是 mujoco passive viewer 原生四件套）：

| 层 | 用的接口 | 看什么 |
|---|---|---|
| 3D 叠加 | `handle.user_scn` | **35 个高度采样点**（按特征值着色）· 足端接触标记 · 基座速度箭头（估计/真值）· 指令箭头 · 轨迹拖尾 |
| 实时曲线 | `handle.set_figures` | 关节力矩 · 基座高度与倾斜 · 线速度（估计 vs 真值）· 动作幅值 · 估计误差 |
| 仪表盘 | `handle.set_images` | height scan 7x5 热图 · 4 腿相位 · 91 维 obs 分段条形 · 关键数字 |
| 文字 HUD | `handle.set_texts` | 健康等级 L0-L3 · 模式 · tilt · z · vx · tau_max · 每一路来源 |

## 为什么这么设计

最值钱的是**把策略"眼里的地形"画回 3D 场景里**：35 个采样点的位置和颜色就是送进策略的
height scan。地面更高（上坡/台阶）为负色、下陷为正色，一眼就能看出策略看到的是不是真地形，
以及 leg_fk / nominal 那种"平地假设"偏了多少。

`FrameCollector`（src/go2_bridge/viz/frame.py）是唯一的采集点，3D/曲线/仪表盘/记录器都只消费
同一个 `VizFrame`，所以四个人并行写也不会接口漂移。

## 常用命令

    # 默认：Go2 + 策略，60 秒，平地，软足端
    .venv/bin/python scripts/view_policy.py

    # 真机同款机型 + 平地假设的高度来源（Go2 上这个组合最稳）
    .venv/bin/python scripts/view_policy.py --robot go2 --scan nominal

    # 带台阶的场景：这时地面高度用射线量（--scene obstacles 自动切 raycast）
    .venv/bin/python scripts/view_policy.py --robot go2 --scene obstacles

    # 只看站立控制器（不加载策略），确认链路
    .venv/bin/python scripts/view_policy.py --controller stand --duration 15

    # 不用图形界面：终端 ANSI 仪表盘 + CSV + summary.png
    .venv/bin/python scripts/view_policy.py --headless --duration 20 --record logs/obs_demo

    # 离屏截图（不需要窗口），用来留证或放进报告
    .venv/bin/python scripts/view_policy.py --snapshot docs/assets/observer.png --snapshot-at 6

    # 相机跟随躯干
    .venv/bin/python scripts/view_policy.py --follow

## 窗口里的键盘

| 键 | 作用 |
|---|---|
| `r` | 重置仿真 + 控制器 + 健康层 |
| `空格` | 暂停 / 继续 |
| `h` | 开关 3D 叠加 |
| `f` | 开关实时曲线 |
| `i` | 开关仪表盘图像 |
| `q` / `Esc` | 退出 |

## 参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `--robot` | go2 | go2=训练侧模型，go2=真机同款 |
| `--controller` | policy | policy / stand / zero / lqr_stub |
| `--scene` | flat | flat=纯平面；obstacles=带台阶（自动用射线量地面高度） |
| `--scan` | 配置里的值 | nominal=按标称站高；leg_fk=接触腿正运动学 |
| `--follow` | 关 | 相机跟随躯干 |
| `--headless` | 关 | 不启动 3D 窗口，只出终端仪表盘 |
| `--record DIR` | 关 | 落盘 CSV + 关键帧 + summary.png + report.md |
| `--snapshot PATH` | 关 | 离屏渲染一张带叠加标记的 PNG |
| `--plots` | window | window=独立数据窗口；inline=内嵌到 MuJoCo 窗口；off=不画 |
| `--plot-hz` | 3 | 独立数据窗口刷新率（会按 RTF 自动减半）|
| `--figures` | tau,height,vel | 仅 inline 模式下生效 |
| `--no-dashboard` | 开 | 仅 inline 模式下生效 |
| `--hud` | on | 左上角文字 HUD（很小一块） |
| `--lite` | on | 关掉软件渲染下最贵的视觉效果 |
| `--trail N` | 120 | 轨迹点数，0 = 不画 |
| `--sync-hz` / `--fig-hz` / `--img-hz` / `--hud-hz` | 20 / 2 / 1 / 3 | 各层刷新率 |

## 两个窗口（默认）

    ┌─ MuJoCo 窗口 ────────────┐   ┌─ 数据窗口 (matplotlib/QtAgg) ────────┐
    │  机器人 + 3D 叠加标记     │   │  力矩 | 高度倾斜 | 速度               │
    │  + 右上角小 HUD           │   │  高度扫描 | 步态相位 | 动作           │
    │  （始终不被遮挡）         │   │  13x7.5 英寸，可缩放、可拖走          │
    └──────────────────────────┘   └──────────────────────────────────────┘

为什么必须拆开：MuJoCo 的窗口尺寸是它自己创建的（本机实测 **740x800**），塞进去的曲线
每块只有 **89x161 px**，根本看不清；把它放大到能看清就必然糊住机器人。
拆成两个窗口后，两边各自都能做对。

用 `--plots inline` 可以退回单窗口内嵌模式（边角布局，遮挡 13.4%），
`--plots off` 完全不画。

## 帧率实测

| 配置 | 结果 |
|---|---|
| 单窗口、曲线铺满整窗、同步 60 Hz | RTF **0.036**（6 s 仿真跑了 2 分 49 秒） |
| 两个窗口：MuJoCo 同步 15 Hz + 数据窗 3 Hz | **RTF 1.00**，sync 中位 **2.4 ms**，12 s 仿真 13.7 s 跑完 |

数据窗口的成本：Agg 每次**全图重绘 ~150-200 ms**（实测拆解：12 条力矩线 21 ms、
每多一个曲线轴 +18 ms、imshow +16 ms、polar 轴 +24 ms、柱状 +19 ms）。所以：

- 数据窗口默认 **3 Hz**（看趋势足够），并带**自适应**：RTF < 0.95 时自动减半刷新率；
- 相位面板用普通 axes 画横条、扫描面板不放 colorbar（省掉 polar 和 colorbar 共 ~32 ms/帧）；
- MuJoCo 窗口这边有**帧率看门狗**：同步中位 > 120 ms 就逐级降级（仪表盘 -> 曲线 -> HUD）。

顺带一个坑：装了 figures/images 之后，GLFW 窗口的 GL context 在**退出时**可能挂住上百秒
（实测 8 s 的 run 卡在退出 190 s）。脚本在写完记录后 `os._exit(0)`，跳过解释器清理。

## 读图指南

- **高度采样点颜色**：`clip(z_base - h_ground - 0.28, ±0.35) * 5.0`。
  平地且站高 0.28 m 时为 0（中性色）；地面更高 -> 负；躯干更高 -> 正。
  它不是"到地面的距离"，而是**相对标称站高的偏差**，这一点很容易读错。
- **速度箭头**：蓝色是送进策略的估计值（腿式里程计），橙色是仿真真值（`qvel[0:3]`）。
  两者分开画，是为了随时看出"观测器估得准不准"——这正是 sim2real 第一号风险。
- **健康等级**：L0 正常 / L1 降级（任务降级）/ L2 保护（下蹲）/ L3 卸力锁存。
  等级一跳，安全层就会改写指令，看 tau 曲线会同时出现断崖。
- **RTF / overrun**：观察器本身会吃 CPU（3D 同步 + 曲线 + 仪表盘），`--sync-hz` 调低可以换更多余量。

## 离屏截图（不需要窗口）

`--snapshot` 走 `mujoco.Renderer` 离屏渲染，会把 3D 叠加一起画进去（先 `update_scene` 再追加 geom），
所以 CI / 无显示器环境也能出图。两个注意点：

1. 离屏 framebuffer 默认只有 640x480，脚本会先把 `model.vis.global_.offwidth/offheight` 调大；
2. Renderer 在进程退出时销毁 GL context 会偶发 SIGSEGV（WSL/GLFW 已知问题），
   脚本在 `finally` 里主动 `renderer.close()` 规避。

## 已知限制

- 观察器是**单进程**的（Go2Sim + 控制器 + 健康层 + 安全层都在一个进程里），
  好处是能看到全部内部状态；想看**两个独立进程走 ROS 2** 的那条链路，请用 `scripts/demo_ros2_policy.sh`。
- 终端仪表盘只在 `--headless` 下实时打印；有窗口时它只统计不打印（避免和窗口抢终端）。
- 曲线/仪表盘的刷新率是抽帧的（默认 20 / 10 Hz），不会拖慢 500 Hz 物理。
- 场景里的地面高度：`--scene flat` 恒为 0；`--scene obstacles` 用 `mj_ray` 只打 geomgroup 0（地面/障碍），
  所以机器人自己的腿不会被当成地面。
