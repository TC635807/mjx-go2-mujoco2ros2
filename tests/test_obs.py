#!/usr/bin/env python3
"""tests/test_obs.py —— Go2 观测构造 (243 维) 逐字段验证。可直接运行, 无 pytest。

布局 (Go2 地形行走策略):
    0:3    基座线速度 (训练是 body 系, *2.0)
    3:6    IMU 陀螺 (*0.25)
    6:9    projected gravity
    9:12   速度指令 (*2.0)
    12:24  关节角 - 训练 home (12)
    24:36  关节速度 (不缩放)
    36:48  上一拍动作 (12)
    48:235 高度扫描 (17x11=187, 特征 = clip(z-h-base, ±0.35)*5)
    235:243 步态相位 (cos,sin) x 4 腿
history_frames=3 时再追加 112 维历史帧 (base48+phase8 每帧)。
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from signal_io.msg import IMUState, LowState, MotorState  # noqa: E402
from ros2_controller.obs import (  # noqa: E402
    TROT_PHASE, WalkObsBuilder, WalkObsConfig,
)

GO2_HOME = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
                     -0.1, 1.0, -1.5, 0.1, 1.0, -1.5], dtype=np.float64)
SCAN_N = 17 * 11
FAILS = []


def check(name, ok, detail=""):
    print(("  [OK]   " if ok else "  [FAIL] ") + name + (("   " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def quat_yaw(yaw):
    return [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]


def make_state(q=None, dq=0.1, gyro=(0.04, -0.08, 0.12), quat=(1, 0, 0, 0)):
    st = LowState()
    st.imu_state = IMUState(quaternion=list(quat),
                            gyroscope=list(gyro),
                            accelerometer=[0.0, 0.0, 9.81], rpy=[0, 0, 0])
    qq = GO2_HOME.copy() if q is None else np.asarray(q, dtype=np.float64)
    for i in range(12):
        st.motor_state[i] = MotorState(mode=1, q=float(qq[i]),
                                       dq=float(dq) if np.isscalar(dq) else float(dq[i]),
                                       tau_est=0.0)
    return st


def make_builder(**kw):
    cfg = WalkObsConfig(scan_nx=17, scan_ny=11,
                        scan_x_range=(-0.4, 1.2), scan_y_range=(-0.5, 0.5),
                        command=(0.5, 0.0, 0.0), linvel_frame="body",
                        include_gait_phase=kw.pop("phase", True),
                        history_frames=kw.pop("history", 1), **kw)
    return WalkObsBuilder(cfg, home_q=GO2_HOME)


def main() -> int:
    print("[1] 维度: 243 = 48 + 187 + 8")
    b = make_builder()
    check("obs_dim == 243", b.obs_dim == 243, "obs_dim=%d" % b.obs_dim)
    check("scan_grid 形状 (187,2)", b.scan_grid().shape == (SCAN_N, 2))
    g = b.scan_grid()
    check("scan 网格角点 = (-0.4,-0.5) / (1.2,0.5)",
          np.allclose(g[0], [-0.4, -0.5]) and np.allclose(g[-1], [1.2, 0.5]), "%s / %s" % (g[0], g[-1]))

    scan = np.linspace(-0.5, 1.5, SCAN_N)
    st = make_state()
    obs, meta = b.observe(st, 0.02, base_lin_vel_world=np.array([0.3, 0.2, 0.0]),
                          height_scan=scan)
    check("meta.valid", meta.valid and meta.linvel_source == "estimator")
    obs = np.asarray(obs, dtype=np.float64)
    check("0:3 = body 线速度 * 2", np.allclose(obs[0:3], [0.6, 0.4, 0.0], atol=1e-6), str(obs[:3]))
    check("3:6 = gyro * 0.25", np.allclose(obs[3:6], np.array([0.04, -0.08, 0.12]) * 0.25, atol=1e-6))
    check("6:9 = projected_gravity ≈ (0,0,-1)", np.allclose(obs[6:9], [0, 0, -1], atol=1e-6), str(obs[6:9]))
    check("9:12 = 指令 * 2 (vx=0.5 -> 1.0)", np.allclose(obs[9:12], [1.0, 0, 0], atol=1e-6))
    check("12:24 = q - 训练 home (0)", np.allclose(obs[12:24], 0.0, atol=1e-6))
    check("24:36 = 关节速度 (不缩放)", np.allclose(obs[24:36], 0.1, atol=1e-6))
    check("36:48 = 上一拍动作 (初始 0)", np.allclose(obs[36:48], 0.0))
    check("48:235 = 高度扫描原样", np.allclose(obs[48:48 + SCAN_N], scan, atol=1e-6))

    phi = np.arctan2(obs[239], obs[235])
    check("235:243 = [cos,sin] 步态相位", np.allclose(
        obs[235:243], np.concatenate([np.cos(TROT_PHASE + phi), np.sin(TROT_PHASE + phi)]),
        atol=1e-5))

    print("[2] 机体系速度: yaw=90°, 世界 (0.3,0.2) -> 机体 (0.2,-0.3)")
    b2 = make_builder()
    st2 = make_state(quat=quat_yaw(np.pi / 2))
    o2, _ = b2.observe(st2, 0.02, base_lin_vel_world=np.array([0.3, 0.2, 0.0]),
                       height_scan=np.zeros(SCAN_N))
    check("0:3 ≈ (0.4, -0.6, 0)", np.allclose(o2[0:3], [0.4, -0.6, 0.0], atol=1e-5), str(o2[:3]))

    print("[2b] projected_gravity == R^T @ [0,0,-1] (含 roll/pitch; 回归: 曾把第三列当第三行)")
    from ros2_controller.obs import projected_gravity, quat_to_mat
    rng = np.random.default_rng(0)
    worst_g = 0.0
    for _ in range(200):
        qq = rng.normal(size=4)
        qq = qq / np.linalg.norm(qq)
        g = projected_gravity(qq)
        ref = quat_to_mat(qq).T @ np.array([0.0, 0.0, -1.0])
        worst_g = max(worst_g, float(np.max(np.abs(g - ref))))
    check("200 个随机四元数 max|diff| < 1e-12", worst_g < 1e-12, "max|diff|=%.2e" % worst_g)
    q_tilt = [np.cos(0.1), np.sin(0.1), 0.0, 0.0]   # 绕 x 轴 11.5° 侧倾
    g_tilt = projected_gravity(q_tilt)
    check("侧倾 11.5° 时 gravity y < 0 (=-sin(0.2), 旧实现是 +sin)", float(g_tilt[1]) < -0.19,
          str(np.round(g_tilt, 4)))

    print("[3] 指令可切换 / commit 进入下一拍")
    b3 = make_builder()
    b3._cmd = np.array([0.0, 0.0, 0.5])
    o3, _ = b3.observe(st, 0.02, base_lin_vel_world=np.zeros(3), height_scan=np.zeros(SCAN_N))
    check("9:12 = (0,0,0.5)*2", np.allclose(o3[9:12], [0, 0, 1.0], atol=1e-6), str(o3[9:12]))
    act = np.linspace(-1, 1, 12)
    b3.commit(act)
    o4, _ = b3.observe(st, 0.04, base_lin_vel_world=np.zeros(3), height_scan=np.zeros(SCAN_N))
    check("commit 后 36:48 = 上一拍动作", np.allclose(o4[36:48], act, atol=1e-6))

    print("[4] obs 截断 (obs_clip=100)")
    b4 = make_builder()
    o5, _ = b4.observe(st, 0.02, base_lin_vel_world=np.array([1e4, 0, 0]),
                       height_scan=np.full(SCAN_N, 1e4))
    check("线速度特征被截到 100", abs(float(o5[0]) - 100.0) < 1e-3, "obs[0]=%.3f" % o5[0])

    print("[5] 无相位档: 235 维")
    b5 = make_builder(phase=False)
    check("无相位 obs_dim == 235", b5.obs_dim == 235, "obs_dim=%d" % b5.obs_dim)

    print("[6] 历史帧 (n=3): 355 维 + 帧顺序 + reset 清零")
    b6 = make_builder(history=3)
    check("history=3 -> obs_dim 355", b6.obs_dim == 355, "obs_dim=%d" % b6.obs_dim)
    o6, _ = b6.observe(st, 0.02, base_lin_vel_world=np.array([0.1, 0.0, 0.0]),
                       height_scan=np.zeros(SCAN_N))
    # 部署侧 warm-start: 第一拍历史 = [当前帧, 0...] (模拟训练 reset 时先写一帧)
    check("第一拍 history[0:56] == 当前帧 base+phase",
          np.allclose(o6[243:243 + 56], np.concatenate([o6[0:48], o6[235:243]]), atol=1e-5)
          and np.allclose(o6[243 + 56:355], 0.0))
    o7, _ = b6.observe(st, 0.04, base_lin_vel_world=np.array([0.1, 0.0, 0.0]),
                       height_scan=np.zeros(SCAN_N))
    check("第二拍 hist[0:56] == 第一拍 base+phase",
          np.allclose(o7[243:243 + 56], np.concatenate([o6[0:48], o6[235:243]]), atol=1e-5))
    b6.reset()
    o8, _ = b6.observe(st, 0.02, base_lin_vel_world=np.zeros(3), height_scan=np.zeros(SCAN_N))
    check("reset 后 history 重新 warm-start (第 1 帧=当前帧, 其余 0) + last_action 清零",
          np.allclose(o8[243:243 + 56], np.concatenate([o8[0:48], o8[235:243]]), atol=1e-5)
          and np.allclose(o8[243 + 56:355], 0.0) and np.allclose(o8[36:48], 0.0))

    print("[7] 缺状态: valid=False, 全 0 (契约)")
    o9, meta9 = b.observe(None, 0.02, base_lin_vel_world=None, height_scan=None)
    check("state=None -> 全 0 obs / valid=False", (not meta9.valid) and np.allclose(o9, 0.0))

    print("[8] scan_feature 定义: clip(z-h-base, ±clip)*scale")
    f = b.scan_feature(0.35, 0.0)
    check("平地 z=0.35 -> +0.35 (base=0.28)", abs(float(f) - 0.35) < 1e-9, str(f))
    f = b.scan_feature(0.0, 0.30)
    check("地面更高 -> 负特征", float(f) < 0.0, str(f))

    print()
    if FAILS:
        print("FAILED %d 项: %s" % (len(FAILS), "; ".join(FAILS)))
        return 1
    print("OK test_obs: 全部通过 (Go2 243 维观测)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
