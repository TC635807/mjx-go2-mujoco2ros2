"""tests/test_estimator.py —— LegKinematics (FK/雅可比) 与 LegOdometry 的验证。

可直接 python 运行 (无 pytest):   python tests/test_estimator.py

验收 (契约 §3):
 1. FK 与 MuJoCo 的 geom_xpos 对齐 (max|diff| < 2e-3 m，实测 ~1e-16);
 2. 已知基座速度下 0.5 s 内收敛到真值 ±0.1 m/s;
 3. 单腿接触 / 零接触 -> confidence == 0 且不输出野值 (并在 stats 里计数)。

速度回放的做法 (诚实说明): 用 Go2Sim + MuJoCo **自带的点雅可比** mj_jac 反解出
"接触足在世界系不动"所需的关节速度，再把基座速度设成已知真值 v，
每拍只做 mj_forward (不积分) —— 这是一次**运动学回放**: 足端接触力、四元数、
关节速度都来自 MuJoCo，但基座速度是外部注入的常量，不是闭环动力学跑出来的。
因此它检验的是本模块的 FK/雅可比/接触判据/滤波链路，而不是控制器的稳定性。
角速度一路: Go2Sim._imu() 的 gyroscope 目前**不是**机体系角速度 (见 _replay 注释),
测试按真机语义用 MuJoCo 的世界角速度转回机体系后再喂给里程计。
"""

from __future__ import annotations

import copy
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

import mujoco  # noqa: E402

from signal_io.estimator import LEG_ORDER, LegKinematics, LegOdometry  # noqa: E402
from sim_env.robot import Go2Sim, make_stand_cmd  # noqa: E402

XML = os.path.join(_ROOT, "sim_env", "models", "go2", "go2.xml")
SCENE = os.path.join(_ROOT, "sim_env", "models", "go2", "scene.xml")

#: Go2 训练/部署关节顺序下的 home (与 Go2Sim 的 keyframe 一致)
HOME = np.array([0.0, 0.9, -1.8] * 4)

_FAILURES = []


def check(name, fn):
    try:
        fn()
        print("  ok   %s" % name)
    except AssertionError as exc:
        _FAILURES.append((name, str(exc)))
        print("  FAIL %s: %s" % (name, exc))
    except Exception as exc:  # noqa: BLE001
        _FAILURES.append((name, "%s: %s" % (type(exc).__name__, exc)))
        print("  ERR  %s: %s: %s" % (name, type(exc).__name__, exc))


def settle(sim, steps=500, kp=60.0, kd=5.0):
    cmd = make_stand_cmd(sim.stand_pose, kp=kp, kd=kd)
    for _ in range(steps):
        sim.write_cmd(cmd)
        sim.step()


def _calf_body_ids(sim):
    return [mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, leg + "_calf")
            for leg in LEG_ORDER]


# ---------------------------------------------------------------- 1. FK 对齐


def test_fk_alignment():
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)                      # 真实落地 1 s 后的位姿
    rng = np.random.default_rng(0)
    worst = 0.0
    for trial in range(300):
        if trial == 0:
            q = HOME.copy()
        else:
            q = HOME + rng.uniform(-0.8, 0.8, 12)
        sim.data.qpos[:3] = rng.uniform(-1.0, 1.0, 3)
        sim.data.qpos[2] = 0.3
        quat = rng.normal(size=4)
        sim.data.qpos[3:7] = quat / np.linalg.norm(quat)
        sim.data.qpos[sim.qpos_adr] = q
        mujoco.mj_forward(sim.model, sim.data)
        R = sim.data.xmat[sim.base_body].reshape(3, 3)
        rel = (sim.data.geom_xpos[sim.foot_geom]
               - sim.data.xpos[sim.base_body]) @ R
        mine = kin.foot_positions_body(q)
        worst = max(worst, float(np.abs(rel - mine).max()))
    assert worst < 2e-3, "FK 与 MuJoCo 偏差 %.3g m >= 2e-3" % worst
    assert worst < 1e-9, "FK 应当**精确**对齐 (实测 %.3g)" % worst
    print("       FK max|diff| = %.3g m (300 个随机位姿, 阈值 2e-3)" % worst)


def test_fk_matches_settled_state():
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)
    mujoco.mj_forward(sim.model, sim.data)
    st = sim.read_state()
    R = sim.data.xmat[sim.base_body].reshape(3, 3)
    rel = (sim.data.geom_xpos[sim.foot_geom] - sim.data.xpos[sim.base_body]) @ R
    mine = kin.foot_positions_body(st.q12())
    d = float(np.abs(rel - mine).max())
    assert d < 2e-3, "settle 后 FK 偏差 %.3g" % d
    # 接触力应超过阈值, 供里程计用
    assert min(st.foot_force) > 30, st.foot_force
    print("       settled: base_z=%.4f foot_force=%s FK diff=%.3g"
          % (sim.data.qpos[2], st.foot_force, d))


def test_jacobian_vs_mujoco():
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    q = np.array([0.05, 0.8, -1.9, -0.1, 1.1, -1.5, 0.3, 1.3, -2.1, -0.4, 0.7, -1.2])
    sim.data.qpos[:3] = [0.0, 0.0, 0.3]
    sim.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    sim.data.qpos[sim.qpos_adr] = q
    mujoco.mj_forward(sim.model, sim.data)
    R = sim.data.xmat[sim.base_body].reshape(3, 3)
    mine = kin.foot_jacobians_body(q)
    jp = np.zeros((3, sim.model.nv))
    jr = np.zeros((3, sim.model.nv))
    worst = 0.0
    for i, g in enumerate(sim.foot_geom):
        # 注意 body 参数必须是该足所在的 calf body (不是 base)
        mujoco.mj_jac(sim.model, sim.data, jp, jr, sim.data.geom_xpos[g],
                      _calf_body_ids(sim)[i])
        Jm = jp[:, sim.dof_adr[3 * i:3 * i + 3]]
        worst = max(worst, float(np.abs(R.T @ Jm - mine[i]).max()))
    assert worst < 1e-6, "雅可比与 mj_jac 偏差 %.3g" % worst
    print("       |J - mj_jac| max = %.3g" % worst)


# ---------------------------------------------------------------- 2. 速度收敛


def _replay(sim, kin, v_true, omega_body, steps=250, dt=0.002,
            contact_force=None):
    """运动学回放: 用 mj_jac 反解接触腿关节速度，注入已知基座速度。"""
    calf_ids = _calf_body_ids(sim)
    odo = LegOdometry(kin)
    jp = np.zeros((3, sim.model.nv))
    jr = np.zeros((3, sim.model.nv))
    p6 = np.zeros(6)
    errs = []
    for k in range(steps):
        mujoco.mj_forward(sim.model, sim.data)
        sim.data.qvel[:] = 0.0
        sim.data.qvel[0:3] = v_true
        sim.data.qvel[3:6] = omega_body
        for i in range(4):
            mujoco.mj_jac(sim.model, sim.data, jp, jr,
                          sim.data.geom_xpos[sim.foot_geom[i]], calf_ids[i])
            Jleg = jp[:, sim.dof_adr[3 * i:3 * i + 3]]
            rhs = jp[:, 0:6] @ sim.data.qvel[0:6]
            sim.data.qvel[sim.dof_adr[3 * i:3 * i + 3]] = -np.linalg.solve(Jleg, rhs)
        mujoco.mj_forward(sim.model, sim.data)
        st = sim.read_state()
        # 重要: Go2Sim._imu() 现在返回的 gyroscope **不是**机体系角速度
        # (实测 qvel[3:6]=[0.3,0.4,0.5] 时报 [0.504,-0.402,0.290]，即按躯干
        # 惯性系(body_iquat)旋转过的量, robot.py 侧问题, 已报 Lead)。
        # 真机 LowState 的 gyro 就是机体系角速度, 所以这里按契约语义用
        # MuJoCo 的世界角速度转回机体系, 保证测的是里程计本身而不是 sim 的 bug。
        mujoco.mj_objectVelocity(sim.model, sim.data, mujoco.mjtObj.mjOBJ_BODY,
                                 sim.base_body, p6, 0)
        R_base = sim.data.xmat[sim.base_body].reshape(3, 3)
        st.imu_state.gyroscope = [float(v) for v in (R_base.T @ p6[0:3])]
        if contact_force is not None:
            st.foot_force = list(contact_force)
        v = odo.update(st, dt, k * dt)
        errs.append(float(np.linalg.norm(v - v_true)))
    return odo, np.asarray(errs)


def test_velocity_convergence():
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)
    for v_true, omega in ((np.array([0.4, 0.0, 0.0]), np.zeros(3)),
                          (np.array([0.3, 0.2, 0.0]), np.zeros(3)),
                          (np.array([0.4, 0.0, 0.0]), np.array([0.0, 0.0, 0.5]))):
        odo, errs = _replay(sim, kin, v_true, omega)
        t_conv = next((k * 0.002 for k in range(len(errs)) if errs[k] < 0.1), None)
        assert t_conv is not None, "0.5 s 内未收敛: final err=%.4f" % errs[-1]
        assert t_conv <= 0.5, "收敛耗时 %.3f s > 0.5 s" % t_conv
        assert errs[-1] < 0.1, "终值误差 %.4f" % errs[-1]
        assert odo.confidence > 0.9, "四足接触时 confidence 应接近 1: %.3f" % odo.confidence
        v = odo.velocity
        print("       v_true=%s omega=%s -> v_est=[%.4f %.4f %.4f] "
              "err=%.4f t_conv=%.3fs conf=%.2f"
              % (v_true, omega, v[0], v[1], v[2], errs[-1], t_conv, odo.confidence))


def test_velocity_follows_step():
    """基座速度突变 -> 估计器单调跟随 (不产生野值/超调)。"""
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)
    odo, errs = _replay(sim, kin, np.array([0.5, 0.0, 0.0]), np.zeros(3), steps=150)
    v = odo.velocity
    assert abs(v[0] - 0.5) < 0.1, "vx=%.4f" % v[0]
    assert abs(v[1]) < 0.05 and abs(v[2]) < 0.05, "串扰过大: %s" % v
    assert np.max(errs) < 0.6, "出现过冲/野值: max err=%.3f" % np.max(errs)


# ---------------------------------------------------------------- 3. 接触降级


def test_zero_and_single_contact():
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)
    odo, _ = _replay(sim, kin, np.array([0.4, 0.0, 0.0]), np.zeros(3), steps=60)
    v_ok = odo.velocity.copy()
    assert odo.confidence > 0.9
    st_ok = sim.read_state()

    # --- 零接触: 不得输出野值, 返回上一帧, confidence -> 0
    st0 = copy.deepcopy(st_ok)
    st0.foot_force = [0, 0, 0, 0]
    v0 = odo.update(st0, 0.002, 1.0)
    assert odo.confidence == 0.0, "零接触 confidence=%s" % odo.confidence
    assert np.allclose(v0, v_ok), "零接触应保持上一帧: %s vs %s" % (v0, v_ok)
    assert odo.stats()["n_zero_contact"] >= 1
    assert odo.stats()["n_low_contact"] >= 1

    # --- 单腿接触: 同样 confidence -> 0
    st1 = copy.deepcopy(st_ok)
    st1.foot_force = [50, 0, 0, 0]
    n_low = odo.stats()["n_low_contact"]
    v1 = odo.update(st1, 0.002, 1.002)
    assert odo.confidence == 0.0, "单腿接触 confidence=%s" % odo.confidence
    assert np.allclose(v1, v_ok)
    assert odo.stats()["n_low_contact"] == n_low + 1

    # --- 零接触 + 疯狂关节速度: 输出仍然不动 (不发出野值)
    st2 = copy.deepcopy(st_ok)
    st2.foot_force = [0, 0, 0, 0]
    for i in range(12):
        st2.motor_state[i].dq = 1e3
    v2 = odo.update(st2, 0.002, 1.004)
    assert odo.confidence == 0.0 and np.allclose(v2, v_ok), v2

    # --- state=None
    v3 = odo.update(None, 0.002, 1.006)
    assert odo.confidence == 0.0 and np.allclose(v3, v_ok)
    assert odo.stats()["n_no_state"] >= 1

    s = odo.stats()
    assert s["n_contact_hist"][0] >= 2, s["n_contact_hist"]      # 零接触
    assert s["n_contact_hist"][1] >= 1, s["n_contact_hist"]      # 单腿接触
    assert s["n_contact_hist"][4] >= 60, s["n_contact_hist"]     # 四足接触回放帧
    print("       零/单接触降级 OK: n_zero=%d n_low=%d n_no_state=%d hist=%s"
          % (s["n_zero_contact"], s["n_low_contact"], s["n_no_state"],
             s["n_contact_hist"]))


def test_contact_threshold():
    """接触判据用 foot_force 阈值: 低于阈值不算接触。"""
    sim = Go2Sim(SCENE, sim_dt=0.002)
    kin = LegKinematics(XML)
    settle(sim, 500)
    st = sim.read_state()
    odo = LegOdometry(kin, contact_force_thresh=30.0)
    st_low = copy.deepcopy(st)
    st_low.foot_force = [29, 29, 29, 29]
    v = odo.update(st_low, 0.002, 0.0)
    assert odo.confidence == 0.0, "29N 不应算接触"
    assert np.allclose(v, 0.0)
    st_high = copy.deepcopy(st)
    st_high.foot_force = [31, 31, 31, 31]
    odo.update(st_high, 0.002, 0.002)
    assert odo.confidence > 0.0, "31N 应算接触"


def test_reset():
    kin = LegKinematics(XML)
    odo = LegOdometry(kin)
    rng = np.random.default_rng(1)
    st = None
    # 用真实 sim 造一个有效 state, 再确认 reset 归零
    sim = Go2Sim(SCENE, sim_dt=0.002)
    settle(sim, 400)
    st = sim.read_state()
    for k in range(20):
        odo.update(st, 0.002, k * 0.002)
    assert odo.confidence > 0.0
    odo.reset()
    assert odo.confidence == 0.0
    assert np.all(odo.velocity == 0.0)
    assert odo.stats()["n_update"] == 0
    _ = rng


def test_kinematics_describe():
    kin = LegKinematics(XML)
    d = kin.describe()
    assert d["base_body"] == "base_link"
    assert d["hip_pos"][0][0] > 0 and d["hip_pos"][2][0] < 0
    assert abs(d["foot_radius"][0] - 0.022) < 1e-9
    assert np.allclose(kin.hip_axis, np.array([[1.0, 0, 0]] * 4))
    assert np.allclose(kin.thigh_axis, np.array([[0, 1.0, 0]] * 4))
    try:
        LegKinematics(os.path.join(_ROOT, "sim_env", "models", "go2", "nope.xml"))
        raise AssertionError("缺文件应当 FileNotFoundError")
    except FileNotFoundError:
        pass


# ---------------------------------------------------------------- main


def main():
    print("test_estimator.py: 腿部 FK/雅可比 + 腿式里程计")
    check("FK 与 MuJoCo 对齐 (<2e-3 m)", test_fk_alignment)
    check("FK 对齐 settled state", test_fk_matches_settled_state)
    check("雅可比与 mj_jac 一致", test_jacobian_vs_mujoco)
    check("已知基座速度 0.5s 收敛 ±0.1 m/s", test_velocity_convergence)
    check("速度突变跟随/无超调", test_velocity_follows_step)
    check("零/单接触 confidence=0 无野值", test_zero_and_single_contact)
    check("接触力阈值", test_contact_threshold)
    check("reset 归零", test_reset)
    check("几何解析", test_kinematics_describe)
    if _FAILURES:
        print("\nFAILED %d:" % len(_FAILURES))
        for name, msg in _FAILURES:
            print("  - %s: %s" % (name, msg))
        raise SystemExit(1)
    print("OK test_estimator: 9 项全部通过")


if __name__ == "__main__":
    main()
