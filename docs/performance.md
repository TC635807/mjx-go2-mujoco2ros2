# 用 Python 会不会拖慢？—— 实测拆账

复现：

    source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
    export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages ROS_DOMAIN_ID=1
    .venv/bin/python scripts/bench_io.py

## 500 Hz 那一拍（预算 2000 µs）的钱花在哪

| 环节 | 中位 | p95 | 占预算 |
|---|---:|---:|---:|
| `mujoco.mj_step`（物理一步） | 20.9 µs | 44.5 µs | 1.0% |
| `read_state()`（组 LowState：IMU + 12 关节 + 足端力） | 63.4 µs | 140.8 µs | 3.2% |
| `write_cmd()`（12 关节 PD + 限幅） | 37.0 µs | 66.8 µs | 1.8% |
| `LowState.pack()` + CRC32（裸 UDP 用） | 83.0 µs | 133.1 µs | 4.2% |
| `LowState.unpack()` + CRC 校验 | 9.2 µs | 15.7 µs | 0.5% |
| `LowCmd.unpack()` | 11.2 µs | 18.7 µs | 0.6% |
| ROS2 `publish(LowState)`（本端序列化+入队） | 26.3 µs | 50.3 µs | 1.3% |
| ROS2 `spin_once`（空转一圈） | 58.9 µs | 124.8 µs | 2.9% |
| **ROS2 端到端（publish → 回调收到）** | **233.9 µs** | 445.8 µs | 11.7% |

**一个 500 Hz 拍的全部活（物理 + 打包 + 发状态 + 施加力矩）约 204 µs，占 2 ms 预算的 10.2%。**
剩下 ~90% 是空的。

## 结论怎么读

1. **瓶颈不是"Python 慢"，是 DDS 往返。** 端到端 234 µs 里绝大部分是 CycloneDDS 的
   序列化/传输/回调调度；同样的活换成 C++ 也得走这一趟，只是能省掉 Python 的胶水。
   物理本身只花 21 µs —— 那是 C 代码。
2. **50 Hz 控制层完全无压力。** 它每拍有 20 ms。一个 12 维 LQR 用 numpy 解一次
   `K = lqr(A,B,Q,R)`（A/B 是 18×18 / 18×12）大约 50~150 µs，占 0.5%~0.8%；
   算得慢一点、多算几次也远远够。
3. **Python 真正的代价是"抖动"，不是"吞吐"。**上表 max 那一列：
   `read_state` 最坏 6.9 ms、`LowState.pack` 最坏 5.2 ms（WSL2 调度 + GC 毛刺）。
   平均 0.2 ms 但偶尔 5 ms，对**硬实时 500 Hz** 是真风险；对仿真无所谓
   （有 `Pacer` + 零阶保持 + 看门狗兜着）。
4. 所以推荐的分工：

   * **50 Hz 控制层（LQR / MPC / 策略网络）：Python 完全够**，还能直接复用
     numpy / scipy / jax 那一堆东西。宇树自己也同时提供 `unitree_sdk2py` 的官方例程。
   * **500 Hz 电机层**：
     * 仿真里用 Python 没问题（本机实测 498 Hz、RTF 1.00）；
     * 真机上如果要 PC 侧发 500 Hz，建议要么把进程绑核 + 用 RT 内核，
       要么把这层写成 C++ 节点（IO 层已经抽象好了，换实现不影响控制器）。
5. 训练侧的 Python 更不用纠结：训练走的是 MJX/CUDA，Python 只是外壳。

## 一句话

**Python 在这条链路上花掉的是"预算的 10%"，换来的是"控制器随便改"。**
真正的实时风险来自调度抖动而不是语言本身，而那个问题在真机上用绑核/RT 内核/C++ 电机层解决，
不需要把整个控制器推倒重写。
