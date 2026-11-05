"""指令掉线兜底（failsafe）：真机上唯一能替"已经死掉的控制器"发阻尼的东西。

===============================================================================
1. 它解决什么问题（与 health / supervisor 的分工）
===============================================================================

  已经在进程内的两层保护：
      HealthMonitor        判 LowState 还来不来（真机 40/80/250 ms -> L1/L2/L3）
      SafetySupervisor     按等级出指令（track/crouch/damping/estop）
  它们有一个共同的盲区：**必须由控制器进程活着才有意义**。当 controller 被
  SIGKILL / 段错误 / OOM / 拔网线导致 DDS 线程卡死 / PC 合盖休眠时，进程要么直接
  消失，要么不再发 /lowcmd。此时机器人固件的行为官方没有公开（可能是"保持最后
  一条指令"），而 run_controller 的 finally 只在 Python 正常栈退出时才生效。

  本模块 + scripts/failsafe_guard.py 是**另一个进程**：
      * 只要 /lowcmd 以正常频率在发，它什么都不做；
      * 一旦超过 timeout_ms（默认 250 ms）没有收到任何 LowCmd，就按 publish_hz
        （默认 500 Hz）发送 kp=0 / kd=8 的阻尼指令；
      * 一旦重新看到 /lowcmd（控制器活了 / 另一个发布者接管），**立刻**停止发送；
      * 单次最多补发 max_duration_s（默认 5 s），之后停下并计数：
        无限补发会把"控制器已死"这个事实掩盖掉。

  WARN 本模块是**纯决策内核**：不做任何 IO、不读时钟（now 全部由调用方传入），
       所以可以完全离线单测（tests/test_failsafe.py 不需要 ROS 2）。

===============================================================================
2. 状态机（四态，只有 FIRING 会发布）
===============================================================================

      WAITING    还没见过任何 /lowcmd              -> 永不发布
      ARMED      见过 /lowcmd，且间隔 <= timeout_ms -> 不发布
      FIRING     间隔 > timeout_ms                 -> 按 publish_hz 发布阻尼
      EXHAUSTED  本次补发已超过 max_duration_s     -> 不发布，等新的 /lowcmd 重新 arm

  迁移：
      WAITING   --见到 cmd--> ARMED
      ARMED     --超时-------> FIRING          (n_trips += 1)
      FIRING    --见到 cmd---> ARMED           (n_recoveries += 1；立即停发，不留尾巴)
      FIRING    --超过 max_duration_s--> EXHAUSTED
      EXHAUSTED --见到 cmd--> ARMED

  为什么 WAITING 不发布：guard 可能比控制器先起来（launch 顺序不保证），也可能
  是操作员只想"旁观"。如果 guard 单独启动就开始发阻尼，等于在没有控制器的情况
  下把机器人按住 —— 那不是兜底，是新的风险。要"从启动第一秒就算超时"的用法，
  显式传 require_ever_seen=False（对应 CLI 的 --assume-always-armed）。

===============================================================================
3. 真机要点 / 常见失败模式 / 残留风险（诚实清单）
===============================================================================

  * **未在真机验证过**。先在 sim_ros 上把整条链路跑通，再上真机。
  * 默认**不真的发**（scripts/failsafe_guard.py 要 --enable-publish 才发），
    默认只打印 WOULD PUBLISH，避免"以为是旁观，结果接管了机器人"。
  * 它自己也会发 /lowcmd -> 与控制器**抢同一个话题**。gap 判定保证控制器一恢复
    它立刻停，但 DDS 里可能还残留 1~2 帧阻尼被固件执行（kp=0，安全）。
    所以**不要**在控制器正常跑的时候开 --enable-publish 做实验。
  * 没有覆盖：guard 自己死掉 / PC 断电 / 网线全断（DDS 发不出去）。
    这些情况下唯一的兜底是"机器人固件自己的超时行为"，未知，必须吊装试验确认。
  * 本模块不看 /lowstate、不看姿态、不看电池：它只回答"上一条 LowCmd 还是不是新鲜
    的"。姿态/电池的保护仍在控制器进程内（见 health.py）。

相关文件：
  scripts/failsafe_guard.py        ROS 2 守护进程（本模块唯一的调用方）
  ros2_controller/health.py        /lowstate 断流分级（进程内）
  ros2_controller/safety.py        按等级出指令（进程内）
  signal_io/msg.py                 LowCmd 本地消息（make_damping_cmd 用它）
  tests/test_failsafe.py           状态机单测（不需要 ROS 2）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .msg import NUM_JOINT, LowCmd

__all__ = ["STATE_WAITING", "STATE_ARMED", "STATE_FIRING", "STATE_EXHAUSTED",
           "FailsafeKernel", "Decision", "make_damping_cmd", "DEFAULT_KD"]

# 阻尼指令的 kd：与 ControllerNode.emergency_stop 的 create_damping_cmd 保持一致
# （官方 unitree_rl_gym 同款：kp=0 / kd=8 / q=0，纯关节阻尼）。
DEFAULT_KD = 8.0

STATE_WAITING = "waiting"      # 还没见过任何 LowCmd：永不发布
STATE_ARMED = "armed"          # 见过 LowCmd，链路正常：不发布
STATE_FIRING = "firing"        # 指令断流中：按 publish_hz 发布阻尼
STATE_EXHAUSTED = "exhausted"  # 本次补发到上限：停止发布，等新指令重新 arm


@dataclass
class Decision:
    """一次 update() 的结论（纯数据，便于单测与日志）。"""

    publish: bool          # 这一拍要不要发一条阻尼指令
    state: str             # 当前状态（四态之一）
    reason: str            # 人类可读原因（状态迁移/节流/耗尽）
    gap_ms: float          # 距离上一条 /lowcmd 的间隔；没见过指令时为 -1
    fired_s: float         # 本次 FIRING 已持续多久
    n_trips: int           # 累计触发次数
    n_fire_ticks: int      # 累计决定发布的拍数（节流后）

    @property
    def firing(self) -> bool:
        return self.state == STATE_FIRING


class FailsafeKernel:
    """指令断流判定 + 补发节流。无 IO、无时钟、可重放（纯函数式状态机）。

    典型用法（500 Hz 循环，见 scripts/failsafe_guard.py）：
        k = FailsafeKernel(timeout_ms=250, publish_hz=500, max_duration_s=5)
        while True:
            saw = <这一拍有没有收到新的 /lowcmd>
            d = k.update(now=monotonic(), saw_cmd=saw)
            if d.publish:
                <发低层阻尼指令>   # 不发也要调用 stats() 看 n_fire_ticks
    """

    def __init__(self, timeout_ms: float = 250.0, publish_hz: float = 500.0,
                 max_duration_s: float = 5.0, require_ever_seen: bool = True):
        """建内核。

        Args:
            timeout_ms: 多久没收到 /lowcmd 就判定"控制器掉了"（默认 250 ms，
                与真机 health_profiles.real_ros 的 estop_ms 一致）。
            publish_hz: 补发频率，默认 500 Hz（与固件低层周期一致）。
            max_duration_s: 单次最多补发多久（<=0 表示不限制，**不建议**）。
            require_ever_seen: True（默认）= 没见过任何 /lowcmd 就永不发布
                （WAITING）；False = 从第一次 update() 开始计时（--assume-always-armed）。
        """
        self.timeout_s = max(0.0, float(timeout_ms)) / 1000.0
        self.publish_hz = float(publish_hz)
        if self.publish_hz <= 0.0:
            raise ValueError("publish_hz 必须 > 0，得到 %r" % (publish_hz,))
        self.max_duration_s = float(max_duration_s)
        self.require_ever_seen = bool(require_ever_seen)
        self.reset()

    # ---------------------------------------------------------------- 生命周期
    def reset(self, now: Optional[float] = None) -> None:
        """回到初始状态；给 now 时把它当作"上一次收到指令"的时刻。"""
        self.state = STATE_WAITING if self.require_ever_seen else STATE_ARMED
        self._last_cmd_t: Optional[float] = None if now is None else float(now)
        self._last_update_t: Optional[float] = None
        self._fire_start_t: Optional[float] = None
        self._next_pub_t: Optional[float] = None
        self.n_trips = 0
        self.n_recoveries = 0
        self.n_fire_ticks = 0
        self.n_exhausted = 0
        self.max_gap_ms = 0.0
        self.total_fire_s = 0.0
        self.n_updates = 0

    # ---------------------------------------------------------------- 主入口
    def note_cmd(self, now: float) -> Decision:
        """单独上报"这一拍收到了 /lowcmd"（等价 update(now, saw_cmd=True)）。"""
        return self.update(now, saw_cmd=True)

    def update(self, now: float, saw_cmd: bool = False) -> Decision:
        """推进一拍。

        Args:
            now: 单调时间（秒），由调用方给；本类不读时钟。
            saw_cmd: 自上次 update 以来是否收到过至少一条 /lowcmd。
        Returns:
            Decision：publish=True 表示"这一拍应该发一条阻尼指令"。
        """
        now = float(now)
        dt = 0.0 if self._last_update_t is None else max(0.0, now - self._last_update_t)
        self._last_update_t = now
        self.n_updates += 1
        # 本次 FIRING 的累计时长（每拍累加；耗尽检查用 _fire_start_t 的绝对值）
        if self.state == STATE_FIRING:
            self.total_fire_s += dt

        # require_ever_seen=False（--assume-always-armed）：第一拍就当作"刚收到指令"，
        # 从这里开始计时；否则 _last_cmd_t 恒 None，永远不会从 ARMED 升到 FIRING。
        if self._last_cmd_t is None and not self.require_ever_seen:
            self._last_cmd_t = now

        # 统计"两次指令之间实际看到的空洞"必须在重置 _last_cmd_t **之前**算：
        # 收到指令的那一拍 gap 会被重置成 0，事后看 max_gap 就永远偏小。
        gap_before = self._gap_ms(now)
        if gap_before > 0.0:
            self.max_gap_ms = max(self.max_gap_ms, gap_before)

        reason = ""
        if saw_cmd:
            was = self.state
            self._last_cmd_t = now
            self.state = STATE_ARMED
            self._fire_start_t = None
            self._next_pub_t = None
            if was == STATE_FIRING:
                self.n_recoveries += 1
                reason = "recovered:lowcmd_back"
            elif was == STATE_EXHAUSTED:
                reason = "rearmed:lowcmd_back"
            else:
                reason = "armed"

        gap_ms = self._gap_ms(now)

        # ARMED -> FIRING：只有"见过指令并且真的断了"才升级（WAITING 永不升级）
        if self.state == STATE_ARMED and self._last_cmd_t is not None:
            if (now - self._last_cmd_t) > self.timeout_s:
                self.state = STATE_FIRING
                self._fire_start_t = now
                self._next_pub_t = now       # 第一拍立刻发，不等一个周期
                self.n_trips += 1
                reason = "gap=%.1fms>%.1fms" % (gap_ms, self.timeout_s * 1e3)

        publish = False
        fired_s = 0.0
        if self.state == STATE_FIRING:
            fired_s = max(0.0, now - self._fire_start_t)
            if self.max_duration_s > 0.0 and fired_s > self.max_duration_s:
                # 到上限：停止补发，等控制器回来重新 arm（不无限补发掩盖故障）
                self.state = STATE_EXHAUSTED
                self.n_exhausted += 1
                self._next_pub_t = None
                reason = "exhausted_after=%.2fs" % fired_s
            elif self._next_pub_t is not None and now >= self._next_pub_t:
                publish = True
                self.n_fire_ticks += 1
                period = 1.0 / self.publish_hz
                self._next_pub_t += period
                if self._next_pub_t <= now:   # 落后太多就对齐，别补发一串
                    self._next_pub_t = now + period
                if not reason:
                    reason = "fire:t=%.3fs" % fired_s

        if not reason:
            if self.state == STATE_WAITING:
                reason = "waiting:no_lowcmd_ever"
            elif self.state == STATE_ARMED:
                reason = "armed:gap=%.1fms" % gap_ms
            elif self.state == STATE_EXHAUSTED:
                reason = "exhausted:waiting_rearm"

        return Decision(publish=publish, state=self.state, reason=reason,
                        gap_ms=gap_ms, fired_s=fired_s,
                        n_trips=self.n_trips, n_fire_ticks=self.n_fire_ticks)

    # ---------------------------------------------------------------- 观测
    def _gap_ms(self, now: float) -> float:
        if self._last_cmd_t is None:
            return -1.0
        return max(0.0, now - self._last_cmd_t) * 1e3

    def stats(self) -> dict:
        return {
            "state": self.state,
            "timeout_ms": self.timeout_s * 1e3,
            "publish_hz": self.publish_hz,
            "max_duration_s": self.max_duration_s,
            "require_ever_seen": self.require_ever_seen,
            "ever_seen_cmd": self._last_cmd_t is not None,
            "last_gap_ms": round(self._gap_ms(self._last_update_t or 0.0), 3),
            "max_gap_ms": round(self.max_gap_ms, 3),
            "n_trips": self.n_trips,
            "n_recoveries": self.n_recoveries,
            "n_fire_ticks": self.n_fire_ticks,
            "n_exhausted": self.n_exhausted,
            "total_fire_s": round(self.total_fire_s, 3),
            "n_updates": self.n_updates,
        }


def make_damping_cmd(kd: float = DEFAULT_KD) -> LowCmd:
    """构造"纯阻尼"低层指令（kp=0 / kd=kd / q=0 / tau=0）。

    与 ControllerNode.emergency_stop 的 create_damping_cmd 一致：只写前 12 个
    真实关节槽位，后 8 个保留槽位不动（Go2 不用）。
    真机语义：固件算 tau = 0*(q_des-q) + kd*(0-dq) + 0 = -kd*dq，即"只按关节
    速度泄能"，机器人会软下来/靠重力站住，而不是被顶住。
    """
    cmd = LowCmd()
    for i in range(NUM_JOINT):
        m = cmd.motor_cmd[i]
        m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0x01, 0.0, 0.0, 0.0, 0.0, float(kd)
    return cmd
