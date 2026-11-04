import sys, time
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
rclpy.init()
n = Node("lyrical_lowstate_probe")
cnt = {"n": 0, "last": None}
def cb(m):
    cnt["n"] += 1
    cnt["last"] = m
n.create_subscription(LowState, "/lowstate", cb, 10)
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
m = cnt["last"]
print("ROS2 Lyrical subscriber: rx=%d" % cnt["n"])
if m is not None:
    print("  tick=%d motor_state[0].q=%.4f" % (m.tick, m.motor_state[0].q))
rclpy.shutdown()
sys.exit(0 if cnt["n"] > 0 else 1)
