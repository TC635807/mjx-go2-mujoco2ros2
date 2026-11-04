import sys, time
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowCmd

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
rclpy.init()
n = Node("lyrical_lowcmd_pub")
p = n.create_publisher(LowCmd, "/lowcmd", 10)
m = LowCmd()
m.head[0] = 0xFE
m.head[1] = 0xEF
m.level_flag = 0xFF
for i in range(20):
    m.motor_cmd[i].mode = 0x01
sent = 0
t0 = time.time()
while time.time() - t0 < dur:
    p.publish(m)
    sent += 1
    rclpy.spin_once(n, timeout_sec=0)
    time.sleep(0.005)
print("ROS2 Lyrical publisher: tx=%d" % sent)
rclpy.shutdown()
