#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

KP = 3.0
V_MAX = 0.1
MAX_ACCEL = 0.15
DT = 0.02
POS_TOLERANCE = 0.0015

class Joint3PositionHold(Node):
    def __init__(self):
        super().__init__("joint3_position_hold")
        self.target = 0.0
        self.current = 0.0
        self.have_state = False
        self.cmd_vel = 0.0
        self.pub = self.create_publisher(Float64MultiArray, "/joint3_velocity_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Float64, "/joint3_target", self.on_target, 10)
        self.create_timer(DT, self.control_loop)

    def on_joint_states(self, msg):
        if "Joint_3" not in msg.name:
            return
        idx = msg.name.index("Joint_3")
        self.current = msg.position[idx]
        self.have_state = True

    def on_target(self, msg):
        self.target = float(msg.data)

    def control_loop(self):
        if not self.have_state:
            return
        error = self.target - self.current
        if abs(error) < POS_TOLERANCE:
            desired_vel = 0.0
        else:
            desired_vel = KP * error
            desired_vel = max(-V_MAX, min(V_MAX, desired_vel))
        max_step = MAX_ACCEL * DT
        delta = desired_vel - self.cmd_vel
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.cmd_vel += delta
        m = Float64MultiArray()
        m.data = [self.cmd_vel]
        self.pub.publish(m)

def main():
    rclpy.init()
    node = Joint3PositionHold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
