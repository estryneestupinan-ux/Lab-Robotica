#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import tkinter as tk

class JointGuiNode(Node):
    def __init__(self):
        super().__init__("joint_angle_gui")
        self.pub1 = self.create_publisher(Float64MultiArray, "/joint1_position_controller/commands", 10)
        self.pub2 = self.create_publisher(Float64MultiArray, "/joint2_position_controller/commands", 10)
        self.pub3_target = self.create_publisher(Float64, "/joint3_target", 10)

    def send(self, pub, value):
        msg = Float64MultiArray()
        msg.data = [float(value)]
        pub.publish(msg)

    def send_joint3_target(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub3_target.publish(msg)

def main():
    rclpy.init()
    node = JointGuiNode()

    root = tk.Tk()
    root.title("Control de Angulos - Brazo R4")

    tk.Label(root, text="Joint_1 (rad)").pack()
    s1 = tk.Scale(root, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, length=400,
                  command=lambda v: node.send(node.pub1, v))
    s1.pack()

    tk.Label(root, text="Joint_2 (rad)").pack()
    s2 = tk.Scale(root, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, length=400,
                  command=lambda v: node.send(node.pub2, v))
    s2.pack()

    tk.Label(root, text="Joint_3 (m)").pack()
    s3 = tk.Scale(root, from_=0.0, to=0.105, resolution=0.001, orient=tk.HORIZONTAL, length=400,
                  command=lambda v: node.send_joint3_target(v))
    s3.pack()

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
