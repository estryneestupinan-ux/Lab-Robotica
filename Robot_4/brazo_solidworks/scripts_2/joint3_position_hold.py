#!/usr/bin/env python3
"""
Control de posición del eje Z (Joint_3) en Gazebo, SCARA R4.

En la simulación, Joint_3 se maneja con un controlador de VELOCIDAD
(/joint3_velocity_controller). Este nodo cierra un lazo proporcional encima:
lee la posición de Joint_3 en /joint_states, la compara con el objetivo que
llega por /joint3_target (std_msgs/Float64, en metros, 0 = arriba y positivo
hacia abajo) y publica la velocidad.

  velocidad = KP * error, saturada a ±V_MAX y con rampa de aceleración MAX_ACCEL
  |error| < POS_TOLERANCE -> velocidad 0 (el eje se detiene)

La rampa evita los saltos de velocidad que desestabilizan a gz-sim.
Lo usan joint_angle_gui.py y secuencia_gui.py, que publican /joint3_target.
No correrlo junto con ik_gui_teleop.py ni trayectoria_lineal.py: esos scripts
mandan su propia velocidad al eje Z.

Uso:  python3 joint3_position_hold.py
"""
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
