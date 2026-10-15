#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ScaraTerminalInterface(Node):
    def __init__(self):
        super().__init__('scara_terminal_interface')
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, 
            '/scara_controller/commands', 
            10
        )
        self.L1 = 0.238688
        self.L2 = 0.120000

    def solve_ik_and_send(self, x, y, z):
        r_sq = x**2 + y**2
        cos_q2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

        if abs(cos_q2) > 1.0:
            self.get_logger().warn(f'¡Punto ({x}, {y}, {z}) fuera del espacio de trabajo!')
            return False

        sin_q2 = math.sqrt(1 - cos_q2**2)
        q2 = math.atan2(sin_q2, cos_q2)

        k1 = self.L1 + self.L2 * cos_q2
        k2 = self.L2 * sin_q2
        q1 = math.atan2(y, x) - math.atan2(k2, k1)
        d3 = z

        cmd = Float64MultiArray()
        # Mapeo según el orden de articulaciones: [Revoluci_n_1, Revoluci_n_2, Corredera_4]
        cmd.data = [q1, q2, d3]
        self.pub_cmd.publish(cmd)
        
        print(f"-> Ángulos calculados: q1={math.degrees(q1):.1f}°, q2={math.degrees(q2):.1f}°, d3={d3:.3f}m")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ScaraTerminalInterface()

    print("\n=============================================")
    print("      INTERFAZ DE CONTROL SCARA (IK)        ")
    print("=============================================")
    print("Escribe 'q' en cualquier momento para salir.\n")

    try:
        while rclpy.ok():
            str_x = input("Ingrese X (metros, ej. 0.20): ")
            if str_x.lower() == 'q': break
            
            str_y = input("Ingrese Y (metros, ej. 0.10): ")
            if str_y.lower() == 'q': break
            
            str_z = input("Ingrese Z (metros, ej. 0.02): ")
            if str_z.lower() == 'q': break

            try:
                x = float(str_x)
                y = float(str_y)
                z = float(str_z)
                node.solve_ik_and_send(x, y, z)
            except ValueError:
                print("Error: Por favor ingresa números válidos.\n")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()