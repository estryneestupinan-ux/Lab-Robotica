#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

NOMBRES_ROBOTS = ["robot1", "robot2", "robot3", "robot4", "robot5"]
DURACION_PASO = 3.0   # s que se mueve cada robot


class Simulador(Node):
    def __init__(self):
        super().__init__("simulador_robots")
        qos_emerg = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Bool, "/secuencia/inicio", self.cb_inicio, 10)
        self.create_subscription(Bool, "/emergencia", self.cb_emergencia, qos_emerg)

        self.pubs = {n: self.create_publisher(Bool, f"/{n}/en_movimiento", 10)
                     for n in NOMBRES_ROBOTS}
        self.emergencia = False
        self.paso = -1          # -1 = secuencia inactiva
        self.t_paso = 0.0
        self.create_timer(0.1, self.ciclo)

    def cb_inicio(self, msg):
        if msg.data and not self.emergencia and self.paso < 0:
            self.paso = 0
            self.t_paso = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Secuencia iniciada")

    def cb_emergencia(self, msg):
        if msg.data and not self.emergencia:
            self.get_logger().warn("EMERGENCIA: deteniendo todo")
            self.paso = -1
        self.emergencia = msg.data

    def ciclo(self):
        ahora = self.get_clock().now().nanoseconds / 1e9
        if self.paso >= 0 and ahora - self.t_paso > DURACION_PASO:
            self.paso += 1
            self.t_paso = ahora
            if self.paso >= len(NOMBRES_ROBOTS):
                self.paso = -1
                self.get_logger().info("Secuencia terminada")

        for i, n in enumerate(NOMBRES_ROBOTS):
            moviendo = (i == self.paso) and not self.emergencia
            self.pubs[n].publish(Bool(data=moviendo))


def main():
    rclpy.init()
    nodo = Simulador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
