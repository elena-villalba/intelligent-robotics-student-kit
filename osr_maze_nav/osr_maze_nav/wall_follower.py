import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CorridorNavigator(Node):
    def __init__(self):
        super().__init__('corridor_navigator')

        # Publicador a cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscripción al LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            'laser_controller/out',
            self.scan_callback,
            10
        )

        # Frecuencia de publicación
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables internas
        self.regions = {
            'left': float('inf'),
            'right': float('inf'),
            'front': float('inf')
        }

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        self.regions = {
            'right': min(min(ranges[0:30]), 3.5),  # Derecha
            'front': min(min(ranges[330:] + ranges[:30]), 3.5),  # Frente
            'left': min(min(ranges[330:360]), 3.5),  # Izquierda
        }

    def control_loop(self):
        msg = Twist()
        front = self.regions['front']
        left = self.regions['left']
        right = self.regions['right']

        # Seguridad: si está demasiado cerca de la pared de frente, gira
        if front < 0.5:
            msg.linear.x = 0.1
            msg.angular.z = -0.4  # Gira a la izquierda
            self.get_logger().info('🔄 Obstacle ahead! Turning left.')
        else:
            # Ir hacia delante ajustando para mantenerse centrado
            msg.linear.x = 0.2
            error = left - right  # Si error > 0: está más cerca de la derecha

            # Control proporcional simple
            k_p = 1.0
            msg.angular.z = -k_p * error
            self.get_logger().info(
                f"🚗 Forward | L: {left:.2f} R: {right:.2f} Err: {error:.2f}"
            )

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()