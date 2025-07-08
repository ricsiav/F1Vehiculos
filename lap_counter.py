import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class LapCounter(Node):
    def __init__(self):
        super().__init__('lap_counter_node')
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.last_in_zone = False
        self.lap_count = -1
        self.zone_x, self.zone_y, self.radius = 0.0, 0.0, 1.0  # Ajusta esto a tu checkpoint

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = math.hypot(x - self.zone_x, y - self.zone_y)

        in_zone = dist < self.radius
        if in_zone and not self.last_in_zone:
            self.lap_count += 1
            self.get_logger().info(f'🚗 Vuelta completada: {self.lap_count}')
        self.last_in_zone = in_zone


def main(args=None):
    rclpy.init(args=args)
    node = LapCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
