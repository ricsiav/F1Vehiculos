import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer_node')
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.last_in_zone = False
        self.zone_x, self.zone_y, self.radius = 0.0, 0.0, 1.0  # Ajusta el checkpoint
        self.lap_times = []
        self.start_time = time.time()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = math.hypot(x - self.zone_x, y - self.zone_y)

        in_zone = dist < self.radius
        if in_zone and not self.last_in_zone:
            now = time.time()
            lap_time = now - self.start_time
            self.start_time = now
            self.lap_times.append(lap_time)
            self.get_logger().info(f'⏱️ Tiempo vuelta {len(self.lap_times)-1}: {lap_time:.2f} s')
        self.last_in_zone = in_zone


def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
