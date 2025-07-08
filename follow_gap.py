# follow_gap.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowGap(Node):
    def __init__(self):
        super().__init__('follow_gap_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        self.speed = 4.0  # velocidad constante
        self.bubble_radius = 0.8  # espacio de seguridad

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, 0, 10)  # filtrar errores

        # Burbuja de seguridad para evitar obstáculos muy cercanos
        closest = np.argmin(ranges)
        angle_min = max(0, closest - 10)
        angle_max = min(len(ranges), closest + 10)
        ranges[angle_min:angle_max] = 0

        # Detectar los gaps
        gaps = self.find_gaps(ranges)
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        # Centro del gap para dirigir el volante
        gap_center = (best_gap[0] + best_gap[1]) // 2
        angle = msg.angle_min + gap_center * msg.angle_increment

        max_speed = 9.3    # velocidad en recta
        min_speed = 2.0     # velocidad mínima en curva fuerte
        abs_angle = abs(angle)
        speed = max(min_speed, max_speed  *(1 - abs_angle / (np.pi / 4)))
        # Reducción de velocidad si hay mucho giro
        self.speed = speed

        # Publicar comando
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.speed
        self.publisher.publish(drive_msg)



    def find_gaps(self, ranges):
        """Devuelve las secuencias (start, end) donde los valores son > 0"""
        gaps = []
        start = None
        for i, r in enumerate(ranges):
            if r > 1.5:
                if start is None:
                    start = i
            else:
                if start is not None:
                    gaps.append((start, i))
                    start = None
        if start is not None:
            gaps.append((start, len(ranges)))
        return gaps


def main(args=None):
    rclpy.init(args=args)
    node = FollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()