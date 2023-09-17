import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

DRIVE_SPEED = 0.1
TURN_SPEED = 0.6
FORWARD_ALPHA = 1000.0
POINT_ALPHA = 1

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.laser_topic= self.create_subscription(LaserScan,'scan',self.process_scan,10)
        self.did_thing = False

    def process_scan(self, lidar_scan):
        angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max, lidar_scan.angle_increment)
        # Flip angles around the y axis b/c straight forward is 180??
        angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
        ranges = np.array(lidar_scan.ranges)
        points = np.stack([angles,ranges])
        point_filter = (ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
        filtered_points = points[:, point_filter]

        cart_x = filtered_points[1, :] * np.cos(filtered_points[0, :])
        cart_y = filtered_points[1, :] * np.sin(filtered_points[0, :])
        cartesian = np.stack([cart_x, cart_y])
        
        dist_sq = cart_x ** 2 + cart_y ** 2
        source_deriv = 2 * cartesian / (dist_sq ** 2) / cartesian.shape[1] * POINT_ALPHA
        gradient = -np.sum(source_deriv, axis=1) + np.array([FORWARD_ALPHA / 500, 0])
        target_rad = np.hypot(gradient[0], gradient[1])
        target_ang = -np.arctan2(gradient[1], gradient[0])
        print(gradient)

        drive_msg = Twist()
        drive_msg.linear.x = min(target_rad * DRIVE_SPEED, 1.0)
        drive_msg.angular.z = np.clip(target_ang * TURN_SPEED, -1.0, 1.0)
        print(f'Forward {drive_msg.linear.x}, Turn {drive_msg.angular.z} ({"L" if drive_msg.angular.z > 0 else "R"})')
        self.publisher.publish(drive_msg)

def main():
    rclpy.init()
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
