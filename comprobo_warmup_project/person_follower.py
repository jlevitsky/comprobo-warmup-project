
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


DRIVE_P = 0.2
TURN_P = 0.5


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.follow_person, 10)
    
    def follow_person(self, lidar_scan):
        angles=np.arange(lidar_scan.angle_min,lidar_scan.angle_max, lidar_scan.angle_increment)
        ranges=np.array(lidar_scan.ranges)
        points=np.stack([angles,ranges])      
        point_filter=(ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
        filtered_points=points[:, point_filter]

        cartx = filtered_points[1, :] * np.cos(filtered_points[0, :])
        carty = filtered_points[1, :] * np.sin(filtered_points[0, :])
        cartesian = np.stack([cartx, carty])

        x_filter = (cartesian[0, :] > 0) & (cartesian[0, :] < 1)
        y_filter = (cartesian[1, :] > -1) & (cartesian[1, :] < 1)
        filtered_cartesian = cartesian[:, x_filter & y_filter]
        if filtered_cartesian.size == 0:
            print('No points found')
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            return

        center_of_mass = np.mean(filtered_cartesian, 1)
        com_dist = np.hypot(center_of_mass[0], center_of_mass[1])
        com_angle = np.arctan2(center_of_mass[1], center_of_mass[0])
        drive_speed = DRIVE_P * com_dist
        turn_speed = TURN_P * com_angle

        drive_msg = Twist()
        drive_msg.linear.x = np.clip(drive_speed, -1.0, 1.0)
        drive_msg.angular.z = np.clip(turn_speed, -1.0, 1.0)
        self.publisher.publish(drive_msg)


def main():
    rclpy.init()
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
