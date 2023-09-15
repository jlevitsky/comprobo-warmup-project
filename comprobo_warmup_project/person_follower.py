
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
        self.filtered_scan_pub = self.create_publisher(LaserScan, 'filter_scan', 10)
        self.create_subscription(LaserScan, 'scan', self.follow_person, 10)
    
    def follow_person(self, lidar_scan: LaserScan):
        angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max, lidar_scan.angle_increment)
        ranges = np.array(lidar_scan.ranges)
        points = np.stack([angles,ranges])      
        point_filter = (ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
        filtered_points = points[:, point_filter]

        min_angle = -np.pi/2
        max_angle = np.pi/2
        min_range = 0.3
        max_range = 2.0
        r_filter = (filtered_points[1, :] > min_range) & (filtered_points[1, :] < max_range)
        t_filter = (filtered_points[0, :] >= min_angle) & (filtered_points[0, :] <= max_angle)
        points_in_range = filtered_points[:, r_filter & t_filter]
        if points_in_range.size == 0:
            print('No points found')
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            return

        center_of_mass = np.mean(points_in_range, 1)
        drive_speed = DRIVE_P * center_of_mass[1]
        turn_speed = TURN_P * center_of_mass[0]

        drive_msg = Twist()
        drive_msg.linear.x = np.clip(drive_speed, -1.0, 1.0)
        drive_msg.angular.z = np.clip(turn_speed, -1.0, 1.0)
        # self.publisher.publish(drive_msg)

        filter_msg = LaserScan()
        filter_msg.angle_min = np.min(points_in_range[0])
        filter_msg.angle_max = np.max(points_in_range[0])
        filter_msg.angle_increment = lidar_scan.angle_increment
        filter_msg.range_min = lidar_scan.range_min
        filter_msg.range_max = lidar_scan.range_max
        filter_msg.time_increment = lidar_scan.time_increment
        filter_msg.scan_time = lidar_scan.scan_time

        full_t_filter = (angles >= min_angle) & (angles <= max_angle)
        truncated_ranges = ranges[full_t_filter]
        full_r_filter = (truncated_ranges > min_range) & (truncated_ranges < max_range)
        truncated_ranges[~full_r_filter] = 0
        range_list = [float(r) for r in truncated_ranges]
        filter_msg.ranges = range_list
        self.filtered_scan_pub.publish(filter_msg)


def main():
    rclpy.init()
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
