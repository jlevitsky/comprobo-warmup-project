import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

DRIVE_SPEED = 0.5
TURN_SPEED = 0.5
SCAN_RANGE=3

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.laser_topic= self.create_subscription(LaserScan,'scan',self.process_scan,10)

    def process_scan(self, lidar_scan):
        
        angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max, lidar_scan.angle_increment)
        # Flip angles around the y axis b/c straight forward is 180??
        angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
        ranges = np.array(lidar_scan.ranges)
        points = np.stack([angles,ranges])    

        point_filter = (ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
        filtered_points = points+SCAN_RANGE*(~point_filter)

        obstacle_avoid_wave=filtered_points[1,:]
        go_forward_wave=-np.abs(angles)
        combined_wave=obstacle_avoid_wave+go_forward_wave

        preffered_angle=np.argmax(combined_wave)
        command_angle=angles[preffered_angle]


        motion_command=Twist()
        motion_command.linear.x=DRIVE_SPEED
        motion_command.angular.z=float(TURN_SPEED*command_angle)
        self.publisher.publish(motion_command)

def main():
    rclpy.init()
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
