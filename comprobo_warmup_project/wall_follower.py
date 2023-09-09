import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

DRIVE_SPEED = 0.5
TURN_SPEED = 0.5

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.laser_topic= self.create_subscription(LaserScan,'scan',self.process_scan,10)

    def process_scan(self, lidar_scan):
#         angles=np.arange(lidar_scan.angle_min,lidar_scan.angle_max, lidar_scan.angle_increment)
#         angles=np.round(angles*180/np.pi)
#         ranges=np.array(lidar_scan.ranges)
#         print(angles)
#         print(ranges)
#          points=np.stack([angles,ranges])      
#         point_filter=(ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
#         filtered_points=points[:, point_filter]
#         print(filtered_points)

        distance_1=lidar_scan.ranges[-45]
        distance_2=lidar_scan.ranges[-135]
        if distance_1==0:
            distance_1=lidar_scan.range_max
        if distance_2==0:
            distance_2=lidar_scan.range_max
        

      #  distance_1=filtered_points[1, filtered_points[0,:]==45]     #Take out the distance 45 degrees from the horizonzal
      #  distance_2=filtered_points[1, filtered_points[0,:]==135]    #Take out the distance -45 degrees from the horizonzal
        turn_speed=-TURN_SPEED*(distance_1-distance_2)
        if turn_speed>1:
            turn_speed=1.0
        if turn_speed<-1:
            turn_speed=-1.0

        motion_command=Twist()
        motion_command.linear.x=DRIVE_SPEED
        motion_command.angular.z=float(turn_speed)
        self.publisher.publish(motion_command)

def main():
    rclpy.init()
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    