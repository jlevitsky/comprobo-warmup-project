
import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# from dance import Dance
# from person_follower import PersonFollower
# from obstacle_avoider import ObstacleAvoider
# from wall_follower import WallFollower

import numpy as np


# class FinitePersonFollower(PersonFollower):
#     def __init__(self):
#         super().__init__()
#         self.do_action = False
#         self.state_topic = self.create_subscription(String, 'action', self.change_state, 10)
    
#     def follow_person(self, lidar_scan: LaserScan):
#         if self.do_action:
#             super().follow_person(lidar_scan)
    
#     def change_state(self, new_state: String):
#         self.do_action = new_state.data == 'person'


# class FiniteObstacleAvoider(ObstacleAvoider):
#     def __init__(self):
#         super().__init__()
#         self.do_action = False
#         self.state_topic = self.create_subscription(String, 'action', self.change_state, 10)
    
#     def process_scan(self, lidar_scan: LaserScan):
#         if self.do_action:
#             super().process_scan(lidar_scan)
    
#     def change_state(self, new_state: String):
#         self.do_action = new_state.data == 'avoid'


# class FiniteWallFollower(WallFollower):
#     def __init__(self):
#         super().__init__()
#         self.do_action = False
#         self.state_topic = self.create_subscription(String, 'action', self.change_state, 10)
    
#     def process_scan(self, lidar_scan: LaserScan):
#         if self.do_action:
#             super().process_scan(lidar_scan)
    
#     def change_state(self, new_state: String):
#         self.do_action = new_state.data == 'wall'


# class FiniteDance(Dance):
#     def __init__(self):
#         super().__init__()
#         self.do_action = False
#         self.state_topic = self.create_subscription(String, 'action', self.change_state, 10)
    
#     def run_loop(self):
#         if self.do_action:
#             super().run_loop()
#         else:
#             self.state = 0
    
#     def change_state(self, new_state: String):
#         self.do_action = new_state.data == 'dance'


settings = termios.tcgetattr(sys.stdin)


def getKey():
    tty.setraw(sys.stdin.fileno())
    inp_list, _, _ = select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.state_publisher = self.create_publisher(String, 'action', 10)
        self.create_timer(0.1, self.run_loop)
    
    def run_loop(self):
        key = getKey()
        action = String()
        if key == 'w':
            action.data = 'wall'
        elif key == 'a':
            action.data = 'avoid'
        elif key == 's':
            action.data = 'person'
        elif key == 'd':
            action.data = 'dance'
        else:
            action.data = 'stop'
        self.state_publisher.publish(action)


DRIVE_TIME = 0.5
TURN_TIME = 0.25
FORWARD_ALPHA = 1000.0
POINT_ALPHA = 1.0


class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')
        self.create_timer(0.1, self.run_loop)
        self.action = 'stop'
        self.state = 0
        self.state_start_time = -1
        self.drive_speed = 0
        self.turn_speed = 0
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def run_loop(self):
        key = getKey()
        if key =='w':
            self.drive_speed = 0.5
            self.turn_speed = 0.5
            self.action = 'wall'
        elif key =='a':
            self.drive_speed = 0.1
            self.turn_speed = 0.6
            self.action = 'avoid'
        elif key =='s':
            self.drive_speed = 0.15
            self.turn_speed = 2.0
            self.action = 'person'
        elif key =='d':
            self.state = 0
            self.state_start_time = -1
            self.drive_speed = 0.75
            self.turn_speed = 1.0
            self.action = 'dance'
        else:
            self.action = 'stop'
    
    def process_scan(self, lidar_scan: LaserScan):
        if self.action == 'dance':
            current_time, _ = self.get_clock().now().seconds_nanoseconds()
            if self.state_start_time == -1:
                drive_msg = Twist()
                drive_msg.angular.z = self.turn_speed
                self.publisher.publish(drive_msg)
                self.state_start_time = current_time
            if self.state % 8 < 3:
                if current_time - self.state_start_time > TURN_TIME:
                    self.state += 1
                    drive_msg = Twist()
                    drive_msg.angular.z = self.turn_speed * (-1 if self.state % 2 == 1 else 1)
                    self.publisher.publish(drive_msg)
                    self.state_start_time = current_time
            elif self.state % 8 == 3:
                if current_time - self.state_start_time > TURN_TIME:
                    self.state += 1
                    drive_msg = Twist()
                    drive_msg.linear.x = self.drive_speed
                    self.publisher.publish(drive_msg)
                    self.state_start_time = current_time
            elif self.state % 8 < 7:
                if current_time - self.state_start_time > DRIVE_TIME:
                    self.state += 1
                    drive_msg = Twist()
                    drive_msg.linear.x = self.drive_speed * (-1 if self.state % 2 == 1 else 1)
                    self.publisher.publish(drive_msg)
                    self.state_start_time = current_time
            else:
                if current_time - self.state_start_time > DRIVE_TIME:
                    self.state += 1
                    drive_msg = Twist()
                    drive_msg.linear.x = self.turn_speed
                    self.publisher.publish(drive_msg)
                    self.state_start_time = current_time
                    self.state = 0
        elif self.action == 'wall':
            distance_1=lidar_scan.ranges[-45]
            distance_2=lidar_scan.ranges[-135]
            if distance_1==0:
                distance_1=lidar_scan.range_max
            if distance_2==0:
                distance_2=lidar_scan.range_max

            turn_speed=-self.turn_speed*(distance_1-distance_2)
            if turn_speed>1:
                turn_speed=1.0
            if turn_speed<-1:
                turn_speed=-1.0

            motion_command=Twist()
            motion_command.linear.x=self.drive_speed
            motion_command.angular.z=float(turn_speed)
            self.publisher.publish(motion_command)
        elif self.action == 'person':
            angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max, lidar_scan.angle_increment)
            # Flip angles around the y axis b/c straight forward is 180??
            angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
            ranges = np.array(lidar_scan.ranges)
            points = np.stack([angles,ranges])      
            point_filter = (ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
            filtered_points = points[:, point_filter]

            min_angle = -np.pi/4
            max_angle = np.pi/4
            min_range = 0.3
            max_range = 1.5
            r_filter = (filtered_points[1, :] > min_range) & (filtered_points[1, :] < max_range)
            t_filter = (filtered_points[0, :] >= min_angle) & (filtered_points[0, :] <= max_angle)
            points_in_range = filtered_points[:, r_filter & t_filter]
            if points_in_range.size == 0:
                print('No points found')
                stop_msg = Twist()
                self.publisher.publish(stop_msg)
                return

            print(points_in_range)
            center_of_mass = np.mean(points_in_range, 1)
            print(f'Going to {center_of_mass[0] * 180.0/np.pi} deg, {center_of_mass[1]} m')
            drive_speed = self.drive_speed * center_of_mass[1]
            turn_speed = -self.turn_speed * center_of_mass[0]

            drive_msg = Twist()
            drive_msg.linear.x = np.clip(drive_speed, -1.0, 1.0)
            drive_msg.angular.z = np.clip(turn_speed, -1.0, 1.0)
            self.publisher.publish(drive_msg)
        elif self.action == 'avoid':
            angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max, lidar_scan.angle_increment)
            # Flip angles around the y axis b/c straight forward is 180??
            angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
            ranges = np.array(lidar_scan.ranges)
            points = np.stack([angles,ranges])      
            point_filter = (ranges >= lidar_scan.range_min) & (ranges <= lidar_scan.range_max)
            filtered_points = points[:, point_filter]

            min_angle = -np.pi/4
            max_angle = np.pi/4
            min_range = 0.3
            max_range = 1.5
            r_filter = (filtered_points[1, :] > min_range) & (filtered_points[1, :] < max_range)
            t_filter = (filtered_points[0, :] >= min_angle) & (filtered_points[0, :] <= max_angle)
            points_in_range = filtered_points[:, r_filter & t_filter]
            if points_in_range.size == 0:
                print('No points found')
                stop_msg = Twist()
                self.publisher.publish(stop_msg)
                return

            print(points_in_range)
            center_of_mass = np.mean(points_in_range, 1)
            print(f'Going to {center_of_mass[0] * 180.0/np.pi} deg, {center_of_mass[1]} m')
            drive_speed = self.drive_speed * center_of_mass[1]
            turn_speed = -self.turn_speed * center_of_mass[0]

            drive_msg = Twist()
            drive_msg.linear.x = np.clip(drive_speed, -1.0, 1.0)
            drive_msg.angular.z = np.clip(turn_speed, -1.0, 1.0)
            self.publisher.publish(drive_msg)
        else:
            stop_msg = Twist()
            self.publisher.publish(stop_msg)


def main():
    rclpy.init()
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()
