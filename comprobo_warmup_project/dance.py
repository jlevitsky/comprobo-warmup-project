
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


TURN_TIME = .3
DRIVE_TIME = .75

DRIVE_SPEED = 0.75
TURN_SPEED = 0.75


class Dance(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.timer = self.create_timer(0.1, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = 0
        self.state_start_time = -1
    
    def run_loop(self):
        current_time, _ = self.get_clock().now().seconds_nanoseconds()
        if self.state_start_time == -1:
            drive_msg = Twist()
            drive_msg.linear.x = DRIVE_SPEED
            self.publisher.publish(drive_msg)
            self.state_start_time = current_time
        
        if self.state % 2 == 0:
            if current_time - self.state_start_time > DRIVE_TIME:
                self.state += 1
                turn_msg = Twist()
                turn_msg.angular.z = TURN_SPEED
                self.publisher.publish(turn_msg)
                turn_msg.angular.z = -TURN_SPEED
                self.publisher.publish(turn_msg)
                turn_msg.angular.z = TURN_SPEED
                self.publisher.publish(turn_msg)
                turn_msg.angular.z = -TURN_SPEED
                self.publisher.publish(turn_msg)
                self.state_start_time = current_time

        else:
            if current_time - self.state_start_time > TURN_TIME:
                self.state += 1
                drive_msg = Twist()
                drive_msg.linear.x = DRIVE_SPEED
                self.publisher.publish(drive_msg)
                drive_msg.linear.x = -DRIVE_SPEED
                self.publisher.publish(drive_msg)
                drive_msg.linear.x = -DRIVE_SPEED
                self.publisher.publish(drive_msg)
                self.state_start_time = current_time


def main():
    rclpy.init()
    node = Dance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
