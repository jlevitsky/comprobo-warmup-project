import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    inp_list, _, _ = select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.create_timer(0.1, self.run_loop)
        self.pub = self.create_publisher(Twist,'cmd_vel',10)

    def run_loop(self):
        speed=0.5
        key=getKey()
        cmd_vel=Twist()
        if key =='w':
            cmd_vel.linear.x=speed
        if key =='a':
            cmd_vel.angular.z=speed
        if key =='s':
            cmd_vel.linear.x=-speed
        if key =='d':
            cmd_vel.angular.z=-speed
        self.pub.publish(cmd_vel)

def main():
    rclpy.init()
    node=TeleopKey()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
