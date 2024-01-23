#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #find this using ros2 topic info /turtle1/cmd_vel

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle") #node name
        #initialize publisher
        self.cmd_vel_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5,self.send_velocity_command)
        #print to screen
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist()
        #use the following to find what to sent, ros2 interface show geometry_msgs/msg/Twist
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()