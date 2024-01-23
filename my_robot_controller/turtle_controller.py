#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen #service package #already added in the package.xml
from functools import partial 

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0
        #Create Publisher
        self.cmd_vel_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        #Create Subscriber
        self.pose_subscriber_ = self.create_subscription(
            Pose,"/turtle1/pose",self.pose_callback,10)
        self.get_logger().info("Turtle Controller has been started")

    def pose_callback(self, msg: Pose):
        cmd = Twist()
        
        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        
        if msg.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = msg.x
            self.get_logger().info("Set color to red!")
            self.call_set_pen_service(255,0,0,3,0)
        elif msg.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = msg.x
            self.get_logger().info("Set color to green!")
            self.call_set_pen_service(0,255,0,3,0)

        self.cmd_vel_.publish(cmd)
    
    def call_set_pen_service(self,r,g,b,width,off):
        #create a client
        client = self.create_client(SetPen, "/turtle1/set_pen") # use ros2 service list
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service...")
        
        #creating request
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        #call client to respond immediately
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen)) 
        #the above code calls the callbac_set_pen function when the service has replied


    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e: 
            self.get_logger().error("Service call failed: %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()              
