#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

#use OOP to create class to create node
#inherit from the Node class of the rclpy
class mynode(Node): 

    def __init__(self): #building constructor
        super().__init__("firstnode") #node name firstname, initialized node
        self.counter_ = 0
        self.create_timer(1.0,self.timer_callback) #calls the timer_callback every 1 seconds

#print every certain time 
    def timer_callback(self):
        self.get_logger().info("Nisath Labee " + str(self.counter_)) #self will get th functionalities of Node 
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    #args equalls the args coming from the main input

    #run multiple nodes
    #name_ownchoice = class name
    node = mynode() #node created
    #node kept alive indefinitly until kill them or interrupt
    rclpy.spin(node) #kill this with Ctrl + C

    rclpy.shutdown() #shutdown ros2 communication
    pass

#directly execute the file from the terminal
if __name__ == '__main__':
    main()              

#file name: myfirstnode.py
#node name: firstnode
#executable name: node name after colcon

#need to run this with ros2 functionalities 
#1 install the node in the setup.py