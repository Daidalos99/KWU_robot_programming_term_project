#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist #i have imported my own msg types

from roboid import *

class MyNode(Node):

    def __init__(self):
        super().__init__("get_something")
        # we don't need timer for subscriber
        # self.create_timer(0.5, self.timer_callback) # function which Node gives 
        #                                             # create_timer( period between two callbacks, function to call )
        self.subscriber_ = self.create_subscription(Twist,"/cmd_vel",self.subscriber_callback,10)
                                #create subscriber(msg_type, "topic_name", callback_function, queue_size)
        self.beagle = Beagle()
        self.get_logger().info("sub open")
        

    def subscriber_callback(self,msg):
        self.get_logger().info('%d,%d'%(msg.linear.x,msg.angular.z))
        self.beagle._motion_sec( 0.1, msg.linear.x , msg.angular.z )
        


def main(args = None):
    rclpy.init(args=args) # we need to contain this in first line of all program
    node = MyNode()
    rclpy.spin(node) # continue to be your node alive 
                     # when we enter ctrl+c spin goes stop
    rclpy.shutdown() # in the end we need to shut down ros2 program


if __name__=="_main__":
    main()