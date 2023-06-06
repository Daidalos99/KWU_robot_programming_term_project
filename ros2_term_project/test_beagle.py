#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboid import *
import numpy as np

from sensor_msgs.msg import LaserScan  #i have imported my own msg types
from geometry_msgs.msg import Twist

class MyNode(Node):

    def __init__(self):
        super().__init__("beagle_encoder_pub")
        self.create_timer(0.2, self.timer_callback) # function which Node gives 
                                                    # create_timer( period between two callbacks, function to call )
        self.publisher_ = self.create_publisher(LaserScan,"/scan", 10)
                                            # (msg type, "topic_name", queue_size)
        self.timer_ = self.create_timer(0.2, self.timer_callback)

        self.subscriber_ = self.create_subscription(Twist,"/cmd_vel",self.subscriber_callback,10)
      
        self.beagle = Beagle()

        self.left_encoder = 0
        self.right_encoder = 0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.left_encoder_err = 0
        self.right_encoder_err = 0
    
    def timer_callback(self):
        #print("left = %d, right = %d"%(self.beagle.left_encoder(),self.beagle.right_encoder()))
        self.left_encoder = self.beagle.left_encoder()
        self.right_encoder = self.beagle.right_encoder()

        self.left_encoder_err = self.left_encoder - self.last_left_encoder
        self.right_encoder_err = self.right_encoder - self.last_right_encoder

        print("left = %d, right = %d"%(self.left_encoder_err,self.right_encoder_err))

        self.last_left_encoder = self.left_encoder
        self.last_right_encoder = self.right_encoder

    def subscriber_callback(self,msg):
        self.beagle._motion_sec(0.1,msg.linear.x,msg.angular.z)

def main(args = None):
    
    rclpy.init(args=args) # we need to contain this in first line of all program
    node = MyNode()
    
    rclpy.spin(node) # continue to be your node alive 
                     # when we enter ctrl+c spin goes stop
    rclpy.shutdown() # in the end we need to shut down ros2 program


if __name__=="_main__":
    main()