#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboid import *
import numpy as np

from sensor_msgs.msg import LaserScan  #i have imported my own msg types
from geometry_msgs.msg import Twist


class MyNode(Node):

    def __init__(self):
        super().__init__("beagle_laser_pub")
        self.create_timer(0.2, self.timer_callback) # function which Node gives 
                                                    # create_timer( period between two callbacks, function to call )
        self.publisher_ = self.create_publisher(LaserScan,"/scan", 10)
                                            # (msg type, "topic_name", queue_size)
        self.timer_ = self.create_timer(0.2, self.timer_callback)

        self.subscriber_ = self.create_subscription(Twist,"/cmd_vel",self.subscriber_callback,10)
      
        self.beagle = Beagle()

        self.value = [0.0,]
        self.nvalue = [0.0,]
    
    def timer_callback(self):
        self.publish_lidar()
        print("left = %d, right = %d"%(self.beagle.left_encoder(),self.beagle.right_encoder()))

    def subscriber_callback(self,msg):
        self.beagle._motion_sec(0.1,msg.linear.x,msg.angular.z)

    def publish_lidar(self):
        self.value = self.beagle.lidar()
        msg = LaserScan()
        msg.header.frame_id = 'laser'
        msg.angle_min = -3.14
        msg.angle_max = -3.14
        msg.angle_increment = 3.14/180
        msg.time_increment = 0.001
        msg.scan_time = 0.2
        msg.range_max = 5.0
        msg.range_min = 0.05
        print(len(self.value))
        print(str(type(self.value[0])))
        for i in range (0,359):
            if self.value[i] >= 0 and self.value[i] <= 550 :
                self.value[i] = self.value[i]/100.0
            else :
                self.value[i] = 5.55
        msg.ranges = list(np.float_(self.value))

        self.publisher_.publish(msg)

def main(args = None):
    
    rclpy.init(args=args) # we need to contain this in first line of all program
    node = MyNode()
    node.beagle.start_lidar()
    node.beagle.wait_until_lidar_ready()
    print('lidar is ready.')
    
    rclpy.spin(node) # continue to be your node alive 
                     # when we enter ctrl+c spin goes stop
    rclpy.shutdown() # in the end we need to shut down ros2 program


if __name__=="_main__":
    main()