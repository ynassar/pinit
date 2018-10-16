#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MotionController:

    def __init__(self):

        self.node_name = 'motion_controller'
        self.topic_name = 'cmd_vel'
        self.vel_linear = 0
        self.vel_linear_max = 0.4
        self.vel_linear_min = -0.5
        self.vel_angular = 0
        self.vel_angular_max = 2.25
        self.vel_angular_min = -2.25
        self.acceleration_steps = 1000
        self.vel_pub = rospy.Publisher(self.topic_name, 
                                        Twist,
                                        queue_size=10)
    
    
    class DIRECTION(Enum):
        FORWARD = 0
        BACKWARD = 1
        LEFT = 2
        RIGHT = 3
        STOP = 4


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)   
    

    def to_range(self, start, end, n):
        """returns a fixed list with n elements between start and end"""
        
        return np.linspace(start, end, n)

    
    def publish(self):
        angular = Vector3(0, 0, self.vel_angular)
        linear = Vector3(self.vel_linear, 0, 0)
        msg = Twist(linear, angular)
        
        self.vel_pub.publish(msg)


    def move(self, direction):
        vel_linear, vel_angular = self.get_speeds(direction)
        n = len(vel_linear)
        for i in range(n):
            self.vel_linear = vel_linear[i]
            self.vel_angular = vel_angular[i]
            self.publish()


    def get_speeds(self, direction):    
        linear = []
        angular = []
        if(direction == self.DIRECTION.FORWARD):
            linear = self.to_range(0, self.vel_linear_max, self.acceleration_steps)
            angular = self.to_range(0, 0, self.acceleration_steps)
        elif(direction == self.DIRECTION.BACKWARD):
            linear = self.to_range(0, self.vel_linear_min, self.acceleration_steps)
            angular = self.to_range(0, 0, self.acceleration_steps)
        elif(direction == self.DIRECTION.RIGHT):
            linear = self.to_range(0, 0, self.acceleration_steps)
            angular = self.to_range(0, self.vel_angular_max, self.acceleration_steps)
        elif(direction == self.DIRECTION.LEFT):
            linear = self.to_range(0, 0, self.acceleration_steps)
            angular = self.to_range(0, self.vel_angular_min, self.acceleration_steps)
        elif(direction == self.DIRECTION.STOP):
            linear = self.to_range(self.vel_linear, 0, self.acceleration_steps)
            angular = self.to_range(self.vel_angular, 0, self.acceleration_steps)
        else:
            raise ValueError("Unknown motion direction. Check \
                            MotionController.DIRECTION")
        
        return linear, angular
          


if __name__ == '__main__':
    controller = MotionController()
    controller.init_node()
