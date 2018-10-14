#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist

class MotionController:

    def __init__(self):

        self.node_name = 'motion_controller'
        self.topic_name = 'cmd_vel'
        self.rate = 0
        self.vel_linear = 0
        self.vel_linear_max = 0.5
        self.vel_linear_min = -0.5
        self.vel_angular = 0
        self.vel_angular_max = 4.25
        self.vel_angular_min = -4.25
        self.acceleration_steps = 100
        self.speed_pub = rospy.Publisher(self.topic_name, 
                                        Twist,
                                        queue_size=10)
    
    
    class MOTION_DIRECTION(Enum):
        FORWARD = 0
        BACKWARD = 1
        LEFT = 2
        RIGHT = 3
        STOP = 4


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)   
    

    def to_range(self, start, end, step):
        return np.linspace(start, end, step)

    
    def publish():
        angular = [0, 0, self.vel_angular]
        linear = [self.vel_linear, 0, 0]
        vel_pub.publish(Twist(linear, angular))


    def control_motion(self, direction):
        vel_linear, vel_angular = get_speeds(direction)
        n = len(vel_linear)
        for i in range(n):
            self.vel_linear = vel_linear
            self.vel_angular = vel_angular
            publish()


    def get_speeds(self, direction):    
        linear = []
        angular = []
        if(direction == MOTION_DIRECTION.FORWARD):
            linear = to_range(0, self.vel_linear_max, self.acceleration_steps)
            angular = to_range(0, 0, self.acceleration_steps)
        elif(direction == MOTION_DIRECTION.BACKWARD):
            linear = to_range(0, self.vel_linear_min, self.acceleration_steps)
            angular = to_range(0, 0, self.acceleration_steps)
        elif(direction == MOTION_DIRECTION.RIGHT):
            linear = to_range(0, 0, self.acceleration_steps)
            angular = to_range(0, self.vel_angular_max, self.acceleartion_steps)
        elif(direction == MOTION_DIRECTION.LEFT):
            linear = to_range(0, 0, self.acceleration_steps)
            angular = to_range(0, self.vel_angular_min, self.acceleartion_steps)
        elif(direction == MOTION_DIRECTION.STOP):
            linear = to_range(self.vel_linear, 0, self.acceleration_steps)
            angular = to_range(self.vel_angular, self.theta_min, self.acceleartion_steps)
        else:
            raise ValueError("Unknown motion direction. Check \
                            MotionController.MOTION_DIRECTION")
        
        return linear, angular
          


if __name__ == '__main__':
    controller = MotionController()
    controller.init_node()


