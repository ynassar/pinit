#!/usr/bin/env python

from enum import Enum
import threading

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MotionController():

    class RobotDirection(Enum):
        FORWARD = 0
        BACKWARD = 1
        LEFT = 2
        RIGHT = 3
        STOP = 4


    def __init__(self):

        self.node_name = 'motion_controller'
        self.topic_name = 'cmd_vel'
        self.vel_linear = 0
        self.vel_angular = 0
        self.robot_direction = self.RobotDirection.STOP
        self.direction_lock = None
        self.vel_linear_range = (-0.5, 0.5)
        self.vel_angular_range = (-1.25, 1.25)
        self.acceleration_steps = 80
        self.vel_linear_increment = (abs(self.vel_linear_range[0] -
                                         self.vel_linear_range[1]) /
                                     self.acceleration_steps)
        self.vel_angular_increment = (abs(self.vel_angular_range[0] -
                                          self.vel_angular_range[1]) /
                                      self.acceleration_steps)
        self.thread_executor = None
        self.vel_pub = rospy.Publisher(self.topic_name,
                                       Twist,
                                       queue_size=10)

        self.init_vel_loop()


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)


    def init_vel_loop(self):
        self.thread_executor = threading.Thread(target=self.vel_loop)
        self.thread_executor.start()
        self.direction_lock = threading.Lock()
        rospy.loginfo("velocity publisher loop init")


    def accelerate_linear(self, vel):
        vel = vel + self.vel_linear_increment
        vel = min(vel, self.vel_linear_range[1])

        return vel


    def decelerate_linear(self, vel):
        vel = vel - self.vel_linear_increment
        vel = max(vel, self.vel_linear_range[0])

        return vel


    def vel_loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            linear = self.vel_linear
            angular = self.vel_angular
            self.direction_lock.acquire()
            direction = self.robot_direction
            self.direction_lock.release()
            if direction == self.RobotDirection.FORWARD:
                linear = self.accelerate_linear(linear)
            elif direction == self.RobotDirection.BACKWARD:
                linear = self.decelerate_linear(linear)
            elif direction == self.RobotDirection.LEFT:
                angular = self.vel_angular_range[1]
            elif direction == self.RobotDirection.RIGHT:
                angular = self.vel_angular_range[0]
            elif direction == self.RobotDirection.STOP:
                angular = 0
                if linear > 0:
                    linear = self.decelerate_linear(linear)
                elif linear < 0:
                    linear = self.accelerate_linear(linear)
                else:
                    linear = linear
                    angular = angular
            else:
                rospy.logwarn("Unknown motion direction. Check \
                                 MotionController.RobotDirection")

            self.vel_linear = linear
            self.vel_angular = angular
            self.publish()


    def publish(self):
        angular = Vector3(0, 0, self.vel_angular)
        linear = Vector3(self.vel_linear, 0, 0)
        msg = Twist(linear, angular)

        self.vel_pub.publish(msg)


    def move(self, direction):
        self.direction_lock.acquire()
        self.robot_direction = direction
        self.direction_lock.release()
        rospy.loginfo("direction set" + str(direction))



if __name__ == '__main__':
    controller = MotionController()
