#!/usr/bin/env python

from concurrent import futures
from enum import Enum
import threading
import time

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
        self.vel_angular_range = (-2.25, 2.25)
        self.acceleration_steps = 100
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

        self.init_publisher_loop()


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)


    def init_publisher_loop(self):
       self.thread_executor = threading.Thread(target=self.publisher_loop)
       self.thread_executor.start()
       self.direction_lock = threading.Lock()
       rospy.loginfo("velocity publisher loop init")


    def publisher_loop(self):
        while True:
            time.sleep(0.01)
            linear = self.vel_linear
            angular = self.vel_angular
            self.direction_lock.acquire()
            direction = self.robot_direction
            self.direction_lock.release()
            if(direction == self.RobotDirection.FORWARD):
                linear = linear + self.vel_linear_increment
                linear = min(linear, self.vel_linear_range[1])
            elif(direction == self.RobotDirection.BACKWARD):
                linear = linear - self.vel_linear_increment
                linear = max(linear, self.vel_linear_range[0])
            elif(direction == self.RobotDirection.LEFT):
                angular = angular + self.vel_angular_increment
                angular = min(angular, self.vel_angular_range[1])
            elif(direction == self.RobotDirection.RIGHT):
                angular = angular - self.vel_angular_increment
                angular = max(angular, self.vel_angular_range[0])
            elif(direction == self.RobotDirection.STOP):
                if linear > 0:
                    linear = linear - self.vel_linear_increment
                    linear = max(linear, 0)
                elif linear < 0:
                    linear = linear + self.vel_linear_increment
                    linear = min(linear, 0)
                else:
                    linear = linear
                if angular > 0:
                    angular = angular - self.vel_angular_increment
                    angular = max(angular, 0)
                elif angular < 0:
                    angular = angular + self.vel_angular_increment
                    angular = min(angular, 0)
                else:
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
