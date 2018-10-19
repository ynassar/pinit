#!/usr/bin/env python

from concurrent import futures
from queue import Queue
from enum import Enum
import time

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MotionController():

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
        self.vel_queue = Queue()
        self.vel_pub = rospy.Publisher(self.topic_name,
                                       Twist,
                                       queue_size=10)

        self.init_publisher_loop()

    class RobotDirection(Enum):
        FORWARD = 0
        BACKWARD = 1
        LEFT = 2
        RIGHT = 3
        STOP = 4


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)


    def init_publisher_loop(self):
       executor = futures.ThreadPoolExecutor(max_workers=1)
       executor.submit(self.publisher_loop)


    def publisher_loop(self):
        while True:
            time.sleep(0.1)
            try:
                vel = self.vel_queue.get_nowait()
                self.vel_linear = vel[0]
                self.vel_angular = vel[1]
                self.publish()
            except Queue.Empty:
                self.publish()


    def to_range(self, start, end):
        """returns a fixed list with length=self.acceleration_steps elements between start and end"""

        return np.linspace(start, end, self.acceleration_steps)


    def publish(self):
        angular = Vector3(0, 0, self.vel_angular)
        linear = Vector3(self.vel_linear, 0, 0)
        msg = Twist(linear, angular)

        self.vel_pub.publish(msg)


    def move(self, direction):
        vel_linear, vel_angular = self.get_speeds(direction)
        n = len(vel_linear)
        for i in range(n):
            vel_linear = vel_linear[i]
            vel_angular = vel_angular[i]
            vel = (vel_linear, vel_angular)
            self.vel_queue.put(vel)


    def get_speeds(self, direction):
        linear = []
        angular = []
        if(direction == self.RobotDirection.FORWARD):
            linear = self.to_range(0, self.vel_linear_max)
            angular = self.to_range(self.vel_angular, 0)
        elif(direction == self.RobotDirection.BACKWARD):
            linear = self.to_range(0, self.vel_linear_min)
            angular = self.to_range(self.vel_angular, 0)
        elif(direction == self.RobotDirection.RIGHT):
            linear = self.to_range(self.vel_linear, 0)
            angular = self.to_range(0, self.vel_angular_max)
        elif(direction == self.RobotDirection.LEFT):
            linear = self.to_range(self.vel_linear, 0)
            angular = self.to_range(0, self.vel_angular_min)
        elif(direction == self.RobotDirection.STOP):
            linear = self.to_range(self.vel_linear, 0)
            angular = self.to_range(self.vel_angular, 0)
        else:
            raise ValueError("Unknown motion direction. Check \
                              MotionController.RobotDirection")

        return linear, angular



if __name__ == '__main__':
    controller = MotionController()
    controller.init_node()
