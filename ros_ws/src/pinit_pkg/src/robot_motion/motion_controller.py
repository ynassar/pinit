#!/usr/bin/env python

from enum import Enum
import threading

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MotionController():
    """Handles the robot, iCreate, velocity"""

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
        """Initialize the class as a ros node

        Args:
            None

        Returns:
            None
        """

        rospy.init_node(self.node_name, anonymous=True)


    def init_vel_loop(self):
        """Start a separate thread to publish velocities

        Args:
            None

        Returns:
            None
        """

        self.thread_executor = threading.Thread(target=self.vel_loop)
        self.thread_executor.start()
        self.direction_lock = threading.Lock()
        rospy.loginfo("velocity publisher loop init")


    def accelerate_linear(self, vel):
        """Accelerate linear velocity

        Args:
            vel: the velocity to be accelerated

        Returns:
            accelerated_vel: the velocity after acceleration
        """

        accelerated_vel = vel + self.vel_linear_increment
        accelerated_vel = min(accelerated_vel, self.vel_linear_range[1])

        return accelerated_vel


    def decelerate_linear(self, vel):
        """Decelerate linear velocity

        Args:
            vel: the velocity to be decelerated

        Returns:
            decelerated_vel: the velocity after deceleration
        """

        decelerated_vel = vel - self.vel_linear_increment
        decelerated_vel = max(decelerated_vel, self.vel_linear_range[0])

        return decelerated_vel


    def vel_loop(self):
        """Change the velocity based on robot direction

        Args:
            None

        Returns:
            None
        """

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
                    linear = max(0, linear)
                elif linear < 0:
                    linear = self.accelerate_linear(linear)
                    linear = min(0, linear)
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
        """Publish to the robot velocity topic

        Args:
            None

        Returns:
            None
        """

        angular = Vector3(0, 0, self.vel_angular)
        linear = Vector3(self.vel_linear, 0, 0)
        msg = Twist(linear, angular)

        self.vel_pub.publish(msg)


    def move(self, direction):
        """Set the robot direction

        Args:
            direction: new direction of the robot

        Returns:
            None
        """

        self.direction_lock.acquire()
        self.robot_direction = direction
        self.direction_lock.release()
        rospy.loginfo("direction set" + str(direction))



if __name__ == '__main__':
    controller = MotionController()
