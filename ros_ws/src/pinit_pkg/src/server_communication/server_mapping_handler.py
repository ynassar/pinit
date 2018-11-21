#!/usr/bin/env python

from Queue import Queue
from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from robot_motion.motion_controller import MotionController

import roslaunch
import rospy
import rospkg


class ServerMappingHandler():
    """Handles server mapping requests"""


    def __init__(self, robot_manager):
        self.robot_manager = robot_manager


    def handle_request(self, request):
        """A big switch for different mapping_request messages

        Args:
            None

        Returns:
            None
        """

        request_type = request.request_type
        direction = request.direction
        if request_type == ros_pb2.ServerToRosMappingRequest.START_MAPPING:
            self.start_mapping()
        elif request_type == ros_pb2.ServerToRosMappingRequest.STOP_MAPPING:
            self.stop_mapping()
        else:
            if direction == ros_pb2.ServerToRosMappingRequest.FORWARD:
                self.move(MotionController.RobotDirection.FORWARD)
            elif direction == ros_pb2.ServerToRosMappingRequest.BACKWARD:
                self.move(MotionController.RobotDirection.BACKWARD)
            elif direction == ros_pb2.ServerToRosMappingRequest.LEFT:
                self.move(MotionController.RobotDirection.LEFT)
            elif direction == ros_pb2.ServerToRosMappingRequest.RIGHT:
                self.move(MotionController.RobotDirection.RIGHT)
            else:
                self.move(MotionController.RobotDirection.STOP)


    def move(self, direction):
        """Move the robot in a specific direction

        Args:
            direction: The direction of motion

        Returns:
            None
        """

        self.robot_manager.go_to(self.robot_manager.States.MAPPING, direction)


    def start_mapping(self):
        """Start ros mapping nodes and #TODO streams the map

        Args:
            None

        Returns:
            None
        """

        self.robot_manager.go_to(self.robot_manager.States.MAPPING)


    def stop_mapping(self):
        """Stop ros mapping nodes and #TODO stop streaming the map

        Args:
            None

        Returns:
            None
        """

        self.robot_manager.go_to(self.robot_manager.States.IDLE)
