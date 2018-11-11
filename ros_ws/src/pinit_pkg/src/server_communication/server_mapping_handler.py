#!/usr/bin/env python

from Queue import Queue
from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from robot_motion.motion_controller import MotionController
from map.map_streamer import MapStreamer
from map.map_creator import MapCreatorFactory

import roslaunch
import rospy
import rospkg


class ServerMappingHandler():
    """Handles server mapping requests"""

    class RobotState(Enum):
        """An enum for robot mapping states"""
        IDLE = 1
        MAPPING = 2
        MOVING_AND_MAPPING = 3


    def __init__(self, queue):

        self.communication_queue = queue
        self.map_streamer = MapStreamer(self.communication_queue)
        self.map_creator = MapCreatorFactory()
        self.motion_controller = MotionController()

        self.current_state = self.RobotState.IDLE
        self.transitions = None
        self.init_valid_transitions()


    def init_valid_transitions(self):
        """Initializes a dictionary of state transitions

        Args:
            None

        Returns:
            None
        """
        self.transitions = {
            self.RobotState.MAPPING :
            [self.RobotState.IDLE,
             self.RobotState.MOVING_AND_MAPPING],
            self.RobotState.IDLE:
            [self.RobotState.MAPPING,
             self.RobotState.IDLE],
            self.RobotState.MOVING_AND_MAPPING :
            [self.RobotState.IDLE,
             self.RobotState.MOVING_AND_MAPPING]
        }


    def goto_state(self, state):
        """Transistion to another state

        Args:
            state: the state which the robot attempts to transition to

        Returns:
            True or False if transition is successfull
        """

        valid_transitions = self.transitions[self.current_state]
        success = False

        if state in valid_transitions:
            self.current_state = state
            success = True
        else:
            print("Warning invalid state transistion")
            success = False

        return success


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

        if self.goto_state(self.RobotState.MOVING_AND_MAPPING):
            self.motion_controller.move(direction)


    def start_mapping(self):
        """Start ros mapping nodes and #TODO streams the map

        Args:
            None

        Returns:
            None
        """
        self.map_creator.start()
        self.map_streamer.start()


    def stop_mapping(self):
        """Stop ros mapping nodes and #TODO stop streaming the map

        Args:
            None

        Returns:
            None
        """

        self.map_creator.finish()
        self.map_streamer.finish()
