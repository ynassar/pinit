#!/usr/bin/env python

from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from robot_motion.motion_controller import MotionController

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


    def __init__(self):

        self.motion_controller = MotionController()
        self.current_state = self.RobotState.IDLE
        self.launch_file_path = rospkg.RosPack().get_path('pinit_pkg') + \
            "/launch/gmapping_launch.launch"
        self.launch = None
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


    def start_mapping(self):
        """Start ros mapping nodes and #TODO streams the map

        Args:
            None

        Returns:
            None
        """

        if (self.goto_state(self.RobotState.MAPPING)):
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid,\
            [self.launch_file_path])

            self.launch.start()


    def stop_mapping(self):
        """Stop ros mapping nodes and #TODO stop streaming the map

        Args:
            None

        Returns:
            None
        """

        if self.goto_state(self.RobotState.IDLE):
            self.launch.shutdown()


    def move(self, direction):
        """Move the robot in a specific direction

        Args:
            direction: The direction of motion

        Returns:
            None
        """

        if self.goto_state(self.RobotState.MOVING_AND_MAPPING):
            self.motion_controller.move(direction)
