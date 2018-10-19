#!/usr/bin/env python
from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from ros_ws.src.pinit_pkg.src.robot_motion.motion_controller import MotionController

import roslaunch
import rospy
import rospkg



class ServerMappingHandler():

    class RobotState(Enum):
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
        """initializes a dictionary of state transitions"""
        self.transitions = {
            self.RobotState.MAPPING :
            [self.RobotState.IDLE,
             self.RobotState.MOVING_AND_MAPPING],
            self.RobotState.IDLE:
            [self.RobotState.MAPPING,
             self.RobotState.MOVING_AND_MAPPING,
             self.RobotState.IDLE],
            self.RobotState.MOVING_AND_MAPPING :
            [self.RobotState.IDLE,
             self.RobotState.MOVING_AND_MAPPING]
        }


    def handle_request(self, request):
        """handle grpcs mapping_request"""
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
        """transistion to another state if valid"""
        valid_transitions = self.transitions[self.current_state]

        if state in valid_transitions:
            self.current_state = state
        else:
            print("Warning invalid state transistion")


    def start_mapping(self):
        """starts ros mapping launch file"""
        self.goto_state(self.RobotState.MAPPING)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid,\
                    [self.launch_file_path])


    def stop_mapping(self):
        """kills ros mapping launch file"""
        self.goto_state(self.RobotState.IDLE)

        self.launch.shutdown()


    def move(self, direction):
        """moves the robot by publishing to /cmd_vel"""
        self.goto_state(self.RobotState.MOVING_AND_MAPPING)
        self.motion_controller.move(direction)
