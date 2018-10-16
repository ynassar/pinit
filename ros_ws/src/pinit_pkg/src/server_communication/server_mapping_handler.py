#!/usr/bin/env python
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from ros_ws.src.pinit_pkg.src.robot_motion.motion_controller import MotionController

from enum import Enum
import roslaunch
import rospy



class ServerMappingHandler():
    
    class ROBOT_STATE(Enum):
        IDLE = 1
        MAPPING = 2


    def __init__(self):

        self.motion_controller = MotionController()
        self.current_state = self.ROBOT_STATE.IDLE
        self.launch_file_path = ""
        self.launch = None
        self.transitions = None

        self.init_valid_transitions()


    def init_valid_transitions(self):
        """initializes a dictionary of state transitions"""
        self.transitions = {
            self.ROBOT_STATE.MAPPING : 
                [self.ROBOT_STATE.IDLE,
                self.ROBOT_STATE.MAPPING],
            self.ROBOT_STATE.IDLE:
                [self.ROBOT_STATE.MAPPING,
                self.ROBOT_STATE.IDLE]        
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
                self.move(MotionController.DIRECTION.FORWARD) 
            elif direction == ros_pb2.ServerToRosMappingRequest.BACKWARD:
                self.move(MotionController.DIRECTION.BACKWARD)
            elif direction == ros_pb2.ServerToRosMappingRequest.LEFT:
                self.move(MotionController.DIRECTION.LEFT)
            elif direction == ros_pb2.ServerToRosMappingRequest.RIGHT:
                self.move(MotionController.DIRECTION.RIGHT)
            else:
                self.move(MotionController.DIRECTION.STOP)

    
    def goto_state(self, state):
        """transistion to another state if valid"""
        valid_transitions = self.transitions[self.current_state]

        if state in valid_transitions:
            self.current_state = state
        else:
            raise Exception("Invalid transition from %s to %s" \
                % self.current_state \
                % state)
        

    def start_mapping(self):
        """starts ros mapping launch file"""
        try:
            self.goto_state(self.ROBOT_STATE.MAPPING)
            
            uid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid,\
                [self.launch_file_path])
        except Exception as e:
            print e

    
    def stop_mapping(self):
        """kills ros mapping launch file"""
        try:
            self.goto_state(self.ROBOT_STATE.IDLE)
            
            self.launch.shutdown()
        except Exception as e:
            print e

    
    def move(self, direction):
        """moves the robot by publishing to /cmd_vel"""
        try:
            self.goto_state(self.ROBOT_STATE.MAPPING)
            self.motion_controller.move(direction)
        except Exception as e:
            print e
        