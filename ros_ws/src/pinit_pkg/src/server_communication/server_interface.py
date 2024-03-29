#!/usr/bin/env python

from Queue import Queue

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from server_communication.server_mapping_handler import ServerMappingHandler
from server_communication.server_navigation_handler import ServerNavigationHandler

import rospy

from robot_state.state_manager import RobotStateManager
from map.map_publisher import MapPublisher
from nodes.node_manager import NodeManager
from robot_motion.robot_pose import PoseListenerFactory
from server_communication.server_pose_streamer import ServerPoseStreamerFactory
from gps import gps_cal
from map.map_streamer import MapStreamer
from robot_motion.motion_controller import MotionController


class ServerHandler():

    """Handles all communication with the server"""

    def __init__(self):

        self.robot_name = "nemo"
        self.node_name = "robot_grpc_server_handler"
        self.max_message_length = 1024 * 1024 * 10
        self.server_address = '10.40.47.229:50052'
        #self.server_address = 'localhost:50052'
        self.init_node()

        self.robot_manager = RobotStateManager.create(self.server_address, self.robot_name)
        self.robot_manager.go_to(self.robot_manager.States.IDLE)
        self.mapping_hanlder = ServerMappingHandler(self.robot_manager)
        self.navigation_handler = ServerNavigationHandler(self.robot_manager)
        #TODO we need to fix this
        self.robot_manager.nav_controller.state_manager = self.robot_manager

        self.main_loop()

    def init_node(self):
        """Initiallize the class as a ros node

        Args:
            None

        Returns:
            None
        """

        rospy.init_node(self.node_name,
                        anonymous=False,
                        disable_signals=True)


    def stub_callback(self):
        """Respond back to server

        Args:
            None

        Yields:
            RosToServerCommunication message with the robot name
            then pops messages from the server communication_queue
        """

        yield ros_pb2.RosToServerCommunication(robot_name=self.robot_name)
        for communication in iter(self.robot_manager.communication_queue.get, None):
            communication = self.fill_communication_msg(communication)
            yield communication


    def fill_communication_msg(self, msg):
        msg.robot_name = self.robot_name
        return msg


    def main_loop(self):
        """The main thread for communicating with the server

        Args:
            None

        Returns:
            None

        Raises:
            grpc exceptions. please refer to grpc
        """


        with grpc.insecure_channel(self.server_address,
                                   options=[
                                       ("grpc.max_send_message_length", self.max_message_length),
                                       ("grpc.max_receive_message_length", self.max_message_length)
                                   ]) as channel:
            stub = ros_pb2_grpc.RosServiceStub(channel)
            for communication in stub.Communicate(self.stub_callback()):
                print(communication)
                if communication.HasField("mapping_request"):
                    mapping_request = communication.mapping_request
                    self.mapping_hanlder.handle_request(mapping_request)
                elif communication.HasField("navigation_request"):
                    navigation_request = communication.navigation_request
                    self.navigation_handler.handle_request(navigation_request)
                        
                        




if __name__ == '__main__':
    try:
        print("running as main")
        handler = ServerHandler()
    except rospy.ROSInterruptException as e:
        pass

