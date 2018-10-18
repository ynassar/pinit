#!/usr/bin/env python

import grpc

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

from enum import Enum

from server_mapping_handler import ServerMappingHandler

import rospy


class ServerHandler():

    def __init__(self):

        self.robot_name = "nemo"
        self.node_name = "robot_grpc_server_handler"
        self.mapping_hanlder = ServerMappingHandler()
    
        self.init_node()
        self.main_loop()


    def init_node(self):
        rospy.init_node(self.node_name, anonymous=False)


    def stub_callback(self):
        yield ros_pb2.RosToServerCommunication(robot_name=self.robot_name)


    def main_loop(self):
        while not rospy.is_shutdown():
            with grpc.insecure_channel('10.40.54.241:50052') as channel:
                stub = ros_pb2_grpc.RosServiceStub(channel)
                for communication in stub.Communicate(self.stub_callback()):
                    print communication
                    if communication.HasField("mapping_request"):
                        mapping_request = communication.mapping_request
                        self.mapping_hanlder.handle_request(mapping_request)



if __name__ == '__main__':
    try:
        handler = ServerHandler()
    except rospy.ROSInterruptException:
                pass