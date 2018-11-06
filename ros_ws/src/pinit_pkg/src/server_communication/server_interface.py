#!/usr/bin/env python

from Queue import Queue

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2


from server_mapping_handler import ServerMappingHandler
from server_map_streamer import ServerMapStreamer

import rospy


class ServerHandler():

    """Handles all communication with the server"""

    def __init__(self):

        self.robot_name = "nemo"
        self.node_name = "robot_grpc_server_handler"
        self.max_message_length = 1024 * 1024 * 10
        self.init_node()
        self.communication_queue = Queue()

        self.mapping_hanlder = ServerMappingHandler(self.communication_queue)
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
        for communication in iter(self.communication_queue.get, None):
            yield communication


    def main_loop(self):
        """The main thread for communicating with the server

        Args:
            None

        Returns:
            None

        Raises:
            grpc exceptions. please refer to grpc
        """

        with grpc.insecure_channel('10.40.37.149:50052',
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



if __name__ == '__main__':
    try:
        print("running as main")
        handler = ServerHandler()
    except rospy.ROSInterruptException as e:
        pass

