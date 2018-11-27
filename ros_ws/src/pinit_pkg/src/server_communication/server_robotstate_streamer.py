#!/usr/bin/env python

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import utils

class ServerStateStreamer():

    @classmethod
    def create(cls, robot_name, state_manager, communication_queue):
        return ServerStateStreamer(
            robot_name=robot_name,
            state_manager=state_manager,
            communication_queue=communication_queue)


    def __init__(self, robot_name, state_manager, communication_queue):
        self.robot_name = robot_name
        self.state_manager = state_manager
        self.communication_queue = communication_queue
        self.stream_thread = utils.ros_thread.SimpleThread()
        self.stream_thread.set_loop_function(self.stream)


    def start(self):
        self.stream_thread.start()


    def stop(self):
        self.stream_thread.stop()


    def stream(self):
        state = self.get_state_grpc()
        self.communication_queue.put(state)


    def get_state_grpc(self):
        state = self.get_state()
        grpc_state = self.ros_state_to_grpc(state)

        return grpc_state


    def get_state(self):
        return self.state_manager.get_current_state()


    def ros_state_to_grpc(self, state):
        grpc_state_msg = ros_pb2.RobotStatusUpdate()
        grpc_state_msg.status_update = state.value            #TODO I am not sure if this is valid

        grpc_communication_msg = ros_pb2.RosToServerCommunication(
                robot_name=self.robot_name,
                status_update=grpc_state_msg)

        return grpc_communication_msg

