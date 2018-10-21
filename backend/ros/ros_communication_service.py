import base64
import cv2
import grpc
import numpy as np
import threading
import time
import queue

from concurrent import futures

from backend.models import map as map_model
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2



class RosService(ros_pb2_grpc.RosServiceServicer):
    def __init__(self):
        self._robot_name_to_queue = {}

    def HandleRosData(self, request_iterator, robot_name):
        for request in request_iterator:
            if request.HasField("raw_map"):
                map_data = request.raw_map.data
                map_height = request.raw_map.height
                map_width = request.raw_map.width
                map_array = np.frombuffer(map_data, dtype='uint8').reshape(map_height, map_width)
                print(map_array)
                
    def Communicate(self, request_iterator, context):
        first_request = next(request_iterator)
        robot_name = first_request.robot_name
        threading.Thread(
            target=self.HandleRosData,
            args=(request_iterator, robot_name)
        ).start()
        if robot_name not in self._robot_name_to_queue:
            self._robot_name_to_queue[robot_name] = queue.Queue()
        communication_queue = self._robot_name_to_queue[robot_name]
        for communication in iter(communication_queue.get, None): 
            # Iterate forever, only ROS node can terminate connection.
            yield communication

    def SendMovement(self, request, context):
        if request.robot_name not in self._robot_name_to_queue:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Specified robot not connected.")
        else:
            communication_queue = self._robot_name_to_queue[request.robot_name]
            communication_queue.put(ros_pb2.ServerToRosCommunication(
                mapping_request = request.mapping_request
            ))
            return ros_pb2.MappingResponse()