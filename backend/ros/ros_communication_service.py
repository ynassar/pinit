"""Implements a service for ROS-related communications."""
import base64
import jwt
import threading
import queue

import cv2
import grpc
import numpy as np
import mongoengine

from backend import constants
from backend.models import map as map_model
from backend.ros import map_utils
from backend.ros import request_utils
from backend.ros import ros_data_handler
from backend.ros import greyscale_map_renderer
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

class NoMapFound(Exception):
    pass

class RobotNotConnected(Exception):
    pass


def BuildRosService(ignore_unhandled_communication_types=False):
    """Factory method for RosService instances."""
    map_renderer = greyscale_map_renderer.GreyscaleMapRenderer()
    handler = ros_data_handler.RosDataHandler(map_renderer, ignore_unhandled_communication_types)
    return RosService(handler, constants.DEFAULT_RSA_KEY)

class RosService(ros_pb2_grpc.RosServiceServicer):
    """A servicer for the RosService, which implements RPCs related to ROS communication."""
    def __init__(self, ros_data_handler, rsa_key):
        self._robot_name_to_queue = {}
        self._ros_data_handler = ros_data_handler
        self._rsa_key = rsa_key

    def Communicate(self, request_iterator, context):
        """Handles two-way communication to and from a robot.
        
        Incoming data is handled by self._ros_data_handler, and outgoing messages should be
        pushed to self._robot_name_to_queue[robot_name], where robot_name is the name of the
        robot to send the message to.

        The first message sent by the robot should contain the robot name for identification.
        """
        first_request = next(request_iterator)
        robot_name = request_utils.RobotNameFromRequest(first_request, self._rsa_key)
        threading.Thread(
            target=self._ros_data_handler.HandleRequests,
            args=(request_iterator, robot_name)
        ).start()

        if robot_name not in self._robot_name_to_queue:
            self._robot_name_to_queue[robot_name] = queue.Queue()
        communication_queue = self._robot_name_to_queue[robot_name]
        for communication in iter(communication_queue.get, None):
            # Iterate forever, only ROS node can terminate connection.
            yield communication

    def SendMovement(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        if robot_name not in self._robot_name_to_queue:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Specified robot not connected.")
            raise RobotNotConnected()
        else:
            communication_queue = self._robot_name_to_queue[robot_name]
            if request.mapping_request.request_type == ros_pb2.ServerToRosMappingRequest.START_MAPPING:
                map_model.Map.objects(robot_name = robot_name).delete()
            communication_queue.put(ros_pb2.ServerToRosCommunication(
                mapping_request=request.mapping_request
            ))
            return ros_pb2.MappingResponse()

    def GetMapImage(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        try:
            map = map_model.Map.objects.get(robot_name = robot_name)
            return ros_pb2.MapImage(resolution=map.resolution, encoded_image=map.b64_image)
        except mongoengine.DoesNotExist:
            raise NoMapFound()
    
    def GetRawMap(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        try:
            map = map_model.Map.objects.get(robot_name = request.robot_name)
            return map_utils.RawMapFromMapDocument(map)
        except mongoengine.DoesNotExist:
            raise NoMapFound()