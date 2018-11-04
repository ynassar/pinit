"""Implements a service for ROS-related communications."""
import base64
import threading
import queue

import cv2
import grpc
import numpy as np
import mongoengine

from backend.models import map as map_model
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

LOW_CONFIDENCE_VALUE = 1
HIGH_CONFIDENCE_VALUE = 101
UNKNOWN_VALUE = 0

HIGH_CONFIDENCE_COLOR = 0
LOW_CONFIDENCE_COLOR = 255
UNKNOWN_COLOR = 127

class NoMapFound(Exception):
    pass

class RobotNotConnected(Exception):
    pass

def RenderToArray(occupancy_grid_array):
    """Returns a numpy array representing a rendering of an occupancy grid."""
    image = occupancy_grid_array.copy()
    low_confidence_mask = occupancy_grid_array == LOW_CONFIDENCE_VALUE
    high_confidence_mask = occupancy_grid_array == HIGH_CONFIDENCE_VALUE
    unknown_mask = occupancy_grid_array == UNKNOWN_VALUE
    image[low_confidence_mask] = LOW_CONFIDENCE_COLOR
    image[high_confidence_mask] = HIGH_CONFIDENCE_COLOR
    image[unknown_mask] = UNKNOWN_COLOR
    return image


class RosService(ros_pb2_grpc.RosServiceServicer):
    """A servicer for the RosService, which implements RPCs related to ROS communication."""
    def __init__(self):
        self._robot_name_to_queue = {}

    def HandleRosData(self, request_iterator, robot_name):
        """Handles messages passed from the ROS communication node to the server."""
        for request in request_iterator:
            if request.HasField("raw_map"):
                map_data = request.raw_map.data
                map_height = request.raw_map.height
                map_width = request.raw_map.width
                map_array = np.frombuffer(map_data, dtype='uint8').reshape(map_height, map_width)
                image_array = RenderToArray(map_array)
                _, encoded_image = cv2.imencode('.jpg', image_array)
                b64_image = base64.b64encode(encoded_image).decode('utf-8')
                map_model.Map.objects(robot_name=robot_name).update_one(
                    upsert=True,
                    set__resolution=request.raw_map.resolution,
                    set__b64_image=b64_image,
                    set__raw_map=map_data.decode('utf-8'),
                    set__height=map_height,
                    set__width=map_width)

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
            raise RobotNotConnected()
        else:
            communication_queue = self._robot_name_to_queue[request.robot_name]
            if request.mapping_request.request_type == ros_pb2.ServerToRosMappingRequest.START_MAPPING:
                map_model.Map.objects(robot_name = request.robot_name).delete()
            communication_queue.put(ros_pb2.ServerToRosCommunication(
                mapping_request=request.mapping_request
            ))
            return ros_pb2.MappingResponse()

    def GetMapImage(self, request, context):
        try:
            map = map_model.Map.objects.get(robot_name = request.robot_name)
            return ros_pb2.MapImage(resolution=map.resolution, encoded_image=map.b64_image)
        except mongoengine.DoesNotExist:
            raise NoMapFound()
    
    def GetRawMap(self, request, context):
        try:
            map = map_model.Map.objects.get(robot_name = request.robot_name)
            return ros_pb2.RawMap(resolution=map.resolution, 
                                  height=map.height,
                                  width=map.width,
                                  data=map.raw_map.encode('utf-8'))
        except mongoengine.DoesNotExist:
            raise NoMapFound()