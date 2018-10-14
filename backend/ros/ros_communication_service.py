import grpc
import time
import queue

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

class RosService(ros_pb2_grpc.RosServiceServicer):
    def __init__(self):
        self._robot_name_to_queue = {}

    def Communicate(self, request_iterator, context):
        first_request = next(request_iterator)
        robot_name = first_request.robot_name
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