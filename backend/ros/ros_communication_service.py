import time
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

class RosService(ros_pb2_grpc.RosServiceServicer):
    def __init__(self):
        pass

    def Communicate(self, request_iterator, context):
        yield ros_pb2.ServerToRosCommunication(
            mapping_request=ros_pb2.MappingRequest(
                request_type=ros_pb2.MappingRequest.START_MAPPING
            )
        )
        time.sleep(1)
        yield ros_pb2.ServerToRosCommunication(
            mapping_request=ros_pb2.MappingRequest(
                request_type = ros_pb2.MappingRequest.DIRECTION,
                direction = ros_pb2.MappingRequest.FORWARD
            )
        )
        time.sleep(10)
        yield ros_pb2.ServerToRosCommunication(
            mapping_request=ros_pb2.MappingRequest(
                request_type=ros_pb2.MappingRequest.DIRECTION,
                direction = ros_pb2.MappingRequest.RIGHT
            )
        )
        time.sleep(10)
        yield ros_pb2.ServerToRosCommunication(
            mapping_request=ros_pb2.MappingRequest(
                request_type=ros_pb2.MappingRequest.STOP_MAPPING,
            )
        )