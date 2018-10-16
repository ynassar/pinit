import grpc

from absl import app
from absl import flags

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

flags.DEFINE_integer("port", 50051, "The port to connect to on localhost.")
FLAGS = flags.FLAGS

def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        stub.SendMovement(
            ros_pb2.MappingRequest(
                robot_name='client_test',
                mapping_request=ros_pb2.ServerToRosMappingRequest(
                    request_type=ros_pb2.ServerToRosMappingRequest.START_MAPPING
                )))        

if __name__ == '__main__':
    app.run(main)