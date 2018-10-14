import grpc

from absl import app
from absl import flags

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

flags.DEFINE_integer("port", 50051, "The port to connect to on localhost.")
FLAGS = flags.FLAGS

def GenerateRequest():
    yield ros_pb2.RosToServerCommunication(robot_name='client_test')

def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        for communication in stub.Communicate(GenerateRequest()):
            print(communication)
        

if __name__ == '__main__':
    app.run(main)