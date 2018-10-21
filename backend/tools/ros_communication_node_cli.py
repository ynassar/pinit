import grpc

import time

from absl import app
from absl import flags

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

flags.DEFINE_integer("port", 50052, "The port to connect to on localhost.")
FLAGS = flags.FLAGS

def GenerateRequest():
    yield ros_pb2.RosToServerCommunication(robot_name='client_test')
    time.sleep(1)
    yield ros_pb2.RosToServerCommunication(raw_map=ros_pb2.RawMap(
        resolution=1,
        height=5,
        width=5,
        data=[0,0,0,0,0,
              0,1,1,1,0,
              0,1,0,1,0,
              0,1,1,1,0,
              0,0,0,0,0]
    ))


def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        for communication in stub.Communicate(GenerateRequest()):
            print(communication)
        

if __name__ == '__main__':
    app.run(main)