import grpc

import numpy as np
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
        data=np.array([0,0,0,0,0,
                       0,101,101,101,0,
                       0,101,0,101,0,
                       0,101,101,101,0,
                       0,0,0,0,0], dtype='uint8').tobytes()
    ))


def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        for communication in stub.Communicate(GenerateRequest()):
            print(communication)
        

if __name__ == '__main__':
    app.run(main)