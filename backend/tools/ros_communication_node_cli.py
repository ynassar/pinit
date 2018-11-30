import grpc

import numpy as np
import time
import queue

from absl import app
from absl import flags

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

flags.DEFINE_integer("port", 50052, "The port to connect to on localhost.")
FLAGS = flags.FLAGS

status = 'Idle'
state_update_requests = queue.Queue()

def GenerateRequest():
    yield ros_pb2.RosToServerCommunication(robot_name='client_test')
    yield ros_pb2.RosToServerCommunication(raw_map=ros_pb2.RawMap(
        resolution=1,
        height=5,
        width=5,
        data=np.array([0,0,0,0,0,
                       0,101,101,101,0,
                       0,101,0,101,0,
                       0,101,101,101,0,
                       0,0,0,0,0], dtype='uint8').tobytes(),
        origin=ros_pb2.GpsCoordinates(longitude=10, latitude=20),
        shift_x = 10.0,
        shift_y = 20.0
    ))
    time.sleep(1)
    yield ros_pb2.RosToServerCommunication(
        robot_pose = ros_pb2.LocalMapPose(
            row = 0,
            column = 4,
            angle = 45
        )
    )
    for update_request in iter(state_update_requests.get, None):
        yield update_request
    

def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        communication_generator = stub.Communicate(GenerateRequest())
        try:
            for communication in communication_generator:
                print(communication)
                if communication.HasField("navigation_request"):
                    state_update_requests.put(
                        ros_pb2.RosToServerCommunication(
                            status_update = ros_pb2.RobotStatusUpdate(
                                status_update=ros_pb2.RobotStatusUpdate.NAVIGATING
                            )
                        )
                    )
                    state_update_requests.put(
                        ros_pb2.RosToServerCommunication(
                            status_update = ros_pb2.RobotStatusUpdate(
                                status_update=ros_pb2.RobotStatusUpdate.NAVIGATING_AND_IDLE
                            )
                        )
                    )
        except KeyError:
            communication_generator.cancel()
        

if __name__ == '__main__':
    app.run(main)