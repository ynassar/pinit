import grpc
import time

from absl import app
from absl import flags

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
from proto.login import login_pb2_grpc
from proto.login import login_pb2


flags.DEFINE_integer("port", 50052, "The port to connect to on localhost.")
FLAGS = flags.FLAGS

def main(argv):
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        with grpc.insecure_channel(f'localhost:50051') as auth_channel:
            auth_stub = login_pb2_grpc.AuthenticationServiceStub(auth_channel)
            response = auth_stub.Login(login_pb2.LoginRequest(username="Admin", password="123"))
            token = response.token
        print(token)
        stub = ros_pb2_grpc.RosServiceStub(channel)
        waypoint = stub.RequestRobotToLocation(
                ros_pb2.RobotNavigationRequest(
                    token=token,
                    coordinates=ros_pb2.GpsCoordinates(
                        latitude=10,
                        longitude=20)))

        print(waypoint)

    #rpc RequestRobotToLocation(RobotNavigationRequest) returns (WaypointList) {}
if __name__ == '__main__':
    app.run(main)
