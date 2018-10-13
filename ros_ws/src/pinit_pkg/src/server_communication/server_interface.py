from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
import grpc

def SendData():
    # generate some data
    yield ros_pb2.RosToServerCommunication()


def main():
    with grpc.insecure_channel('10.40.37.234:50051') as channel:
        stub = ros_pb2_grpc.RosServiceStub(channel)
        for communication in stub.Communicate(SendData()):
            print(communication)
            

if __name__ == '__main__':
    main()