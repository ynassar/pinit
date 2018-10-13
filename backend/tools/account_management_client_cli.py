import grpc

from absl import app

from proto.account_management import account_management_pb2
from proto.account_management import account_management_pb2_grpc

def main(argv):
    username = input("Enter a username to register.\n")
    password = input("Enter a password.\n")
    email = input("Enter an email.\n")
    request = account_management_pb2.RegisterRequest(
        username=username, password=password, email=email)
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = account_management_pb2_grpc.AccountManagementServiceStub(channel)
        print(stub.Register(request))

if __name__ == '__main__':
    app.run(main)