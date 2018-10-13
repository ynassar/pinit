import grpc

from absl import app
from absl import flags

from proto.account_management import account_management_pb2
from proto.account_management import account_management_pb2_grpc

from proto.login import login_pb2
from proto.login import login_pb2_grpc

flags.DEFINE_enum("method", "Register", ["Login", "Register"], "The RPC method to invoke.")
flags.DEFINE_integer("port", 50051, "The port to connect to on localhost.")

FLAGS = flags.FLAGS

def Register():
    username = input("Enter a username to register.\n")
    password = input("Enter a password.\n")
    email = input("Enter an email.\n")
    request = account_management_pb2.RegisterRequest(
        username=username, password=password, email=email)
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = account_management_pb2_grpc.AccountManagementServiceStub(channel)
        print(stub.Register(request))

def Login():
    username = input("Enter a username to login with.\n")
    password = input("Enter a password to login with.\n")
    request = login_pb2.LoginRequest(username=username, password=password)
    with grpc.insecure_channel(f'localhost:{FLAGS.port}') as channel:
        stub = login_pb2_grpc.AuthenticationServiceStub(channel)
        print(stub.Login(request))

def main(argv):
    if FLAGS.method == "Login":
        Login()
    elif FLAGS.method == "Register":
        Register()

if __name__ == '__main__':
    app.run(main)