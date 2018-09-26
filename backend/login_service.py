from login_proto import login_pb2_grpc
from login_proto import login_pb2

class LoginService(login_pb2_grpc.AuthenticationServiceServicer):
    """TODO: Add docstring."""

    def Login(self, login_request, context):
        return login_pb2.LoginResponse(token="hello!")