from pinit.proto import login_pb2_grpc

class LoginServer(login_pb2_grpc.AuthenticationServiceServicer):
    """TODO: Add docstring."""

    def Login(self, login_request, context):
        return login_pb2.LoginResponse(token="hello!")