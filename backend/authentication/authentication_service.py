import bcrypt
import grpc
import mongoengine
import jwt

from backend.models import user as user_model

from proto.login import login_pb2
from proto.login import login_pb2_grpc

class NoSuchUser(Exception):
    pass

class WrongPassword(Exception):
    pass

class AuthenticationService(login_pb2_grpc.AuthenticationServiceServicer):
    def __init__(self, rsa_key):
        self._rsa_key = rsa_key.encode('utf-8')

    def Login(self, request, context):
        try:
            user = user_model.User.objects().get(username=request.username)
        except mongoengine.DoesNotExist:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("No user found with the given details.")
            raise NoSuchUser()
        if bcrypt.checkpw(request.password.encode('utf-8'), user.password_hash.encode('utf-8')):
            jwt_data = {
                'username':user.username,
                'is_owner':user.is_owner
            }
            if user.is_owner:
                jwt_data['owned_robot'] = user.owned_robot
            token = jwt.encode(jwt_data, self._rsa_key)
            return login_pb2.LoginResponse(token=token)
        else:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("Invalid password.")
            raise WrongPassword()