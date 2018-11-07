import bcrypt
import grpc
import mongoengine

from proto.account_management import account_management_pb2
from proto.account_management import account_management_pb2_grpc

from backend.models import user

class UsernameOrEmailAlreadyExists(Exception):
    pass

class InvalidRegistrationParameters(Exception):
    pass

class AccountManagementService(account_management_pb2_grpc.AccountManagementServiceServicer):

    def __init__(self):
        pass

    def Register(self, request, context):
        """Implements the Register RPC, that saves a user into the database.
        
        Args:
            request: An account_management_pb2.RegisterRequest instance.
            context: A grpc context.
        Returns:
            An account_management_pb2.RegisterResponse instance.
        """
        email = request.email
        username = request.username
        password_hash = bcrypt.hashpw(request.password.encode('utf-8'), bcrypt.gensalt())

        new_user = user.User(email=email, 
                             username=username,
                             password_hash=password_hash,
                             is_owner=False)
        try:
            new_user.save()
        except mongoengine.NotUniqueError:
            context.set_code(grpc.StatusCode.ALREADY_EXISTS)
            context.set_details("Username or Email already exists.")
            raise UsernameOrEmailAlreadyExists()
        except mongoengine.ValidationError as validation_error:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(validation_error.message)
            raise InvalidRegistrationParameters()
        return account_management_pb2.RegisterResponse()