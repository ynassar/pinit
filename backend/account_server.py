import grpc
import mongoengine
import time

from absl import flags
from absl import app
from backend import constants
from backend.account_management import account_management_service
from backend.authentication import authentication_service
from concurrent import futures
from proto.account_management import account_management_pb2_grpc
from proto.login import login_pb2_grpc


flags.DEFINE_integer("port", 50051, "The port on which to run the server.")
flags.DEFINE_string("database", "local", "The name of the mongodb databse to connect to.")
flags.DEFINE_string("rsa_key", constants.DEFAULT_RSA_KEY, "The RSA key for signing JWT tokens.")

FLAGS = flags.FLAGS

_ONE_DAY_IN_SECONDS = 60*60*24

def main(argv):
    mongoengine.connect(FLAGS.database)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    account_management_pb2_grpc.add_AccountManagementServiceServicer_to_server(
        account_management_service.AccountManagementService(), server)
    login_pb2_grpc.add_AuthenticationServiceServicer_to_server(
        authentication_service.AuthenticationService(FLAGS.rsa_key), server
    )
    server.add_insecure_port(f'[::]:{FLAGS.port}')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == '__main__':
    app.run(main)