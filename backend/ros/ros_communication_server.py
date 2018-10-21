import grpc
import mongoengine
import time

from absl import flags
from absl import app
from backend.ros import ros_communication_service
from concurrent import futures
from proto.ros import ros_pb2_grpc

flags.DEFINE_integer("port", 50052, "The port on which to run the server.")

FLAGS = flags.FLAGS

_ONE_DAY_IN_SECONDS = 60*60*24

def main(argv):
    mongoengine.connect('local_ros')
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    ros_pb2_grpc.add_RosServiceServicer_to_server(
        ros_communication_service.RosService(), server)
    server.add_insecure_port(f'[::]:{FLAGS.port}')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == '__main__':
    app.run(main)