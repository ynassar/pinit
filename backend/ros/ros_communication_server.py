"""A server application for RosCommunicationService."""
import time

from concurrent import futures

import grpc
import mongoengine

from absl import flags
from absl import app
from backend.ros import ros_communication_service
from proto.ros import ros_pb2_grpc

flags.DEFINE_integer("port", 50052, "The port on which to run the server.")
flags.DEFINE_boolean("ignore_unhandled_communication_types", False,
                     "Whether to ignore communication types from the robot for which there is no handler. "
                     "The alternative is throwing an exception.")

FLAGS = flags.FLAGS

_ONE_DAY_IN_SECONDS = 60*60*24

def main(_):
    """Runs the server."""
    mongoengine.connect('local_ros')
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    ros_pb2_grpc.add_RosServiceServicer_to_server(
        ros_communication_service.BuildRosService(FLAGS.ignore_unhandled_communication_types),
        server)
    server.add_insecure_port(f'[::]:{FLAGS.port}')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == '__main__':
    app.run(main)
