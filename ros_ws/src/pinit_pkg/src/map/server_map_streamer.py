import numpy as np

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid

from gps.gps_controller import GpsController


class ServerMapStreamer():
    """Fetch the map and send it to the server"""

    def __init__(self, queue):
        self.communication_queue = queue
        self.map_topic_name = "map"
        self.map_grpc = None
        self.subscriber = None
        self.global_origin = None


    def start_stream(self):
        """Start listenning and streaming the mapping and gets global reference

        Args:
            None

        Returns:
            None
        """

        self.fetch_global_origin() #This should block until we receive gps coordinates
        self.subscriber = rospy.Subscriber(self.map_topic_name,
                                           OccupancyGrid,
                                           self.map_callback)


    def fetch_global_origin(self):
        """Fetch the map global origin

        Args:
            None

        Returns:
            None
        """

        self.global_origin = GpsController().get_coordinates()


    def stop_stream(self):
        """Stops listenning and streaming the map

        Args:
            None

        Returns:
            None
        """

        self.subscriber.unregister()


    def map_callback(self, occupancy_grid):
        """Queue the map in the server main queue


        Args:
            None

        Returns:
            None
        """

        metadata = occupancy_grid.info
        map_raw_data = occupancy_grid.data
        map_raw_data_encoded = self.encode(map_raw_data)

        gps_coordinates_msg = ros_pb2.GpsCoordinates(
            longitude=self.global_origin.long,
            latitude=self.global_origin.lat)

        self.map_grpc = ros_pb2.RosToServerCommunication(
            raw_map=ros_pb2.RawMap(
                resolution=metadata.resolution,
                height=metadata.height,
                width=metadata.width,
                data=map_raw_data_encoded,
                coordinates=gps_coordinates_msg))

        self.communication_queue.put(self.map_grpc)
        rospy.loginfo("Sending map to server...")


    def encode(self, list_of_ints):
        """Encode an int list

        Args:
            list_of_ints: An int list

        Returns:
            A uint8 numpy list
        """

        return (np.array(list_of_ints) + 1).astype('uint8').tobytes()

