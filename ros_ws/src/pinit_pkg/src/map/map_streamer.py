import numpy as np

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid

from gps.gps_controller import GpsController


class MapStreamer():
    """Fetch the map and send it to the server"""

    def __init__(self, queue, gps_callibrator):
        self.communication_queue = queue
        self.map_topic_name = "map"
        self.subscriber = None
        self.global_origin = None
        self.gps_callibrator = gps_callibrator
 

    def start(self):
        """Start listenning and streaming the mapping and gets global reference

        Args:
            None

        Returns:
            None
        """

        #self.fetch_global_origin() #This should block until we receive gps coordinates
        self.subscriber = rospy.Subscriber(self.map_topic_name,
                                           OccupancyGrid,
                                           self.map_callback)
        rospy.loginfo("Started streaming map to server...")

        self.gps_callibrator.start_cal()


    def fetch_global_origin(self):
        """Fetch the map global origin

        Args:
            None

        Returns:
            None
        """

        self.global_origin = GpsController().get_coordinates()


    def finish(self):
        """Stops listenning and streaming the map

        Args:
            None

        Returns:
            None
        """

        self.subscriber.unregister()
        rospy.loginfo("Stopped streaming map to server...")


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

        gps_origin = self.gps_callibrator.get_origin()
        gps_coordinates_msg = ros_pb2.GpsCoordinates(
            longitude=0,
            latitude=0)
        if gps_origin is not None:
            gps_coordinates_msg.longitude = gps_origin.long
            gps_coordinates_msg.latitude = gps_origin.lat

        delta = 0

        grpc_raw_map = ros_pb2.RosToServerCommunication(
            raw_map=ros_pb2.RawMap(
                resolution=metadata.resolution,
                height=metadata.height,
                width=metadata.width,
                data=map_raw_data_encoded,
                origin_angle_shift=delta,
                origin=gps_coordinates_msg))

        self.communication_queue.put(grpc_raw_map)
        rospy.loginfo("Sending map to server...")


    def encode(self, list_of_ints):
        """Encode an int list

        Args:
            list_of_ints: An int list

        Returns:
            A uint8 numpy list
        """

        return (np.array(list_of_ints) + 1).astype('uint8').tobytes()

