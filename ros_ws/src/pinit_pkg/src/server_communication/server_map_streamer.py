import grpc
import sys
import numpy as np

from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid


class ServerMapStreamer():

    def __init__(self, queue):
        self.communication_queue = queue
        self.map_topic_name = "map"
        self.map_grpc = None

        rospy.Subscriber(self.map_topic_name,
                         OccupancyGrid,
                         self.map_callback)


    def encode(self,list_of_ints):
        return (np.array(list_of_ints) + 1).astype('uint8').tobytes()


    def map_callback(self, occupancy_grid):
        metadata = occupancy_grid.info
        map_raw_data = occupancy_grid.data
        print metadata
        map_raw_data_encoded = self.encode(map_raw_data)

        self.map_grpc = ros_pb2.RosToServerCommunication(
            raw_map=ros_pb2.RawMap(
                resolution=metadata.resolution,
                height=metadata.height,
                width=metadata.width,
                data=map_raw_data_encoded))

        self.communication_queue.put(self.map_grpc)
