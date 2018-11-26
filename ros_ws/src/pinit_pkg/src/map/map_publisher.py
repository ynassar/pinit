#!/usr/bin/env python

import threading

import numpy as np

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import rospy
import ros_numpy
import nav_msgs.msg as ros_nav_msgs
import nav_msgs.srv as ros_nav_srv


class MapPublisher():
    #TODO test this

    @classmethod
    def create(cls, server_address, robot_name):
        map_topic_name = "map"
        map_metadata_topic_name = "map_metadata"
        map_publisher = rospy.Publisher(map_topic_name, ros_nav_msgs.OccupancyGrid, queue_size=10)
        map_metadata_publisher = rospy.Publisher(map_metadata_topic_name, ros_nav_msgs.MapMetaData, queue_size=10)
        #TODO we might need to publish map_metadata
        return MapPublisher(server_address, robot_name, map_publisher, map_metadata_publisher)


    def __init__(self, server_address, robot_name, map_publisher, metadata_publisher):
        self.server_address = server_address
        self.robot_name = robot_name
        self.map_publisher = map_publisher
        self.metadata_publisher = metadata_publisher
        self.stop_lock = threading.Lock()
        self.stop_flag = False
        self.ros_map = None
        self.gps_origin = None
        self.delta_theta = None
        self.srv = None


    def start(self):
#        self.publish_loop()
        self.srv = rospy.Service("static_map", ros_nav_srv.GetMap, self.get_map)
#        self.set_stop_flag(False)
        thread = threading.Thread(target=self.publish_loop)
        thread.start()


    def get_map(self, req):
        rospy.loginfo("Sending map :)")
        return self.ros_map


    def stop(self):
        self.set_stop_flag(True)


    def get_origin(self):
        return self.gps_origin


    def get_theta(self):
        return self.delta_theta


    def fetch_remote_map(self):
        with grpc.insecure_channel(self.server_address) as channel:
            stub = ros_pb2_grpc.RosServiceStub(channel)
            map_request = ros_pb2.GetMapRequest(
                robot_name=self.robot_name)
            remote_map = stub.GetRawMap(map_request)
            self.ros_map = self.grpc_map_to_rosmap(remote_map)
            self.delta_theta = remote_map.origin_angle_shift
            self.gps_origin = remote_map.origin


    def publish_loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.get_stop_flag():
            self.ros_map.header.stamp = rospy.Time.now()
            self.ros_map.info.map_load_time = rospy.Time.now()
            self.ros_map.header.frame_id = "map"    #TODO check if this is correct
            self.map_publisher.publish(self.ros_map)
            self.metadata_publisher.publish(self.ros_map.info)
            rate.sleep()


    def grpc_map_to_rosmap(self, grpc_map):
        resolution = grpc_map.resolution
        time_now = rospy.Time.now()
        width = grpc_map.width
        height = grpc_map.height
        map_encoded = grpc_map.data
        map_decoded = self.decode(map_encoded)

        ros_map = ros_nav_msgs.OccupancyGrid()
        ros_map.info.resolution = resolution
        ros_map.info.map_load_time = time_now
        ros_map.info.width = width
        ros_map.info.height = height
        ros_map.info.origin.position.x = -1 * (width / 2) * resolution
        ros_map.info.origin.position.y = -1 * (height / 2) * resolution
        ros_map.info.origin.position.z = 0
        ros_map.info.origin.orientation.x = 0.
        ros_map.info.origin.orientation.y = 0.
        ros_map.info.origin.orientation.z = 0.
        ros_map.info.origin.orientation.w = 1.
        ros_map.header.stamp = rospy.Time.now()
        ros_map.info.map_load_time = rospy.Time.now()
        ros_map.header.frame_id = "map"    #TODO check if this is correct
        ros_map.data = map_decoded

        return ros_map


    def decode(self, map_encoded):
        ros_data = (np.frombuffer(map_encoded, dtype='int8')-1).tolist()
        return ros_data


    def set_stop_flag(self, value):
        self.stop_lock.acquire()
        self.stop_flag = value
        self.stop_lock.release()


    def get_stop_flag(self):
        self.stop_lock.acquire()
        flag = self.stop_flag
        self.stop_lock.release()

        return flag
