#!/usr/bin/env python

import threading

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

import rospy
import nav_msgs.msg as ros_nav_msgs


class   ():

    @classmethod
    def create(cls, server_address, robot_name):
        topic_name = "map"
        publish_topic = rospy.Publisher(topic_name, ros_nav_msgs.OccupancyGrid, queue_size=10)
        return MapPublisher(server_address, robot_name, publish_topic)


    def __init__(self, server_address, robot_name, publisher):
        self.server_address = server_address
        self.robot_name = robot_name
        self.publisher = publisher
        self.stop_lock = threading.Lock()
        self.stop_flag = False
        self.ros_map = None
        self.gps_origin = None
        self.delta_theta = None


    def start(self):
        self.set_stop_flag(False)
        thread = threading.Thread(target=self.publish_loop)
        thread.start()


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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.get_stop_flag():
            self.ros_map.header.stamp = rospy.Time.now()
            self.ros_map.header.frame_id = "map"    #TODO check if this is correct
            self.publisher.publish(self.ros_map)

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
        ros_map.info.origin.position.x = 0 #TODO we will probably need to set this with some value
        ros_map.info.origin.position.y = 0 #TODO we will probably need to set this with some value
        ros_map.data = map_decoded

        return ros_map


    def decode(self, map_encoded):
        #TODO decode the message here
        pass


    def set_stop_flag(self, value):
        self.stop_lock.acquire()
        self.stop_flag = value
        self.stop_lock.release()


    def get_stop_flag(self):
        self.stop_lock.acquire()
        flag = self.stop_flag
        self.stop_lock.release()

        return flag
