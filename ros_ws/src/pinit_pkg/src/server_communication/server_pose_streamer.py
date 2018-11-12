#!/usr/bin/env python

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
from google.protobuf.timestamp_pb2 import Timestamp

import tf
import rospy
import threading
import time

from geometry_msgs.msg import PoseStamped
from robot_motion.robot_pose import PoseListenerFactory

class ServerPoseStreamer():
    """Streames the robot pose to the server"""


    def __init__(self, queue, robot_pose_listener):
        self.communication_queue = queue
        self.pose_listener = robot_pose_listener

        self.stream_thread = None
        self.init_stream_loop()


    def init_stream_loop(self):
        self.stream_thread = threading.Thread(target=self.stream_loop)
        self.stream_thread.start()


    def stream_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ros_pose = self.pose_listener.get_pose()
            if ros_pose is not None:
                grpc_pose = self.ros_to_grpc_pose(ros_pose)
                self.communication_queue.put(grpc_pose)
                rospy.loginfo("Sending pose to server...")
            rate.sleep()


    def ros_to_grpc_pose(self, pose):
        x = int(pose.pose.position.x)
        y = int(pose.pose.position.y)
        angle = 0               #TODO do we really need the angle?
        current_time = self.get_time_now()

        grpc_pose = ros_pb2.RosToServerCommunication(
            robot_pose=ros_pb2.LocalMapPose(
                row=x,
                column=y,
                angle=angle,
                timestamp=current_time))


    def get_time_now(self):
        current_time = Timestamp()
        current_time = current_time.GetCurrentTime() #TODO is this local time??
        return current_time




def ServerPoseStreamerFactory(queue):
    pose_listener = PoseListenerFactory()
    return ServerPoseStreamer(queue, pose_listener)

