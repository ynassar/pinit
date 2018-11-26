#!/usr/bin/env python

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
from google.protobuf.timestamp_pb2 import Timestamp

import tf
import rospy
import time

from geometry_msgs.msg import PoseStamped
from robot_motion.robot_pose import PoseListenerFactory
from utils import ros_thread

class ServerPoseStreamer():
    """Streames the robot pose to the server"""


    def __init__(self, queue, robot_pose_listener, server_streamer):
        self.communication_queue = queue
        self.pose_listener = robot_pose_listener
        self.streamer = server_streamer
        self.streamer.set_loop_function(self.stream_loop)


    def start(self):
        self.streamer.start()


    def stop(self):
        self.streamer.stop()


    def stream_loop(self):
        ros_pose = self.pose_listener.get_pose()
        if ros_pose is not None:
            grpc_pose = self.ros_to_grpc_pose(ros_pose)
            self.communication_queue.put(grpc_pose)


    def ros_to_grpc_pose(self, pose):
        x = (pose.pose.position.x)
        y = (pose.pose.position.y)
        orientation = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)
        yaw = euler[2]
        angle = yaw
        current_time = self.get_time_now()

        grpc_pose = ros_pb2.RosToServerCommunication(
            robot_pose=ros_pb2.LocalMapPose(
                row=x,
                column=y,
                angle=angle,
                timestamp=current_time))

        return grpc_pose


    def get_fake_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = 2.5
        pose.pose.position.y = 4.7

        return pose

    def get_time_now(self):
        current_time = Timestamp()
        current_time = current_time.GetCurrentTime()
        return current_time




def ServerPoseStreamerFactory(queue, pose_listener):
    streamer = ros_thread.SimpleThread()
    return ServerPoseStreamer(queue, pose_listener, streamer)

