#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Int32

import grpc
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
import tf


class InitialPosePublisher(object):
    def __init__(self, server_address, robot_name):
        self.publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.header.stamp = rospy.Time.now()
        self.server_address = server_address
        self.robot_name = robot_name


    def publish_initial_pose(self):
        # waits for amcl to run
        rospy.wait_for_service('set_map')
        rospy.loginfo("publishing initial_pose")
        rospy.sleep(5)
        self.publisher.publish(self.initial_pose)

    def fetch_pose(self):
        with grpc.insecure_channel(self.server_address) as channel:
            stub = ros_pb2_grpc.RosServiceStub(channel)
            pose_request = ros_pb2.GetPoseRequest(robot_name=self.robot_name)

            remote_pose = stub.GetPose(pose_request)

            initial_pose_row = remote_pose.row
            initial_pose_col = remote_pose.column
            initial_pose_angle = remote_pose.angle

            point = Point(initial_pose_row, initial_pose_col, 0)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, initial_pose_angle)
            orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            self.initial_pose.pose.pose.position = point
            self.initial_pose.pose.pose.orientation = orientation




if __name__ == '__main__':
    initial_pose_publisher = InitialPosePublisher()
    
    point = Point(5.107, 4244, 0.000)
    orientation = Quaternion(0.000, 0.000, 0.0936, -0.0353)

    initial_pose_publisher.send_initial_pose(point, quaternion)
