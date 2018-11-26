#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Int32


class InitialPosePublisher(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped) 
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.header.stamp = rospy.Time.now()
        rospy.sleep(15)

    def send_initial_pose(self, point, quaternion):        
        self.initial_pose.pose.pose.position = point
        self.initial_pose.pose.pose.orientation = orientation
        self.publisher.publish(self.initial_pose)


if __name__ == '__main__':
    initial_pose_publisher = InitialPosePublisher()
    
    point = Point(5.107, 4244, 0.000)
    orientation = Quaternion(0.000, 0.000, 0.0936, -0.0353)

    initial_pose_publisher.send_initial_pose(point, quaternion)
