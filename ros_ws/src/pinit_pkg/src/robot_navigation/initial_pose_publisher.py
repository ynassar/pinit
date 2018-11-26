#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Int32


class InitialPosePublisher(object):
    def __init__(self):
        publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)

        # rate = rospy.Rate(10)

        # while not rospy.is_shutdown():

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()

        #Position(5.107, 4.244, 0.000), Orientation(0.000, 0.000, 0.936, -0.353)
        point = Point(5.107, 4244, 0.000)
        orientation = Quaternion(0.000, 0.000, 0.0936, -0.0353)



        initial_pose.pose.pose.position = point
        initial_pose.pose.pose.orientation = orientation

        rospy.sleep(15)
        print "published the pose"
        publisher.publish(initial_pose)


if __name__ == '__main__':
    initial_pose_publisher = InitialPosePublisher()