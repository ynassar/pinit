#!/usr/bin/env python

import tf
import rospy
import threading

from geometry_msgs.msg import PoseStamped

class PoseListener():
    """Calculates the robot pose"""


    def __init__(self, tf_listener, source_tf_name, target_tf_name):
        self.tf_listener = tf_listener
        self.source_tf_name = source_tf_name
        self.target_tf_name = target_tf_name
        self.stream_thread = None


    def get_pose(self):
        ros_pose = None
        try:
            (translation, rotation) = self.tf_listener.lookupTransform(
                self.source_tf_name,
                self.target_tf_name,
                rospy.Time(0))
            ros_pose = self.fill_ros_pose(translation, rotation)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as E:
            rospy.logerr("Error while getting calculating pose: ")

        return ros_pose


    def fill_ros_pose(self, translation, rotation):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.source_tf_name
        return pose_msg


    def run(self):
        rospy.init_node('pose_publisher')
        listener = tf.TransformListener()
        pub = rospy.Publisher('Pose', PoseStamped, queue_size=10);
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                (translation, rotation) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
                print "trans: ", translation
                print "rot: ", rotation
                pose = PoseStamped()
                pose.pose.position.x = translation[0]
                pose.pose.position.y = translation[1]
                pose.pose.position.z = translation[2]
                pose.pose.orientation.x = rotation[0]
                pose.pose.orientation.y = rotation[1]
                pose.pose.orientation.z = rotation[2]
                pose.pose.orientation.w = rotation[3]
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'map'
                pub.publish(pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()



def PoseListenerFactory():
    tf_listener = tf.TransformListener()
    source_tf = 'map'
    target_tf = 'base_link'
    return PoseListener(tf_listener, source_tf, target_tf)



if  __name__ == '__main__':
    rospy.init_node('PoseListener_test_node')
    pose_listener = PoseListenerFactory()
    publisher = rospy.Publisher("pose_test", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose = pose_listener.get_pose()
        if pose is not None:
            publisher.publish(pose)
        rate.sleep()
