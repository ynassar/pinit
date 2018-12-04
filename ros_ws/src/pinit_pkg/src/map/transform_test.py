#!/usr/bin/env python

import rospy
import tf

def pose_test(tf_broadcast):
    tf_broadcast.sendTransform((5, 0, 0),
                               tf.transformations.quaternion_from_euler(0, 0, 0),
                               rospy.Time.now(),
                               "tf_tests",
                               "base_link")


if __name__ == "__main__":
    rospy.init_node("tf_test")
    tf_broadcast = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.loginfo("publishing transform")
        pose_test(tf_broadcast)
