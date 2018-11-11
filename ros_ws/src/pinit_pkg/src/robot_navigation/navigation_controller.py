#!/usr/bin/env python

import rospy
from initial_pose_publisher import InitialPosePublisher
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion

class NavigationController():

    def __init__(self):
        rospy.init_node('navigation_controller_node')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = Pose(Point(101.665, 100.262, 0.000), Quaternion(0.000, 0.000, -0.014, 1.000))

    def active_cb(self):
        rospy.loginfo("active cb")

    def feedback_cb(self):
        rospy.loginfo("feedback cb")

    def done_cb(self):
         rospy.loginfo("done cb")
    
    def send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.goal
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
    
if __name__ == '__main__':
    nav = NavigationController()
    nav.send_goal()
