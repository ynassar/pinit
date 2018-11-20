#!/usr/bin/env python

import rospy
from initial_pose_publisher import InitialPosePublisher
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from gps import gps_utils

class NavigationController():

    def __init__(self):
        rospy.init_node('navigation_controller_node')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Position(3.500, -0.344, 0.000), Orientation(0.000, 0.000, -0.827, 0.562) 
        self.goal = Pose(Point(3.500, -0.344, 0.000), Quaternion(0.000, 0.000, -0.827, 0.562))

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
    
    def got_to_gps(self, dest, origin):
        distance, theta = get_vector(origin, dest)
        dest_x, dest_y = convert_gps(distance, theta)
        self.goal = Pose(Point(dest_x, dest_y, 0.000), Quaternion(0, 0, 0, 0))
        self.send_goal() 

    def go_to_map_goal(self, dest_x, dest_y):
        self.goal = Pose(Point(dest_x, dest_y, 0.000), Quaternion(0, 0, 0, 0))
        self.send_goal()

    


if __name__ == '__main__':
    nav = NavigationController()
    nav.send_goal()
