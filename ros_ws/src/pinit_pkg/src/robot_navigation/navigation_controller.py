#!/usr/bin/env python

import rospy
from initial_pose_publisher import InitialPosePublisher
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from gps import gps_utils

class MapPoint():

    def __init__(self, x, y):
        self.x = x
        self.y = y


class NavigationController():

    def __init__(self, MapPub):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #TODO run the client in a separate thread
        #TODO this blocks untill movebase (which contains a ros action server) starts so it should be in a separate thread
        #self.client.wait_for_server()
        # Position(3.500, -0.344, 0.000), Orientation(0.000, 0.000, -0.827, 0.562) 
        self.goal = Pose(Point(3.500, -0.344, 0.000), Quaternion(0.000, 0.000, -0.827, 0.562))
        self.map_info = MapPub

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
    
    def got_to_gps(self, dest):
        origin = self.map_info.get_origin()
        distance, theta = gps_utils.get_vector(origin, dest)
        dest_x, dest_y = gps_utils.convert_gps(distance, theta)
        self.goal = Pose(Point(dest_x, dest_y, 0.000), Quaternion(0, 0, 0, 0))
        self.send_goal() 

    def go_to_map_goal(self, dest):
        self.goal = Pose(Point(dest.x, dest.y, 0.000), Quaternion(0, 0, 0, 0))
        self.send_goal()
    
    def start_nav(self, dest_type, dest):
        print dest_type, dest
#        if dest_type == 'pose':
#            self.go_to_map_goal(dest)
#        elif dest_type == 'gps':
#            self.got_to_gps(dest)
            




    


if __name__ == '__main__':
    nav = NavigationController()
    nav.send_goal()
