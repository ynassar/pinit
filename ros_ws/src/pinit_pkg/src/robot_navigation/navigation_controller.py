#!/usr/bin/env python

import rospy
from initial_pose_publisher import InitialPosePublisher
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from gps import gps_utils
import threading

class MapPoint():

    def __init__(self, x, y):
        self.x = x
        self.y = y


class NavigationController():

    def __init__(self, MapPub):

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.state_manager = None
        #TODO run the client in a separate thread
        #TODO this blocks untill movebase (which contains a ros action server) starts so it should be in a separate thread
        
        # Position(3.500, -0.344, 0.000), Orientation(0.000, 0.000, -0.827, 0.562) 
        self.goal = Pose(Point(3.500, -0.344, 0.000), Quaternion(0.000, 0.000, -0.827, 0.562))
        self.map_info = MapPub

    def active_cb(self):
        rospy.loginfo("active cb")

    def feedback_cb(self, arg1):
        #rospy.loginfo("feedback cb")
        pass

    def done_cb(self, arg1, arg2):
        self.state_manager.go_to(self.state_manager.States.NAVIGATING_AND_IDLE)
        rospy.loginfo("done cb")
    
    def send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.goal
        self.client.wait_for_server()
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
    
    def go_to_gps(self, dest):
        origin = self.map_info.get_origin()
        origin = gps_utils.grpc_to_ros(origin)
        print "gps origin degrees", gps_utils.coordinates_to_deg(dest)
        dest = gps_utils.coordinates_to_rad(dest)
        print "gps origin", origin
        distance, theta = gps_utils.get_vector(origin, dest)
        print "gps distance, theta: ", distance, theta
        dest_x, dest_y = gps_utils.convert_gps(distance, theta)
        print "destination: ", dest_x, dest_y
        self.goal = Pose(Point(dest_x, dest_y, 0.000), Quaternion(0, 0, 0, 1))
        self.send_goal() 

    def go_to_map_goal(self, dest):
        self.goal = Pose(Point(dest.x, dest.y, 0.000), Quaternion(0, 0, 0, 1))
        self.send_goal()
    
    def start_nav(self, dest_type, dest):
        # print dest_type, dest
        print dest_type, dest
        # TODO comment below for testing 
        if dest_type == 'pose':
           self.go_to_map_goal(dest)
        elif dest_type == 'gps':
           self.go_to_gps(dest)
            




    


if __name__ == '__main__':
    nav = NavigationController()
    nav.send_goal()
