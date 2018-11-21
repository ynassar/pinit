#!/usr/bin/env python

from Queue import Queue
from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
from gps.gps_utils import GpsPoint

from robot_navigation.navigation_controller import NavigationController, MapPoint

import roslaunch
import rospy
import rospkg

class ServerNavigationHandler():
    
    def __init__(self, robot_manager):
        self.robot_manager = robot_manager
        self.nav_controller = NavigationController()

    def handle_request(self, request):
        
        request_type = None
        if request.HasField("pose"):
            pose = request.pose
            dest = MapPoint(pose.row, pose.column)
            request_type = 'pose'
        elif request.HasField("coordinates"):
            coordinates = request.coordinates
            latitude = coordinates.lat
            longitude =  coordinates.long
            dest = GpsPoint(long=longitude, lat=latitude)
            request_type = 'gps'

        self.robot_manager.go_to(self.robot_manager.States.NAVIGATING, request_type, dest)        
        
