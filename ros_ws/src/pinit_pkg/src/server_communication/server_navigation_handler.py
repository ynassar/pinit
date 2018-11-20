#!/usr/bin/env python

from Queue import Queue
from enum import Enum
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2
from gps.gps_utils import GpsPoint

from robot_navigation import NavigationController

import roslaunch
import rospy
import rospkg

class ServerNavigationHandler():
    
    def __init__(self, robot_manager, MapPublisher):
        self.robot_manager = robot_manager
        self.nav_controller = NavigationController()
        self.map_info = MapPublisher

    def handle_request(self, request):
        
        if request.HasField("pose"):
            pose = request.pose
            self.nav_controller.go_to_map_goal(pose.row, pose.column)
        elif request.HasField("coordinates"):
            coordinates = request.coordinates
            latitude = coordinates.lat
            longitutde =  coordinates.long
            
            dest = GpsPoint(long=longitude, lat=latitude)
            origin = self.map_info.get_origin()
            self.nav_controller.got_to_gps(origin, dest)
        
        
