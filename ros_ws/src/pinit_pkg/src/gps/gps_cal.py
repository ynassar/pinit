#!/usr/bin/env python
import numpy as np
import math
import rospy
import threading
from gps.gps_controller import GpsController
import gps.gps_utils as gps_utils

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from robot_motion.robot_pose import PoseListenerFactory

class GPSCallibrtor():

    def __init__(self):
        self.controller = GpsController()
        self.thread = threading.Thread(target=self.callibration_loop)
        self.origin_lock = threading.Lock()
        self.gps_origin = None


    # This now just gets the origin we don't need theta anymore
    #TODO handle multiple calls of start_cal()
    def start_cal(self):
        self.thread.start()


    def callibration_loop(self):
        self.origin_lock.acquire()
        self.gps_origin = self.controller.get_coordinates()
        while self.gps_origin is None:
            self.gps_origin = self.controller.get_coordinates()
        self.gps_origin = self.gps_origin
        self.origin_lock.release()

        
    def get_origin(self):
        self.origin_lock.acquire()
        origin = self.gps_origin
        self.origin_lock.release()

        return origin
        
        


if __name__=="__main__":
    rospy.init_node("gps_converstion_test")
    # test = GPSCallibrtor()
    # test.test_loop()

