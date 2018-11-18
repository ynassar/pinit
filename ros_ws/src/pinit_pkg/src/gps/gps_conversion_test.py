#!/usr/bin/env python
import math
import rospy
from gps.gps_controller import GpsController
import gps.gps_utils as gps_utils

class GPSTest():

    def __init__(self):
        self.controller = GpsController()


    def test_loop(self):
        gps_origin = self.controller.get_coordinates()
        while gps_origin is not None and
            not math.isnan(gps_origin.lat):
            gps_origin = self.controller.get_coordinates()

        rate = rospy.rate(10)
        while not rospy.is_shutdown():
            current_coordinates = self.controller.get_coordinates()
            distance, angle = gps_utils.get_vector(gps_origin, current_coordinates)
            print "distance ", distance, " angle ", angle


if __name__=="__main__":
    rospy.init_node("gps/converstion_test")
    test = GPSTest()
    test.test_loop()

