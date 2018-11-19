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
        while gps_origin is None:
            gps_origin = self.controller.get_coordinates()
        gps_origin = gps_utils.coordinates_to_rad(gps_origin)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_coordinates = self.controller.get_coordinates()

            current_coordinates = gps_utils.coordinates_to_rad(current_coordinates)
            long = current_coordinates.long
            lat = current_coordinates.lat
            distance, angle = gps_utils.get_vector(gps_origin, current_coordinates)
            print "distance ", distance, " angle ", angle, "long", long, "lat", lat

            rate.sleep()


if __name__=="__main__":
    rospy.init_node("gps_converstion_test")
    test = GPSTest()
    test.test_loop()

