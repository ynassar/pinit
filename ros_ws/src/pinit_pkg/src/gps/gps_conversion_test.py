#!/usr/bin/env python
import math
import rospy
from gps.gps_controller import GpsController
import gps.gps_utils as gps_utils

from geometry_msgs.msg import PoseStamped, Point, Quaternion

class GPSTest():

    def __init__(self):
        self.controller = GpsController()
        self.pose_pub = rospy.Publisher("pose_pub", PoseStamped, queue_size=10)


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
            map_x, map_y = gps_utils.convert_gps(distance, angle, 1) 
            print "distance ", distance, " angle ", angle, "Map x: ", map_x, "Map y : ", map_y

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()

            pose.pose.position =  Point(-1 * map_y, -1 * map_x, 0)
            pose.pose.orientation = Quaternion(0,0,0,0)

            self.pose_pub.publish(pose)
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("gps_converstion_test")
    test = GPSTest()
    test.test_loop()

