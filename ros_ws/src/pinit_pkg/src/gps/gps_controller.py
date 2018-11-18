#!/usr/bin/env python

import rospy
import rospkg
import math
from sensor_msgs.msg import NavSatFix
import roslaunch


class GpsPoint():

    def __init__(self, long, lat):
        self.long = long
        self.lat = lat



class GpsController():
    
    def __init__(self):
        self.node_name = "gps_controller"
        self.subscriber = rospy.Subscriber("fix", NavSatFix, self.callback)
        self.longitude = None
        self.latitude = None
        

    def init_node(self):
        rospy.init_node(self.node_name, anonymous=False)

    def callback(self, coordinates):       
        self.longitude = coordinates.longitude
        self.latitude = coordinates.latitude


    def get_coordinates(self):

        longitude = self.longitude
        latitude = self.latitude

        point = GpsPoint(
            long=longitude, 
            lat=latitude)
        
        return point



if __name__ == '__main__':
    try:
        controller = GpsController()
        controller.init_node()
        coordinates = controller.get_coordinates()
        print coordinates
    except rospy.ROSInterruptException as e:
        pass
