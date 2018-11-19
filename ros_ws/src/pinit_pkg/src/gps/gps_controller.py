#!/usr/bin/env python

import rospy
import rospkg
import math
from sensor_msgs.msg import NavSatFix
import roslaunch
from gps.gps_utils import GpsPoint


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


    def init_fake_coordinates(self):
        self.latitude = 30.0187716667
        self.longitude = 31.5006533333


    def fake_coordinantes(self):
        self.latitude = self.latitude + 0.000001


    def get_coordinates(self):
        point = None
        
        self.fake_coordinantes()
        longitude = self.longitude
        latitude = self.latitude
        if longitude is not None \
            and latitude is not None:            
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
