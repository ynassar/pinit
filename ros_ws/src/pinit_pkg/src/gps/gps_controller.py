#!/usr/bin/env python

import rospy
import rospkg
import math
from sensor_msgs.msg import NavSatFix
import roslaunch

class GPS_Controller():
    
    def __init__(self):
        self.node_name = "gps_controller"
        self.launch_file_path = rospkg.RosPack().get_path('pinit') + \
            "/launch/gps_launch.launch"

        self.longitude = None
        self.latitude = None
        

    def init_node(self):
        rospy.init_node(self.node_name, anonymous=False)

    def callback(self, coordinates):       
        self.longitude = coordinates.longitude
        self.latitude = coordinates.latitude


    def get_coordinates(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid,\
            [self.launch_file_path])
        self.launch.start()


        sub = rospy.Subscriber("fix", NavSatFix, self.callback)

        while(self.longitude == None or self.latitude == None \
            or math.isnan(self.longitude)):
            # rospy.loginfo("Waiting for coordinates")
            pass

        longitude = self.longitude
        latitude = self.latitude
        print self.latitude
        sub.unregister()
        self.launch.shutdown()
        self.longitude = None
        self.latitude = None
        return longitude, latitude

if __name__ == '__main__':
    try:
        controller = GPS_Controller()
        controller.init_node()
        coordinates = controller.get_coordinates()
        print coordinates
    except rospy.ROSInterruptException as e:
        pass