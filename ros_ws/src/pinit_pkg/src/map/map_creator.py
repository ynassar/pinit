#!/usr/bin/env python

import roslaunch
import rospkg
import rospy

class MapCreator():
    """Creates a local Map via ros gmapping"""

    def __init__(self, launch_file_path):
        self.launch_file_path = launch_file_path
        self.ros_launch = None


    def start(self):
        """Start map creation

        Args:
            None

        Returns:
            None

        """
        #TODO transition to a new robot state
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.ros_launch = roslaunch.parent.ROSLaunchParent(uuid,\
                                                       [self.launch_file_path])

        self.ros_launch.start()
        rospy.loginfo("Starting gmapping...")


    def finish(self):
        #TODO transition to a new robot state
        self.ros_launch.shutdown()
        rospy.loginfo("Stoping gmapping...")



def MapCreatorFactory():
    launch_file_path = rospkg.RosPack().get_path('pinit_pkg') + \
                                                "/launch/gmapping_launch.launch"
    return MapCreator(launch_file_path)
