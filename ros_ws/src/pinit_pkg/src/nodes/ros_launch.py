#!/usr/bin/env python
import roslaunch

class RosLaunch():

    def __init__(self):
        self.launch_path = None
        self.launch_parent = None


    def set_path(self, path):
        self.launch_path = path
        

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch_parent = roslaunch.parent.ROSLaunchParent(uuid,\
                                                       [self.launch_path])

        self.launch_parent.start()


    def stop(self):
        self.launch_parent.shutdown()
