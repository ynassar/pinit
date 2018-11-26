#!/usr/bin/env python
import rospkg
from nodes.ros_launch import RosLaunch


class NodeManager():
    def __init__(self):
        self.base_pkg_path = rospkg.RosPack().get_path('pinit_pkg')

        self.gmapping_relative_path = "/launch/gmapping_launch.launch"
        self.movebase_relative_path = "/launch/move_base.launch"
        self.robot_relative_path = "/launch/my_robot_config.launch"

        self.gmapping_launch = RosLaunch()
        self.movebase_launch = RosLaunch()
        self.robot_launch = RosLaunch()

        self.gmapping_launch.set_path(self.base_pkg_path + self.gmapping_relative_path)
        self.movebase_launch.set_path(self.base_pkg_path + self.movebase_relative_path)
        self.robot_launch.set_path(self.base_pkg_path + self.robot_relative_path)


    def start_gmapping(self):
        self.gmapping_launch.start()


    def stop_gmapping(self):
        self.gmapping_launch.stop()

    
    def start_movebase(self):
        self.movebase_launch.start()


    def stop_movebase(self):
        self.movebase_launch.stop()


    def start_robot(self):
        self.robot_launch.start()


    def stop_robot(self):
        self.robot_launch.stop()


if __name__ == "__main__":
    import rospy
    rospy.init_node("node_manager_test")
    manager = NodeManager()
    manager.start_gmapping()
