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

    def __init__(self, PoseListener):
        self.controller = GpsController()
        self.pose_pub = rospy.Publisher("pose_pub", PoseStamped, queue_size=10)
        self.robot_pose_listener = PoseListener
        self.stop = False
        self.thread = threading.Thread(target=self.callibration_loop)
        self.theta_lock = threading.Lock()
        self.origin_lock = threading.Lock()
        self.stop_lock = threading.Lock()
        self.gps_origin = None
        self.theta_avg = None


    def start_cal(self):
        self.thread.start()


    def callibration_loop(self):
        self.origin_lock.acquire()
        self.gps_origin = self.controller.get_coordinates()
        while self.gps_origin is None:
            self.gps_origin = self.controller.get_coordinates()
        self.gps_origin = gps_utils.coordinates_to_rad(self.gps_origin)
        self.origin_lock.release()

        rate = rospy.Rate(10)
        theta_sum = 0
        angle_count = 0
        while not rospy.is_shutdown():
            current_coordinates = self.controller.get_coordinates()
            current_coordinates = gps_utils.coordinates_to_rad(current_coordinates)
            distance, angle = gps_utils.get_vector(self.gps_origin, current_coordinates)

            robot_pose = self.robot_pose_listener.get_pose()
            robot_x = robot_pose.pose.position.x
            robot_y = robot_pose.pose.position.y
            robot_angle = np.arctan2(robot_y, robot_x)
            
            if distance > 10:
                delta_theta = robot_angle - angle
                theta_sum += delta_theta
                angle_count += 1

            self.stop_lock.acquire()
            stop = self.stop
            self.stop_lock.release()
            if stop:
                self.theta_lock.acquire()
                self.theta_avg = theta_sum / float(angle_count)
                self.theta_lock.release()
                return
    
            rate.sleep()

        
    def stop_cal(self):
            self.stop = True


    def get_theta(self):
        self.theta_lock.acquire()
        theta = self.theta_avg
        self.theta_lock.release()

        return theta


    def get_origin(self):
        self.origin_lock.acquire()
        origin = self.gps_origin
        self.origin_lock.release()

        return origin
        
        


if __name__=="__main__":
    rospy.init_node("gps_converstion_test")
    # test = GPSCallibrtor()
    # test.test_loop()

