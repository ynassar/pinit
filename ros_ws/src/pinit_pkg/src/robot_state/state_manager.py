#!/usr/bin/env python
from enum import Enum
from Queue import Queue
import rospkg
import roslaunch
import rospy

from utils.fsm import FSM
from nodes.node_manager import NodeManager
from map.map_streamer import MapStreamer
from robot_motion.motion_controller import MotionController
from robot_motion.robot_pose import PoseListenerFactory
from server_communication.server_pose_streamer import ServerPoseStreamerFactory
from gps import gps_cal

class RobotStateManager():

    class States(Enum):
        START = 1
        IDLE = 2
        MAPPING = 3
        NAVIGATING = 4
        ERROR = 5


    @classmethod
    def create(cls):
        robot_fsm = FSM()
        communication_queue = Queue()
        node_manager = NodeManager()
        map_streamer = MapStreamer(communication_queue)
        pose_listener = PoseListenerFactory()
        pose_streamer = ServerPoseStreamerFactory(communication_queue, pose_listener)
        gps_calibrator = gps_cal.GPSCallibrtor(pose_listener)
        motion_controller = MotionController()


        return RobotStateManager(
            robot_fsm=robot_fsm,
            com_queue=communication_queue,
            node_manager=node_manager,
            map_streamer=map_streamer,
            pose_streamer=pose_streamer,
            motion_controller=motion_controller,
            gps_calibrator=gps_calibrator)


    def __init__(self, robot_fsm, com_queue, node_manager, map_streamer, pose_streamer,
                 motion_controller, gps_calibrator):

        self.fsm_states = [s for s in self.States]
        self.robot_fsm = robot_fsm
        self.communication_queue = com_queue
        self.node_manager = node_manager
        self.map_streamer = map_streamer
        self.pose_streamer = pose_streamer
        self.motion_controller = motion_controller
        self.gps_calibrator = gps_calibrator

        self.init_states()
        self.init_transitions()


    def go_to(self, state, *args):
        self.robot_fsm.go_to(state, *args)


    def init_states(self):
        for state in self.fsm_states:
            self.robot_fsm.add_state(state)
        self.robot_fsm.set_default(self.States.START)


    def init_transitions(self):
        self.robot_fsm.add_transition(self.States.START,
                                      self.States.IDLE,
                                      self.start_to_idle_cb)
        self.robot_fsm.add_transition(self.States.IDLE,
                                      self.States.IDLE,
                                      self.idle_to_idle_cb)
        self.robot_fsm.add_transition(self.States.IDLE,
                                      self.States.MAPPING,
                                      self.idle_to_mapping_cb)
        self.robot_fsm.add_transition(self.States.IDLE,
                                      self.States.NAVIGATING,
                                      self.idle_to_navigating_cb)
        self.robot_fsm.add_transition(self.States.MAPPING,
                                      self.States.MAPPING,
                                      self.mapping_to_mapping_cb)
        self.robot_fsm.add_transition(self.States.MAPPING,
                                      self.States.IDLE,
                                      self.mapping_to_idle_cb)
        self.robot_fsm.add_transition(self.States.NAVIGATING,
                                      self.States.NAVIGATING,
                                      self.navigating_to_navigating_cb)
        self.robot_fsm.add_transition(self.States.NAVIGATING,
                                      self.States.IDLE,
                                      self.navigating_to_idle_cb)


    def idle_to_idle_cb(self):
        pass


    def idle_to_mapping_cb(self):
        self.node_manager.start_gmapping()
        self.map_streamer.start()


    def idle_to_navigating_cb(self):
        pass


    def mapping_to_idle_cb(self):
        self.map_streamer.finish()
        self.node_manager.stop_gmapping()
        #self.node_manager.start_movebase()


    def mapping_to_mapping_cb(self, *args):
        self.motion_controller.move(*args)


    def navigating_to_idle_cb(self):
        pass


    def navigating_to_navigating_cb(self):
        pass


    def start_to_idle_cb(self):
        #self.node_manager.start_robot()
        self.pose_streamer.init_stream_loop()


if __name__ == "__main__":
    rospy.init_node("robot_state_manager_test")
    manager = RobotStateManager.create()
    manager.init_states()
    manager.init_transitions()
    manager.go_to(RobotStateManager.States.IDLE)
    rospy.sleep(10)


