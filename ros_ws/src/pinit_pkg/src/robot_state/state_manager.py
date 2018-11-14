#!/usr/bin/env python
from enum import Enum
from utils.fsm import FSM
import rospkg
import roslaunch
import rospy


class RobotStateManager():

    class States(Enum):
        START = 1
        IDLE = 2
        MAPPING = 3
        NAVIGATING = 4
        ERROR = 5

    # static vars
    robot_fsm = FSM()
    states = [s for s in States]


    def __init__(self):
        raise NotImplementedError('This class is intended to be used as a static class only')


    @classmethod
    def go_to(cls, state):
        cls.robot_fsm.go_to(state)


    @classmethod
    def init_states(cls):
        for state in cls.states:
            cls.robot_fsm.add_state(state)
        cls.robot_fsm.set_default(cls.States.START)


    @classmethod
    def init_transitions(cls):
        cls.robot_fsm.add_transition(cls.States.START,
                                      cls.States.IDLE,
                                      cls.start_to_idle_cb)
        cls.robot_fsm.add_transition(cls.States.IDLE,
                                      cls.States.IDLE,
                                      cls.idle_to_idle_cb)
        cls.robot_fsm.add_transition(cls.States.IDLE,
                                      cls.States.MAPPING,
                                      cls.idle_to_mapping_cb)
        cls.robot_fsm.add_transition(cls.States.IDLE,
                                      cls.States.NAVIGATING,
                                      cls.idle_to_navigating_cb)
        cls.robot_fsm.add_transition(cls.States.MAPPING,
                                      cls.States.MAPPING,
                                      cls.mapping_to_mapping_cb)
        cls.robot_fsm.add_transition(cls.States.MAPPING,
                                      cls.States.IDLE,
                                      cls.mapping_to_idle_cb)
        cls.robot_fsm.add_transition(cls.States.NAVIGATING,
                                      cls.States.NAVIGATING,
                                      cls.navigating_to_navigating_cb)
        cls.robot_fsm.add_transition(cls.States.NAVIGATING,
                                      cls.States.IDLE,
                                      cls.navigating_to_idle_cb)


    @classmethod
    def idle_to_idle_cb(cls):
        pass

    
    @classmethod
    def idle_to_mapping_cb(cls):
        pass


    @classmethod
    def idle_to_navigating_cb(cls):
        pass


    @classmethod
    def mapping_to_idle_cb(cls):
        pass


    @classmethod
    def mapping_to_mapping_cb(cls):
        pass


    @classmethod
    def navigating_to_idle_cb(cls):
        pass

    
    @classmethod
    def navigating_to_navigating_cb(cls):
        pass


    @classmethod
    def start_to_idle_cb(cls):
        pass       


if __name__ == "__main__":
    rospy.init_node("robot_state_manager_test")
    RobotStateManager.init_states()
    RobotStateManager.init_transitions()
    RobotStateManager.go_to(RobotStateManager.States.IDLE)
    rospy.sleep(10)


