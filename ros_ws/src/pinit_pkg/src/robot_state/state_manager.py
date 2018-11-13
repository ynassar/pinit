#!/usr/bin/env python
from enum import Enum
from utils.fsm import FSM
import rospkg
import roslaunch
import rospy


class RobotStateManager():
    #TODO Make this a static class with static methods w kolo static

    class States(Enum):
        START = 1
        IDLE = 2
        MAPPING = 3
        NAVIGATING = 4
        ERROR = 5

    # static vars
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    gmapping_launch = None
    movebase_launch = None
    robot_launch = None
    pkg_path = rospkg.RosPack().get_path('pinit_pkg')
    gmapping_launch_path = pkg_path + "/launch/gmapping_launch"
    movebase_launch_path = pkg_path + "/launch/movebase_launch"
    robot_launch_path = pkg_path + "/launch/my_robot.launch"

    roslaunch.configure_logging(uuid)
    robot_fsm = FSM()
    states = [s for s in States]


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
        RobotStateManager.gmapping_launch = roslaunch.parent.ROSLaunchParent(RobotStateManager.uuid,\
                [RobotStateManager.gmapping_launch_path])
        RobotStateManager.gmapping_launch.start()


    @classmethod
    def idle_to_navigating_cb(cls):
        RobotStateManager.movebase_launch = roslaunch.parent.ROSLaunchParent(RobotStateManager.uuid,\
                [RobotStateManager.movebase_launch_path])
        RobotStateManager.movebase_launch.start()


    @classmethod
    def mapping_to_idle_cb(cls):
        RobotStateManager.gmapping_launch.shutdown()


    @classmethod
    def mapping_to_mapping_cb(cls):
        pass


    @classmethod
    def navigating_to_idle_cb(cls):
        RobotStateManager.movebase_launch.shutdown()

    
    @classmethod
    def navigating_to_navigating_cb(cls):
        pass


    @classmethod
    def start_to_idle_cb(cls):
        RobotStateManager.robot_launch = roslaunch.parent.ROSLaunchParent(RobotStateManager.uuid,\
                [RobotStateManager.robot_launch_path])
        RobotStateManager.robot_launch.start()
        


if __name__ == "__main__":
    rospy.init_node("robot_state_manager_test")
    RobotStateManager.init_states()
    RobotStateManager.init_transitions()
    RobotStateManager.go_to(RobotStateManager.States.IDLE)
    rospy.sleep(10)


