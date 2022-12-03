#!/usr/bin/env python 

import roslib 
roslib.load_manifest("leo_autonomy")

import rospy

from smach import StateMachine, State
from smach_ros import ConditionState, IntrospectionServer

class IdleState(State):
    def __init__(self):
        State.__init__(self, outcomes=[""])

    def execute(self, userdata):
        pass

class TeleopState(State):
    pass

class MappingState(State):
    pass 

class CollectionState(State):
    pass 

class ReturnState(State):
    pass

class EstopState(State):
    def __init__(self):
        State.__init__(self, outcomes=[])

    def execute(self, userdata):
        rospy.logwarn_throttle(10, "ESTOPPED")


def construct_state_machine():
    sm = StateMachine(outcomes=[''])

    with sm:
        pass

    return sm


def main():
    rospy.init_node("leo_state_machine_node")

    sm = construct_state_machine()
    intro_server = IntrospectionServer("leo_state_machine_code", sm, "/leo")
    intro_server.start()

    sm.execute()
    rospy.spin()
    intro_server.stop()
    

if __name__ == "__main__":
    main()

