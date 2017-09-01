#!/usr/bin/env python
import rospy
from smach import State, StateMachine,UserData, Concurrence
from smach_ros import MonitorState
from xm_msgs.msg import *
class State1(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded'])

    def execute(self,UserData):
        rospy.logwarn('I am State1')
        rospy.sleep(2.0)
        return 'succeeded'
       

class State2(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded'])

    def execute(self,UserData):
        while True:
            rospy.logwarn('this is the state2')
            rospy.sleep(0.5)
            if self.preempt_requested():
                return 'succeeded'
  

class SmachHaha():
    def __init__(self):
        rospy.init_node('smach_haha')
        rospy.on_shutdown(self.shutdown)
        self.smach_top = Concurrence(outcomes=['succeeded','aborted','preempted'],default_outcome ='succeeded',
                                        outcome_map={'succeeded':{'STATE_1':'succeeded'}},
                                        child_termination_cb = self.child_termination_cb)
        with self.smach_top:
            self.sm_state2 =StateMachine(outcomes=['succeeded'])
            with self.sm_state2:
                StateMachine.add('FIRST',
                                    State2(),
                                    transitions={'succeeded':'SECOND'})
                StateMachine.add('SECOND',
                                    State1(),
                                    transitions={'succeeded':'FIRST'})
            Concurrence.add('STATE_1',
                                State1())
            Concurrence.add('STATE_2',
                                self.sm_state2)
        haha = self.smach_top.execute()

    def child_termination_cb(self,outcome_map):
        if outcome_map['STATE_1'] =='succeeded':
            rospy.logerr('fuck you')
            return True
    def people_cb(self,UserData,msg):
        return False

    def shutdown(self):
        rospy.logerr('smach test over')
    
if __name__ =="__main__":
    try:

        SmachHaha()
    except KeyboardInterrupt:
        pass