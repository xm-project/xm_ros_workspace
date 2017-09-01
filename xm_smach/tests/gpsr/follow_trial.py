#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from smach import State, StateMachine, UserData, Concurrence
from xm_smach.gpsr_lib import *
from geometry_msgs.msg import *
from xm_smach.common_lib import *
from subprocess import *
class sm_Follow():
    def __init__(self):
        rospy.init_node('sm_Follow')
        self.sm_FollowMe = Concurrence(outcomes=['succeeded','aborted','error'],
                                        default_outcome ='succeeded',
                                        outcome_map={'succeeded':{'STOP':'succeeded'}},
                                        child_termination_cb =self.child_cb)
        # if one state complete, the Concurrence will give a preempted signal, and will stop the current state in the preempt outcomes
        with self.sm_FollowMe:
            self.meta_Follow = StateMachine(outcomes =['succeeded','aborted','error'])
            with self.meta_Follow:
                self.meta_Follow.userdata.pos_xm = Pose()
                StateMachine.add('FIND_PEOPLE',
                                    FindPeople().find_people_,
                                    remapping ={'pos_xm':'pos_xm'},
                                    transitions ={'invalid':'MOVE','valid':'FIND_PEOPLE','preempted':'aborted'})
                StateMachine.add('MOVE',
                                    NavStack(),
                                    remapping ={'pos_xm':'pos_xm'},
                                    transitions={'succeeded':'FIND_PEOPLE','aborted':'MOVE','error':'error'})
            self.meta_Stop = StateMachine(outcomes =['succeeded','aborted'])
            with self.meta_Stop:
                StateMachine.add('STOP',
                                    StopFollow(),
                                    transitions ={'succeeded':'succeeded','aborted':'STOP'})
            Concurrence.add('FOLLOW',
                                self.meta_Follow)
            Concurrence.add('STOP',
                                self.meta_Stop)
        self.sm_FollowMe.execute()

    def child_cb(self,outcome_map):
        if outcome_map['STOP']== 'succeeded':
            rospy.logerr('fuck')
            subprocess.call("/home/xiong/Recognition/kinect2/terminate_people", shell=True)
            return True
    # def shutdown(self):
    #     rospy.logerr('byebye')


if __name__ =="__main__":
    try:
        sm_Follow()
    except KeyboardInterrupt:
        pass