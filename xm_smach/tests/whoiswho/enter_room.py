#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import State,UserData,StateMachine
from smach_ros import ServiceState
from xm_msgs.msg import *
from xm_msgs.srv import *
from xm_smach.common_lib import *
from xm_smach.whoiswho_lib import *
from math import pi
from geometry_msgs.msg import *
# test smach for enter the room of people
class EnterRoom():
    def __init__(self):
        rospy.init_node('enter_room')
        rospy.on_shutdown(self.shutdown)
        self.sm_enter = StateMachine(outcomes= ['succeeded','aborted','preempted'])
        self.nav_states_ = WayPoint().nav_states_
        with self.sm_enter :
            # navpose is the pose of xm_arm
            self.sm_enter.userdata.mode = 0
            StateMachine.add('NAV_POSE',
                                ArmCmd(),
                                transitions={'succeeded':'WAIT1','aborted':'NAV_POSE'},
                                remapping = {'mode':'mode'})
            # this time (rec =1) may need to justfy to satisfy the scene
            self.sm_enter.userdata.delay_time =1.0
            StateMachine.add('WAIT1',
                                Wait(),
                                transitions={'succeeded':'DOOR'},
                                remapping = {'rec':'delay_time'})
            StateMachine.add('DOOR', 
                                DoorDetect().door_detect_,
                                transitions={'invalid':'NAV1','valid':'DOOR'})  
            StateMachine.add('NAV1',
                                self.nav_states_[0],
                                transitions={'succeeded':'TURN_POSE','aborted':'NAV1'})
            pt = Point()
            pt.x =0
            pt.y=0
            pt.z= pi/8
            self.sm_enter.userdata.turn_pose = pt
            StateMachine.add('TURN_POSE',
                                SimpleMove(),
                                transitions={'succeeded':'SPEAK','aborted':'SPEAK'},
                                remapping = {'turn_pose':'turn_pose'})
            self.sm_enter.userdata.sentences = 'please look at me'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded'},
                                remapping ={'sentences':'sentences'})    
        self.smach_bool = False      
        self.sm_enter.execute()
        self.smach_bool =True
    
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('xm completes the whole tasks , >3<')
        else:
            rospy.loginfo('Stopping..., you may write the wrong code , but this is not the fault of xm #_   #')

  
if __name__ == "__main__":
    try:
        EnterRoom()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")        
     