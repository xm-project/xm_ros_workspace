# /usr/bin/env python
# encoding:utf8
import rospy
from xm_smach.gpsr_lib import *
from xm_smach.common_lib import *
from xm_smach.target_gpsr import *
class SmFind():
    def __init__(self):
        rospy.init_node('sm_find')
        rospy.on_shutdown(self.shutdown)
        self.current_task =0
        self.goal = ['juice']
        self.action =['nav','pick','nav']
        self.sm_find=StateMachine(outcomes=
            ['succeeded','aborted','preempted','person','position','valid','invalid','again'])
            with self.sm_find:
                self.sm_person=StateMachine(outcomes=
                ['succeeded','aborted','preempted','valid','invalid','again'])
                # nav to the person is after nav the ordered room and then through camera find the person pos
                with self.sm_person:
                    sm_person.userdata.wait_time = 2.0
                    StateMachine.add('WAIT',
                                     Wait(),
                                     transitions={'succeeded':'GET_PEOPLE_POSITION'},
                                     remapping = {'rec':'wait_time'})
                    StateMachine.add('GET_PEOPLE_POSITION',
                                        FindPeople().find_people_,
                                        transitions={'succeeded':'NAV2_PEOPLE','aborted':'SPEAK','again':'GET_PEOPLE_POSITION'},
                                        remapping={'pos_person':'sm_pos_person','pos_xm':'sm_pos_xm'})
                    sm_person.userdata.mode = 1
                    StateMachine.add('NAV2_PEOPLE', 
                                        Nav(),
                                        transitions={'succeeded':'SPEAK'},
                                        remapping={'mode':'mode','pos_person':'sm_pos_person','pos_xm':'sm_pos_xm'})  
                    sm_person.userdata.sentences = 'I find you'
                    StateMachine.add('SPEAK',
                                        Speak(),transitions={'succeeded':''},
                                        remapping={'sentences':'sentences'})
                # nav to thing is directly nav to the place in the target_gpsr 
                self.sm_position=StateMachine(outcomes=
                ['succeeded','aborted','preempted'])
                with self.sm_position:
                    sm_position.userdata.mode =0
                    sm_position.userdata.pos_thing=  target[self.goal[self.current_task]]['pos']
                    StateMachine.add('NAV_POSITION',
                                        Nav(),
                                        transitions={'succeeded':'SPEAK','aborted':'NAV_POSITION','preempted':''},
                                        remapping={'mode':'mode','pos_xm':'pos_thing'})
                    sm_position.userdata.sentences ='I find it'
                    StateMachine.add('SPEAK',
                                        Speak(),
                                        transitions={'succeeded':''},
                                        remapping={'sentences':'sentences'})
                sm_find.userdata.goal = self.goal
                sm_find.userdata.current_task = self.current_task
                StateMachine.add('PERSON_OR_POS',
                                    PersonOrPosition(),
                                    transitions={'person':'PERSON','position':'POSITION'},
                                    remapping={'goal':'goal','current_task':'current_task'})   
                StateMachine.add('PERSON',self.sm_person,transitions={'succeeded':''})
                StateMachine.add('POSITION',self.sm_position,transitions={'succeeded':''})
                self.smach_bool =False
                sm_find.execute()
                self.smach_bool = True
    def on_shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach task take successfully')
        else:
            rospy.loginfo('smach task is terminating due to wrong snippet')


if __name__=='__main__':
    SmFind()
