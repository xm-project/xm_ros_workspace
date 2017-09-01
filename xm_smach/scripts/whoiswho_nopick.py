#!/usr/bin/env python
# encoding:utf8

# you need not to import the rospy module ,but import it here you can enjoy the vscode Tab fun -_-
# should use my modules as much as possible ^_^
# and the output_keys cannot be read
# the input_keys cannot be write
# this task is only 8 minutes so when xm return the room the cv should give me the position 
# unless the time must be beyond limit
# if userdata no instance , it must not be used
# this state is no-pick tasks included , for the arm-pick is unstable
# due to the new cv strategy, when we know the information of people position, we have already remember each person
# we should also pay attention to that a node can only contain one tf_listener, two or more tf_listener may cause conflicts
import rospy  
from xm_smach.common_lib import *
from xm_smach.whoiswho_lib import * 
from smach import State, StateMachine, Concurrence
from math import pi
from geometry_msgs.msg import *
import subprocess
# NavStack in this smach is always nav to place
# donnot specified the people_pos

# the snippet is used in the August China-Robot-Competition, but it have some shortcoming
# due to I donnot want to make the snippet a lot bugs, you should improve the snippet by yourselves:
# TODO:
# 1.time 
# only 8 minutes for us, so we should recognize all the face in the detect-place and ache the 
# 5 Pose with their name nad target, and then go to each of them
# instead of recognize->go->return->recognize->..., this way is quite time consuming
# 2.concurrence
# the concurrence we use in the snippet in quite terrible, due to I donnot know how to preempt the 
# concurrence-state in the snippet..., so please remove the global and rewrite the snippet of concurrence

# global
fuck =True
class Hehe(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded'])
    def execute(self,userdata):
        global fuck
        i =50
        while i>0 and fuck ==True:
            rospy.sleep(1.0)
            i-=1
        fuck = True
        return 'succeeded'        
class WhoisWho():
    def __init__(self):
        rospy.init_node('whoiswho_nopick')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)
        # subprocess.call("xterm -e rosrun xm_speech xm_speech_client.py &", shell = True) 
        # add waypoints into the list
        self.waypoints=[]

        location= (Point(4.095, -2.573, 0),  #找人数的点
                   Point(3.105, 3.466, 0),   #出门抓东西的点
                   Point(8.145, -9.663, 0))     #结束出门的点
        quaternions=[]

        euler_angles=[-1.503,-3.049,-1.473]
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        for i in range(3):
            self.waypoints.append(Pose(location[i], quaternions[i]))

        # the locate can specified by ourselves
        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            # rospy.logerr('sm_EnterRoom add State')
            # arm go to the nav_pose in the srdf
            # self.sm_EnterRoom.userdata.arm_mode =0
            # self.sm_EnterRoom.userdata.arm_ps_1 = PointStamped()
            # StateMachine.add('NAV_POSE',
            #                     ArmCmd(),
            #                     transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
            #                     remapping ={'mode':'arm_mode','arm_ps':'arm_ps_1'})
            # wait the door to open
            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'error'})
            # simple delay 5s
            self.sm_EnterRoom.userdata.rec = 5.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            self.sm_EnterRoom.userdata.point = Point(1.5,0.0,0.0)
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove(),
                                transitions={'succeeded':'NAV_1','aborted':'NAV_1','error':'error'},
                                remapping={'point':'point'})
            
            # navstack to room of crowds
            # waypoints are the list of Pose fit the data type need
            self.sm_EnterRoom.userdata.start_waypoint  = self.waypoints[0]
            # self.sm_EnterRoom.userdata.nav_mode =0
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'SELF_INTRO','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            
            # self-introduce
            self.sm_EnterRoom.userdata.sentences_2 = 'I am robot xiaomeng, I come from the nwpu'
            StateMachine.add('SELF_INTRO',
                                Speak(),
                                remapping ={'sentences':'sentences_2'},
                                transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        
        # we have already identity the people from 0-4 when the first recongization
        self.sm_FaceDetect = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            output_keys = ['people_position'])
        with self.sm_FaceDetect:

            
            self.sm_FaceDetect.userdata.people_position =list()
            
            self.sm_FaceDetect.userdata.sentences = 'please look at me'
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})
            # call face_reco service for get all the people position which is a list
            self.sm_FaceDetect.userdata.name_id =-1
            StateMachine.add('GET_POSITION',
                                FaceReco(),
                                remapping  ={'name_id':'name_id','position':'people_position'},
                                transitions ={'succeeded':'succeeded',
                                              'again':'GET_POSITION',
                                              'aborted':'GET_POSITION',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'})

            # if the face-recognize failed, we should make some remedy to make the state continue
            self.sm_FaceDetect.userdata.turn_point_1 = Point(0.0,0.0,pi/8)                   
            StateMachine.add('TURN_L',
                                SimpleMove(),
                                transitions={'succeeded':'SPEAK_2','error':'error'},
                                remapping ={'point':'turn_point_1'})
            
            self.sm_FaceDetect.userdata.turn_point_2 = Point(0.0,0.0,-pi/8)
            StateMachine.add('TURN_R',
                                SimpleMove(),
                                remapping ={'point':'turn_point_2'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})
            StateMachine.add('SPEAK_2',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})
        
        self.sm_GetTarget = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['target'])#the target is a string.....
        with self.sm_GetTarget:

            # because xm is nav to pose in the nav_pose 
            self.sm_GetTarget.userdata.nav_ps = self.waypoints[1]
            # this smach code donnot to grasp ,so this part is useless
            StateMachine.add('NAV_TARGET',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'FIND_OBJECT','aborted':'NAV_TARGET','error':'error'})
            self.sm_GetTarget.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                remapping ={'current_goal':'target','object_pos':'object_pos'},
                                transitions ={'succeeded':'TALK','aborted':'FIND_OBJECT','error':'error'})
            self.sm_GetTarget.userdata.sentences = 'I find the target'
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'PICK','aborted':'TALK','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode_1','arm_ps':'object_pos'},
                                transitions ={'succeeded':'PUT','aborted':'PICK','error':'error'})
            self.sm_GetTarget.userdata.arm_mode_2 = 0
            self.sm_GetTarget.userdata.arm_ps_2 = PointStamped()
            StateMachine.add('PUT',
                                ArmCmd(),
                                remapping ={'mode':'arm_mode_2','arm_ps':'arm_ps_2'},
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
        # concurrence for the speech_node
        # we want to make the speech-meaning a timeout, so we use the concurrence function
        # but in fact we can also use multithread to slove the problem 
        self.meta_Remember = Concurrence(outcomes=['succeeded','aborted','error'],
                                            output_keys=['name','target'],
                                            default_outcome ='succeeded',
                                            outcome_map={'succeeded':{'NAME_AND_THING':'succeeded'}},
                                            child_termination_cb =self.child_cb)
        with self.meta_Remember:
            Concurrence.add('GET_EMPTY_NAME',
                                Hehe()
                                )
            
            Concurrence.add('NAME_AND_THING',
                                NameAndThing(),
                                remapping ={'name':'name','target':'target'}
                                )


        # input  one position is a list
        # io_keys can wirte and read 
        # userdata can update real-time 
        # no need to call the cv service
        self.sm_Remember = StateMachine(outcomes =['succeeded','aborted','error'],
                                        input_keys =['person_position'],
                                        output_keys =['name','target']
                                        )
        with self.sm_Remember:
            # rospy.logerr("sm_Remember add state")
            # self.sm_Remember.userdata.nav_mode =1
            # rospy.logerr("Is here1?")
            
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'TALK','aborted':'NAV_GO','error':'error'}
                                )
                                
            self.sm_Remember.userdata.sentences = "what is your name and what do you want"
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'RUNNODE','aborted':'TALK','error':'error'})
            
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions ={'succeeded':'GET_BOTH','aborted':'aborted'})             
            StateMachine.add('GET_BOTH',
                                self.meta_Remember,
                                remapping ={'name':'name','target':'target'},
                                transitions ={'succeeded':'WAIT','aborted':'GET_BOTH','error':'error'})
            self.sm_Remember.userdata.rec =2.0
            StateMachine.add('WAIT',
                                Wait(),
                                remapping={'rec':'rec'},
                                transitions ={'succeeded':'succeeded','error':"error"})
           
        
        self.sm_GiveBack = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['name_id','name_list','target_list'])# the name is a string
        with self.sm_GiveBack:
            # rospy.logerr('sm_GiveBack add State')
            
            # self.sm_GiveBack.userdata.nav_ps = self.waypoints[0]   
            # self.sm_GiveBack.userdata.nav_mode_1 =0     
            # StateMachine.add('NAV_ROOM',
            #                     NavStack(),
            #                     remapping ={'pos_xm':'nav_ps','mode':'nav_mode_1'},
            #                     transitions ={'succeeded':'FACE_RECO','aborted':'NAV_ROOM','error':'error'})
            # self.sm_GiveBack.userdata.name_id =0
            # self.sm_GiveBack.userdata.name_list =list()
            self.sm_GiveBack.userdata.sentences = "please look at me"
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'WAIT','aborted':'aborted','error':'error'})
            self.sm_GiveBack.userdata.rec =5.0
            StateMachine.add('WAIT',
                                Wait(),
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'FACE_RECO','error':'error'})
            self.sm_GiveBack.userdata.person_position =PointStamped()
            StateMachine.add('FACE_RECO',
                                FaceReco(),
                                remapping ={'position':'person_position','name_id':'name_id'},
                                transitions ={'succeeded':'NAV_GO',
                                              'again':'FACE_RECO',
                                              'aborted':'FACE_RECO',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'}
                                )
            self.sm_GiveBack.userdata.turn_point_1 = Point(0.0,0.0,pi/8)                   
            StateMachine.add('TURN_L',
                                SimpleMove(),
                                transitions={'succeeded':'SPEAK_2','error':'error'},
                                remapping ={'point':'turn_point_1'})
            
            self.sm_GiveBack.userdata.turn_point_2 = Point(0.0,0.0,-pi/8)
            StateMachine.add('TURN_R',
                                SimpleMove(),
                                remapping ={'point':'turn_point_2'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})

            StateMachine.add('SPEAK_2',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'FACE_RECO','aborted':'aborted','error':'error'})
            #please pay attention! 
            # self.sm_GiveBack.userdata.nav_mode_2 =1     
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'GET_NAME','aborted':'NAV_GO','error':'error'})
            StateMachine.add("GET_NAME",
                                NameHehe(),
                                remapping ={'name_list':'name_list','name_id':'name_id','target_list':'target_list', 'sentences':'sentences_1'},
                                transitions ={'succeeded':'TALK_1','aborted':'aborted','error':'error'})
            self.sm_GiveBack.userdata.sentences_1 =''
            StateMachine.add('TALK_1',
                                Speak(),
                                remapping ={'sentences':"sentences_1"},
                                transitions ={'succeeded':'succeeded','aborted':"succeeded",'error':'error'})

            # this pose should be specified to make the people get it
            # self.sm_GiveBack.userdata.arm_mode =2
            # self.sm_GiveBack.userdata.place_ps = PointStamped()
            # self.sm_GiveBack.userdata.place_ps.header.frame_id ='base_footprint'
            # self.sm_GiveBack.userdata.place_ps.point.x =0.7
            # self.sm_GiveBack.userdata.place_ps.point.y =0.0
            # self.sm_GiveBack.userdata.place_ps.point.z =0.3
            # StateMachine.add('PLACE',
            #                     ArmCmd(),
            #                     remapping ={'mode':'arm_mode','arm_ps':'place_ps'},
            #                     transitions ={'succeeded':'TALK_2','aborted':'PLACE','error':'error'})
            # self.sm_GiveBack.userdata.sentences_2 = 'I get the thing you want, am I?'
            # StateMachine.add('TALK_2',
            #                 Speak(),
            #                 remapping ={'sentences':'sentences_2'},
            #                 transitions ={'succeeded':'succeeded','aborted':"aborted",'error':'error'})
            


        self.sm_EndTask = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_EndTask:
            # rospy.logerr('sm_EndTask add State')
            
            self.sm_EndTask.userdata.nav_ps = self.waypoints[2] 
            # self.sm_EndTask.userdata.nav_mode = 0     
            StateMachine.add('NAV_BYEBYE',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'SIMPLE_MOVE','aborted':'NAV_BYEBYE','error':'error'})
            self.sm_EndTask.userdata.point = Point(1.8,0.0,0.0)
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove(),
                                remapping={'point':'point'},
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

        self.WHOISWHO = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.WHOISWHO:

            
            # we can generate some state which contant the list_value
            StateMachine.add('ENTERROOM',
                                self.sm_EnterRoom,
                                transitions ={'succeeded':'FACEDETECT','aborted':'aborted','error':'error'})
            self.WHOISWHO.userdata.people_position =list()
            StateMachine.add('FACEDETECT',
                                self.sm_FaceDetect,
                                remapping ={'people_position':'people_position'},
                                transitions = {'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            self.WHOISWHO.userdata.person_position = PointStamped()
            # if someone is recongized failed, we should make the total list in the same order
            # get the last element of the list
            StateMachine.add('GETPERSON',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'REMEMBER','aborted':"NAV_ROOM",'error':'error'}
                                )
            # the cv will remember the face with the increasing ID
            self.WHOISWHO.userdata.name =''
            self.WHOISWHO.userdata.target= ''
            self.WHOISWHO.userdata.name_list =list()
            self.WHOISWHO.userdata.target_list =list()
            # if donnot need to remember, the order of ids should be the same with the positions return 
            # the first of the position should be the 0
            # last be the 4
            
            StateMachine.add('REMEMBER',
                                self.sm_Remember,
                                remapping ={'person_position':'person_position','name':'name','target':'target'},
                                transitions ={'succeeded':'NAMEINLIST','aborted':'aborted','error':'error'}
                                )
            
            #this state use for joining the name and the target into the name_list and target_list 
            # insert in the head of the lists
            # if the name and target are empty, we should also append them to the lists
            StateMachine.add('NAMEINLIST',
                                NameInList(),
                                remapping ={'name':'name','target':'target','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            # ############################################################################
            # so far here ,the tasks of remember is completed , the rest is the tasks to return the target to the people
            # we should take out the name and the matched target for xm to grasp and give back
            # in fact, we will pop the name and target out the list,so the name_id is gradually reduced
            # ############################################################################
            # StateMachine.add('GETTARGET',
            #                     GetValue(),
            #                     remapping ={'element_list':'target_list','element':'target'},
            #                     transitions ={'succeeded':'CATCHTARGET','aborted':'ENDTASK','error':'error'})
            # StateMachine.add('CATCHTARGET',
            #                     self.sm_GetTarget,
            #                     transitions= {'succeeded':'GETID','aborted':'GETID','error':'error'},
            #                     remapping = {'target':'target'})
            # ############################################################################
            self.WHOISWHO.userdata.nav_ps = self.waypoints[0]
            # self.WHOISWHO.userdata.nav_mode_1 =0
            StateMachine.add('NAV_ROOM',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps'},
                                transitions ={'succeeded':'CLEAR_MAP','aborted':'NAV_ROOM','error':'error'})
            StateMachine.add('CLEAR_MAP',
                                ClearMap(),
                                transitions ={'succeeded':'GETID','aborted':'GETID'})
            # get the last num of the list, ID
            # if the name and target list is empty, will skipped this item
            self.WHOISWHO.userdata.sentences_1 = 'please change the order'
            StateMachine.add('SPEAK_1',
                                Speak(),
                                remapping ={'sentences':'sentences_1'},
                                transitions ={'succeeded':'WAIT_HEHE','aborted':'WAIT_HEHE','error':'error'})
            self.WHOISWHO.userdata.rec_hehe =10.0
            StateMachine.add('WAIT_HEHE',
                                Wait(),
                                remapping ={'rec':'rec_hehe'},
                                transitions ={'succeeded':'SPEAK_2','error':'error'})
            self.WHOISWHO.userdata.sentences_2 = 'I will make the recongization task'
            StateMachine.add('SPEAK_2',
                                Speak(),
                                transitions ={'succeeded':'GETID','aborted':'GETID','error':'error'},
                                remapping ={'sentences':'sentences_2'})

            self.WHOISWHO.userdata.name_id = 0   
            StateMachine.add('GETID',
                                GetId(),
                                remapping ={'output_id':'name_id','input_list':'name_list'},
                                transitions ={'succeeded':'GIVEBACK','aborted':'ENDTASK','error':'error'}
                                )
            StateMachine
            StateMachine.add('GIVEBACK',
                                self.sm_GiveBack,
                                remapping ={'name_id':'name_id','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'NAV_ROOM','aborted':'aborted','error':'error'}
                                )
            StateMachine.add('ENDTASK',
                                self.sm_EndTask,
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})

            # rospy.logerr('sm_Top execute begin')
                                
            self.WHOISWHO.execute()
            self.smach_bool =True
    
    # use for concurrence
    def child_cb(self,outcome_map):
        global fuck
        if  outcome_map['NAME_AND_THING'] == 'succeeded':
            # rospy.logerr('time out')
            # string_ = "I cannot understand the meaning, go to the next state"
            # print string_
            fuck =False
            print 'Concurrence over'
            # subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(string_)} , shell = True)

            return True

    def shutdown(self):
        if self.smach_bool ==False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')


if __name__ =="__main__":
    WhoisWho()          
