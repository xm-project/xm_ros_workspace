#!/usr/bin/env python
# encoding:utf8
# you need not to import the rospy module ,but import it here you can enjoy the vscode Tab fun -_-
# should use my modules as much as possible ^_^
# the gpsr task may not have the specified person names, they are all 'person'
import rospy  
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math

class Gpsr():
    def __init__(self):
        rospy.init_node('GpsrSmach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('you are all sillyb')
        self.smach_bool = False 

        self.sm_EnterRoom = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            # if use arm, use this state
            # self.sm_EnterRoom.userdata.arm_mode_1 =0
            # self.sm_EnterRoom.userdata.arm_ps = PointStamped()
            # StateMachine.add('NAV_POSE',
            #                     ArmCmd(),
            #                     transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
            #                     remapping ={'arm_ps':'arm_ps','mode':'arm_mode_1'})
            
            #first detect the door is open or not  
            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'aborted'})

            # after clear the costmap, wait for some seconds  
            self.sm_EnterRoom.userdata.rec = 3.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            # before executing the nav-stack, move directly through the door may make the nav-stack easier
            self.sm_EnterRoom.userdata.go_point = Point(1.5,0.0,0.0)
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove(),
                                remapping ={'point':'go_point'},
                                transitions={'succeeded':'NAV_1','aborted':'NAV_1','error':'error'})

            # go to the known Pose in the map to get the gpsr-command
            self.sm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'SPEAK1','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})

            self.sm_EnterRoom.userdata.sentences = 'I arrived,please ask me a question'                    
            StateMachine.add('SPEAK1',
                                Speak(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping ={'sentences':'sentences'})   

            
        self.sm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        # find task
        with self.sm_Find:

            # for the people-finding task, pay attention that at the end of the task,
            # run a simple executable file to make the cv-node release the KinectV2
            # otherwise, the find-object task may throw error and all the cv-node may
            # break down
            self.sm_Person = StateMachine(outcomes = ['succeeded','aborted','error']
                                    )

            with self.sm_Person:
                self.sm_Person.userdata.rec = 2.0
                # run the people_tracking node
                StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'WAIT','aborted':'succeeded'})
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                    remapping ={'rec':'rec'})
                #the data should be PointStamped() 
                self.sm_Person.userdata.pos_xm  =Pose()
                StateMachine.add('GET_PEOPLE_POS',
                                    FindPeople().find_people_,
                                    transitions ={'invalid':'NAV_PEOPLE','valid':'SPEAK','preempted':'aborted'},
                                    remapping = {'pos_xm':'pos_xm'}
                                    )
                # this state will use the userdata remapping in the last state
                
                StateMachine.add('NAV_PEOPLE',
                                    NavStack(),
                                    transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                    remapping = {'pos_xm':'pos_xm'})
                self.sm_Person.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'CLOSEKINECT','error':'error'},
                                    remapping = {'sentences':'sentences'})

                # close the KinectV2
                StateMachine.add('CLOSEKINECT',
                                    CloseKinect(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted'})
            
            self.sm_Pos = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])
            with self.sm_Pos:
                # this command may be foolish, but it may be used in the next competition
                # get the name of the target

                # in the target_gpsr.py we define some Pose() , they are all named as ${room_name}+'_table_1(2)'
                # because each room we have some best detecting positions, we should make xm go to these places
                # for object-find tasks
                self.sm_Pos.userdata.pose = Pose()
                self.sm_Pos.userdata.mode_1 =1
                StateMachine.add('GET_POS',
                                    GetPos(),
                                    remapping ={'target':'target','current_task':'current_task','pose':'pose','mode':'mode_1'},
                                    transitions={'succeeded':'NAV_HEHE','aborted':'aborted','error':'error'})

                StateMachine.add('NAV_HEHE',
                                    NavStack(),
                                    remapping ={'pos_xm':'pose'},
                                    transitions ={'succeeded':'GET_TARGET','aborted':'NAV_HEHE','error':'error'})

                self.sm_Pos.userdata.target_mode = 0
                self.sm_Pos.userdata.name = ''
                StateMachine.add('GET_TARGET',
                                    GetTarget(),
                                    remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
                                    transitions ={'succeeded':'FIND_OBJECT','aborted':'aborted','error':'error'})
                
                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'GET_POS_1','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 

                # if xm not finds the object in the first detecting-place, she should go to the second
                # specified place for detect-task, but as U can see in the target_gpsr.py, some room are
                # only one best detecting-place
                self.sm_Pos.userdata.pose_1 = Pose()
                self.sm_Pos.userdata.mode_2 =2
                StateMachine.add('GET_POS_1',
                                    GetPos(),
                                    remapping ={'target':'target','current_task':'current_task','pose':'pose_1','mode':'mode_2'},
                                    transitions={'succeeded':'NAV_HAHA','aborted':'aborted','error':'error'})

                StateMachine.add('NAV_HAHA',
                                    NavStack(),
                                    remapping ={'pos_xm':'pose_1'},
                                    transitions ={'succeeded':'FIND_OBJECT_1','aborted':'NAV_HAHA','error':'error'})

                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT_1',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 


                self.sm_Pos.userdata.sentences = 'I find it'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping ={'sentences':'sentences'})

            # this state is used for swithing the mode, finding people or finding object?
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.sm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('POS',
                                self.sm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})

        #nav tasks  
        self.sm_Nav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        with self.sm_Nav:
            self.sm_Nav.userdata.pos_xm =Pose()
            self.sm_Nav.userdata.target_mode =1
            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions ={'succeeded':"NAV_GO",'aborted':'aborted','error':'error'},
                                remapping ={'target':'target','current_task':"current_task",'current_target':'pos_xm','mode':'target_mode'})
            
            StateMachine.add('NAV_GO',
                                NavStack(),
                                transitions ={'succeeded':'SPEAK','aborted':'NAV_GO','error':'error'},
                                remapping ={'pos_xm':'pos_xm'})

            self.sm_Nav.userdata.sentences = 'I arrive here'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping ={'sentences':'sentences'})
        
        #follow task 
        # we will use the concurrence fun 
        # the tutorials in the wiki too easy, it is no help for you to understand the concurrence
        # please see the test-snippet of the source of executive-smach
        # https://github.com/ros/executive_smach/blob/indigo-devel/smach_ros/test/concurrence.py
        self.sm_Follow = Concurrence(outcomes=['succeeded','aborted','error'],
                                        default_outcome ='succeeded',
                                        outcome_map={'succeeded':{'STOP':'succeeded'}},
                                        child_termination_cb =self.child_cb)

        with self.sm_Follow:
            self.meta_Follow = StateMachine(outcomes =['succeeded','aborted','error'])
            with self.meta_Follow:
                
                StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'FIND_PEOPLE','aborted':'succeeded'})
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

        # speak task 
        self.sm_Talk = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Talk:
            StateMachine.add('SPEAK',
                                Anwser(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})
        
        self.sm_Pick = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])
        
        with self.sm_Pick:
            
            self.sm_Pick.userdata.name =''
            self.sm_Pick.userdata.target_mode =0
            StateMachine.add('GETNAME',
                                GetTarget(),
                                remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
                                transitions={'succeeded':'FIND_OBJECT','aborted':'aborted','error':'error'})
            self.sm_Pick.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'FIND_OBJECT','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'})
            
            #making the xm foreward the object may make the grasping task easier  
            self.sm_Pick.userdata.pose = Pose()
            StateMachine.add('POS_JUSTFY',
                                PosJustfy(),
                                remapping={'object_pos':'object_pos','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'FIND_AGAIN','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})
            StateMachine.add('FIND_AGAIN',
                                FindObject(),
                                transitions ={'succeeded':'PICK','aborted':'FIND_AGAIN','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'})
            self.sm_Pick.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})

        
        self.sm_Place = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Place:
            # place_ps please specified due to the scene
            self.sm_Place.userdata.place_ps = PointStamped()
            self.sm_Place.userdata.place_ps.header.frame_id ='base_link'
            self.sm_Place.userdata.place_ps.point.x =0.8
            self.sm_Place.userdata.place_ps.point.y =0.0
            self.sm_Place.userdata.place_ps.point.z =0.6 
            # without moveit, if is just place it in a open space
            self.sm_Place.userdata.arm_mode_1 =3         
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'PLACE','error':'error'},
                                remapping ={'arm_ps':'arm_ps','mode':'arm_mode_1'})
        
        self.sm_GPSR =StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_GPSR:
            self.sm_GPSR.userdata.target = list()
            self.sm_GPSR.userdata.action = list()
            self.sm_GPSR.userdata.task_num =0
            self.sm_GPSR.userdata.current_task =-1
            # get the task due to the speech-node
            StateMachine.add('RECEIVE_TASKS',
                                GetTask(),
                                transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                                remapping ={'target':'target','action':'action','task_num':'task_num'}
                                )
            #get the task in order and execute the task 
            # if want to go to the door at the end of the task, please modify:
            # succeeded->succeeded --> succeeded->BYEBYE
            StateMachine.add('GET_NEXT_TASK',
                                NextDo(),
                                transitions ={'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'error':'error',
                                            'find':'FIND',
                                            'go':'GO',
                                            'follow':'FOLLOW',
                                            'pick':'GET_NEXT_TASK',
                                            'place':'GET_NEXT_TASK',
                                            'talk':'TALK',
                                            'aborted':'aborted',
                                            'error':'error'},
                                remapping   ={'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'}
                                )
            # all the task-group
            StateMachine.add('GO',
                                self.sm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            StateMachine.add('FIND',
                                self.sm_Find,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',
                                self.sm_Follow,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FOLLOW'})
            StateMachine.add('PICK',
                                self.sm_Pick,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK'})
            StateMachine.add('TALK',
                                self.sm_Talk,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
            StateMachine.add('PLACE',
                                self.sm_Place,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PLACE'}) 


            self.sm_GPSR.userdata.string = 'I finish all the three tasks, I will go out the stage'
            StateMachine.add('BYEBYE',
                                Speak(),
                                transitions = {'succeeded':'GOOUT','error':'error'},
                                remapping ={'sentences':'string'})  
            #no need 
            self.sm_GPSR.userdata.waypoint = gpsr_target['end']['pos'] 
            StateMachine.add('GOOUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'})
                                
        out_1 = self.sm_EnterRoom.execute()
        # only execute for the 1 time, 
        out_2 = self.sm_GPSR.execute()    
        self.smach_bool = True 

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
        
    # use for concurrence
    def child_cb(self,outcome_map):
        if outcome_map['STOP']== 'succeeded':
            rospy.logerr('xm_bot will stop follow stack')
            string_ = "I will stop following you"
            subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(string_)} , shell = True)
            # here are one global path, please modify this to the computer you use
            subprocess.call("/home/xiong/Recognition/kinect2/terminate_people", shell=True)
            return True


if __name__ == "__main__":
    try:
        Gpsr()
    except KeyboardInterrupt:
        pass    

                