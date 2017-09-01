#!/usr/bin/env python
# encoding:utf8
from xm_smach.target_gpsr import gpsr_target
import rospy
import tf
import actionlib
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool,Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
        

# state decide the smach of gpsr ,it is used for choosing the meta-smach due to the order of speech
class NextDo(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                    outcomes=['succeeded','aborted','go','find','follow','pick','talk','place','error'],
                    input_keys =['action','task_num'],
                    io_keys =['current_task'])
        
    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task
            task_num = userdata.task_num
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_task+=1

        # tasks have been executed successfully
        if userdata.current_task ==  task_num:
            return 'succeeded'
        current_action =  action[userdata.current_task]
        if current_action == 'go':
            return 'go'
        elif current_action == 'find':
            return 'find'
        elif current_action == 'follow':
            return 'follow'
        elif current_action == 'grasp':
            return 'pick'
        elif current_action == 'talk':
            return 'talk'
        elif current_action == 'place':
            return 'place'
        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -=1
            return 'aborted'
  
#   function as the name of state -_-
class PersonOrPosition(State):### switch the goal among person , speaker, thing, position
    def __init__(self):
        State.__init__(self, 
                        outcomes=['error','person','position'],
                        input_keys=['target','current_task'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'current_task')
        except:
            rospy.logerr('No param specified')
            return 'error'
        self.target = userdata.target[userdata.current_task]
        if self.target=='person' or self.target=='speaker':
            return 'person'
        else :
            return 'position'


# state used for find the people to follow 
# use for the position of people and xm
# xm must face to the people
# this state may last for 5 turns because the camera may not find the people fast
'''@param max_checks the number of messages to receive and evaluate. If cond_cb returns False for any
               of them, the state will finish with outcome 'invalid'. If cond_cb returns True for 
               all of them, the outcome will be 'valid' '''
# here we want to use the features of MonitorState mentioned above
class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])

        self.tf_listener = tf.TransformListener()

# if camrea find the people ,this state will return False
# else, if after 5 times camera still cannot find people, will return True
    def people_cb(self,userdata,msg):

        if msg is not None:
            self.tmp_pos = msg.position
            ps =self.data_deal(self.tmp_pos)
            userdata.pos_xm = ps
            
            return False
        else:
            return True

    def data_deal(self,pos_xm):
        # the simple deal for data from the cv from PointStmp() to Pose()
        # due to we have change the frame of camera_link,so the data may not be dealed as so
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.6*cos(angle)
        person_y = person_y -0.6*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        # init the stamped of the Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

 
# simple state used for get meaning from the speech node
# command mean invoke the speech node to return the meaning from the order
class GetTask(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['target','action','task_num'])
                        
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
                
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
        except:
            rospy.logerr('No param specified')
            return 'error'
        subprocess.call("xterm -e rosrun xm_speech xm_speech_client.py &", shell = True)
        self.speech_client.wait_for_service(timeout=10)
        response =self.speech_client.call(command =2)
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = response.action
            userdata.target= response.target
            print(userdata.task_num)
            print(userdata.action)
            print(userdata.target)
            return 'succeeded'
        else:
            return 'aborted'


# state used for getting the information of the specified target
# the mode ==0 mean return the name of the target
# the mode==1 mean return the Pose of the target
class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                            input_keys =['target','current_task','mode'],
                            output_keys =['current_target'])
    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        if userdata.mode ==0:
            # due to the cv may need a different name-style from the speech_node
            # this should be modify
            userdata.current_target = userdata.target[userdata.current_task]
        elif userdata.mode ==1:
            userdata.current_target = gpsr_target[userdata.target[userdata.current_task]]['pos']
        else:
            return 'aborted'
        return 'succeeded'

# close the Kinect use a simple executable file
class CloseKinect(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            subprocess.call("/home/xiong/Recognition/kinect2/terminate_people" , shell = True)
        except:
            return 'aborted'
        return 'succeeded'

# the state is used for invoking the speech service to get the stop-signal
class StopFollow(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    
    def execute(self,userdata):
        try:
            subprocess.call("xterm -e rosrun xm_speech xm_speech_client.py &", shell = True) 
        except:
            return 'aborted'
        
        self.speech_client.wait_for_service(timeout=10)
        res= self.speech_client.call(command=4)
        if res.num>0:
            return 'succeeded'

# run the people_tracking node
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.call('xterm -e rosrun xm_vision people_tracking &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'
    
# get the detect Pose where can make the find-object task execute best
class GetPos(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['target','current_task','mode'],
                        output_keys =['pose'])

    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            last_task = userdata.current_task -1
            last_target = str(userdata.target[last_task]) +'_table' +'_' +str(userdata.mode)
            print last_target
            userdata.pose = gpsr_target[last_target]['pos']
            return 'succeeded'
            if False:
                return 'aborted'
        