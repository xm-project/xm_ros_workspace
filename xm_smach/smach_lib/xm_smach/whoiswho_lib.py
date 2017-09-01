#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import State, UserData
from smach_ros import SimpleActionState, MonitorState,ServiceState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from xm_msgs.srv import *
import subprocess
import time
import tf
from geometry_msgs.msg import *
from math import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from copy import deepcopy

# the vital state used in whoiswho smach 


# the face_reco state 
# the position is a list of PointStamped()
# we cannot do any job for reading the output_keys
# io_keys can avoid the problem of reading or writing only of keys
# the name_id is the Id need we to recognize which is int32
# we should deal the data in this state, or the data from cv may be error
# then you will see that the type of navstack can be all Pose()
class FaceReco(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r'],
                        input_keys = ['name_id'],
                        io_keys = ['position'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('get_position',xm_ObjectDetect)
        self.tf_listener =tf.TransformListener()
        self.ps = Pose()
        self.position =list()
    def execute(self,userdata):
        rospy.loginfo('face recongize begin')
        req  = xm_ObjectDetectRequest()
        try:
            name_id = userdata.name_id
        except:
            rospy.logerr('No param specified')
            return 'error'
        req.object_name ="people_position"
        req.people_id = name_id
        # call the cv service
        try:
            res= self.face_reco_client.call(req)
        except:
            return 'aborted'
        else:
            if res.object is  None:
                return 'aborted'
            if res.object[0].state ==0:
                for i in range(len(res.object)):
                    self.data_deal(res.object[i].pos)
                    self.position.append(self.ps) 
                if name_id !=-1:
                    userdata.position =self.position.pop()
                else:
                    userdata.position.extend(self.position)
                return 'succeeded'
            elif res.object[0].state ==-1:
                rospy.logwarn("I will recognize again")
                return 'again'
            elif res.object[0].state ==-2:
                rospy.logwarn("the train may cause error")
                return 'train_error'
            elif res.object[0].state ==-3:
                rospy.logwarn("the position is not fit,turn left")
                return 'turn_l'
            elif res.object[0].state ==-4:
                rospy.logwarn("the position is not fit, turn right")
                return 'turn_r'
            else :
                return 'aborted'
    def data_deal(self,pos_xm):
        # the simple deal for data from the cv
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.45*cos(angle)
        person_y = person_y -0.45*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))  
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        
        
        # pos_xm =self.tf_listener.transformPoint('base_footprint',pos_xm)
        # pos_xm =self.tf_listener.transformPoint('odom',pos_xm)
        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)
    
        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)

        
# state return the name and the thing heard , use for remember face and the information
# serivice name 'name_get'
# serivice type xm_Speech_meaning
# mode ==false mean that is the 2nd time reco

class NameAndThing(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['name','target'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    def execute(self,userdata):
        try:
            a=1
        except:
            rospy.logerr('ros is error, please buy a new computer')
            return 'error'
        # name and target recognize
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            res = self.client.call(command=3)
            # the name from the speech_node is the form of list
            name = res.name.pop()
            target = res.object.pop()
            userdata.name = name
            userdata.target = target
            self.string_ ='I know that you are '+str(name)+'and you want '+str(target)
            print self.string_
            subprocess.call("espeak -vf5 -s 150 '%(a)s'"%{'a':str(self.string_)} , shell = True)
            return 'succeeded'

# simple state check the people_position if empty
class CheckEmpty(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['list'])
    
    def execute(self,userdata):
        try:
            length = len(userdata.list)
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            if length ==0 :
                return 'aborted'
            else:
                return 'succeeded'

# this state is used for generating the sentences that shows xm understands the speaker
class CheckName(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted','error'],
                        input_keys =['name_list','name_id','name'])
    def execute(self, userdata):
        try:
            getattr(userdata,'name_list')
            getattr(userdata,'name_id')
            getattr(userdata,'name')
        except:
            rospy.logerr('No params specified')
            return 'error'
        if userdata.name ==userdata.name_list[userdata.name_id]:
            return 'succeeded'
        else:
            return 'aborted'

# this state used for extract the position of each person
# get the last element of the list
class GetValue(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list'],
                        output_keys =['element'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'


# this state used for joining the name and target in a list
# insert in the head of the list
class NameInList(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys=['name','target'],
                        output_keys=['name_list','target_list'])
        self.name_list = list()
        self.target_list = list()
    def execute(self, userdata):
        try:
            getattr(userdata,'name')
            getattr(userdata,'target')

        except :
            rospy.logerr('No params specified')
            return 'error'
        else:
            # first insert
            # 2 list with the same order
            self.name_list.insert(0,userdata.name)            
            self.target_list.insert(0,userdata.target)
            userdata.name_list =self.name_list
            userdata.target_list =self.target_list
            print self.target_list
            print self.name_list
            return 'succeeded'
        # oh hehe 
        if False:
            return 'aborted'

# this state may look quitly silly , so if you find good way please rewrite 
# get the last of the list
class GetId(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['input_list'],
                        output_keys =['output_id'])
        self.input_list = list()
        self.mode = True
    
    def execute(self,userdata):
        try:
            getattr(userdata,'input_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            print userdata.input_list
            if self.mode ==True:
                self.mode =False
                self.input_list = deepcopy(userdata.input_list)
            else:
                pass
            try:
                last_element =''
                while last_element =='':
                    last_element = self.input_list.pop()
            except :
                rospy.logerr('pop from empty list')
                return 'aborted'
            else:
                userdata.output_id = len(self.input_list)
                return 'succeeded'

# this state is used for rosrun the xm_speech node 
# donnot ask me why the smach code need to run the node 
# I donnot want to say -_-
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.call("xterm -e rosrun xm_speech xm_speech_client.py &", shell = True) 
        except:
            return 'aborted'
        else:
            return 'succeeded'

# get the name
class NameHehe(State):
    def __init__(self):
        State.__init__(self,outcomes =["succeeded",'aborted','error'],
                            input_keys =['name_id','name_list','target_list'],
                            output_keys =['sentences'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'name_id')
            getattr(userdata,'name_list')
            getattr(userdata,'target_list')
            
        except:
            rospy.logerr("No params specified")
            return 'error'
        else:
            print userdata.name_list
            print userdata.name_id
            sentences = 'your name is '+str(userdata.name_list[userdata.name_id]) +' and' +'you want ' +str(userdata.target_list[userdata.name_id])
            userdata.sentences = sentences
            print sentences
            return 'succeeded'
        if False:
            return 'aborted'

# generate the pdf of the object-recognize
class GenePdf(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'])
        self.cv_client = rospy.ServiceProxy('get_object', xm_ObjectDetect)
    
    def execute(self,userdata):
        pass

# this state is used for generating a empty name for order
class EmptyName(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                            output_keys=['target','name'])
    def execute(self,userdata):
        try:
            rospy.sleep(40.0)
        except:
            return 'aborted'
        userdata.target = ''
        userdata.name = ''
        return 'succeeded'

#  clean the costmap
class ClearMap(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            # subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            subprocess.call('rosservice call /move_base/clear_costmaps "{}" ' , shell = True)
            
        except:
            return 'aborted'
        return 'succeeded'