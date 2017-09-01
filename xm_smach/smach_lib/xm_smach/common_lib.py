#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
import rospy
from smach import State,UserData
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool,Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


# simple state for nav_stack 
# may usually use for whoiswho
# we may notice the very important detail:
# if you are nav to the front of person, the data must from the cv ,so it must be PointStamped,
# but if you are just nav to place, the waypoints usually specified, so it must be Pose
# there are no nav_people and nav_thing distinguish
class NavStack(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()

        

    def execute(self, userdata):
        # mode ==1 mean that xm is going the place of people, which require xm face the person
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            # wait 60s for the action server to become available 
            self.nav_client.wait_for_server(rospy.Duration(60))
            return self.nav_thing(userdata.pos_xm)


    def nav_thing(self,pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        self.nav_client.send_goal(goal)
        nav_counter = 0
        # if nav stack is failed, we can plan again
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter<50:
            nav_counter +=1
            # if the navstack state is preempted, we will terminate the nav-task
            # so here we should have a more outcomes--'preempted' to make the state 
            # jump out the statemachine of the higher level
            # donnot forget to refresh the preempted flag to make the cocurrence execute more safely
            # TODO:add the outcome 'preempted' and modify all the python-snippet
            # if self.preempt_requested():
            #     self.nav_client .cancel_goal()
            #     return 'succeeded'
            # else:
            #     pass
            rospy.sleep(0.5)
        if self.nav_client.get_goal_status_text() == 'Goal reached.' :
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'
    
#  simple state used for xm_speak ,we use the espeak ,but very un-lovely -_-
class Speak(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['sentences'])
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try :
            self.string_ = str(userdata.sentences)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                subprocess.call("espeak -vf5 -s 100 '%(a)s'"%{'a':str(self.string_)} , shell = True)
                print self.string_
            except:
                return 'aborted'
            else:
                return 'succeeded'

# simple state used to invoke the /move service to justfy the xm position
# it contains all the /move service call state

class SimpleMove(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        input_keys=['point'])
        self.move_client = rospy.ServiceProxy('/move',xm_Move)
        self.move_client.wait_for_service(timeout= 25.0)

    def execute(self,userdata):
        move_value = xm_MoveRequest()
        try:
            move_value.position = userdata.point
        except:
            rospy.logerr('No param specified')
            return 'error'
        print str(move_value)
        result =self.move_client.call(move_value)
        
        if result.arrived ==True :
            return 'succeeded'
        else:
            return 'aborted'
    
# armcmd should be written into a unified state for 3 different poses
# nav_ps is just used for making the arm understand this action is nav_pose instead of error,
# so it must is specified as true
class ArmCmd(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=[ 'succeeded', 'aborted', 'error'],
                        io_keys=['mode','arm_ps'])
        self.xm_arm_client = rospy.ServiceProxy('arm_stack', xm_PickOrPlace)
        # make sure that the PointStamped used for arm pick_or_place is in the frame base_footprint
        self.tf_listener = tf.TransformListener()
        
    def execute(self, userdata):
        rospy.loginfo('Moving arm')
        try:
            getattr(userdata,'mode')
            getattr(userdata,'arm_ps')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
           
            req = xm_PickOrPlaceRequest()
            req.action = userdata.mode
            self.arm_ps_ =PointStamped()
            if userdata.arm_ps.header.frame_id is not '':
                rospy.loginfo('frame checked')
                self.arm_ps_ = self.tf_listener.transformPoint('base_link',userdata.arm_ps)
            else:
                pass
            req.goal_position = self.arm_ps_
            res =self.xm_arm_client.call(req)
            if res.result ==True:
                return 'succeeded'
            else:
                return 'aborted'

# simple state use for delay some time
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"


# monitor state used for detect if the door is open and xm want to go to there ^_^
class DoorDetect():
    def __init__(self):
        self.door_detect_ = MonitorState('DoorState', Bool, self.door_cb,max_checks =1)

    def door_cb(self,userdata,msg):
        if msg.data == True:
            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
            req = EmptyRequest()
            res = clear_client.call(req)
            return False
        else:
            return True

# anwser the qusetions with the people
# the anwser is spell error ->_<-
class Anwser(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted'])
        self.anwser_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    
    def execute(self,usedata):
        try:
            subprocess.call('xterm -e rosrun xm_speech xm_speech_client.py &', shell=True)
            # res =self.anwser_client.call(command =1)
        except:
            return 'aborted'
        else:
            self.anwser_client.wait_for_service(timeout =10)
            self.anwser_client.call(command =1)
            return 'succeeded'


# state use for object reco
# because in different task smach ,this state may make different tasks,so donnot 
# write this state class in the common_lib.py
class FindObject(State):
    def __init__(self):
        State.__init__(self,
                         outcomes=['succeeded','aborted','error'],
                         input_keys=['name'],
                         output_keys =['object_pos'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()
       
    def execute(self, userdata):
        goal = Point()
        try:
            name = userdata.name
        except:
            rospy.logerr('No param specified')
            return 'error'
        for i in range(5):
            try:

                req = xm_ObjectDetectRequest()
                req.object_name = name
                res = self.xm_findobject.call(req)
                if len(res.object)!=0:
                    break
            except:
                return 'aborted'
        if i==2:
            return 'aborted'
        #   object_pos is PointStamped
        object_pos = PointStamped()
      
        object_pos.point.x = res.object[0].pos.point.z -0.11
        object_pos.point.y = res.object[0].pos.point.x -0.125
        object_pos.point.z = 0.917-res.object[0].pos.point.y 
        object_pos.header.frame_id = 'base_link'
       
        userdata.object_pos = object_pos
     
        # userdata.object_pos = res.object[0].pos
        # output_keys cannot be read     
        # print userdata.object_pos
        return 'succeeded'

# this state is used for justfy the position when need to execute arm-stack
# the PointStamped() object_pos have been justfied in the last state, so donnot do it again in this state
class PosJustfy(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys=['object_pos'],
                        output_keys =['pose'])

        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata,'object_pos')
        except:
            rospy.logerr('No params specified')
            return 'error'
        
        object_pos = self.tf_listener.transformPoint('base_link',userdata.object_pos)
        # data deal

        pos_xm = object_pos
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.7*cos(angle)
        person_y = person_y -0.7*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        # init the stamped of the Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        q_angle = quaternion_from_euler(0,0,angle)
        q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
        # if error can directly raise the interupt
        rospy.logwarn('wait for tf succeeded ')    

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        userdata.pose = Pose(pos_xm.point,qs.quaternion)
        return 'succeeded'