#!/usr/bin/env python
#encoding:utf8
# in this test we should test something:
# 1st the appropriate distance for pick-task
# 2nd need to move_back for the place task?
import rospy
from xm_msgs.srv import *
from xm_msgs.msg import *
from smach import State,UserData,StateMachine
from geometry_msgs.msg import *
import tf
import math
import subprocess

# 这是一个比较常用的机械臂测试脚本，用来和视觉一起测试识别物品并抓取的任务

object_name = 'milk tea'

# this the state used to recognize the object of the object_name(global variable) 
# and return its PointStamped() information in the frame"camera_link"
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
        # subprocess.call("xterm -e rosrun xm_vision image_test &",shell=True)
        self.xm_findobject.wait_for_service(timeout=60.0)
        
        for i in range(3):
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
        self.tf_listener.waitForTransform('base_link','camera_link',rospy.Time(),rospy.Duration(60.0))
        rospy.logwarn('tf wait succeeded')

        # pay attention to this, this is caused by the low-accuracy of the arm, 
        # if the arm-control is accurate, the 2 ways following may be all ok
         
        # object_pos = self.tf_listener.transformPoint('base_link',res.object[0].pos)
        # rospy.logwarn('tf transform succeeded')
        # # the pos may be some different...
        # object_pos.point.y = -object_pos.point.y
        # object_pos.point.y -=0.125
        # object_pos.point.x -=0.11
        # object_pos.point.z =0.02
        object_pos = PointStamped()
        (tran,rot) = self.tf_listener.lookupTransform('base_link','camera_link',rospy.Time(0))
        object_pos.point.x = res.object[0].pos.point.z -0.16
        object_pos.point.y = res.object[0].pos.point.x -0.10
        object_pos.point.z = 0.917-res.object[0].pos.point.y 
        object_pos.header.frame_id = 'base_link'
        
        userdata.object_pos = object_pos
        print object_pos
        
        return 'succeeded'

# invoking the arm service to implement the grasp and pick tasks 
class GraspSmach(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                            input_keys=['arm_ps','arm_mode'])
        self.arm_client = rospy.ServiceProxy('arm_stack',xm_PickOrPlace)
    def execute(self,userdata):
        try:
            getattr(userdata,'arm_ps')
            getattr(userdata, 'arm_mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        req = xm_PickOrPlaceRequest()
        req.action = userdata.arm_mode
        req.goal_position = userdata.arm_ps
        res = self.arm_client.call(req)
        if res.result == False:
            return 'aborted'
        else:
            return 'succeeded'


# TODO:if arm_ik_server failed, we may need to justify the position of robot, but this way cannot avoid the 
# obstacles in the ground, so this way should be improved
class SimpleMove(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        input_keys=['point'],
                        io_keys=['arm_ps'])
        self.move_client = rospy.ServiceProxy('/move',xm_Move)
        self.move_client.wait_for_service(timeout= 25.0)

    def execute(self,userdata):
        move_value = xm_MoveRequest()
        try:
            move_value.position = userdata.point
            getattr(userdata,'arm_ps')
        except:
            rospy.logerr('No param specified')
            return 'error'
        print str(move_value)
        result =self.move_client.call(move_value)
        
        if result.arrived ==True :
            userdata.arm_ps.point.x-=userdata.point.x
            return 'succeeded'
        else:
            return 'aborted'


        
# simple move_back may cause problem, so can we use the nav?
# but the nav may make the grasp task for more time to execute
# this test pick first and place 

class GraspTask():
    def __init__(self):
        global object_name
        rospy.init_node('grasp_task')
        rospy.logwarn('grasp test is beginning')
        self.sm_Grasp = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.sm_Grasp:
            self.sm_Grasp.userdata.arm_ps = PointStamped()
            
            self.sm_Grasp.userdata.name = object_name
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                remapping={'name':'name','object_pos':'arm_ps'},
                                transitions={'succeeded':'GRASP','aborted':'FIND_OBJECT','error':'error'})

            self.sm_Grasp.userdata.arm_mode_1 =1
            StateMachine.add('GRASP',
                                GraspSmach(),
                                remapping={'arm_ps':'arm_ps','arm_mode':'arm_mode_1'},
                                transitions={'succeeded':'PLACE','aborted':'aborted','error':'error'})
            # self.sm_Grasp.userdata.move_point = Point(-0.1,0.0,0.0)
            # StateMachine.add('MOVE_BACK',
            #                     SimpleMove(),
            #                     remapping={'point':'move_point'},
            #                     transitions={'succeeded':'PLACE','aborted':'PLACE','error':'error'})
            self.sm_Grasp.userdata.arm_mode_2 = 2
            StateMachine.add('PLACE',
                                GraspSmach(),
                                transitions={"succeeded":'succeeded','aborted':'aborted','error':'error'},
                                remapping={'arm_ps':'arm_ps','arm_mode':'arm_mode_2'})

        outcome = self.sm_Grasp.execute()


if __name__ =="__main__":
    try:
        GraspTask()
    except KeyboardInterrupt:
        pass
            
            