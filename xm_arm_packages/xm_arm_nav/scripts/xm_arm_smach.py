#!/usr/bin/env python
#encoding:utf8
import rospy
from smach import State, StateMachine,UserData
from xm_msgs.srv import *
from xm_msgs.msg import *
import math
import tf
from geometry_msgs.msg import *
from control_msgs.msg import *
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name
from std_srvs.srv import Empty,EmptyRequest
import subprocess
from copy import deepcopy

# 机械臂抓取和放置操作具体实现代码
# simple smach use for pick or place stacks
# because the collision of moveit I think may make the planning more difficult, so I will directly control the arm in the end of 
# whole operation
# so in my opinion, make the gripper for a seperate plan_group is unnecessary
# when using the fsm to design your program,sometimes writing the meta-package firstly may do well

# the MoveitArm is used for moveit arm_tasks
# this snippet is the core-code for the whole arm-stack
middle_joints = list()
grasp_joints = list()
class PreArm(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['arm_ps','arm_mode'])
        self.tf_listener = tf.TransformListener()
        self.ik_client = rospy.ServiceProxy('xm_ik_solver',xm_SolveIK)

    def execute(self,userdata):
        global middle_joints, grasp_joints
        try:
            getattr(userdata, 'arm_ps')
            getattr(userdata,'arm_mode')
        except:
            rospy.logerr('No parms specified')
            return 'error'
        if userdata.arm_mode ==0:
            xm_arm_moveit_name('nav_pose')
            return 'succeeded'
        else:
            pass
        # print userdata.arm_ps
        service_bool = self.ik_client.wait_for_service(timeout=10)
        if service_bool ==False:
            rospy.logerr('time-out, ik server is no avaiable')
            return 'aborted'
        else:
            ik_req = xm_SolveIKRequest()
            ik_req.goal = deepcopy(userdata.arm_ps)
            ik_req.goal.point.x -=0.15
            ik_res = self.ik_client.call(ik_req)
            self.ik_client.close()
            if ik_res.result ==True:
                solution_list = list(ik_res.solution)
                solution_list[1] = 1.40  
                if solution_list[4]<2.1:
                    solution_list[4]= 2.0

                middle_joints = deepcopy(solution_list)
                xm_arm_moveit_level(solution_list)
                rospy.logwarn('moveit task compelete')
                rospy.sleep(1.0)
                # JudgeMent the moveit is succeeded?
                (tran,rot) =self.tf_listener.lookupTransform('base_link','gripper_link',rospy.Time())
                if (tran[2] - userdata.arm_ps.point.z)<-0.05:# too low we think is not safe for the following operation
                    rospy.logwarn('moveit may be over, donnot use arm again')
                    return 'aborted'
                else:
                    pass
                solution_list = list(ik_res.solution)

                if solution_list[4] <2.1:
                    solution_list[4] = 2.0
                
                grasp_joints = deepcopy(solution_list)
                lifting_value = solution_list[0]
                joint_value= solution_list[1:]
                arm_controller_level(joint_value)
                # move the lifting 
                lifting_controller_level(lifting_value)
                return 'succeeded'
            else:
                return 'aborted'




# you can reference the move_arm.py in the handsfree_hw package
# the fixed distance is 0.2 may be better for the moveit_planning
# the distance should be changed after testing
# here we will to grasp the target accurately
class ArmDirect(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                            input_keys=['gripper_mode','arm_ps'])
       
        self.ik_client = rospy.ServiceProxy('xm_ik_solver',xm_SolveIK)
        self.tf_listener =tf.TransformListener()

    def execute(self,userdata):
        global middle_joints, grasp_joints
        try:
            getattr(userdata,'gripper_mode')
            getattr(userdata,'arm_ps')
        except:
            rospy.logerr('No params specified')
            return 'error'
        if userdata.gripper_mode ==0:
            rospy.logerr('No need to move')
            return 'aborted'
        # print userdata.arm_ps
        service_bool =self.ik_client.wait_for_service(timeout=10)
        if service_bool ==False :
            rospy.logerr('time out, ik-service is no avaiable')
            return 'aborted'
        else:
            # this is the first motion
            rospy.logwarn('first step')
            ik_req = xm_SolveIKRequest()
            ik_req.goal = deepcopy(userdata.arm_ps)
            ik_res = self.ik_client.call(ik_req)
            ik_bool =True
            self.ik_client.close()
            if ik_res.result ==True:
                solution_list = list(ik_res.solution)          
                lifting_value = solution_list[0]
                joint_value= solution_list[1:]
                arm_controller_level(joint_value)
                # move the lifting 
                lifting_controller_level(lifting_value)
                rospy.sleep(1.0)
            else:
                ik_bool = False
                rospy.logwarn('ik error')

            # this is the second motion, will open or close the gripper
            # true open false close
            rospy.logwarn('second step')
            if userdata.gripper_mode ==0:
                rospy.logerr('you may be joking')
                return 'aborted'
            elif userdata.gripper_mode ==1:
                gripper_mode  = False
            elif userdata.gripper_mode ==2 or userdata.gripper_mode ==3:
                gripper_mode = True
            else:
                rospy.logerr('params error')
                return 'aborted'
            gripper_service_level(gripper_mode)
            rospy.sleep(1.0)
            if ik_bool ==True:

                rospy.logwarn('third step')
                joint_value[2] +=0.3
                joint_value[3] -=0.2
                arm_controller_level(joint_value)
                # move the lifting 
                lifting_controller_level(lifting_value)
                rospy.sleep(1.0)
            else:
                pass

            rospy.logwarn('fourth step')
            lifting_value = grasp_joints[0]
            joint_value = grasp_joints[1:]
            arm_controller_level(joint_value)
            # move the lifting_link
            lifting_controller_level(lifting_value)
            rospy.sleep(1.0)
                
            # this is the fourth motion, will make the arm back
            rospy.logwarn('fifth step')   
            lifting_value = middle_joints[0]
            joint_value= middle_joints[1:]
            arm_controller_level(joint_value)
            # move the lifting
            lifting_controller_level(lifting_value)
            rospy.sleep(1.0)
            return 'succeeded'




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


# simple state used for generating obstacles in the end of gripper
class ObstacleDeal(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                            input_keys=['gripper_mode'])
        self.obstacle_client = rospy.ServiceProxy('xm_obstacles',xm_Obstacles)
    
    def execute(self,userdata):
        try:
            getattr(userdata,'gripper_mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        # if gipper_mode is 0 should throw error
        # elif gripper_mode is 1 should gen obstacle
        # elif gripper_mode is 2 should remove the obstacle
        obstacle_req = xm_ObstaclesRequest()
        target_name = 'target'
        target_size = [0.05,0.05,0.23]
        obstacle_req.size = target_size
        obstacle_req.name = target_name
        if userdata.gripper_mode ==0:
            rospy.logerr('you may be kidding')
            return 'aborted'
        elif userdata.gripper_mode ==1:
            # gen the obstacle
            obstacle_req.deal_method =2
            obstacle_res= self.obstacle_client.call(obstacle_req)
            if obstacle_res.result ==True:
                return 'succeeded'
            else:
                return 'aborted'
        elif userdata.gripper_mode ==2 or userdata.gripper_mode ==3:
            obstacle_req.deal_method =3
            obstacle_res= self.obstacle_client.call(obstacle_req)
            if obstacle_res.result ==False:
                return 'succeeded'
            else:
                return 'aborted'
        else:
            return 'aborted'

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

# create the Octomap use the first 30 PointCloud data
class CreatMap(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            subprocess.call('rosrun libfreenect2_ros_grabber libfreenect2_ros_grabber ',shell=True)
            rospy.sleep(2.0)
        except:
            return 'aborted'
        else:
            return 'succeeded'


# justfy the arm-mode to save time and success rate
class ChangeMode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error','preempted'],
                        input_keys=['arm_mode'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'arm_mode')
        except:
            rospy.logwarn('No params specified')
            return 'error'
        if userdata.arm_mode ==3:
            rospy.logwarn('we just want to deliver the target to person, so why to use moveit ^_^')
            return 'aborted'
        if userdata.arm_mode ==0:
            rospy.logwarn('we just want to init the arm pose, only do one pose')
            return 'preempted'
        else:
            rospy.logwarn('we need to avoid the obstacles, so use the moveit')
            return 'succeeded'

class xm_arm_smach():
    def __init__(self):
        rospy.init_node('xm_arm_smach')
        rospy.on_shutdown(self.shutdown)
        self.arm_stack_service  = rospy.Service('arm_stack',xm_PickOrPlace,self.callback)
        self.sm_ArmStack = StateMachine(outcomes=['succeeded','aborted','error'],
                                            input_keys=['arm_point','arm_mode'])
        #there will be no info in the registered of statemachine  
        # we should make the target be a fit-fixed distance with the robot
        # we make the rules:
        # the target should be 1m with the base_link
        # the arm should be 0.2m away from the target
        # all the state which using the moveit should be 'aborted' if failed
        # we should notice that, sometimes we need to give the target we get to person, here we donnot need to 
        # avoid the obstacle

        with self.sm_ArmStack:
            
            StateMachine.add('EASY',
                                ChangeMode(),
                                remapping={'arm_mode':"arm_mode"},
                                transitions={'succeeded':'CREAT_MAP','aborted':'ARM_DIRECT','preempted':'ARM_INIT', 'error':'error'})
            StateMachine.add('CREAT_MAP',
                                CreatMap(),
                                transitions={'succeeded':'INIT_POSE','aborted':'aborted'})
            self.sm_ArmStack.userdata.arm_mode_1 =0
            StateMachine.add('INIT_POSE',
                                PreArm(),
                                remapping={'arm_ps':'arm_point','arm_mode':'arm_mode_1'},
                                transitions={'succeeded':'ARM_MOVEIT','aborted':'ARM_INIT' ,'error':'error'}
                                )

            StateMachine.add('ARM_MOVEIT',
                                PreArm(),
                                remapping={'arm_ps':'arm_point','arm_mode':'arm_mode'},
                                transitions={'succeeded':'ARM_DIRECT','aborted':'ARM_INIT','error':'error'})
            
            StateMachine.add('ARM_DIRECT',
                                ArmDirect(),
                                transitions={'succeeded':'OBSTACLE_DEAL','aborted':'aborted','error':'error'},
                                remapping={'gripper_mode':'arm_mode','arm_ps':'arm_point'})
            
            StateMachine.add('OBSTACLE_DEAL',
                                ObstacleDeal(),
                                remapping={'gripper_mode':'arm_mode'},
                                transitions={'succeeded':'WAIT','aborted':'WAIT','error':'error'})

            self.sm_ArmStack.userdata.rec = 2.0
            StateMachine.add('WAIT',
                                Wait(),
                                remapping={'rec':'rec'},
                                transitions={'succeeded':'ARM_INIT','error':'error'})
            StateMachine.add('ARM_INIT',
                                PreArm(),
                                remapping={'arm_ps':'arm_point','arm_mode':'arm_mode_1'},
                                transitions={'succeeded':'succeeded','aborted':'aborted' ,'error':'error'})                                
            
            rospy.spin()

    def shutdown(self):
        rospy.logwarn('byebye^_^')

    def callback(self,req):
        self.arm_point = req.goal_position
        self.arm_mode  = req.action
        sm_result = self.sm_ArmStack.execute(parent_ud ={'arm_point':self.arm_point,'arm_mode':self.arm_mode})
        res = xm_PickOrPlaceResponse()        
        if sm_result == 'succeeded':
            
            rospy.logwarn('arm_stack succeeded')
            res.result =True
        else:
            rospy.logwarn('moveit failed, please justfy the position of robot')
            res.result =False
        return res

if __name__ =="__main__":
    try:
        xm_arm_smach()
    except KeyboardInterrupt:
        pass
