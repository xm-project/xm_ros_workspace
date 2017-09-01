#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import *
import math
import tf
from geometry_msgs.msg import *
from control_msgs.msg import *
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

#这个模块用来封装控制器级别的机械臂关节控制
# 爪子的控制器也可以使用service来代替
 
def joint_check(joint_name, joint_value):
    if joint_name =='Joint_lifting':
        return (joint_value>-0.18)&(joint_value<0.09)
    elif joint_name =='Joint_waist':
        return (joint_value>-0.35)&(joint_value<1.57)
    elif joint_name =='Joint_big_arm':
        return (joint_value>-1.57)&(joint_value<1.57)
    elif joint_name =='Joint_fore_arm':
        return (joint_value>-2.00)&(joint_value<2.00)
    elif joint_name =='Joint_wrist':
        return (joint_value>-2.2)&(joint_value<2.2)
    else:
        return False

def arm_controller_level(joint_value):
    arm_client = actionlib.SimpleActionClient('/mobile_base/xm_arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction)
    service_bool =arm_client.wait_for_server(timeout=rospy.Duration(10))
    joint_names = ['Joint_waist','Joint_big_arm','Joint_fore_arm','Joint_wrist']
    if service_bool ==False:
        rospy.logwarn('time out, no available service exists')
        return False
    else:
        # check the data if avaiable
        if isinstance(joint_value, list) ==False:
            rospy.logwarn('joint_value is not list')
            return False
        else:
            pass
        if len(joint_value)!=4:
            rospy.logwarn('joint_value length is error')
            return False
        else:
            pass
        # the joint_value should in the order of the joint_names
        arm_goal = FollowJointTrajectoryGoal()

        arm_goal.trajectory.joint_names.extend(joint_names)
        # arm_goal.trajectory.joint_names.append('Joint_waist')
        # arm_goal.trajectory.joint_names.append('Joint_big_arm')
        # arm_goal.trajectory.joint_names.append('Joint_fore_arm')
        # arm_goal.trajectory.joint_names.append('Joint_wrist')
        
        arm_joint=JointTrajectoryPoint()
    
        arm_joint.positions= joint_value
        arm_joint.accelerations=[0.0,0.0,0.0,0.0]
        arm_joint.velocities=[0.0,0.0,0.0,0.0]
        arm_joint.effort=[0.0,0.0,0.0,0.0]
        arm_joint.time_from_start=rospy.Duration(secs=2.0)
        arm_goal.trajectory.points.append(arm_joint)

        arm_client.send_goal(arm_goal)
        result = False
        while result ==False:
            result =arm_client.wait_for_result(rospy.Duration.from_sec(1.0))
            print result
        return True

def lifting_controller_level(lifting_value):
    lifting_client = actionlib.SimpleActionClient('/mobile_base/xm_lifting_controller/lifting_cmd', FollowJointTrajectoryAction)
    service_bool =lifting_client.wait_for_server(timeout=rospy.Duration(10))
    if service_bool ==False:
        rospy.logwarn('server timeout ,no avaiable controller')
        return False
    else:
       

        lifting_goal = FollowJointTrajectoryGoal()
        lifting_goal.trajectory.joint_names.append('Joint_lifting')
        lifting_joint=JointTrajectoryPoint()
        lifting_joint.positions.append(lifting_value)
        lifting_joint.accelerations=[0.0]
        lifting_joint.velocities=[0.0]
        lifting_joint.effort=[0.0]
        lifting_joint.time_from_start=rospy.Duration(secs=2.0)
        lifting_goal.trajectory.points.append(lifting_joint)
        lifting_client.send_goal(lifting_goal)    
        result = False
        while result ==False:
            result =lifting_client.wait_for_result(rospy.Duration.from_sec(1.0))
            print result
        return True

def gripper_service_level(gripper_cmd):
    # due to the instability of gripper_controller, we choose the gripper_service to controller the gripper
    gripper_client = rospy.ServiceProxy('/mobile_base/gripper_command',xm_Gripper)
    service_bool =gripper_client.wait_for_service(timeout=10)   
    if service_bool ==False:
        rospy.logwarn('server timeout, no avaiable controller')
        return False
    else:
        gripper_goal = xm_GripperRequest()
        gripper_goal.command = gripper_cmd
        res = gripper_client.call(gripper_goal)
        return res.result 

 
