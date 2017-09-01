#!/usr/bin/env python
#encoding:utf8
import rospy
import tf
import actionlib
import math
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from std_msgs.msg import String,Int32,Bool
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from math import *
import actionlib
#from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal,FollowJointTrajectoryAction
from trajectory_msgs.msg import *

# 这是控制器层的简单测试脚本，直接通过升降和机械臂的action接口对相应关节进行控制，
# 一般用来简单的测试你拿到的机械臂有没有接好线什么的-_-

if __name__ == '__main__':
    
    rospy.init_node('move_arm_node')
    ###################################this is lifting_link=￣ω￣=
    client = actionlib.SimpleActionClient('/mobile_base/xm_lifting_controller/lifting_cmd', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names.append('Joint_lifting')

    temp_joint=JointTrajectoryPoint()
    temp_joint.positions=[0.0]

    temp_joint.accelerations=[0.0]
    temp_joint.velocities=[0.0]
    temp_joint.effort=[0.0]
    temp_joint.time_from_start=rospy.Duration(secs=2.0)
    # here I find the bug , so the last people should pay attention too
    # the bug is that, the goal.trajectory.points is the list type
    goal.trajectory.points.append(temp_joint)
    # goal.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(secs=1)
    
    client.send_goal(goal)

    
    client.wait_for_result(rospy.Duration.from_sec(5.0))



##########################this is arm=￣ω￣=#####################################
#     client = actionlib.SimpleActionClient('/mobile_base/xm_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#     client.wait_for_server()
#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory.joint_names.append('Joint_waist')
#     goal.trajectory.joint_names.append('Joint_big_arm')
#     goal.trajectory.joint_names.append('Joint_fore_arm')
#     goal.trajectory.joint_names.append('Joint_wrist')
   
#     temp_joint=JointTrajectoryPoint()
#     temp_joint.positions=[0.0,0.0,0.0,0.0]

#     temp_joint.accelerations=[0.0,0.0,0.0,0.0]
#     temp_joint.velocities=[0.0,0.0,0.0,0.0]
#     temp_joint.effort=[0.0,0.0,0.0,0.0]
#     temp_joint.time_from_start=rospy.Duration(secs=2.0)

#     goal.trajectory.points.append(temp_joint)
#     goal.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(secs=1)

#     client.send_goal(goal)
#     result =False
#     while result ==False:

#         result =  client.wait_for_result(rospy.Duration.from_sec(1.0))


