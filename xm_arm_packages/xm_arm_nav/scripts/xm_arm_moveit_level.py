#!/usr/bin/env python
#encoding:utf8
import rospy, sys
import moveit_commander
from xm_msgs.srv import *
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def xm_arm_moveit_level(joint_values):
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        xm_arm_ = moveit_commander.MoveGroupCommander('xm_arm')
        # set the goal_joint_tolerance and make the value bigger may make the motion-plan easier, but 
        # will sacrifice some accuracy, if moveit-move is not the last move of arm, this value if 0.01 may
        # be ok(accuracy may be low of course) 
        xm_arm_.set_goal_joint_tolerance(0.01)
        
        # make the motion-plan allow replan, but usually if the motion-planner misunderstand you are in the collison
        # not existing in fact, plan may be failed each time
        xm_arm_.allow_replanning(True)

        # this is not the fastest motion-planner, but it is not so sensitive to the obstacle(compared to the RRT)
        xm_arm_.set_planner_id('SBLkConfigDefault')
        # set the plan-goal
        # due to we use our custom-ik-solver, so we cannot use the set_position_value_target() due to kdl-solver
        # cannot be used for our 4-dof arm
        # but you can make our custom-ik-solver to be a plugin of moveit ik-solver, and you can choose it 
        # in the moveit-setup-assistant
        xm_arm_.set_start_state_to_current_state()
        xm_arm_.set_joint_value_target(joint_values)
        traj = xm_arm_.plan()
        # in fact, the result may be True though the plan be failed ,but the traj will be empty if failed 
        result = xm_arm_.execute(traj)
    except:
        return False
    else:
        return result

# set arm to the named-pose in the srdf
def xm_arm_moveit_name(pose_name):
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        xm_arm_ = moveit_commander.MoveGroupCommander('xm_arm')
        xm_arm_.set_goal_joint_tolerance(0.01)
        xm_arm_.allow_replanning(True)
        xm_arm_.set_planner_id('SBLkConfigDefault')
        xm_arm_.set_start_state_to_current_state()
        xm_arm_.set_named_target(pose_name)
        xm_arm_.go()
    except:
        return False
        rospy.sleep(1.0)
    else:
        
        return True





