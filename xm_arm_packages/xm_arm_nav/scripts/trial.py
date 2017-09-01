#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from xm_msgs.srv import *

# 这个测试脚本用来测试obstacles_gen.py的障碍物生成效果，
# 修改deal_method 来实现对障碍物的不同操作
class trial_hehe:
    def __init__(self):
        rospy.init_node('trial_demo')
        client = rospy.ServiceProxy('xm_obstacles', xm_Obstacles)
        hehe = xm_ObstaclesRequest()
        rospy.loginfo("start")
        haha =True
        if haha is True:
            deal_method =1
            hehe.deal_method = deal_method
            hehe.size =[0.3, 1.5, 0.07]
            hehe.locate = [0.95,0,0.465]
            hehe.name = 'table'
            hehe_1 =client.call(hehe)
            # rospy.sleep(1.0)
            # hehe.deal_method = deal_method
            # hehe.size =[0.01, 2.2, 0.5]
            # hehe.locate = [0.65,0,0.35]
            # hehe.name = 'forward'
            # hehe_2 =client.call(hehe)
        else:
            deal_method =2
            hehe.deal_method = deal_method
            hehe.name = 'target'
            hehe.size = [0.05,0.05,0.25]
            hehe_2 =client.call(hehe)
        # Give the scene a chance to catch up
        rospy.sleep(1)
        rospy.loginfo('end')
        rospy.sleep(1.0)
if __name__ =="__main__":
    trial_hehe()
    
    
    