#!/usr/bin/env python
# encoding:utf8
# axis: x->front y->left z->head
# this ik-solver is compeleted on 08/06/2017 ^_^

import rospy
import tf
import math
from xm_msgs.srv import *
import subprocess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from std_msgs.msg import *

# python实现的ik解算器，解算是基于当前的位置建立的坐标系，所以不是绝对坐标系解算

class XmArmIkServer():
    def __init__(self):
        rospy.init_node('xm_arm_ik_server')
        rospy.on_shutdown(self.shutdown)
        # basic geometry information of arm
        self.arm_joint = ['Joint_lifting','Joint_waist','Joint_big_arm','Joint_fore_arm','Joint_wrist']
        # strange error when using the arm_tolerance in the data_check function
        self.arm_tolerance = {'max':[0.09,1.57,1.57,2.10,2.20],
                              'min':[-0.18,-0.35,-1.57,-2.10,-2.20]}
        self.arm_length = {'big_arm_len':0.35,'fore_arm_len':0.25,'gripper_len':0.25}
        self.ik_bool =True
        self.ik_message = 'ik succeeded'
        #basic kinematics information of arm
        # the length of shrink big_arm and fore_arm
        self.arm_min_len = 0.3347312354710866
        self.arm_max_len = 0.6
        # ros service interface
        self.ik_server = rospy.Service('xm_ik_solver',xm_SolveIK,self.call_back)
        # ros tf interface setup
        self.tf_listener = tf.TransformListener()
        rospy.logwarn('start ik_server')
        
        rospy.spin()

    def call_back(self,req):
        res = xm_SolveIKResponse()
        joint_solution = {'Joint_lifting':0,
                          'Joint_waist':0,
                          'Joint_big_arm':0,
                          'Joint_fore_arm':0,
                          'Joint_wrist':0}

        if req.goal.header.frame_id is None:
            rospy.logerr('No frame specified')
            res.result = False
            res.message ="No frame specified"
        else:
            # first we will deal the position for the arm_stack,due to we need the end-efforter should be horizoned
            msg = rospy.wait_for_message('/joint_states',JointState,timeout=10.0)
            current_joint_state = msg.position
            new_header = Header()
            new_header.frame_id = req.goal.header.frame_id
            # only need the first 5 elements
            current_joint_state = current_joint_state[:5]
            req.goal.header = new_header
            waist_ps= self.tf_listener.transformPoint('waist_link',req.goal)
            # calculate the joint-angle of waist-arm
            angle = math.atan2(waist_ps.point.y, waist_ps.point.x)
            # depend the current_big_arm_link to calculate the joint_value is difficult, so I will use the position only to 
            # calcuate the joint_value
            joint_solution['Joint_waist'] = angle + current_joint_state[1]
            if self.data_check('Joint_waist',joint_solution['Joint_waist']) ==False:
                self.ik_bool =False
                rospy.logwarn('waist_link out range')
                self.ik_message ='waist_link out range'
            # here we will modify the target_pos with the gripper_len
            waist_ps.point.x -=self.arm_length['gripper_len']*math.cos(angle)
            waist_ps.point.y -=self.arm_length['gripper_len']*math.sin(angle) 
            (trans,rot) = self.tf_listener.lookupTransform('waist_link','big_arm_link',rospy.Time(0))
            target_state = Point()
            target_state.x= waist_ps.point.x -trans[0]
            target_state.y= waist_ps.point.y -trans[1]
            target_state.z= waist_ps.point.z -trans[2]
            
            # the target_state is the Point information of the target in the big_arm_link
            # calculate the 2d-distance between target and the big_arm_link
            temp_len = math.sqrt(target_state.x**2 +target_state.y**2)
            # out_len is the 2d-distance between big_arm_link and the waist_link
            out_len = 0.130
            
            target_len_temp = math.sqrt(temp_len**2 - (out_len*math.sin(angle))**2) + out_len*math.cos(angle)
            #target_len_temp is the 2d-distance between the big_arm_link and the target after waist_link move    
            target_len_temp -=out_len

            # this is the 3d-distance
            target_len = math.sqrt(target_len_temp**2 + target_state.z**2)

            # the lifting_link should not direct add or minus the height of the arm_ps.point.z
            # after deal the bug of direct ik, I notice the ik-solver have trouble with lifting_link

            if target_len<self.arm_max_len and target_len>self.arm_min_len:
                # the arm_len is in the range without moving of lifting_link
                temp_len = target_state.z
                joint_solution['Joint_lifting'] = current_joint_state[0]
                
            elif target_len>self.arm_max_len:
                rospy.logwarn('the target is so far, I will move the lifting_link to catch it')
                if self.arm_max_len <target_len_temp:
                    self.ik_bool =False
                    rospy.logwarn('too far cannot receive with lifting_link')
                    self.ik_message ='too far cannot receive with lifting_link'
                else:  
                    # before here the temp_len is the 2d-distance between target and the big_arm_link
                    temp_len = math.sqrt(self.arm_max_len**2 - target_len_temp**2)
                    if target_state.z<0:
                        temp_len = -temp_len
                    else:
                        pass
                    move_len = target_state.z - temp_len

                    joint_solution['Joint_lifting'] = move_len + current_joint_state[0]
                    if self.data_check('Joint_lifting',joint_solution['Joint_lifting']) ==False:
                        self.ik_bool =False
                        rospy.logwarn('lifting_link up out range')
                        self.ik_message ='lifting_link up out range'
                    else:
                        pass
               
            elif target_len<self.arm_min_len:
                rospy.logwarn('the target is so close, I will move the lifting_link to catch it')
                
                # before here the temp_len is the 2d-distance between target and the big_arm_link     
             
                temp_len = math.sqrt(self.arm_min_len**2 - target_len_temp**2)
                if target_state.z<0:
                    temp_len = -temp_len
                else:
                    pass
                move_len = target_state.z - temp_len

                joint_solution['Joint_lifting'] = move_len + current_joint_state[0]
                if self.data_check('Joint_lifting',joint_solution['Joint_lifting']) ==False:
                    self.ik_bool =False
                    rospy.logwarn('lifting_link down out range')
                    self.ik_message ='lifting_link down out range'
                else:
                    pass

            else:
                pass
            # the calculate of joint_lifting is end, now we will calculate the big_arm and fore_arm
            # here we should consider the angle of current_big_arm_link joint when check the data

            arm_joints = self.calculate_angle(target_len_temp, temp_len)

            joint_solution['Joint_big_arm'] = arm_joints[0] 
            joint_solution['Joint_fore_arm'] = arm_joints[1]
            joint_solution['Joint_wrist'] =-(joint_solution['Joint_big_arm']+ joint_solution['Joint_fore_arm'])

            res.result = self.ik_bool
            res.message =self.ik_message
            # the dict may in a strange order
            res.solution.append(joint_solution['Joint_lifting'])
            res.solution.append(joint_solution['Joint_waist'])
            res.solution.append(joint_solution['Joint_big_arm'])
            res.solution.append(joint_solution['Joint_fore_arm'])
            res.solution.append(joint_solution['Joint_wrist'])
            
            print res
            return res
            

    def data_check(self,joint_name, joint_value):
        print str(joint_name) +':'+ str(joint_value)
        if joint_name =='Joint_lifting':
           
            return (joint_value> -0.18)&(joint_value <0.09)
        elif joint_name =='Joint_waist':
            return (joint_value> -0.35)&(joint_value <1.57)   
        elif joint_name =='Joint_big_arm':
            return (joint_value> -1.57)&(joint_value <1.57)   
        elif joint_name =='Joint_fore_arm':
            return (joint_value> -2.10)&(joint_value <2.10)   
        elif joint_name =='Joint_wrist':
            return (joint_value> -2.20)&(joint_value <2.20)     
        else:
            return False    

    def calculate_angle(self,distance, height):
        #this function is used for calculating the angle of 2-dof arm with specified distance and height
        # this ik may get 2-solution, and we will choose only one of them due to the ik_data_check
        print 'distance'+':'+str(distance)
        print 'height' +':' +str(height)
        target_len = math.sqrt(distance**2 + height**2)
        acos_angle = (self.arm_length['big_arm_len']**2 + self.arm_length['fore_arm_len']**2 - target_len**2)\
                                                    /(2*self.arm_length['big_arm_len']*self.arm_length['fore_arm_len'])
        # 1-solution
        if acos_angle>=1.0:
            acos_angle =1.0
        elif acos_angle<=-1.0:
            acos_angle =-1.0
        else:
            pass
        joint_fore_arm = math.acos(acos_angle) - math.pi
        joint_stick = math.atan2(height, distance)

        acos_angle = (self.arm_length['big_arm_len']**2 + target_len**2 - self.arm_length['fore_arm_len']**2)\
                                            /(2*self.arm_length['big_arm_len']*target_len)
        # we should make a over limit deal
        if acos_angle>=1.0:
            acos_angle =1.0
        elif acos_angle<=-1.0:
            acos_angle =-1.0
        else:
            pass

        joint_big_arm = math.acos(acos_angle) + joint_stick 
        if self.data_check('Joint_big_arm',joint_big_arm)==True and self.data_check('Joint_fore_arm',joint_fore_arm)==True:
            # if without [], the return value will be tuple
            return [joint_big_arm,joint_fore_arm]
        else:
            # 2-solution
            joint_fore_arm = -joint_fore_arm 
            joint_big_arm =  joint_stick*2 - joint_big_arm 
            if self.data_check('Joint_big_arm',joint_big_arm)==True and self.data_check('Joint_fore_arm',joint_fore_arm)==True:
                # if without [], the return value will be tuple
                return [joint_big_arm,joint_fore_arm]
            else:
                self.ik_bool = False
                rospy.logwarn('joint_big_arm and joint_fore_arm out range')
                self.ik_message = 'joint_big_arm and joint_fore_arm out range'
                # the data is out of range but also return something
                return [joint_big_arm,joint_fore_arm]                

    def shutdown(self):
        rospy.logerr('ik_server is over, byebye^_^')

if __name__ == "__main__":
    XmArmIkServer()




