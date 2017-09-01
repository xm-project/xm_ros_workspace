#!/usr/bin/env python
#encoding:utf8
import rospy, sys
import moveit_commander
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from xm_msgs.srv import *

# 这个脚本用来生成桌型的固定障碍物和方形的附着于机械爪的障碍物

class ObstaclesGen:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('obstacles_gen')
        # move group
        xm_arm_ = moveit_commander.MoveGroupCommander("xm_arm")
        self.end_effector_link =xm_arm_.get_end_effector_link()
        print self.end_effector_link
        self.scene_interface = moveit_commander.PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(0.6)
        # planningsceneinterface
        self.scene_interface.remove_attached_object(self.end_effector_link, name = None)
        self.scene_interface.remove_world_object(name = None)#remove all thing when starting
        self.obstacles_pub = rospy.Service('xm_obstacles',xm_Obstacles, self.service_callback)
        rospy.spin()
        # log out
        rospy.logwarn("bye bye ^^")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os.exit(0)
    
    def service_callback(self, req):
        
        #check value when create thing 
        res = xm_ObstaclesResponse()
        reference_frame = 'base_link'
        thing_name = str(req.name)
        deal_method_ = req.deal_method
        # deal_method_ 0 ->remove
        #              1 ->add
        #              2 ->attached
        #              3 ->deattached and remove
        if deal_method_ ==0:
            self.scene_interface.remove_world_object(name = thing_name)
            
        else:
            if deal_method_ ==1:
                thing_size = list(req.size)
                thing_locate = list(req.locate)
                if len(thing_size)!=3 or len(thing_locate)!=3:
                    rospy.logerr('value size error , i cannot create thing -_-.')
                    res.result = False
                else :
                    thing_pose = PoseStamped()
                    thing_pose.header.frame_id = reference_frame
                    thing_pose.pose.position.x = thing_locate[0]
                    thing_pose.pose.position.y = thing_locate[1]
                    thing_pose.pose.position.z = thing_locate[2]
                    thing_pose.pose.orientation.w = 1.0
                    self.scene_interface.add_box(thing_name, thing_pose, thing_size)
                    self.setColor(thing_name, 0.8,0,0,1.0)
                    self.sendColors()
                    rospy.sleep(3.0)
            else :
                if deal_method_ ==2:
                    rospy.loginfo("attached sth")
                    target_size = list(req.size)
                    print str(target_size)
                    # target_locate is not needed 
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = self.end_effector_link
                    self.scene_interface.attach_mesh
                    target_pose.pose.position.x = target_size[0]/2.0 +0.25
                    target_pose.pose.position.y =0.0
                    target_pose.pose.position.z =0.0
                    target_pose.pose.orientation.x=0
                    target_pose.pose.orientation.y=0
                    target_pose.pose.orientation.z=0
                    target_pose.pose.orientation.w =1
                    self.scene_interface.attach_box(self.end_effector_link,thing_name,target_pose,target_size)
                else :
                    if deal_method_==3:
                       self.scene_interface.remove_attached_object(self.end_effector_link,name = thing_name) 
                       rospy.sleep(0.8)
                       self.scene_interface.remove_world_object(name = thing_name)
                    else:
                        rospy.logerr("i cannot know how you are thinking @_@")
        res.result =True           
        return res


    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)


if __name__ == "__main__":
    try:
        ObstaclesGen()
    except KeyboardInterrupt:
        raise
    
            
         



