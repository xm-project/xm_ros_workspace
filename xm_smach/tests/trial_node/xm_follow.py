#!/usr/bin/env python
#encoding:utf8
import rospy
from xm_msgs.msg import *

class xm_follow():
    def __init__(self):
        rospy.init_node('xm_follow')
        rospy.on_shutdown(self.shutdown)
        publisher = rospy.Publisher('follow',xm_FollowPerson,queue_size=5)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = xm_FollowPerson()
            msg.position.header.frame_id ='camera_link'
            msg.position.point.x =0.3
            msg.position.point.y =0.0
            msg.position.point.z =0.7
            publisher.publish(msg)
            rate.sleep()

    def shutdown(self):
        rospy.logerr('fuck you bye bye^_^')

if __name__ =="__main__":
    xm_follow()