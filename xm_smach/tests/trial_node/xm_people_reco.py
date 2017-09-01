#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import xm_People, xm_PeopleRequest,xm_PeopleResponse
from geometry_msgs.msg import *
class xm_people_reco():
    def __init__(self):
        rospy.init_node('xm_people_reco')
        servicer = rospy.Service('people_reco',xm_People,self.callback)
        rospy.loginfo('people_reco service is beginning')
        rospy.spin()


    def callback(self,req):
        res = xm_PeopleResponse()
        if req is not None:
            print str(req)
            if req.command==0:
                people_list = list()
                pos_person = PointStamped()
                pos_person.header.frame_id='camera_link'
                pos_person.point.x=0.7 
                pos_person.point.y= 0.1
                pos_person.point.z= -0.2
                # we specified the 4 data
                people_list.append(pos_person)
                pos_person.point.y+= 0.1
                people_list.append(pos_person)
                pos_person.point.y+= 0.1
                people_list.append(pos_person)
                pos_person.point.y+= 0.1
                people_list.append(pos_person)
                res.pos_person =people_list
            elif req.command==1:
                rospy.loginfo('hehe')
            elif req.command==2:
                people_list = list()
                pos_person = PointStamped()
                pos_person.header.frame_id='camera_link'
                pos_person.point.x=0.7 
                pos_person.point.y= 0.1
                pos_person.point.z= -0.2
                people_list.append(pos_person)
                res.pos_person = people_list
            else:
                raise KeyboardInterrupt
        else:
            raise KeyboardInterrupt
        return res

if __name__=="__main__":
    xm_people_reco()
