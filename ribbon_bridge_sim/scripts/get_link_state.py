#!/usr/bin/env python
#coding: utf-8
import rospy

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

def Get_state(link_name):
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

    try:
        res = get_link_state(link_name=link_name)
        print res.link_state

    except rospy.ServiceException as e:
        rospy.logerr("Service Exception")

def main():
    rospy.wait_for_service("/gazebo/get_link_state")

    link_name = "tug_boat::body"
    while not rospy.is_shutdown():
        Get_state(link_name)


if __name__ == "__main__":
    rospy.init_node("GetLinkState", anonymous=True)
    main()
