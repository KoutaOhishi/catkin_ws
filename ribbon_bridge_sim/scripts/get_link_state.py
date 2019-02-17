#!/usr/bin/env python
#coding: utf-8
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

def Distance(x1, y1, x2, y2):
    """ ２点間の距離を返す関数 """
    value = pow((x1-x2), 2) + pow((y1-y2), 2)
    return math.sqrt(value)


def Get_state(link_name):
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

    try:
        res = get_link_state(link_name=link_name)
        print res.link_state
        dist = Distance(res.link_state.pose.position.x, res.link_state.pose.position.y, 0.0, 0.0)
        print "中心との距離:%s"%str(round(dist, 2))
        rospy.sleep(1)

    except rospy.ServiceException as e:
        rospy.logerr("Service Exception")

def main():
    rospy.wait_for_service("/gazebo/get_link_state")

    link_name = "tug_boat_0::body"
    while not rospy.is_shutdown():
        Get_state(link_name)


if __name__ == "__main__":
    rospy.init_node("GetLinkState", anonymous=True)
    main()
