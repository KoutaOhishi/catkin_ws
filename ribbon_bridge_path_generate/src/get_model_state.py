#!/usr/bin/env python
#coding: utf-8
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

#Global
Model_name = ""

def Distance(x1, y1, x2, y2):
    """ ２点間の距離を返す関数 """
    value = pow((x1-x2), 2) + pow((y1-y2), 2)
    return math.sqrt(value)

def CB(msg):
    if Model_name in msg.name:
        target_index = msg.name.index(Model_name)
        print Model_name
        print msg.pose[target_index]
        print msg.twist[target_index]
        print "-"
        #rospy.sleep(1)
    else:
        rospy.logerr("[%s] is NOT exist."%Model_name)
def main():
    global Model_name

    rospy.Subscriber("/gazebo/model_states", ModelStates, CB)

    Model_name = "tug_boat_0"




if __name__ == "__main__":
    rospy.init_node("GetModelState", anonymous=True)
    main()
    rospy.spin()
