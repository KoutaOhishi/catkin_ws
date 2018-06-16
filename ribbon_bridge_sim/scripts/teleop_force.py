#!/usr/bin/env python
#coding: utf-8
import rospy

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.srv import *

def Add_force():
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    body_name = "tug_boat::body"
    reference_frame = "world"

    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 500
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 1000

    duration = rospy.Duration(.3)


    try:
        apply_body_wrench(body_name=body_name,
        reference_frame=reference_frame,
        wrench=wrench,
        duration=duration)
        #print res

    except rospy.ServiceException as e:
        print e

def main():
    Add_force()

if __name__ == "__main__":
    rospy.init_node("Add_force", anonymous=True)
    main()
