#!/usr/bin/env python
#coding: utf-8
import rospy
import sys
from subprocess import *

import tf
from ribbon_bridge_sim.msg import *
from gazebo_msgs.msg import *

#global
Model_list = []

def cb(msg):
    global Model_list
    Model_list = msg.name

def main():
    global Model_list
    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, cb)

    while not rospy.is_shutdown():
        print len(Model_list)

        if len(Model_list) == 0:
            pass

        else:
            for i in range(len(Model_list)):
                modelname = str(Model_list[i])
                print modelname
                Popen(["python", "/home/rg26/catkin_ws/src/ribbon_bridge_sim/scripts/tf_argv.py", modelname])
            break


if __name__ == "__main__":
    rospy.init_node("tf_mul")
    main()
