#!/usr/bin/env python
#coding: utf-8
import rospy, rospkg
import cv2
import numpy as np
import math
import time

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *
from matplotlib import pyplot as plt

#Global
RibbonBridgeTwist1 = Twist()
RibbonBridgeTwist2 = Twist()
RibbonBridgeTwist3 = Twist()

VelocityList1 = []
VelocityList2 = []
VelocityList3 = []

isRibbonBridgeArrived1 = False
isRibbonBridgeArrived2 = False
isRibbonBridgeArrived3 = False

loop_count = 0
loop_list = []

def model_states_CB(msg):
    global RibbonBridgeTwist1, RibbonBridgeTwist2, RibbonBridgeTwist3

    for i in range(len(msg.name)):
        if msg.name[i] == "tug_boat":
            RibbonBridgeTwist1 = msg.twist[i]
        if msg.name[i] == "tug_boat_0":
            RibbonBridgeTwist2 = msg.twist[i]
        if msg.name[i] == "tug_boat_1":
            RibbonBridgeTwist3 = msg.twist[i]

def is_RibbonBridge1_arrived_CB(msg):
    global isRibbonBridgeArrived1
    isRibbonBridgeArrived1 = msg.data

def is_RibbonBridge2_arrived_CB(msg):
    global isRibbonBridgeArrived2
    isRibbonBridgeArrived2 = msg.data

def is_RibbonBridge3_arrived_CB(msg):
    global isRibbonBridgeArrived3
    isRibbonBridgeArrived3 = msg.data

def update_bridges_velocity():
    global VelocityList1, VelocityList2, VelocityList3
    global loop_count, loop_list

    loop_list.append(loop_count)
    loop_count += 1

    #if isRibbonBridgeArrived1 == False:
    vel = (abs(RibbonBridgeTwist1.linear.x) + abs(RibbonBridgeTwist1.linear.y))/2
    VelocityList1.append(round(vel,1))

    #if isRibbonBridgeArrived2 == False:
    vel = (abs(RibbonBridgeTwist2.linear.x) + abs(RibbonBridgeTwist2.linear.y))/2
    VelocityList2.append(round(vel,1))

    #if isRibbonBridgeArrived3 == False:
    vel = (abs(RibbonBridgeTwist3.linear.x) + abs(RibbonBridgeTwist3.linear.y))/2
    VelocityList3.append(round(vel,1))

def draw_graph():
     plt.plot(loop_list, VelocityList1, color="black", linestyle='--')
     plt.plot(loop_list, VelocityList2, color="black", linestyle=":")
     plt.plot(loop_list, VelocityList3,  color="black")


     plt.xlim(0, loop_count)
     plt.ylim(-0.5, 3.0)
     plt.pause(.001)


def main():
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_CB)

    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge1_arrived", Bool, is_RibbonBridge1_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge2_arrived", Bool, is_RibbonBridge2_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge3_arrived", Bool, is_RibbonBridge3_arrived_CB)

    rospy.sleep(1) #浮橋の位置をsubscribeする時間を確保

    while not rospy.is_shutdown():
        update_bridges_velocity()
        draw_graph()
        time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("DrawVelocityGraph")
    main()
    rospy.spin()
