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

"""
Gazeboから得られた浮橋の位置情報を元に、浮橋の移動の軌跡を描画する
"""

#Global
RibbonBridgePose1 = Pose()
RibbonBridgePose2 = Pose()
RibbonBridgePose3 = Pose()

PassengePointList1 = []
PassengePointList2 = []
PassengePointList3 = []

isRibbonBridgeArrived1 = False
isRibbonBridgeArrived2 = False
isRibbonBridgeArrived3 = False

def model_states_CB(msg):
    global RibbonBridgePose1, RibbonBridgePose2, RibbonBridgePose3

    for i in range(len(msg.name)):
        if msg.name[i] == "tug_boat":
            RibbonBridgePose1 = msg.pose[i]
        if msg.name[i] == "tug_boat_0":
            RibbonBridgePose2 = msg.pose[i]
        if msg.name[i] == "tug_boat_1":
            RibbonBridgePose3 = msg.pose[i]

def is_RibbonBridge1_arrived_CB(msg):
    global isRibbonBridgeArrived1
    isRibbonBridgeArrived1 = msg.data

def is_RibbonBridge2_arrived_CB(msg):
    global isRibbonBridgeArrived2
    isRibbonBridgeArrived2 = msg.data

def is_RibbonBridge3_arrived_CB(msg):
    global isRibbonBridgeArrived3
    isRibbonBridgeArrived3 = msg.data

def update_bridges_trajectory():
    global PassengePointList1, PassengePointList2, PassengePointList3

    if isRibbonBridgeArrived1 == False:
        PassengePointList1.append(RibbonBridgePose1.position)

    if isRibbonBridgeArrived2 == False:
        PassengePointList2.append(RibbonBridgePose2.position)

    if isRibbonBridgeArrived3 == False:
        PassengePointList3.append(RibbonBridgePose3.position)

def draw_trajectory():
    #白い画像の生成(1600x900) (1260x700)
    width = 1260
    height = 700
    blank_img = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(blank_img,(0,0),(width, height),(255,255,255),-1)

    marker_size = 20
    marker_line_size = 2
    line_size = 3
    marker_interval = 20 #10個おきにマーカーを描画

    for i in range(len(PassengePointList1)):
        if i == 0 or i == len(PassengePointList1)-1:
            cv2.circle(
                blank_img,
                (int(width/2 + PassengePointList1[i].x*10), int(height/2 - PassengePointList1[i].y*10)), marker_size,
                (0,0,0),
                marker_line_size )

        else:
            pts = np.array([
                    (int(width/2 + PassengePointList1[i].x*10), int(height/2 - PassengePointList1[i].y*10)),
                    (int(width/2 + PassengePointList1[i-1].x*10), int(height/2 - PassengePointList1[i-1].y*10))
                ], dtype=np.int32)
            cv2.polylines(blank_img, [pts], False, (0,0,0), line_size)

            if i%marker_interval == 0:
                cv2.circle(
                    blank_img,
                    (int(width/2 + PassengePointList1[i].x*10), int(height/2 - PassengePointList1[i].y*10)), marker_size,
                    (0,0,0),
                    marker_line_size )


    for i in range(len(PassengePointList2)):
        if i == 0 or i == len(PassengePointList2)-1:
            cv2.rectangle(
                blank_img,
                (int(width/2 + PassengePointList2[i].x*10 - marker_size), int(height/2 - PassengePointList2[i].y*10 - marker_size)),
                (int(width/2 + PassengePointList2[i].x*10 + marker_size), int(height/2 - PassengePointList2[i].y*10 + marker_size)),
                (0,255,0),
                marker_line_size )

        else:
            pts = np.array([
                    (int(width/2 + PassengePointList2[i].x*10), int(height/2 - PassengePointList2[i].y*10)),
                    (int(width/2 + PassengePointList2[i-1].x*10), int(height/2 - PassengePointList2[i-1].y*10))
                ], dtype=np.int32)
            cv2.polylines(blank_img, [pts], False, (0,0,0), line_size)

            if i%marker_interval == 0:
                cv2.rectangle(
                    blank_img,
                    (int(width/2 + PassengePointList2[i].x*10 - marker_size), int(height/2 - PassengePointList2[i].y*10 - marker_size)),
                    (int(width/2 + PassengePointList2[i].x*10 + marker_size), int(height/2 - PassengePointList2[i].y*10 + marker_size)),
                    (0,255,0),
                    marker_line_size )


    for i in range(len(PassengePointList3)):
        if i == 0 or i == len(PassengePointList3)-1:
            #三角形の３つの角の座標
            pts = np.array([
                    (int(width/2 + PassengePointList3[i].x*10), int(height/2 - PassengePointList3[i].y*10 - marker_size-1)),
                    (int(width/2 + PassengePointList3[i].x*10 + marker_size), int(height/2 - PassengePointList3[i].y*10 + marker_size)),
                    (int(width/2 + PassengePointList3[i].x*10 - marker_size), int(height/2 - PassengePointList3[i].y*10 + marker_size))
                ], dtype=np.int32)
            cv2.polylines(blank_img, [pts], True, (0,0,0), marker_line_size)

        else:
            pts = np.array([
                    (int(width/2 + PassengePointList3[i].x*10), int(height/2 - PassengePointList3[i].y*10)),
                    (int(width/2 + PassengePointList3[i-1].x*10), int(height/2 - PassengePointList3[i-1].y*10))
                ], dtype=np.int32)
            cv2.polylines(blank_img, [pts], False, (0,0,0), line_size)

            if i%marker_interval == 0:
                #三角形の３つの角の座標
                pts = np.array([
                        (int(width/2 + PassengePointList3[i].x*10), int(height/2 - PassengePointList3[i].y*10 - marker_size-1)),
                        (int(width/2 + PassengePointList3[i].x*10 + marker_size), int(height/2 - PassengePointList3[i].y*10 + marker_size)),
                        (int(width/2 + PassengePointList3[i].x*10 - marker_size), int(height/2 - PassengePointList3[i].y*10 + marker_size))
                    ], dtype=np.int32)
                cv2.polylines(blank_img, [pts], True, (0,0,0), marker_line_size)


    show_img_size = (width/5, height/5)
    show_img = cv2.resize(blank_img, show_img_size)
    cv2.imshow("trajectory", show_img)
    cv2.waitKey(1)

    pkg_path = rospkg.RosPack().get_path('ribbon_bridge_path_generate')
    cv2.imwrite(pkg_path + "/img/trajectory.png", blank_img)

def main():
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_CB)

    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge1_arrived", Bool, is_RibbonBridge1_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge2_arrived", Bool, is_RibbonBridge2_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge3_arrived", Bool, is_RibbonBridge3_arrived_CB)


    rospy.sleep(1) #浮橋の位置をsubscribeする時間を確保

    while not rospy.is_shutdown():
        update_bridges_trajectory()
        draw_trajectory()
        time.sleep(1)


if __name__ == "__main__":
    rospy.init_node("Trajectory")
    main()
    rospy.spin()
