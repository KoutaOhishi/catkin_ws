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
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

"""
Gazeboから得られた浮橋の位置情報を元に、浮橋の移動の軌跡を描画する
"""

#Global
RibbonBridgePose1 = Pose()
RibbonBridgePose2 = Pose()
RibbonBridgePose3 = Pose()

RibbonBridgePath1 = Path()
RibbonBridgePath2 = Path()
RibbonBridgePath3 = Path()

#通過位置を格納していく
PassengePointList1 = []
PassengePointList2 = []
PassengePointList3 = []

#目標経路を格納する
PassengePathList1 = []
PassengePathList2 = []
PassengePathList3 = []

isRibbonBridgeArrived1 = False
isRibbonBridgeArrived2 = False
isRibbonBridgeArrived3 = False

aerial_camera = Image()

def sub_Image_CB(msg):
    global aerial_camera
    try:
        #rospy.loginfo("Subscribed Image Topic !")
        cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        aerial_camera = cv_img

    except:
        rospy.logerr("Failed to Subscribe Image Topic")

def RibbonBridgePose1_CB(msg):
    global RibbonBridgePose1
    RibbonBridgePose1 = msg

def RibbonBridgePose2_CB(msg):
    global RibbonBridgePose2
    RibbonBridgePose2 = msg

def RibbonBridgePose3_CB(msg):
    global RibbonBridgePose3
    RibbonBridgePose3 = msg

def is_RibbonBridge1_arrived_CB(msg):
    global isRibbonBridgeArrived1
    isRibbonBridgeArrived1 = msg.data

def is_RibbonBridge2_arrived_CB(msg):
    global isRibbonBridgeArrived2
    isRibbonBridgeArrived2 = msg.data

def is_RibbonBridge3_arrived_CB(msg):
    global isRibbonBridgeArrived3
    isRibbonBridgeArrived3 = msg.data

def path1_CB(msg):
    global RibbonBridgePath1
    RibbonBridgePath1 = msg

def path2_CB(msg):
    global RibbonBridgePath2
    RibbonBridgePath2 = msg

def path3_CB(msg):
    global RibbonBridgePath3
    RibbonBridgePath3 = msg

def add_bridge_poses():
    global PassengePointList1, PassengePointList2, PassengePointList3

#    if isRibbonBridgeArrived1 == False:
    PassengePointList1.append(RibbonBridgePose1.position)
    #else:
        #PassengePathList1[len(PassengePathList1)-1] = [RibbonBridgePose1.position.y, RibbonBridgePose1.position.x]

    #if isRibbonBridgeArrived2 == False:
    PassengePointList2.append(RibbonBridgePose2.position)

    #if isRibbonBridgeArrived3 == False:
    PassengePointList3.append(RibbonBridgePose3.position)

def add_paths():
    global PassengePathList1, PassengePathList2, PassengePathList3

    if len(PassengePathList1) == 0:
        if len(RibbonBridgePath1.poses) <= 10:
            PassengePathList1.append(RibbonBridgePath1.poses[len(RibbonBridgePath1.poses)-1].pose.position)
        else:
            PassengePathList1.append(RibbonBridgePath1.poses[10].pose.position)
    else:
        if len(RibbonBridgePath1.poses) <= 10:
            #if RibbonBridgePath1.poses[len(RibbonBridgePath1.poses)-1].pose.position != PassengePathList1[len(PassengePathList1)-1]:
            PassengePathList1.append(RibbonBridgePath1.poses[len(RibbonBridgePath1.poses)-1].pose.position)
        else:
            #if RibbonBridgePath1.poses[10].pose.position != PassengePathList1[len(PassengePathList1)-1]:
            PassengePathList1.append(RibbonBridgePath1.poses[10].pose.position)

    """if len(PassengePathList2) == 0:
        if len(RibbonBridgePath2.poses) <= 10:
            PassengePathList2.append(RibbonBridgePath2.poses[len(RibbonBridgePath2.poses)-1].pose.position)
        else:
            PassengePathList2.append(RibbonBridgePath2.poses[10].pose.position)
    else:
        if len(RibbonBridgePath2.poses) <= 10:
            #if RibbonBridgePath2.poses[len(RibbonBridgePath2.poses)-1] != PassengePathList2[len(PassengePathList2)-1]:
            PassengePathList2.append(RibbonBridgePath2.poses[len(RibbonBridgePath2.poses)-1].pose.position)
        else:
            #if RibbonBridgePath2.poses[10] != PassengePathList2[len(PassengePathList2)-1]:
            PassengePathList2.append(RibbonBridgePath2.poses[10].pose.position)

    if len(PassengePathList3) == 0:
        if len(RibbonBridgePath3.poses) <= 10:
            PassengePathList3.append(RibbonBridgePath3.poses[len(RibbonBridgePath3.poses)-1].pose.position)
        else:
            PassengePathList3.append(RibbonBridgePath3.poses[10].pose.position)
    else:
        if len(RibbonBridgePath3.poses) <= 10:
            #if RibbonBridgePath3.poses[len(RibbonBridgePath3.poses)-1] != PassengePathList3[len(PassengePathList3)-1]:
            PassengePathList3.append(RibbonBridgePath3.poses[len(RibbonBridgePath3.poses)-1].pose.position)
        else:
            #if RibbonBridgePath3.poses[10] != PassengePathList3[len(PassengePathList3)-1]:
            PassengePathList3.append(RibbonBridgePath3.poses[10].pose.position)"""

def draw_trajectory():
    #白い画像の生成(1600x900) (1260x700)
    width = 1600
    height = 900

    #blank_img = np.zeros((height, width, 3), dtype=np.uint8)
    #cv2.rectangle(blank_img,(0,0),(width, height),(255,255,255),-1)

    pkg_path = rospkg.RosPack().get_path('ribbon_bridge_path_generate')

    img_path = pkg_path + "/img/aerial_camera_.png"

    #blank_img = cv2.imread(img_path, 0)
    blank_img = aerial_camera
    #blank_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    marker_size = 20
    marker_line_size = 1
    line_size = 30
    marker_interval = 10 #何個おきにマーカーを描画するか
    passenge_point_size = 20

    #目標経路の描画
    for i in range(len(PassengePathList1)):
        if i == 0:
            pts = np.array([
                    (int( PassengePointList1[i].x), int( PassengePointList1[i].y)),
                    (int( PassengePathList1[0].y), int( PassengePathList1[0].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)
        else:
            pts = np.array([
                    (int( PassengePathList1[i].y), int( PassengePathList1[i].x)),
                    (int( PassengePathList1[i-1].y), int( PassengePathList1[i-1].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)

    """for i in range(len(PassengePathList2)):
        if i == 0:
            pts = np.array([
                    (int( PassengePointList2[i].x), int( PassengePointList2[i].y)),
                    (int( PassengePathList2[0].y), int( PassengePathList2[0].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)
        else:
            pts = np.array([
                    (int( PassengePathList2[i].y), int( PassengePathList2[i].x)),
                    (int( PassengePathList2[i-1].y), int( PassengePathList2[i-1].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)


    for i in range(len(PassengePathList3)):
        if i == 0:
            pts = np.array([
                    (int( PassengePointList3[i].x), int( PassengePointList3[i].y)),
                    (int( PassengePathList3[0].y), int( PassengePathList3[0].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)
        else:
            pts = np.array([
                    (int( PassengePathList3[i].y), int( PassengePathList3[i].x)),
                    (int( PassengePathList3[i-1].y), int( PassengePathList3[i-1].x))
                ], dtype=np.int32)
            color = (0,0,255)
            cv2.polylines(blank_img, [pts], False, color, line_size)"""



    #通過点の描画
    for i in range(len(PassengePointList1)):
        #if i == 0 or i == len(PassengePointList1)-1 or i%marker_interval == 0:
            #cv2.circle(blank_img,(int(PassengePointList1[i].x), int(PassengePointList1[i].y)), marker_size,(0,0,0),marker_line_size )
        if i != 0:
            cv2.line(blank_img, (int(PassengePointList1[i].x), int(PassengePointList1[i].y)), (int(PassengePointList1[i-1].x), int(PassengePointList1[i-1].y)), (0,0,0), passenge_point_size)


    """for i in range(len(PassengePointList2)):
        #if i == 0 or i == len(PassengePointList2)-1 or i%marker_interval == 0:
            #cv2.rectangle(blank_img,(int(PassengePointList2[i].x - marker_size), int(PassengePointList2[i].y - marker_size)),(int(PassengePointList2[i].x + marker_size), int(PassengePointList2[i].y + marker_size)),(0,0,0),marker_line_size )
        if i != 0:
            cv2.line(blank_img, (int(PassengePointList2[i].x), int(PassengePointList2[i].y)), (int(PassengePointList2[i-1].x), int(PassengePointList2[i-1].y)), (0,0,0), passenge_point_size)


    for i in range(len(PassengePointList3)):
        #if i == 0 or i == len(PassengePointList3)-1 or i%marker_interval == 0:
            #三角形の３つの角の座標
            #pts = np.array([(int(PassengePointList3[i].x), int(PassengePointList3[i].y - marker_size-1)),(int( PassengePointList3[i].x + marker_size), int( PassengePointList3[i].y + marker_size)),(int(PassengePointList3[i].x - marker_size), int( PassengePointList3[i].y + marker_size))], dtype=np.int32)
            #cv2.polylines(blank_img, [pts], True, (0,0,0), marker_line_size)
        if i != 0:
            cv2.line(blank_img, (int(PassengePointList3[i].x), int(PassengePointList3[i].y)), (int(PassengePointList3[i-1].x), int(PassengePointList3[i-1].y)), (0,0,0), passenge_point_size)"""





    #show_img_size = (width/5, height/5)
    #show_img = cv2.resize(blank_img, show_img_size)

    cv2.imshow("trajectory", blank_img)
    cv2.waitKey(1)

    cv2.imwrite(pkg_path + "/img/make_trajectory.png", blank_img)

def main():
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, RibbonBridgePose1_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, RibbonBridgePose2_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, RibbonBridgePose3_CB)

    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge1_arrived", Bool, is_RibbonBridge1_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge2_arrived", Bool, is_RibbonBridge2_arrived_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/is_RibbonBridge3_arrived", Bool, is_RibbonBridge3_arrived_CB)

    rospy.Subscriber("ribbon_bridge_path_generate/path", Path, path1_CB)
    rospy.Subscriber("ribbon_bridge_path_generate/path2", Path, path2_CB)
    rospy.Subscriber("ribbon_bridge_path_generate/path3", Path, path3_CB)

    rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, sub_Image_CB)

    global aerial_camera
    aerial_camera = Image()

    step = 0
    rospy.sleep(1) #浮橋の位置をsubscribeする時間を確保
    print "start"

    while not rospy.is_shutdown():
        add_bridge_poses()
        add_paths()
        draw_trajectory()
        print "step:[%s]"%(str(step))
        step += 1
        time.sleep(1)




if __name__ == "__main__":
    rospy.init_node("Trajectory")
    main()
    rospy.spin()
