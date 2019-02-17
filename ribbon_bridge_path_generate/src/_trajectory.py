#!/usr/bin/env python
#coding: utf-8
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from nav_msgs.msg import Path
from std_msgs.msg import Int8

""" 浮体の移動の軌跡画像を作成するノード """
Cv_img = np.ndarray(255)
Cv_height = 0
Cv_Width = 0
isGetImg = False

RibbonBridgePose_1 = Pose()
RibbonBridgePose_2 = Pose()
RibbonBridgePose_3 = Pose()

Poses_1 = PoseArray()
Poses_2 = PoseArray()
Poses_3 = PoseArray()

def sub_RibbonBridgePose_1_CB(msg):
    global RibbonBridgePose_1
    RibbonBridgePose_1 = msg

def sub_RibbonBridgePose_2_CB(msg):
    global RibbonBridgePose_2
    RibbonBridgePose_2 = msg

def sub_RibbonBridgePose_3_CB(msg):
    global RibbonBridgePose_3
    RibbonBridgePose_3 = msg


def save_start_img():
    ## 軌跡画像の作成開始の最初の画像を保存する関数
    cv2.imwrite("/home/rg26/catkin_ws/src/ribbon_bridge_path_generate/img/trajectory.jpg", Cv_img)

def trajectory():
    now_img = Cv_img
    past_img = cv2.imread("/home/rg26/catkin_ws/src/ribbon_bridge_path_generate/img/trajectory.jpg")

    alpha = 0.01
    blend_img = cv2.addWeighted(now_img, alpha, past_img, 1-alpha, 0)

    #cv2.imwrite("/home/rg26/catkin_ws/src/ribbon_bridge_path_generate/img/trajectory.jpg", blend_img)

    Poses_1.poses.append(RibbonBridgePose_1)
    Poses_2.poses.append(RibbonBridgePose_2)
    Poses_3.poses.append(RibbonBridgePose_3)

    for i in range(len(Poses_1.poses)):
        cv2.circle(blend_img, (int(Poses_1.poses[i].position.x), int(Poses_1.poses[i].position.y)), 10, (0,0,255), -1)

    for i in range(len(Poses_2.poses)):
        cv2.circle(blend_img, (int(Poses_2.poses[i].position.x), int(Poses_2.poses[i].position.y)), 10, (0,0,0), -1)

    for i in range(len(Poses_3.poses)):
        cv2.circle(blend_img, (int(Poses_3.poses[i].position.x), int(Poses_3.poses[i].position.y)), 10, (200,255,127), -1)

    #cv2.circle(blend_img, (int(RibbonBridgePose_1.position.x), int(RibbonBridgePose_1.position.y)), 20, (0,0,255), -1)
    #cv2.circle(blend_img, (int(RibbonBridgePose_2.position.x), int(RibbonBridgePose_2.position.y)), 20, (0,0,255), -1)
    #cv2.circle(blend_img, (int(RibbonBridgePose_3.position.x), int(RibbonBridgePose_3.position.y)), 20, (0,0,255), -1)

    cv2.imwrite("/home/rg26/catkin_ws/src/ribbon_bridge_path_generate/img/trajectory.jpg", blend_img)

    show_img_size = (Cv_width/5, Cv_height/5)
    show_img = cv2.resize(blend_img, show_img_size)
    cv2.imshow("trajectory", show_img)
    cv2.waitKey(10)

def img_cb(msg):
    try:
        #rospy.loginfo("Subscribed Image Topic !")
        global Cv_img, isGetImg, Cv_height, Cv_width
        Cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        isGetImg = True

        Cv_height = Cv_img.shape[0]
        Cv_width = Cv_img.shape[1]

        # 入力画像を読み込む。
        """img1 = cv2.imread('image1.jpg')
        img2 = cv2.imread('image2.jpg')
        # img1, img2 は同じ形状でなければならない。
        print('img1.shape', img1.shape)  # img1.shape (500, 500, 3)
        print('img2.shape', img2.shape)  # img2.shape (500, 500, 3)

        alpha = 0.4
        blended = cv2.addWeighted(img1, alpha, img2, 1 - alpha, 0)  # img1 * 0.4 + img2 * 0.6

        plt.imshow(cv2.cvtColor(blended, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        plt.show()"""

        # windowサイズの調整
        #show_img_size = (cv_width/5, cv_height/5)
        #show_img = cv2.resize(cv_img, show_img_size)

        #cv2.imshow("ribbon_bridge", show_img)
        #cv2.waitKey(1)

    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")

def main():
    rospy.init_node("Trajectory", anonymous=True)
    sub_img = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, img_cb)

    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, sub_RibbonBridgePose_1_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, sub_RibbonBridgePose_2_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, sub_RibbonBridgePose_3_CB)


    while not rospy.is_shutdown():
        if isGetImg == True:
            break


    save_start_img()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        #rospy.sleep(1)
        trajectory()


if __name__ == "__main__":
    main()
    rospy.spin()
