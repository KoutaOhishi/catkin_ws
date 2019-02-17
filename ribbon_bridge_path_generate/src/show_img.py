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

from ribbon_bridge_measurement.msg import *

RibbonBridgePose_1 = Pose()
RibbonBridgePose_2 = Pose()
RibbonBridgePose_3 = Pose()

RibbonBridgePath_1 = Path()
RibbonBridgePath_2 = Path()
RibbonBridgePath_3 = Path()

def sub_RibbonBridgePose_1_CB(msg):
    global RibbonBridgePose_1
    RibbonBridgePose_1 = msg

def sub_RibbonBridgePose_2_CB(msg):
    global RibbonBridgePose_2
    RibbonBridgePose_2 = msg

def sub_RibbonBridgePose_3_CB(msg):
    global RibbonBridgePose_3
    RibbonBridgePose_3 = msg

def sub_RibbonBridgePath_1_CB(msg):
    global RibbonBridgePath_1
    RibbonBridgePath_1 = msg

def sub_RibbonBridgePath_2_CB(msg):
    global RibbonBridgePath_2
    RibbonBridgePath_2 = msg

def sub_RibbonBridgePath_3_CB(msg):
    global RibbonBridgePath_3
    RibbonBridgePath_3 = msg


def img_cb(msg):
    #try:
    #rospy.loginfo("Subscribed Image Topic !")
    cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    cv_height = cv_img.shape[0]
    cv_width = cv_img.shape[1]


    for i in range(len(RibbonBridgePath_1.poses)):
        if i == 0:
            cv2.circle(cv_img, (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), 20, (255,0,0), -1)

        elif i == len(RibbonBridgePath_1.poses)-1:
            cv2.circle(cv_img, (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), 20, (255,255,0), -1)
            cv2.line(cv_img, (int(RibbonBridgePath_1.poses[i-1].pose.position.y), int(RibbonBridgePath_1.poses[i-1].pose.position.x)), (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), (255, 255, 255), thickness=4)

        else:
            cv2.circle(cv_img, (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), 20, (0,0,255), -1)

            if i == 1:
                cv2.arrowedLine(cv_img, (int(RibbonBridgePath_1.poses[i-1].pose.position.y), int(RibbonBridgePath_1.poses[i-1].pose.position.x)), (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), (0, 0, 255), thickness=4)
            else:
                cv2.line(cv_img, (int(RibbonBridgePath_1.poses[i-1].pose.position.y), int(RibbonBridgePath_1.poses[i-1].pose.position.x)), (int(RibbonBridgePath_1.poses[i].pose.position.y), int(RibbonBridgePath_1.poses[i].pose.position.x)), (255, 255, 255), thickness=4)


        for i in range(len(RibbonBridgePath_2.poses)):
            if i == 0:
                cv2.circle(cv_img, (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), 20, (255,0,0), -1)

            elif i == len(RibbonBridgePath_2.poses)-1:
                cv2.circle(cv_img, (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), 20, (255,255,0), -1)
                cv2.line(cv_img, (int(RibbonBridgePath_2.poses[i-1].pose.position.y), int(RibbonBridgePath_2.poses[i-1].pose.position.x)), (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), (255, 255, 255), thickness=4)

            else:
                cv2.circle(cv_img, (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), 20, (0,0,255), -1)

                if i == 1:
                    cv2.arrowedLine(cv_img, (int(RibbonBridgePath_2.poses[i-1].pose.position.y), int(RibbonBridgePath_2.poses[i-1].pose.position.x)), (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), (0, 0, 255), thickness=4)
                else:
                    cv2.line(cv_img, (int(RibbonBridgePath_2.poses[i-1].pose.position.y), int(RibbonBridgePath_2.poses[i-1].pose.position.x)), (int(RibbonBridgePath_2.poses[i].pose.position.y), int(RibbonBridgePath_2.poses[i].pose.position.x)), (255, 255, 255), thickness=4)


        for i in range(len(RibbonBridgePath_3.poses)):
            if i == 0:
                cv2.circle(cv_img, (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), 20, (255,0,0), -1)

            elif i == len(RibbonBridgePath_3.poses)-1:
                cv2.circle(cv_img, (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), 20, (255,255,0), -1)
                cv2.line(cv_img, (int(RibbonBridgePath_3.poses[i-1].pose.position.y), int(RibbonBridgePath_3.poses[i-1].pose.position.x)), (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), (255, 255, 255), thickness=4)

            else:
                cv2.circle(cv_img, (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), 20, (0,0,255), -1)

                if i == 1:
                    cv2.arrowedLine(cv_img, (int(RibbonBridgePath_3.poses[i-1].pose.position.y), int(RibbonBridgePath_3.poses[i-1].pose.position.x)), (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), (0, 0, 255), thickness=4)
                else:
                    cv2.line(cv_img, (int(RibbonBridgePath_3.poses[i-1].pose.position.y), int(RibbonBridgePath_3.poses[i-1].pose.position.x)), (int(RibbonBridgePath_3.poses[i].pose.position.y), int(RibbonBridgePath_3.poses[i].pose.position.x)), (255, 255, 255), thickness=4)



    #except CvBridgeError, e:
        #rospy.logerror("Failed to Subscribe Image Topic")




    # windowサイズの調整
    show_img_size = (cv_width/5, cv_height/5)
    show_img = cv2.resize(cv_img, show_img_size)



    cv2.imshow("ribbon_bridge", show_img)
    cv2.waitKey(1)




def main():
    rospy.init_node("img_subscriber", anonymous=True)

    sub_img = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, img_cb)

    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, sub_RibbonBridgePose_1_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, sub_RibbonBridgePose_2_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, sub_RibbonBridgePose_3_CB)

    rospy.Subscriber("/ribbon_bridge_path_generate/path", Path, sub_RibbonBridgePath_1_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/path2", Path, sub_RibbonBridgePath_2_CB)
    rospy.Subscriber("/ribbon_bridge_path_generate/path3", Path, sub_RibbonBridgePath_3_CB)


if __name__ == "__main__":
    main()
    rospy.spin()
