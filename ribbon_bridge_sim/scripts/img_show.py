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

ribbon_bridge = RibbonBridge()
pastModel_pose = Pose()
path = Path()
path2 = Path()
path3 = Path()

def result_data_cb(msg):
    global ribbon_bridge, pastModel_pose
    #ribbon_bridge = msg.RibbonBridges[0]

    dist_list = []
    for i in range(len(msg.RibbonBridges)):
        dist = math.sqrt(pow(msg.RibbonBridges[i].center.x-pastModel_pose.position.x, 2)+pow(msg.RibbonBridges[i].center.y-pastModel_pose.position.y, 2))
        dist_list.append(dist)

    target_index = dist_list.index(min(dist_list))
    ribbon_bridge = msg.RibbonBridges[target_index]
    pastModel_pose.position.x = msg.RibbonBridges[target_index].center.x
    pastModel_pose.position.y = msg.RibbonBridges[target_index].center.y

def path_cb(msg):
    global path
    path = msg

def path_cb2(msg):
    global path2
    path2 = msg

def path_cb3(msg):
    global path3
    path3 = msg

def yolo_cb(msg):
    pass

def img_cb(msg):

    #rospy.loginfo("Subscribed Image Topic !")
    cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    cv_img2 = cv_img
    cv_img3 = cv_img

    cv_height = cv_img.shape[0]
    cv_width = cv_img.shape[1]

    #corner_1 = (int(ribbon_bridge.corners[0].x), int(ribbon_bridge.corners[0].y))
    #corner_2 = (int(ribbon_bridge.corners[1].x), int(ribbon_bridge.corners[1].y))
    #corner_3 = (int(ribbon_bridge.corners[2].x), int(ribbon_bridge.corners[2].y))
    #corner_4 = (int(ribbon_bridge.corners[3].x), int(ribbon_bridge.corners[3].y))

    #center = (int(ribbon_bridge.center.x),int(ribbon_bridge.center.y))

    #cv2.arrowedLine(cv_img,corner_1,corner_4,(0,0,255),5)
    #cv2.arrowedLine(cv_img,corner_1,corner_2,(0,255,255),5)
    #cv2.arrowedLine(cv_img, corner_1, (corner_1[0]+100, corner_1[1]), (255,0,255), 5)
    #cv2.circle(cv_img, corner_1, 10, (0,0,255), -1)
    #cv2.circle(cv_img, corner_2, 10, (255,0,255), -1)
    #cv2.circle(cv_img, center, 5, (0,0,255), -1)
    try:
        for i in range(len(path.poses)):
            if i == 1:
                _path = (int(path.poses[i].pose.position.y), int(path.poses[i].pose.position.x))
                cv2.circle(cv_img, _path, 20, (0,0,255), -1)
            elif i == 0:
                _path = (int(path.poses[i].pose.position.y), int(path.poses[i].pose.position.x))
                cv2.circle(cv_img, _path, 20, (255,255,0), -1)

            elif i == len(path.poses):
                _path = (int(path.poses[i].pose.position.y), int(path.poses[i].pose.position.x))
                cv2.circle(cv_img, _path, 20, (0,255,255), -1)
            else:
                _path = (int(path.poses[i].pose.position.y), int(path.poses[i].pose.position.x))
                cv2.circle(cv_img, _path, 20, (0,255,255), -1)
                start = (int(path.poses[i-1].pose.position.y), int(path.poses[i-1].pose.position.x))
                goal = (int(path.poses[i].pose.position.y), int(path.poses[i].pose.position.x))
                green = (0,255,0)
                cv2.line(cv_img, start, goal, green, 5)

        for i in range(len(path2.poses)):
            if i == 1:
                _path = (int(path2.poses[i].pose.position.y), int(path2.poses[i].pose.position.x))
                cv2.circle(cv_img2, _path, 20, (0,0,255), -1)
            elif i == 0:
                _path = (int(path2.poses[i].pose.position.y), int(path2.poses[i].pose.position.x))
                cv2.circle(cv_img2, _path, 20, (255,255,0), -1)
            elif i == len(path2.poses):
                _path = (int(path2.poses[i].pose.position.y), int(path2.poses[i].pose.position.x))
                cv2.circle(cv_img2, _path, 20, (0,255,255), -1)
            else:
                _path = (int(path2.poses[i].pose.position.y), int(path2.poses[i].pose.position.x))
                cv2.circle(cv_img2, _path, 20, (0,255,255), -1)
                start = (int(path2.poses[i-1].pose.position.y), int(path2.poses[i-1].pose.position.x))
                goal = (int(path2.poses[i].pose.position.y), int(path2.poses[i].pose.position.x))
                green = (0,255,0)
                cv2.line(cv_img2, start, goal, green, 5)

        """for i in range(len(path3.poses)):
            if i == 1:
                _path = (int(path3.poses[i].pose.position.y), int(path3.poses[i].pose.position.x))
                cv2.circle(cv_img3, _path, 20, (0,0,255), -1)
            elif i == 0:
                _path = (int(path3.poses[i].pose.position.y), int(path3.poses[i].pose.position.x))
                cv2.circle(cv_img3, _path, 20, (255,255,0), -1)
            elif i == len(path3.poses):
                _path = (int(path3.poses[i].pose.position.y), int(path3.poses[i].pose.position.x))
                cv2.circle(cv_img3, _path, 20, (0,255,255), -1)
            else:
                _path = (int(path3.poses[i].pose.position.y), int(path3.poses[i].pose.position.x))
                cv2.circle(cv_img3, _path, 20, (0,255,255), -1)
                start = (int(path3.poses[i-1].pose.position.y), int(path3.poses[i-1].pose.position.x))
                goal = (int(path3.poses[i].pose.position.y), int(path3.poses[i].pose.position.x))
                green = (0,255,0)
                cv2.line(cv_img3, start, goal, green, 5)"""



        #cv2.line(cv_img,corner_4,corner_1,(0,255,0),5)

        #cv2.arrowedLine(img, (50, 80), (125, 130), (0, 255, 0), thickness=4)
        #cv2.arrowedLine(cv_img, (int(path2.poses[0].pose.position.y), int(path2.poses[0].pose.position.x)), (int(path2.poses[1].pose.position.y), int(path2.poses[1].pose.position.y)) , (0,255,0), thickness=4)

        #現在目指しているsub-goalを指す
        cv2.arrowedLine(cv_img, (int(path.poses[0].pose.position.y), int(path.poses[0].pose.position.x)), (int(path.poses[1].pose.position.y), int(path.poses[1].pose.position.x)) , (0,0,255), thickness=4)
        cv2.arrowedLine(cv_img2, (int(path2.poses[0].pose.position.y), int(path2.poses[0].pose.position.x)), (int(path2.poses[1].pose.position.y), int(path2.poses[1].pose.position.x)) , (0,0,255), thickness=4)
        """cv2.arrowedLine(cv_img3, (int(path3.poses[0].pose.position.y), int(path3.poses[0].pose.position.x)), (int(path3.poses[1].pose.position.y), int(path3.poses[1].pose.position.x)) , (0,0,255), thickness=4)"""

    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")




    # windowサイズの調整
    show_img_size = (cv_width/5, cv_height/5)
    show_img = cv2.resize(cv_img, show_img_size)
    show_img2 = cv2.resize(cv_img2, show_img_size)
    show_img3 = cv2.resize(cv_img3, show_img_size)


    #cv2.imshow("ribbon_bridge1", show_img)
    #cv2.waitKey(1)
    #cv2.imshow("ribbon_bridge2", show_img2)
    #cv2.waitKey(1)
    cv2.imshow("ribbon_bridge3", show_img3)

    cv2.waitKey(1)




def main():
    rospy.init_node("img_subscriber", anonymous=True)

    img_topic_name = "/aerial_camera/camera1/image_raw"

    sub_img = rospy.Subscriber(img_topic_name, Image, img_cb)
    sub_Result_data = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, result_data_cb)
    sub_path = rospy.Subscriber("/ribbon_bridge_path_generate/path", Path, path_cb)
    sub_path2 = rospy.Subscriber("/ribbon_bridge_path_generate/path2", Path, path_cb2)
    sub_path3 = rospy.Subscriber("/ribbon_bridge_path_generate/path3", Path, path_cb3)

    sub_yolo = rospy.Subscriber("/darknet_ros/found_object", Int8, yolo_cb)



if __name__ == "__main__":
    main()
    rospy.spin()
