#!/usr/bin/env python
#coding: utf-8
import rospy
import cv2
import numpy as np
import datetime

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Global
Saved_flag = False

def img_cb(msg):
    global Saved_flag
    try:
        #rospy.loginfo("Subscribed Image Topic !")
        cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        cv_height = cv_img.shape[0]
        cv_width = cv_img.shape[1]

        image_size = cv_height * cv_width


        # windowサイズの調整
        #show_img_size = (cv_width/10, cv_height/10)
        #show_img = cv2.resize(cv_img, show_img_size)

        now = datetime.datetime.now()
        now_str = now.strftime("%Y-%m-%d-%H:%M:%S")
        file_name = "saved_img_%s.png"%now_str

        file_path = "/home/rg26/catkin_ws/src/ribbon_bridge_sim/img/" + file_name


        #cv2.imshow("window", show_img)
        cv2.imwrite(file_path, cv_img)
        #cv2.waitKey(1)
        print "Saved at [%s]."%file_path
        Saved_flag = True


    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")

def main():
    rospy.init_node("img_subscriber", anonymous=True)

    img_topic_name = "/aerial_camera/camera1/image_raw"

    rospy.Subscriber(img_topic_name, Image, img_cb)

    while not rospy.is_shutdown():
        if Saved_flag == True:
            break

if __name__ == "__main__":
    main()
