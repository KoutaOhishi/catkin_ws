#!/usr/bin/env python
#coding: utf-8
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def img_cb(msg):
    try:
        #rospy.loginfo("Subscribed Image Topic !")
        cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        cv_height = cv_img.shape[0]
        cv_width = cv_img.shape[1]

        image_size = cv_height * cv_width


        # windowサイズの調整
        show_img_size = (cv_width/10, cv_height/10)
        show_img = cv2.resize(cv_img, show_img_size)



        cv2.imshow("window", show_img)
        cv2.waitKey(1)


    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")

def main():
    rospy.init_node("img_subscriber", anonymous=True)

    img_topic_name = "/aerial_camera/camera1/image_raw"

    rospy.Subscriber(img_topic_name, Image, img_cb)
    rospy.spin()

if __name__ == "__main__":
    main()
