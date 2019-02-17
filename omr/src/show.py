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
        temp = cv2.imread("/home/rg26/catkin_ws/src/omr/img/tmp.png")

        cv_height = cv_img.shape[0]
        cv_width = cv_img.shape[1]

        template_width = 10
        template_height = 10
        threshold = 0.85

        box_pt = []

        matches = cv2.matchTemplate(cv_img, temp, cv2.TM_CCORR_NORMED)

        for y in xrange(matches.shape[0]):
            for x in xrange(matches.shape[1]):
                if matches[y][x] > threshold:
                    cv2.rectangle(cv_img, (x, y),
                                  (x + template_width, y + template_height),
                                  (0, 0, 255), 1)
                    box_pt.append((x+5,y+5))


        print box_pt

        cv2.rectangle(cv_img, box_pt[0], box_pt[3], (255,0,0), 1)

        cv2.imshow("window", cv_img)
        cv2.waitKey(1)


    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")

def main():
    rospy.init_node("img_subscriber", anonymous=True)

    img_topic_name = "/usb_cam/image_raw"

    rospy.Subscriber(img_topic_name, Image, img_cb)
    rospy.spin()

if __name__ == "__main__":
    main()
