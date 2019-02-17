#!/usr/bin/env python
# coding: utf-8
import yaml
import cv2
import numpy as np
import heapq
import rospy, rospkg
import datetime
import math

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from ribbon_bridge_measurement.msg import *

class Resize_Image():
    def __init__(self):
        #Subscribeするimgトピック名
        self.sub_img_topic_name = "/aerial_camera/camera1/image_raw"

        #Publishするimgトピック名
        self.pub_img_topic_name = "/gazebo/resized_img"

        self.show_img = True

        self.sub_img = rospy.Subscriber(self.sub_img_topic_name, Image, self.sub_img_CB)
        self.pub_img = rospy.Publisher(self.pub_img_topic_name, Image, queue_size=1)

    def sub_img_CB(self, msg):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            img_width = msg.width
            img_height = msg.height

            resize = (img_width/10, img_height/10)

            resize_img = cv2.resize(cv_img, resize)

            publish_img = CvBridge().cv2_to_imgmsg(resize_img, "bgr8")
            self.pub_img.publish(publish_img)

            if self.show_img == True:
                cv2.imshow("resize_img", resize_img)
                cv2.waitKey(1)



        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic[%s]"%self.sub_img_topic_name)

    def main(self):
        rospy.loginfo("ResizeImage Node start")


if __name__ == "__main__":
    rospy.init_node("ResizeImage", anonymous=True)
    c = Resize_Image()
    c.main()
    rospy.spin()
