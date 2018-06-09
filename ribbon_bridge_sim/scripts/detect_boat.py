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

        cv_gray = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)

        # しきい値指定によるフィルタリング
        retval, dst = cv2.threshold(cv_gray, 127, 255, cv2.THRESH_TOZERO_INV )

        # 白黒の反転
        dst = cv2.bitwise_not(dst)
        # 再度フィルタリング
        retval, dst = cv2.threshold(dst, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # 輪郭を抽出
        dst, contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #dst = cv2.imread(cv_img, cv2.IMREAD_COLOR)

        for i, contour in enumerate(contours):
            # 小さな領域の場合は間引く
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            # 画像全体を占める領域は除外する
            if image_size * 0.99 < area:
                continue

            # 外接矩形を取得
            x,y,w,h = cv2.boundingRect(contour)
            dst = cv2.rectangle(cv_img,(x,y),(x+w,y+h),(0,0,255),5)


        # windowサイズの調整
        show_img_size = (cv_width/5, cv_height/5)
        show_img = cv2.resize(dst, show_img_size)



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
