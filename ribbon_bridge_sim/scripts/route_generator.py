#!/usr/bin/env python
# coding: utf-8
import yaml
import cv2
import numpy as np
import heapq
import rospy, rospkg

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from ribbon_bridge_measurement.msg import *


class RouteGenerator:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('ribbon_bridge_sim')

        #yamlファイルの読み込み
        self.info = yaml.load(open(self.pkg_path + "/config/route_generator.yaml", "r+"))

        #imgファイルのpathの設定
        self.img_path = self.pkg_path + "/img/" + self.info["img_name"]

        #生成したmapのファイルpathの設定
        self.map_path = self.pkg_path + "/img/" + self.info["map_name"]

        #map作成のために生成した白紙の画像ファイルのpath
        self.blank_map_path = self.pkg_path + "/img/blank.png"

        #Subscribeするimgトピック名を取得
        self.img_topic = str(self.info["img_topic"])

        #imgのwidthとheightを読み込み
        self.map_width = self.info["img_width"]
        self.map_height = self.info["img_height"]

        #浮体の接触禁止範囲の定義
        self.contact_area = self.info["contact_area"]

        #self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)

        self.map_sub = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.rect_cb)

    def main(self):
        #img_sub = rospy.Subscriber(self.img_topic, Image, self.img_cb)
        #map_sub = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridge, self.rect_cb)
        #self.create_blank_map()
        pass

    def rect_cb(self, msg):
        center_x = msg.RibbonBridges[0].center.x
        center_y = msg.RibbonBridges[0].center.y
        center_theta = msg.RibbonBridges[0].center.theta

        corner_0 = msg.RibbonBridges[0].corners[0]
        corner_1 = msg.RibbonBridges[0].corners[1]
        corner_2 = msg.RibbonBridges[0].corners[2]
        corner_3 = msg.RibbonBridges[0].corners[3]

        map_raw = cv2.imread(self.blank_map_path)

        cv2.rectangle(map_raw, (int(corner_1.x), int(corner_1.y)), (int(corner_3.x), int(corner_3.y)), (0,0,0), -1)

        cv2.imwrite(self.map_path, map_raw)

        ret, map_thresh = cv2.threshold(map_raw, 210, 255, cv2.THRESH_BINARY)

        for x in range(self.map_height/10):
            for y in range(self.map_width/10):
                pixelValue = map_thresh[y*10][x*10][0]
                if pixelValue == 0:
                    cv2.circle(map_raw,(x*10,y*10), self.contact_area, (0,0,255), -1)#costの付与

        cv2.rectangle(map_raw, (int(corner_1.x), int(corner_1.y)), (int(corner_3.x), int(corner_3.y)), (0,0,0), -1)


        save_path = self.pkg_path + "/img/" + "cost.png"

        cv2.imwrite(save_path, map_raw)


        show_img_size = (self.map_height/10, self.map_width/10)
        show_img = cv2.resize(map_raw, show_img_size)
        cv2.imshow("window", show_img)
        cv2.waitKey(1)


    def create_blank_map(self):
        """ 入力画像と同じサイズの白い画像を生成する """
        # cv2.rectangleで埋めるだけ
        blank_img = np.zeros((self.map_width, self.map_height), dtype=np.uint8)

        cv2.rectangle(blank_img,(0,0),(self.map_height, self.map_width),(255,255,255),-1)

        cv2.imwrite(self.blank_map_path, blank_img)


    def img_cb(self, msg):
        try:
            #rospy.loginfo("Subscribed Image Topic !")
            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            cv2.imwrite(self.img_path, cv_img)

        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic")

if __name__ == "__main__":
    rospy.init_node("route_generator_node")
    rg = RouteGenerator()
    rg.main()
    rospy.spin()
