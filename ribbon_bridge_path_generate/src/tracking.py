#!/usr/bin/env python
#coding: utf-8
import rospy
import sys
import time
import math
import cv2
import numpy as np

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import *

class Tracking():
    def __init__(self):
        #Subscriber
        self.sub_img = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, self.subImage_CB)
        self.subResultData = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.subResultData_CB)

        #imgのwidthとheightの設定(Subscribeの時に更新する)
        self.ImageWidth = 1600#4096
        self.ImageHeight = 900#2160

        #制御する浮体ごとに変数を用意する
        self.RibbonBridgePose_1 = Pose()
        self.RibbonBridgePose_2 = Pose()
        self.RibbonBridgePose_3 = Pose()

        #1つ前のタイミングの浮体の位置を保存しておく
        self.pastRibbonBridgePose_1 = Pose()
        self.pastRibbonBridgePose_2 = Pose()
        self.pastRibbonBridgePose_3 = Pose()

        #浮体の位置をpublishするためのPublisherを浮体の数だけ用意する
        self.pubRibbonBridgePose_1 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, queue_size=1)
        self.pubRibbonBridgePose_2 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, queue_size=1)
        self.pubRibbonBridgePose_3 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, queue_size=1)

        self.RibbonBridges = RibbonBridges()#検出した浮体の配列

        self.BoatDiagonal = 1000 #浮体の対角線の長さ、浮体の追跡のしきい値に用いる。最初は大きめに設定しておく
        self.isGetBoatDiagonal = False #浮体の対角線の長さを取得できたらTrue
        self.isStartTrackingPose_1 = False

        self.isYoloExists = True #Yoloのノードの生存状況
        self.subYolo = rospy.Subscriber("/darknet_ros/found_object", Int8, self.subYolo) #YOLOのノードとの接続を確認するために作成

        self.pubTrackerStatus = rospy.Publisher("/ribbon_bridge_path_generate/tracking_status", Bool, queue_size=1) #TrackingができないときはFalse

        self.pubResultImage = rospy.Publisher("/ribbon_bridge_path_generate/tracking_result", Image, queue_size=1)

    def subImage_CB(self, msg):
        try:
            self.ImageWidth = msg.width
            self.ImageHeight = msg.height

            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            if math.isnan(self.RibbonBridgePose_1.position.x) == False and math.isnan(self.RibbonBridgePose_1.position.y) == False:
                #浮体の位置をマークする
                cv2.circle(cv_img, (int(self.RibbonBridgePose_1.position.x), int(self.RibbonBridgePose_1.position.y)), 20, (0,0,255), -1)
                ##他の浮体の立ち入り禁止エリアを描画
                cv2.circle(cv_img, (int(self.RibbonBridgePose_1.position.x), int(self.RibbonBridgePose_1.position.y)), int(self.BoatDiagonal), (0,0,255), 5)
                #浮き橋の番号を付与
                #cv2.text()

            if math.isnan(self.RibbonBridgePose_2.position.x) == False and math.isnan(self.RibbonBridgePose_2.position.y) == False:
                #浮体の位置をマークする
                cv2.circle(cv_img, (int(self.RibbonBridgePose_2.position.x), int(self.RibbonBridgePose_2.position.y)), 20, (255,0,0), -1)
                ##他の浮体の立ち入り禁止エリアを描画
                cv2.circle(cv_img, (int(self.RibbonBridgePose_2.position.x), int(self.RibbonBridgePose_2.position.y)), int(self.BoatDiagonal), (255,0,0), 10)

            if math.isnan(self.RibbonBridgePose_3.position.x) == False and math.isnan(self.RibbonBridgePose_3.position.y) == False:
                #浮体の位置をマークする
                cv2.circle(cv_img, (int(self.RibbonBridgePose_3.position.x), int(self.RibbonBridgePose_3.position.y)), 20, (0,255,0), -1)
                ##他の浮体の立ち入り禁止エリアを描画
                cv2.circle(cv_img, (int(self.RibbonBridgePose_3.position.x), int(self.RibbonBridgePose_3.position.y)), int(self.BoatDiagonal), (0,255,0), 10)



            #for i in range(len(self.RibbonBridges)):
                #cv2.line(cv_img, (int(self.pastRibbonBridgePose_1.position.x), int(self.pastRibbonBridgePose_1.position.y)), (int(self.RibbonBridges[i].center.x), int(self.RibbonBridges[i].center.x)), (255,255,255), thickness=5)
                #cv2.line(cv_img, (int(self.pastRibbonBridgePose_2.position.x), int(self.pastRibbonBridgePose_2.position.y)), (int(self.RibbonBridges[i].center.x), int(self.RibbonBridges[i].center.x)), (255,255,255), thickness=5)


            size = (self.ImageWidth/5, self.ImageHeight/5)
            resize_img = cv2.resize(cv_img, size)

            cv2.imshow("tracker result", resize_img)
            cv2.waitKey(1)

            #pub_img = CvBridge().cv2_to_imgmsg(cv_img, encoding="passthrough")
            #self.pubResultImage.publish(pub_img)


        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic")

    def subYolo(self, msg):
        pass

    def checkYoloIsExists(self):
        ### Yoloのノードの生存状況を確認する
        if self.subYolo.get_num_connections() == 0:
            self.isYoloExists = False
            self.pubTrackerStatus.publish(False)
            rospy.logwarn("Yolo has Dead")
            rospy.sleep(1)
        else:
            self.isYoloExists = True

    def subResultData_CB(self, msg):
        self.RibbonBridges = msg.RibbonBridges

        if len(msg.RibbonBridges) != 0:
            #浮体の長さを取得する
            if self.isGetBoatDiagonal == False:
                self.BoatDiagonal = math.sqrt(pow((msg.RibbonBridges[0].corners[0].x-msg.RibbonBridges[0].corners[2].x),2) + pow((msg.RibbonBridges[0].corners[0].y-msg.RibbonBridges[0].corners[2].y),2))
                self.BoatDiagonal = self.BoatDiagonal * 0.375 * 0.25
                self.isGetBoatDiagonal = True

    def pubRibbonBridges(self):
        #追跡している浮体の位置をpublishする
        #NaN(Not a Number)の確認
        if math.isnan(self.RibbonBridgePose_1.position.x) == True or math.isnan(self.RibbonBridgePose_1.position.y) == True:
            rospy.logerr("RibbonBridgePose_1 has NaN")
        else:
            rospy.loginfo("RibbonBridge_1:[%s][%s]"%(str(int(self.RibbonBridgePose_1.position.x)),str(int(self.RibbonBridgePose_1.position.y))))
            self.pubRibbonBridgePose_1.publish(self.RibbonBridgePose_1)

        if math.isnan(self.RibbonBridgePose_2.position.x) == True or math.isnan(self.RibbonBridgePose_2.position.y) == True:
            rospy.logerr("RibbonBridgePose_2 has NaN")
        else:
            rospy.loginfo("RibbonBridge_2:[%s][%s]"%(str(int(self.RibbonBridgePose_2.position.x)),str(int(self.RibbonBridgePose_2.position.y))))
            self.pubRibbonBridgePose_2.publish(self.RibbonBridgePose_2)

        if math.isnan(self.RibbonBridgePose_3.position.x) == True or math.isnan(self.RibbonBridgePose_3.position.y) == True:
            rospy.logerr("RibbonBridgePose_3 has NaN")
        else:
            rospy.loginfo("RibbonBridge_3:[%s][%s]"%(str(int(self.RibbonBridgePose_3.position.x)),str(int(self.RibbonBridgePose_3.position.y))))
            self.pubRibbonBridgePose_3.publish(self.RibbonBridgePose_3)

    def track(self):
        ### 前のタイミングの浮体の位置から一番近い浮体を現在の位置として採用する
        try:
            ribbon_bridges = self.RibbonBridges

            ##### RibbonBridgePose_1の探索
            if len(ribbon_bridges) == 0:
                rospy.logwarn("There are No RibbonBridges")
                self.pubRibbonBridgePose_1.publish(self.pastRibbonBridgePose_1)
            else:
                dist_list = [] #pastRibbonBridgePose_1からの距離を格納するための配列
                for i in range(len(ribbon_bridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_1.position.x-ribbon_bridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_1.position.y-ribbon_bridges[i].center.y, 2))
                    dist_list.append(dist)
                    #print "No1 [%s]%s"%(str(i), str(int(dist)))

                if min(dist_list) > self.BoatDiagonal:
                    rospy.logwarn("RibbonBridgePose_1 -> LOST")
                    self.pubTrackerStatus.publish(False)
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_1.position.x = ribbon_bridges[target_index].center.x
                    self.RibbonBridgePose_1.position.y = ribbon_bridges[target_index].center.y
                    self.pastRibbonBridgePose_1 = self.RibbonBridgePose_1
                    self.pubTrackerStatus.publish(True)
                    ribbon_bridges.pop(target_index)
                    #ribbon_bridges.remove(ribbon_bridges[target_index])


            ##### RibbonBridgePose_2の探索
            if len(ribbon_bridges) == 0:
                rospy.logwarn("There are No RibbonBridges")
                self.pubRibbonBridgePose_2.publish(self.pastRibbonBridgePose_2)
            else:
                dist_list = [] #pastRibbonBridgePose_2からの距離を格納するための配列
                for i in range(len(ribbon_bridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_2.position.x-ribbon_bridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_2.position.y-ribbon_bridges[i].center.y, 2))
                    dist_list.append(dist)
                    #print "No2 [%s]%s"%(str(i), str(int(dist)))

                if min(dist_list) > self.BoatDiagonal:
                    rospy.logwarn("RibbonBridgePose_2 -> LOST")
                    self.pubTrackerStatus.publish(False)
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_2.position.x = ribbon_bridges[target_index].center.x
                    self.RibbonBridgePose_2.position.y = ribbon_bridges[target_index].center.y
                    self.pastRibbonBridgePose_2 = self.RibbonBridgePose_2
                    self.pubTrackerStatus.publish(True)
                    ribbon_bridges.pop(target_index)
                    #ribbon_bridges.remove(ribbon_bridges[target_index])

            ##### RibbonBridgePose_3の探索
            if len(ribbon_bridges) == 0:
                rospy.logwarn("There are No RibbonBridges")
                self.pubRibbonBridgePose_3.publish(self.pastRibbonBridgePose_3)
            else:
                dist_list = [] #pastRibbonBridgePose_3からの距離を格納するための配列
                for i in range(len(ribbon_bridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_3.position.x-ribbon_bridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_3.position.y-ribbon_bridges[i].center.y, 2))
                    dist_list.append(dist)
                    #print "No3 [%s]%s"%(str(i), str(int(dist)))


                if min(dist_list) > self.BoatDiagonal:
                    rospy.logwarn("RibbonBridgePose_3 -> LOST")
                    self.pubTrackerStatus.publish(False)
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_3.position.x = ribbon_bridges[target_index].center.x
                    self.RibbonBridgePose_3.position.y = ribbon_bridges[target_index].center.y
                    self.pastRibbonBridgePose_3 = self.RibbonBridgePose_3
                    self.pubTrackerStatus.publish(True)
                    ribbon_bridges.pop(target_index)
                    #ribbon_bridges.remove(ribbon_bridges[target_index])"""


        #print "---"


        except:
            rospy.logerr("Tracking Error")

    def main(self):
        #浮体ごとに初期位置を設定する
        self.pastRibbonBridgePose_1.position.x = 300
        self.pastRibbonBridgePose_1.position.y = 200

        #self.pastRibbonBridgePose_2.position.x = 300
        #self.pastRibbonBridgePose_2.position.y = self.ImageHeight/2
        self.pastRibbonBridgePose_2.position.x = 800
        self.pastRibbonBridgePose_2.position.y = self.ImageHeight/2

        #self.pastRibbonBridgePose_3.position.x = 300
        #self.pastRibbonBridgePose_3.position.y = self.ImageHeight-200
        self.pastRibbonBridgePose_3.position.x = 1413 #48
        self.pastRibbonBridgePose_3.position.y = self.ImageHeight/2

        while not rospy.is_shutdown():
            self.checkYoloIsExists()
            self.track()
            self.pubRibbonBridges()
            #rospy.sleep(0.1)
            time.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("TrackingNode")
    c = Tracking()
    c.main()
