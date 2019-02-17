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
from nav_msgs.msg import *


class RouteGenerator():
    def __init__(self):
        self.pkg_path = rospkg.RosPack().get_path('ribbon_bridge_path_generate')

        #空撮画像を保存するpath
        self.img_path = self.pkg_path + "/img/aerial_camera.png"

        #生成したmap画像のpath
        self.map_path = self.pkg_path + "/img/map.png"

        #map作成のために生成した白紙の画像ファイルのpath
        self.blank_map_path = self.pkg_path + "/img/blank.png"

        #空撮画像とmap画像を合わせた画像のpath
        self.result_path = self.pkg_path + "/img/route.png"

        #Subscribeするimgトピック名を取得
        self.img_topic_name = "/aerial_camera/camera1/image_raw"

        #imgのwidthとheightの設定(Subscribeの時に更新する)
        self.map_width = 1600#4096
        self.map_height = 900#2160

        #検出した浮体の縦と横の長さを設定
        self.Boat_width = 0.0
        self.Boat_height = 0.0
        self.Boat_diagonal = 0.0

        #costmapの大きさ（浮体の対角線の２倍にする）
        self.costmap = 0

        #複数の浮体の中で何番目を制御対象にするかを指定する
        self.target_model_index = 0

        self.Start_pose = Pose()
        self.pastStart_pose = Pose()
        self.TargetRibbonBridge = RibbonBridge()
        #self.OtherRibbonBridges = RibbonBridges()

        self.Goal_pose = Pose()

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_position", Pose, self.sub_Goal_pose_CB)

        self.sub_Result_data = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.sub_Result_data_CB)

        self.sub_Image = rospy.Subscriber(self.img_topic_name, Image, self.sub_Image_CB)

        self.pub_path = rospy.Publisher("/ribbon_bridge_path_generate/path", Path, queue_size=1)

        #flagで処理のタイミングを制御する
        self.GetBridgeResultFlag = False
        self.GetImageFlag = False
        self.CreatedBlankImageFlag = False
        self.GeneratePathFlag = False

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg

    def sub_Result_data_CB(self, msg):
        #rospy.loginfo("Subscribed result_data")
        self.GetBridgeResultFlag = True

        #並び替えの処理（暫定版)ribbon_bridge_measurementの方でする予定
        #x_list = []
        #for i in range(len(msg.RibbonBridges)):
            #x_list.append(msg.RibbonBridges[i].corners[0])
        #x_min_index = x_list.index(min(x_list))
        ###########################################################

        #トラッキング処理(暫定版)
        if len(msg.RibbonBridges) == 0:
            print "There are no RibbonBridges"

        else:
            dist_list = []
            for i in range(len(msg.RibbonBridges)):
                dist = math.sqrt(pow(self.pastStart_pose.position.x-msg.RibbonBridges[i].center.x,2)+pow(self.pastStart_pose.position.y-msg.RibbonBridges[i].center.y,2))
                dist_list.append(dist)

            target_index = dist_list.index(min(dist_list))
            self.Start_pose.position.x = msg.RibbonBridges[target_index].center.x
            self.Start_pose.position.y = msg.RibbonBridges[target_index].center.y
            self.pastStart_pose = self.Start_pose


            if self.CreatedBlankImageFlag == True:
                try:
                    #self.TargetRibbonBridge = msg.RibbonBridges[self.target_model_index]
                    self.TargetRibbonBridge = msg.RibbonBridges[target_index]

                    value = pow((self.TargetRibbonBridge.corners[0].x-self.TargetRibbonBridge.corners[2].x),2) + pow((self.TargetRibbonBridge.corners[0].y-self.TargetRibbonBridge.corners[2].y),2)
                    self.Boat_diagonal = math.sqrt(value)

                    #debug用　本来はTragetRibbonBridgeはコストマップにしない
                    #self.create_costmap(self.TargetRibbonBridge.center.x, self.TargetRibbonBridge.center.y, self.Boat_diagonal/2)

                    for i in range(len(msg.RibbonBridges)):
                        if i == target_index:
                            pass
                        else:
                            self.create_costmap(msg.RibbonBridges[i].center.x, msg.RibbonBridges[i].center.y, self.Boat_diagonal/1.5)

                except:
                    rospy.logerr("The index:[%s] is OUT of LENGTH of [/ribbon_bridge_measurement/result_data]"%str(self.target_model_index))

    def sub_Image_CB(self, msg):
        try:
            #rospy.loginfo("Subscribed Image Topic !")
            self.map_width = msg.width
            self.map_height = msg.height

            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            # 1/10にリサイズして保存
            new_size = (self.map_width/10, self.map_height/10)
            resize_img = cv2.resize(cv_img, new_size)
            cv2.imwrite(self.img_path, resize_img)

            # そのままのサイズで保存
            cv2.imwrite(self.pkg_path + "/img/aerial_camera_.png", cv_img)

            #self.create_blank_map()

            self.GetImageFlag = True

        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic")

    def create_blank_map(self):
        """ 入力画像と同じサイズの白い画像を生成する """
        #rospy.loginfo("Created blank_img")
        # cv2.rectangleで埋めるだけ
        blank_img = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        cv2.rectangle(blank_img,(0,0),(self.map_width, self.map_height),(255,255,255),-1)

        #cv2.circle(blank_img, (int(self.TargetRibbonBridge.center.x), int(self.TargetRibbonBridge.center.y)), 10, (0,0,0), -1)
        #cv2.circle(blank_img, (int(self.TargetRibbonBridge.corners[0].x), int(self.TargetRibbonBridge.corners[0].y)), 10, (0,0,255), -1)
        #cv2.circle(blank_img, (int(self.TargetRibbonBridge.corners[2].x), int(self.TargetRibbonBridge.corners[2].y)), 10, (0,0,255), -1)

        cv2.imwrite(self.blank_map_path, blank_img)
        cv2.imwrite(self.map_path, blank_img)

        self.CreatedBlankImageFlag = True

    def create_costmap(self, centerX, centerY, radius):
        costmap = cv2.imread(self.map_path)

        cv2.circle(costmap, (int(centerX), int(centerY)), int(radius), (0,0,0), -1)

        cv2.imwrite(self.map_path, costmap)

    def show_img(self):
        img = cv2.imread(self.map_path)

        try:
            show_img_size = (self.map_width/10, self.map_height/10)
            show_img = cv2.resize(img, show_img_size)
            cv2.imshow("path_image", show_img)
            cv2.waitKey(10)
        except:
            pass

    def astar(self, map_img, init, goal, distance=lambda path: len(path), heuristic=lambda pos: 0):
        """ A*で経路生成、成功するとpathが返ってくる。経路生成に失敗するとNoneを返す """
        queue = []
        init_score = self.distance([init]) + self.heuristic(init, goal)
        checked = {init: init_score}
        heapq.heappush(queue, (init_score, [init]))
        while len(queue) > 0:
            score, path = heapq.heappop(queue)
            last = path[-1]
            if last == goal: return path
            for pos in self.nexts(map_img, last):
                newpath = path + [pos]
                pos_score = self.distance(newpath) + self.heuristic(pos, goal)
                if pos in checked and checked[pos] <= pos_score: continue
                checked[pos] = pos_score
                heapq.heappush(queue, (pos_score, newpath))
                pass
            pass
        return None

    def nexts(self, map_img, pos):
        """ 今いる座標から八方の座標を計算する関数 """
        wall = 0
        if map_img[pos[0] - 1][pos[1]][0] != wall: yield (pos[0] - 1, pos[1])
        if map_img[pos[0] + 1][pos[1]][0] != wall: yield (pos[0] + 1, pos[1])
        if map_img[pos[0]][pos[1] - 1][0] != wall: yield (pos[0], pos[1] - 1)
        if map_img[pos[0]][pos[1] + 1][0] != wall: yield (pos[0], pos[1] + 1)
        pass

    def heuristic(self, pos, goal):
        """ スタートからゴールまでの最短距離を算出する関数 """
        return ((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2) ** 0.5

    def distance(self, path):
        """ スタートから探索している座標までの距離を算出する関数 """
        return len(path)

    def make_result_img(self, start, goal, path):
        #map_color = cv2.imread(self.pkg_path + "/img/resize_costmap.png")
        map_color = cv2.imread(self.img_path)
        for i in range(len(path)):
            cv2.circle(map_color,(path[i][1], path[i][0]), 1, (0,0,255), -1)
        cv2.circle(map_color,(path[-1][1], path[-1][0]), 4, (0,255,0), -1)
        cv2.circle(map_color,(start[1], start[0]), 4, (0,0,255), -1)
        cv2.circle(map_color,(goal[1], goal[0]), 4, (255,0,255), -1)
        show_img_size = (self.map_width/10, self.map_height/10)
        show_img = cv2.resize(map_color, show_img_size)
        cv2.imshow("path_image", show_img)
        cv2.waitKey(1)

        cv2.imwrite(self.pkg_path + "/img/path.png", map_color)

    def make_result_img_x10(self, start, goal, path):
        """画像を10倍したもの"""
        map_color = cv2.imread(self.pkg_path + "/img/aerial_camera_.png")
        #map_color = cv2.imread(self.img_path)
        for i in range(len(path)):
            #cv2.circle(map_color,(path[i][1], path[i][0]), 1, (0,0,255), -1)
            #img = cv2.line(img,(start_y,start_x),(goal_y,goal_x),(255,0,0),5)
            if i == 0:
                cv2.line(map_color,(start[1]*10,start[0]*10),(path[i][1]*10,path[i][0]*10),(0,0,255),5)
            else:
                cv2.line(map_color,(path[i-1][1]*10,path[i-1][0]*10),(path[i][1]*10,path[i][0]*10),(0,0,255),5)

        cv2.circle(map_color,(path[-1][1]*10, path[-1][0]*10), 4, (0,255,0), -1)
        cv2.circle(map_color,(start[1]*10, start[0]*10), 20, (0,0,255), -1)
        cv2.circle(map_color,(goal[1]*10, goal[0]*10), 20, (255,0,255), -1)

        cv2.imwrite(self.pkg_path + "/img/path_.png", map_color)

        #show_img_size = (self.map_width/5, self.map_height/5)
        #show_img = cv2.resize(map_color, show_img_size)
        #cv2.imshow("path_image x 10", show_img)
        #cv2.waitKey(1)

    def publish_path_msg(self, start, goal, path):
        ### 引数のpathは1/10のサイズで計測したものなので注意
        path_msg = Path()

        for i in range(len(path)):
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = path[i][0]*10
            poseStamped.pose.position.y = path[i][1]*10
            path_msg.poses.append(poseStamped)

        poseStamped = PoseStamped()
        poseStamped.pose.position.x = goal[0]*10
        poseStamped.pose.position.y = goal[1]*10

        """for i in range(len(path)):
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = path[i][0]
            poseStamped.pose.position.y = path[i][1]
            path_msg.poses.append(poseStamped)

        poseStamped = PoseStamped()
        poseStamped.pose.position.x = goal[0]
        poseStamped.pose.position.y = goal[1]
        path_msg.poses.append(poseStamped)"""

        self.pub_path.publish(path_msg)

        self.GeneratePathFlag = True


    def generate_path(self):
        start_x = int(self.TargetRibbonBridge.center.x)
        start_y = int(self.TargetRibbonBridge.center.y)

        goal_x = int(self.Goal_pose.position.x)
        goal_y = int(self.Goal_pose.position.y)

        start = (start_y, start_x)
        goal = (goal_y, goal_x)

        #print start
        #print goal

        costmap = cv2.imread(self.map_path)

        try:
            #1/10のサイズにして計算する
            new_size = (self.map_width/10, self.map_height/10)

            resize_costmap = cv2.resize(costmap, new_size)
            cv2.imwrite(self.pkg_path + "/img/resize_costmap.png", resize_costmap)

            #resize_costmap = cv2.imread(self.pkg_path + "/img/resize_costmap.png")
            start = (start_y/10, start_x/10)
            goal = (goal_y/10, goal_x/10)
            find_path = self.astar(resize_costmap, start, goal, self.distance, self.heuristic)

            #そのままのサイズで計算
            #find_path = self.astar(costmap, start, goal, self.distance, self.heuristic)


            if find_path != None:
                rospy.loginfo("RouteGenerator -> Found Path ")
                self.publish_path_msg(start, goal, find_path)

                self.make_result_img(start, goal, find_path)
                #self.make_result_img_x10(start, goal, find_path)

                self.create_blank_map() #mapをクリアする


            else:
                rospy.logwarn("RouteGenerator -> Can not Find Path ")
                pass

        except:
            #rospy.logerr("RouteGenerator -> Error ")
            pass

    def main(self):
        rospy.loginfo("***** start *****")

        self.Goal_pose.position.x = self.map_width/2
        self.Goal_pose.position.y = self.map_height/2


        while not rospy.is_shutdown():
            #必要な情報がsubscribeされるまでストップする
            if self.GetImageFlag == True and self.GetBridgeResultFlag == True:
                self.create_blank_map()
                break

        while not rospy.is_shutdown():
            #self.show_img()
            self.generate_path()

            if self.GeneratePathFlag == True:
                break


        rospy.loginfo("***** finish *****")





if __name__ == "__main__":
    rospy.init_node("RouteGenerator", anonymous=True)
    rg = RouteGenerator()
    rg.main()
    rospy.spin()
