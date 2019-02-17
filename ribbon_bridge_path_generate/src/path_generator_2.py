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
from std_msgs.msg import Bool


class RouteGenerator():
    def __init__(self):
        self.pkg_path = rospkg.RosPack().get_path('ribbon_bridge_path_generate')

        #空撮画像を保存するpath
        self.img_path = self.pkg_path + "/img/aerial_camera2.png"

        #生成したmap画像のpath
        self.map_path = self.pkg_path + "/img/map2.png"

        #map作成のために生成した白紙の画像ファイルのpath
        self.blank_map_path = self.pkg_path + "/img/blank2.png"

        #空撮画像とmap画像を合わせた画像のpath
        self.result_path = self.pkg_path + "/img/route2.png"

        #Subscribeするimgトピック名を取得
        self.img_topic_name = "/aerial_camera/camera1/image_raw"

        #imgのwidthとheightの設定(Subscribeの時に更新する)
        self.map_width = 1600#4096
        self.map_height = 900#2160

        #検出した浮体の縦と横の長さを設定
        self.Boat_width = 0.0
        self.Boat_height = 0.0
        self.Boat_diagonal = 1000
        self.GetBoatDiagonalFlag = False

        #環境内に存在する浮体の個数
        self.Boat_num = 2

        #costmapの大きさ（浮体の対角線の２倍にする）
        self.costmap = 0

        #複数の浮体の中で何番目を制御対象にするかを指定する
        self.target_model_index = 0

        self.Start_pose = Pose()
        self.Target_pose = Pose()
        self.pastStart_pose = Pose()
        self.TargetRibbonBridge = RibbonBridge()
        self.RibbonBridges = RibbonBridges()
        #self.OtherRibbonBridges = RibbonBridges()

        self.Goal_pose = Pose()

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_position", Pose, self.sub_Goal_pose_CB)

        self.sub_Target_pose = rospy.Subscriber("/ribbon_bridge_path_generate/control_ribbon_bridge_pose_2", Pose, self.sub_Target_pose_CB)

        self.sub_Result_data = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.sub_Result_data_CB)

        self.sub_Image = rospy.Subscriber(self.img_topic_name, Image, self.sub_Image_CB)

        self.pub_path = rospy.Publisher("/ribbon_bridge_path_generate/path2", Path, queue_size=1)

        self.pub_status = rospy.Publisher("/ribbon_bridge_path_generate/status2", Bool, queue_size=1)
        self.pub_status_counter = 1

        #flagで処理のタイミングを制御する
        self.GetBridgeResultFlag = False
        self.GetImageFlag = False
        self.CreatedBlankImageFlag = False
        self.GeneratePathFlag = False

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg

    def sub_Result_data_CB(self, msg):
        #rospy.loginfo("Subscribed result_data")
        if len(msg.RibbonBridges) == 0:
            print "There are no RibbonBridges"

        else:
            self.GetBridgeResultFlag = True
            self.RibbonBridges = msg.RibbonBridges

    def sub_Target_pose_CB(self, msg):
        self.Target_pose = msg

    def create_costmap(self):
        if len(self.RibbonBridges) == 0:
            print "There are no RibbonBridges"
            return False

        else:
            #トラッキング処理(暫定版)
            try:
                dist_list = []
                for i in range(len(self.RibbonBridges)):
                    dist = math.sqrt(pow(self.pastStart_pose.position.x-self.RibbonBridges[i].center.x,2)+pow(self.pastStart_pose.position.y-self.RibbonBridges[i].center.y,2))

                    dist_list.append(dist)

            #except:
                #rospy.logerr("Index error")
                #pass

                if len(dist_list) != self.Boat_num:
                    #rospy.logwarn("There are only %s ribbon_bridges. We need more %s ribbon_bridges"%(str(len(dist_list)), str(self.Boat_num-len(dist_list))))
                    #return False
                    pass
                else:
                    pass

                if len(dist_list) == 0:#条件を満たす浮体がいない場合
                    rospy.logwarn("Lost Target RibbonBridge")
                    return False

                else:
                    if min(dist_list) < self.Boat_diagonal:
                        target_index = dist_list.index(min(dist_list))

                        self.Start_pose.position.x = self.RibbonBridges[target_index].center.x
                        self.Start_pose.position.y = self.RibbonBridges[target_index].center.y

                        self.pastStart_pose = self.Start_pose

                        #浮体の対角線の長さを調べる（コストマップの円の半径に用いる)
                        diagonal = pow((self.RibbonBridges[target_index].corners[0].x-self.RibbonBridges[target_index].corners[2].x),2) + pow((self.RibbonBridges[target_index].corners[0].y-self.RibbonBridges[target_index].corners[2].y),2)

                        if math.sqrt(diagonal) > 200: #誤検出回避
                            #rospy.logwarn("this ribbon bridge is too large")
                            pass
                        else:
                            self.Boat_diagonal = math.sqrt(diagonal)
                            self.GetBoatDiagonalFlag = True

                            if self.CreatedBlankImageFlag == True:
                                #try:
                                self.TargetRibbonBridge = self.RibbonBridges[target_index]

                                for i in range(len(self.RibbonBridges)):
                                    if i == target_index:
                                        pass
                                    else:
                                        #targetの浮体と近すぎるものはコストマップに付与しない（誤認識なので)
                                        dist_target = math.sqrt(pow(self.TargetRibbonBridge.center.x-self.RibbonBridges[i].center.x,2)+pow(self.TargetRibbonBridge.center.y-self.RibbonBridges[i].center.y,2))
                                        if dist_target > self.Boat_diagonal:
                                            self.add_cost(self.RibbonBridges[i].center.x, self.RibbonBridges[i].center.y, self.Boat_diagonal)

                                return True
                    else:
                        rospy.logwarn("The nearest RibbonBridge is Lost")
                        return False

            except:
                #rospy.logerr("The index:[%s] is OUT of LENGTH of [/ribbon_bridge_measurement/result_data]"%str(target_index))
                return False

    def sub_Image_CB(self, msg):
        try:
            #rospy.loginfo("Subscribed Image Topic !")
            self.map_width = msg.width
            self.map_height = msg.height

            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            # 1/10にリサイズして保存
            new_size = (self.map_width/10, self.map_height/10)
            #new_size = (self.map_width/5, self.map_height/5)

            resize_img = cv2.resize(cv_img, new_size)
            cv2.imwrite(self.img_path, resize_img)

            # そのままのサイズで保存
            cv2.imwrite(self.pkg_path + "/img/aerial_camera_2.png", cv_img)

            self.GetImageFlag = True

        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic")

    def create_blank_map(self):
        """ 入力画像と同じサイズの白い画像を生成する """
        #rospy.loginfo("Created blank_img")
        # cv2.rectangleで埋めるだけ
        blank_img = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        cv2.rectangle(blank_img,(0,0),(self.map_width, self.map_height),(255,255,255),-1)

        cv2.imwrite(self.blank_map_path, blank_img)
        cv2.imwrite(self.map_path, blank_img)

        self.CreatedBlankImageFlag = True

    def add_cost(self, centerX, centerY, radius):
        costmap = cv2.imread(self.map_path)

        #円形のコストマップ
        cv2.circle(costmap, (int(centerX), int(centerY)), int(radius*2), (0,0,0), -1)

        #四角形のコストマップ
        #cv2.rectangle(costmap, (int(centerX-radius*1.5),int(centerY-radius*1.5)), (int(centerX+radius*1.5),int(centerY+radius*1.5)), (0,0,0), -1)

        cv2.imwrite(self.map_path, costmap)

        show_img_size = (self.map_width/10, self.map_height/10)
        #show_img_size = (self.map_width/5, self.map_height/5)

        show_img = cv2.resize(costmap, show_img_size)
        #cv2.imshow("costmap2", show_img)
        #cv2.waitKey(1)

    def show_img(self):
        img = cv2.imread(self.map_path)

        try:
            show_img_size = (self.map_width/10, self.map_height/10)
            #show_img_size = (self.map_width/5, self.map_height/5)
            show_img = cv2.resize(img, show_img_size)
            cv2.imshow("path_image2", show_img)
            cv2.waitKey(1)
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
        #show_img_size = (self.map_width/5, self.map_height/5)
        show_img = cv2.resize(map_color, show_img_size)
        cv2.imshow("path_image2", show_img)
        cv2.waitKey(1)

        cv2.imwrite(self.pkg_path + "/img/path2.png", map_color)

    def make_result_img_x10(self, start, goal, path):
        """画像を10倍したもの"""
        map_color = cv2.imread(self.pkg_path + "/img/aerial_camera_2.png")
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

        cv2.imwrite(self.pkg_path + "/img/path_2.png", map_color)

        #show_img_size = (self.map_width/5, self.map_height/5)
        #show_img = cv2.resize(map_color, show_img_size)
        #cv2.imshow("path_image x 10", show_img)
        #cv2.waitKey(1)

    def publish_path_status(self, status):
        if status == True:
            self.pub_status.publish(True)
            self.pub_status_counter = 1

        elif status == False:
            if self.pub_status_counter > 1:
                self.pub_status.publish(False)
                self.pub_status_counter = 1

            else:
                self.pub_status_counter += 1

        else:
            rospy.logwarn("***")

    def publish_path_msg(self, start, goal, path):
        ### 引数のpathは1/10のサイズで計測したものなので注意
        path_msg = Path()

        previous_slope = 1
        previous_x = 0
        previous_y = 0
        for i in range(len(path)):
            #print "x:[%s] y:[%s]"%(str(path[i][0]),str(path[i][1]))
            if i == 0:
                #poseStamped = PoseStamped()
                #poseStamped.pose.position.x = path[i][0]*10
                #poseStamped.pose.position.y = path[i][1]*10
                #path_msg.poses.append(poseStamped)
                pass

            elif i == len(path)-1:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = path[i][0]*10
                poseStamped.pose.position.y = path[i][1]*10
                #poseStamped.pose.position.x = path[i][0]*5
                #poseStamped.pose.position.y = path[i][1]*5
                path_msg.poses.append(poseStamped)

            else:
                increaseX = path[i+1][0]-path[i-1][0]
                increaseY = path[i+1][1]-path[i-1][1]

                if increaseX == 0 and increaseY == 0:
                    pass

                else:
                    if increaseX == 0:
                        slope = increaseY

                    elif increaseY == 0:
                        slope = increaseX

                    else:
                        slope = increaseY/increaseX

                    if slope != previous_slope:
                        poseStamped = PoseStamped()
                        poseStamped.pose.position.x = path[i-1][0]*10
                        poseStamped.pose.position.y = path[i-1][1]*10
                        #poseStamped.pose.position.x = path[i-1][0]*5
                        #poseStamped.pose.position.y = path[i-1][1]*5
                        path_msg.poses.append(poseStamped)

                    previous_slope = slope
                previous_x = path[i][0]
                previous_y = path[i][1]

        #path間の距離が浮体の対角線よりも短かったらそのpathは除外する
        _path_msg = path_msg
        path_msg = Path()
        for i in range(len(_path_msg.poses)):
            if i == 0:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = _path_msg.poses[i].pose.position.x
                poseStamped.pose.position.y = _path_msg.poses[i].pose.position.y
                path_msg.poses.append(poseStamped)

            elif i == len(_path_msg.poses)-1:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = _path_msg.poses[i].pose.position.x
                poseStamped.pose.position.y = _path_msg.poses[i].pose.position.y
                path_msg.poses.append(poseStamped)

            else:
                path_dist_low = math.sqrt(pow(_path_msg.poses[i-1].pose.position.x-_path_msg.poses[i].pose.position.x,2)+pow(_path_msg.poses[i-1].pose.position.y-_path_msg.poses[i].pose.position.y,2))
                path_dist_up = math.sqrt(pow(_path_msg.poses[i+1].pose.position.x-_path_msg.poses[i].pose.position.x,2)+pow(_path_msg.poses[i+1].pose.position.y-_path_msg.poses[i].pose.position.y,2))

                """print "path_id:[%s]"%str(i)
                print "dist_low:[%s]"%str(path_dist_low)
                print "dist_up:[%s]"%str(path_dist_up)
                print "Boat_diagonal:[%s]"%str(self.Boat_diagonal)
                """
                #print "---"



                if path_dist_low > self.Boat_diagonal/2 and path_dist_up > self.Boat_diagonal/2:
                    poseStamped = PoseStamped()
                    poseStamped.pose.position.x = _path_msg.poses[i].pose.position.x
                    poseStamped.pose.position.y = _path_msg.poses[i].pose.position.y
                    path_msg.poses.append(poseStamped)

                elif path_dist_low > self.Boat_diagonal/10:
                        poseStamped = PoseStamped()
                        poseStamped.pose.position.x = _path_msg.poses[i].pose.position.x
                        poseStamped.pose.position.y = _path_msg.poses[i].pose.position.y
                        path_msg.poses.append(poseStamped)

                else:
                    pass

        #1/10のサイズで計測した時用
        """for i in range(len(path)):
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = path[i][0]*10
            poseStamped.pose.position.y = path[i][1]*10
            path_msg.poses.append(poseStamped)

        poseStamped = PoseStamped()
        poseStamped.pose.position.x = goal[0]*10
        poseStamped.pose.position.y = goal[1]*10"""

        #そのままのサイズで計測した時用
        """for i in range(len(path)):
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = path[i][0]
            poseStamped.pose.position.y = path[i][1]
            path_msg.poses.append(poseStamped)

        poseStamped = PoseStamped()
        poseStamped.pose.position.x = goal[0]
        poseStamped.pose.position.y = goal[1]
        path_msg.poses.append(poseStamped)"""

        print "Path_num:[%s]"%str(len(path_msg.poses))

        self.pub_path.publish(path_msg)
        self.GeneratePathFlag = True

    def generate_path(self):
        #start_x = int(self.TargetRibbonBridge.center.x)
        #start_y = int(self.TargetRibbonBridge.center.y)
        start_x = int(self.Target_pose.position.x)
        start_y = int(self.Target_pose.position.y)

        goal_x = int(self.Goal_pose.position.x)
        goal_y = int(self.Goal_pose.position.y)

        start = (start_y, start_x)
        goal = (goal_y, goal_x)

        costmap = cv2.imread(self.map_path)

        try:
            #1/10のサイズにして計算する
            new_size = (self.map_width/10, self.map_height/10)
            #new_size = (self.map_width/5, self.map_height/5)

            resize_costmap = cv2.resize(costmap, new_size)
            cv2.imwrite(self.pkg_path + "/img/resize_costmap2.png", resize_costmap)

            start = (start_y/10, start_x/10)
            goal = (goal_y/10, goal_x/10)
            #start = (start_y/5, start_x/5)
            #goal = (goal_y/5, goal_x/5)
            find_path = self.astar(resize_costmap, start, goal, self.distance, self.heuristic)

            #そのままのサイズで計算
            #find_path = self.astar(costmap, start, goal, self.distance, self.heuristic)


            if find_path != None:
                rospy.loginfo("RouteGenerator -> Found Path ")

                self.publish_path_msg(start, goal, find_path)
                self.publish_path_status(True)

                self.make_result_img(start, goal, find_path)
                #self.make_result_img_x10(start, goal, find_path)

            else:
                rospy.logwarn("RouteGenerator -> Can not Find Path ")
                #self.publish_path_status(False)
                pass

        except:
            rospy.logerr("RouteGenerator -> Error ")
            self.publish_path_status(False)
            pass


    def main(self):
        self.Goal_pose.position.x = self.map_width/2 + 200
        self.Goal_pose.position.y = self.map_height/2

        self.pastStart_pose.position.x = 0#elf.map_width/2
        self.pastStart_pose.position.y = self.map_height


        while not rospy.is_shutdown():
            #必要な情報がsubscribeされるまでストップする
            if self.GetImageFlag == True and self.GetBridgeResultFlag == True:
                self.create_blank_map()
                break

        rospy.loginfo("***** start *****")

        while not rospy.is_shutdown():
            #self.show_img()
            _cost_map = self.create_costmap()
            if _cost_map == True:
                _path = self.generate_path()
                _blank_map = self.create_blank_map()
            else:
                pass





if __name__ == "__main__":
    rospy.init_node("RouteGenerator2", anonymous=True)
    rg = RouteGenerator()
    rg.main()
    rospy.spin()
