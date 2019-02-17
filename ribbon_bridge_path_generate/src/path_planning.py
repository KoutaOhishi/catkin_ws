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


class PathPlanning():
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

        self.Goal_pose = Pose() #目的地

        self.Boat_diagonal = 180 #75m上空から撮影した時の浮体のサイズ
        self.Costmap_size = 300

        #浮体のPose
        self.RibbonBridgePose_1 = Pose() #制御対象
        self.RibbonBridgePose_2 = Pose() #障害物
        self.RibbonBridgePose_3 = Pose() #障害物

        #それぞれの浮体の位置をsubscribeする
        self.sub_RibbonBridgePose_1 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, self.sub_RibbonBridgePose_1_CB)
        self.sub_RibbonBridgePose_2 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, self.sub_RibbonBridgePose_2_CB)
        self.sub_RibbonBridgePose_3 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, self.sub_RibbonBridgePose_3_CB)

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_position", Pose, self.sub_Goal_pose_CB)

        self.sub_Image = rospy.Subscriber(self.img_topic_name, Image, self.sub_Image_CB)

        self.pub_path = rospy.Publisher("/ribbon_bridge_path_generate/path", Path, queue_size=1)

        self.pub_status = rospy.Publisher("/ribbon_bridge_path_generate/status", Bool, queue_size=1)
        self.pub_status_counter = 1

        #flagで処理のタイミングを制御する
        self.GetBridgeResultFlag = False
        self.GetImageFlag = False
        self.CreatedBlankImageFlag = False
        self.GeneratePathFlag = False

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg

    def sub_RibbonBridgePose_1_CB(self, msg):
        self.RibbonBridgePose_1 = msg

    def sub_RibbonBridgePose_2_CB(self, msg):
        self.RibbonBridgePose_2 = msg

    def sub_RibbonBridgePose_3_CB(self, msg):
        self.RibbonBridgePose_3 = msg

    def create_costmap(self):
        try:
            if self.CreatedBlankImageFlag == True:
                #障害物の浮体の位置をコストマップに追加する
                self.add_cost(self.RibbonBridgePose_2.position.x, self.RibbonBridgePose_2.position.y, self.Costmap_size)
                self.add_cost(self.RibbonBridgePose_3.position.x, self.RibbonBridgePose_3.position.y, self.Costmap_size)
                return True

        except:
            rospy.logerr("Failed to create_costmap")
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
            cv2.imwrite(self.pkg_path + "/img/aerial_camera_.png", cv_img)

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

    def add_cost(self, centerX, centerY, costmap_size):
        costmap = cv2.imread(self.map_path)

        #円形のコストマップ
        #cv2.circle(costmap, (int(centerX), int(centerY)), int(costmap_size/1.25), (0,0,0), -1)

        #四角形のコストマップ
        #cv2.rectangle(costmap, (int(centerX-costmap_size/1.5),int(centerY-costmap_size/2)), (int(centerX+costmap_size/1.5),int(centerY+costmap_size/2)), (0,0,0), -1)
        cv2.rectangle(costmap, (int(centerX-costmap_size),int(centerY-costmap_size/1.5)), (int(centerX+costmap_size),int(centerY+costmap_size/1.5)), (0,0,0), -1)

        cv2.imwrite(self.map_path, costmap)

        #show_img_size = (self.map_width/10, self.map_height/10)
        show_img_size = (self.map_width/5, self.map_height/5)
        show_img = cv2.resize(costmap, show_img_size)

        #cv2.imshow("costmap", show_img)
        #cv2.waitKey(1)


    def show_img(self):
        img = cv2.imread(self.map_path)

        try:
            show_img_size = (self.map_width/10, self.map_height/10)
            #show_img_size = (self.map_width/5, self.map_height/5)
            show_img = cv2.resize(img, show_img_size)
            cv2.imshow("path_image", show_img)
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
        #cv2.imshow("path_image", show_img)
        #cv2.waitKey(1)

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
        path_msg = Path()
        for i in range(len(path)):
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = path[i][0]*10
            poseStamped.pose.position.y = path[i][1]*10
            path_msg.poses.append(poseStamped)

        print "Path_num:[%s]"%str(len(path_msg.poses))
        self.pub_path.publish(path_msg)
        self.GeneratePathFlag = True

    def _publish_path_msg(self, start, goal, path):
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
                print "---"
                """

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
        start_x = int(self.RibbonBridgePose_1.position.x)
        start_y = int(self.RibbonBridgePose_1.position.y)

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
            cv2.imwrite(self.pkg_path + "/img/resize_costmap.png", resize_costmap)

            start = (start_y/10, start_x/10)
            goal = (goal_y/10, goal_x/10)
            #start = (start_y/5, start_x/5)
            #goal = (goal_y/5, goal_x/5)
            find_path = self.astar(resize_costmap, start, goal, self.distance, self.heuristic)

            #そのままのサイズで計算
            #find_path = self.astar(costmap, start, goal, self.distance, self.heuristic)


            if find_path != None:
                rospy.loginfo("PathPlanning -> Found Path ")

                self.publish_path_msg(start, goal, find_path)
                self.publish_path_status(True)

                #self.make_result_img(start, goal, find_path)
                #self.make_result_img_x10(start, goal, find_path)

            else:
                rospy.logwarn("PathPlanning -> Can not Find Path ")
                #self.publish_path_status(False)
                pass

        except:
            rospy.logerr("PathPlanning -> Error ")
            self.publish_path_status(False)
            pass


    def main(self):
        #self.Goal_pose.position.x = self.map_width/2 + 300
        #self.Goal_pose.position.y = self.map_height/2

        #self.Goal_pose.position.x = self.map_width/2 + 500 + 200
        #self.Goal_pose.position.y = self.map_height/2

        self.Goal_pose.position.x = 1106
        self.Goal_pose.position.y = 450



        while not rospy.is_shutdown():
            #必要な情報がsubscribeされるまでストップする
            if self.GetImageFlag == True:
                self.create_blank_map()
                break

        rospy.loginfo("***** start *****")

        while not rospy.is_shutdown():
            _cost_map = self.create_costmap()
            if _cost_map == True:
                _path = self.generate_path()
                _blank_map = self.create_blank_map()
            else:
                pass


if __name__ == "__main__":
    rospy.init_node("PathPlanning", anonymous=True)
    pp= PathPlanning()
    pp.main()
    rospy.spin()
