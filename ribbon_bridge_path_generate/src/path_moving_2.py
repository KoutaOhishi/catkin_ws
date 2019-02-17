#!/usr/bin/env python
#coding: utf-8
import rospy
import sys
import time
import math
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *
from matplotlib import pyplot as plt
from nav_msgs.msg import *

class LinearControl():
    def __init__(self):
        self.Model_name = "tug_boat_0"
        self.Reference_frame = "world"

        self.Goal_pose = Pose()
        self.Model_pose = Pose()

        self.True_Model_state = ModelState() #浮体の位置の真値

        self.Path = Path()
        self.GetPathFlag = False #Pathを受け取れたらTrue
        self.Path_status = False #Pathの生成が上手くできていればTrue

        self.Arrival_distance = 5 #この値より小さくなれば到着したとみなす
        self.NotArrival_distance = self.Arrival_distance + 1

        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

        #self.sub_RibbonBridgePose_1 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, self.sub_RibbonBridgePose_1_CB)
        self.sub_RibbonBridgePose_2 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, self.sub_RibbonBridgePose_2_CB)
        #self.sub_RibbonBridgePose_3 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, self.sub_RibbonBridgePose_3_CB)

        self.sub_True_Model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_True_Model_pose_CB)

        self.sub_Path = rospy.Subscriber("ribbon_bridge_path_generate/path2", Path, self.sub_Path_CB)

        self.sub_Boat_num = rospy.Subscriber("/darknet_ros/found_object", Int8, self.sub_Boat_num_CB) #YOLOのノードとの接続を確認するために作成

        self.sub_Path_status = rospy.Subscriber("/ribbon_bridge_path_generate/status2", Bool, self.sub_Path_status_CB)

        self.pub_Is_arrived = rospy.Publisher("/ribbon_bridge_path_generate/is_RibbonBridge2_arrived", Bool, queue_size=1)

        #PID制御用のパラメータ
        self.DistX_1 = 0.0
        self.DistY_1 = 0.0
        self.DistZ_1 = 0.0
        self.DistX_2 = 0.0
        self.DistY_2 = 0.0
        self.DistZ_2 = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.Fz = 0.0
        self.Kp = 0.005#0.05#0.01
        self.Ki = 0.00000
        self.Kd = 0.00000
        self.count = 0


    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg.data

    def sub_Boat_num_CB(self, msg):
        self.Boat_num = msg.data

    def sub_Path_status_CB(self, msg):
        self.Path_status = msg.data

    def sub_Path_CB(self, msg):
        if self.Path == msg:
            self.GetPathFlag = False
        else:
            self.Path = msg
            self.GetPathFlag = True

    def sub_RibbonBridgePose_2_CB(self, msg):
        self.RibbonBridgePose_2 = msg
        self.Model_pose = msg

        #到着判定
        if self.ArrivedFlag_X == False:
            #if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
            if abs(self.Model_pose.position.x - self.Goal_pose.position.x) < self.Arrival_distance:
                self.stop_ribbon_bridge("x")
                self.ArrivedFlag_X = True
                #rospy.loginfo("Arrived X")

        else: #self.ArrivedFlag_X == True:
            #if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
            if abs(self.Model_pose.position.x - self.Goal_pose.position.x) >= self.NotArrival_distance:
                self.ArrivedFlag_X = False

        if self.ArrivedFlag_Y == False:
            #if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
            if abs(self.Model_pose.position.y - self.Goal_pose.position.y) < self.Arrival_distance:
                self.stop_ribbon_bridge("y")
                self.ArrivedFlag_Y = True
                #rospy.loginfo("Arrived Y")

        else: #self.ArrivedFlag_Y == True:
            #if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
            if abs(self.Model_pose.position.y - self.Goal_pose.position.y) >= self.NotArrival_distance:
                self.ArrivedFlag_Y = False

    def sub_RibbonBridgePose_1_CB(self, msg):
        self.RibbonBridgePose_1 = msg

    def sub_RibbonBridgePose_3_CB(self, msg):
        self.RibbonBridgePose_3 = msg

    def sub_True_Model_pose_CB(self, msg):
        """ gazebo空間における浮体の位置の真値を取得する """
        i = msg.name.index(self.Model_name)
        self.True_Model_state.model_name = msg.name[i]
        self.True_Model_state.pose = msg.pose[i]
        self.True_Model_state.twist = msg.twist[i]

    def stop_ribbon_bridge(self, way):
        """ YOLOが止まった時は強制的に浮き橋を停止させる """
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state_srv = ModelState()

        model_state_srv.model_name = self.Model_name
        model_state_srv.reference_frame = self.Reference_frame
        model_state_srv.pose = self.True_Model_state.pose
        model_state_srv.twist = self.True_Model_state.twist

        if way == "x":
            model_state_srv.twist.linear.x = 0.000

        elif way == "y":
            model_state_srv.twist.linear.y = 0.000

        elif way == "z":
            pass

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        try:
            set_model_state(model_state=model_state_srv)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/set_model_state] Exception")
            rospy.sleep(5)

    def boat_acceleration(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        ### 浮き橋にかかる加速度を設定する
        model_state = ModelState()
        model_state.model_name = self.Model_name
        model_state.reference_frame = self.Reference_frame
        model_state.pose = self.True_Model_state.pose
        model_state.twist = self.True_Model_state.twist

        if linear_x != 0.0:
            model_state.twist.linear.x = linear_x
        if linear_y != 0.0:
            model_state.twist.linear.y = linear_y
        if linear_z != 0.0:
            model_state.twist.linear.z = linear_z

        if angular_x != 0.0:
            model_state.twist.angular.x = angular_x
        if angular_y != 0.0:
            model_state.twist.angular.y = angular_y
        if angular_z != 0.0:
            model_state.twist.angular.z = angular_z

        try:
            ###Publish###
            #pub_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
            #pub_model_state.publish(model_state)

            ###Service###
            set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            set_model_state(model_state=model_state)

        except:
            rospy.logerr("Failed to set ModelState")
            rospy.sleep(5)

    def pid_control(self):
        dist_x = self.Goal_pose.position.x - self.Model_pose.position.x
        dist_y = self.Model_pose.position.y - self.Goal_pose.position.y

        fx = self.Fx + self.Kp * (dist_x-self.DistX_1) + self.Ki * dist_x + self.Kd * ((dist_x-self.DistX_1)-(self.DistX_1-self.DistX_2))
        fy = self.Fy + self.Kp * (dist_y-self.DistY_1) + self.Ki * dist_y + self.Kd * ((dist_y-self.DistY_1)-(self.DistY_1-self.DistY_2))

        if self.count != 0:#１回めの計算は数値が正しくないので飛ばす
            self.boat_acceleration(fx, fy, 0.0, 0.0, 0.0, 0.0)

        self.Fx = fx
        self.Fy = fy
        #self.Fz = tz

        self.DistX_2 = self.DistX_1
        self.DistY_2 = self.DistY_1
        #self.DistZ_2 = self.DistZ_1

        self.DistX_1 = dist_x
        self.DistY_1 = dist_y
        #self.DistZ_1 = dist_z

        self.count += 1
        time.sleep(1)

        print "Goal_position:x[%s] y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
        print "Model_position:x[%s] y[%s]"%(str(round(self.Model_pose.position.x)),str(round(self.Model_pose.position.y)))
        print "Distance:x[%s], y[%s]"%(str(round(dist_x, 2)), str(round(dist_y, 2)))
        print "Acceleration:x[%s], y[%s]"%(str(round(fx, 2)), str(round(fy, 2)))
        print "Arrived:x[%s], y[%s]"%(str(self.ArrivedFlag_X), str(self.ArrivedFlag_Y))
        print "---"

        if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
            self.pub_Is_arrived.publish(True)
        else:
            self.pub_Is_arrived.publish(False)

    def main(self):
        rospy.loginfo("waiting path")
        while not rospy.is_shutdown():
            if len(self.Path.poses) != 0:
                rospy.loginfo("***** Start *****")
                break

        next_goal_flag = False

        ###
        while not rospy.is_shutdown():
            if len(self.Path.poses) <=10:
                break
            else:
                self.Goal_pose.position.x = self.Path.poses[10].pose.position.y
                self.Goal_pose.position.y = self.Path.poses[10].pose.position.x

            if self.sub_Boat_num.get_num_connections() == 0 or self.Path_status == False:
                self.stop_ribbon_bridge("x")
                self.stop_ribbon_bridge("y")
                self.stop_ribbon_bridge("z")

                if self.Path_status == False:
                    #経路生成に問題発生
                    rospy.logwarn("Some error has occured")

                if self.sub_Boat_num.get_num_connections() == 0:
                    #YOLOのノードとの接続が切れた場合
                    rospy.logwarn("YOLO has Dead")
                rospy.sleep(1)

            else:
                self.pid_control()
        ###

        """while not rospy.is_shutdown():
            if len(self.Path.poses) <= 2:
                break

            else:
                pass

            if next_goal_flag == True:
                self.Goal_pose.position.x = self.Path.poses[2].pose.position.y
                self.Goal_pose.position.y = self.Path.poses[2].pose.position.x

            else:
                self.Goal_pose.position.x = self.Path.poses[1].pose.position.y
                self.Goal_pose.position.y = self.Path.poses[1].pose.position.x

            if self.sub_Boat_num.get_num_connections() == 0 or self.Path_status == False:
                self.stop_ribbon_bridge("x")
                self.stop_ribbon_bridge("y")
                self.stop_ribbon_bridge("z")

                if self.Path_status == False:
                    #経路生成に問題発生
                    rospy.logwarn("Some error has occured")

                if self.sub_Boat_num.get_num_connections() == 0:
                    #YOLOのノードとの接続が切れた場合
                    rospy.logwarn("YOLO has Dead")
                rospy.sleep(1)

            else:
                if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                    self.ArrivedFlag_X == False
                    self.ArrivedFlag_Y == False
                    #rospy.loginfo("next sub-Goal Position")
                    #if next_goal_flag == False:
                    next_goal_flag = True
                    #else:
                        #next_goal_flag = False

                self.pid_control()
        """
        self.Goal_pose.position.x = self.Path.poses[len(self.Path.poses)-1].pose.position.y
        self.Goal_pose.position.y = self.Path.poses[len(self.Path.poses)-1].pose.position.x


        #goalでstay
        while not rospy.is_shutdown():
            self.pid_control()

            if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                rospy.loginfo("***** Arrived *****")

if __name__ == "__main__":
    rospy.init_node("LinearControl", anonymous=True)
    c = LinearControl()
    c.main()
    rospy.spin()
