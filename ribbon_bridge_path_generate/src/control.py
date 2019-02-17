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

class SelfMove():
    def __init__(self):
        self.Model_name = "tug_boat"
        self.Reference_frame = "world"

        self.Model_pose = Pose()
        self.pastModel_pose = Pose()
        self.Boat_diagonal = 10.0

        self.Model_corners = [(0,0),(0,0),(0,0),(0,0)]
        self.Model_theta = Quaternion()
        self.Goal_pose = Pose()
        self.True_Model_state = ModelState() #浮体の位置の真値

        self.Duration_time = 0.1 #一回の操作で浮体に力を与える時間
        self.Arrival_distance = 1.0 #この値より小さくなれば到着したと判定する(小さすぎると止まるタイミングを見失う)

        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_pose", Pose, self.sub_Goal_pose_CB)

        self.sub_Model_pose = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.sub_Model_pose_CB)

        self.sub_True_Model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_True_Model_pose_CB)

        self.pub_Model_pose = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        #self.pub_Model_pose = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)

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
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 0.0
        self.count = 1

    def euler_to_quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def degree_to_euler(self, degree):
        return Vector3(x=0.00, y=0.00, z=degree)

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg
        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

    def stop_ribbon_bridge(self, way):
        """ wayの向きにかかるforceを0にする→ブレーキをかける """
        pub_msg = ModelState()
        pub_msg.model_name = self.Model_name

        #pub_msg = LinkState()
        #pub_msg.link_name = self.Model_name + "::body"


        pub_msg.pose = self.True_Model_state.pose
        pub_msg.twist = self.True_Model_state.twist

        if way == "x":
            pub_msg.twist.linear.x = 0.0
            #if self.ArrivedFlag_Y == True:
                #pub_msg.twist.linear.y = 0.0
            #if self.ArrivedFlag_Z == True:
                #pub_msg.twist.angular.z = 0.0
            self.pub_Model_pose.publish(pub_msg)
            #rospy.sleep(0.1)

        elif way == "y":
            pub_msg.twist.linear.y = 0.0
            #if self.ArrivedFlag_X == True:
                #pub_msg.twist.linear.x = 0.0
            #if self.ArrivedFlag_Z == True:
                #pub_msg.twist.angular.z = 0.0
            self.pub_Model_pose.publish(pub_msg)
            #rospy.sleep(0.1)


        elif way == "z":
            pub_msg.twist.angular.z = 0.0
            #if self.ArrivedFlag_X == True:
                #pub_msg.twist.linear.x = 0.0
            #if self.ArrivedFlag_Y == True:
                #pub_msg.twist.linear.y = 0.0
            self.pub_Model_pose.publish(pub_msg)

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        """set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state_srv = ModelState()

        model_state_srv.model_name = self.Model_name
        model_state_srv.reference_frame = self.Reference_frame
        model_state_srv.pose = self.True_Model_state.pose
        model_state_srv.twist = self.True_Model_state.twist

        if way == "x":
            model_state_srv.twist.linear.y = 0.000

        elif way == "y":
            model_state_srv.twist.linear.x = 0.000

        elif way == "z":
            pass

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        try:
            set_model_state(model_state=model_state_srv)
            #print "##########"
            #print res

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/set_model_state] Exception")
            rospy.sleep(5)"""

    def sub_True_Model_pose_CB(self, msg):
        """ gazebo空間における浮体の位置の真値を取得する """
        i = msg.name.index(self.Model_name)
        self.True_Model_state.model_name = msg.name[i]
        self.True_Model_state.pose = msg.pose[i]
        self.True_Model_state.twist = msg.twist[i]

    def sub_Model_pose_CB(self, msg):
        """ 指定した浮体の位置をsubscribeする、Goalに到着したらブレーキをかける """
        i = 0

        if len(msg.RibbonBridges) == 0:
            print "There are no RibbonBridges"

        else:
            #self.Model_pose.position.x = msg.RibbonBridges[i].center.x
            #self.Model_pose.position.y = msg.RibbonBridges[i].center.y
            #１つ前のタイミングでの浮体の位置から最も近い浮体を制御対象にする（トラッキング的な処理）
            dist_list = []
            for i in range(len(msg.RibbonBridges)):
                dist = math.sqrt(pow(self.pastModel_pose.position.x-msg.RibbonBridges[i].center.x,2)+pow(self.pastModel_pose.position.y-msg.RibbonBridges[i].center.y,2))

                #if dist < self.Boat_diagonal:
                dist_list.append(dist)

            if len(dist_list) == 0:#条件を満たす浮体がいない場合
                self.stop_ribbon_bridge("x")
                self.stop_ribbon_bridge("y")
                self.stop_ribbon_bridge("z")

            else:
                target_index = dist_list.index(min(dist_list))
                self.Model_pose.position.x = msg.RibbonBridges[target_index].center.x
                self.Model_pose.position.y = msg.RibbonBridges[target_index].center.y
                self.pastModel_pose = self.Model_pose

                diagonal = pow((msg.RibbonBridges[target_index].corners[0].x-msg.RibbonBridges[target_index].corners[2].x),2) + pow((msg.RibbonBridges[target_index].corners[0].y-msg.RibbonBridges[target_index].corners[2].y),2)
                self.Boat_diagonal = math.sqrt(diagonal)

                if self.ArrivedFlag_X == False:
                    #if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
                    if abs(self.Model_pose.position.x - self.Goal_pose.position.x) < self.Arrival_distance:
                        self.stop_ribbon_bridge("x")
                        self.ArrivedFlag_X = True
                        #rospy.loginfo("Arrived X")

                else: #self.ArrivedFlag_X == True:
                    #if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
                    if abs(self.Model_pose.position.x - self.Goal_pose.position.x) >= self.Arrival_distance:
                        self.ArrivedFlag_X = False

                if self.ArrivedFlag_Y == False:
                    #if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
                    if abs(self.Model_pose.position.y - self.Goal_pose.position.y) < self.Arrival_distance:
                        self.stop_ribbon_bridge("y")
                        self.ArrivedFlag_Y = True
                        #rospy.loginfo("Arrived Y")

                else: #self.ArrivedFlag_Y == True:
                    #if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
                    if abs(self.Model_pose.position.y - self.Goal_pose.position.y) >= self.Arrival_distance:
                        self.ArrivedFlag_Y = False

                #回転の計算
                corner_1 = (msg.RibbonBridges[i].corners[0].x, msg.RibbonBridges[i].corners[0].y)
                corner_2 = (msg.RibbonBridges[i].corners[1].x, msg.RibbonBridges[i].corners[1].y)
                corner_3 = (msg.RibbonBridges[i].corners[2].x, msg.RibbonBridges[i].corners[2].y)
                corner_4 = (msg.RibbonBridges[i].corners[3].x, msg.RibbonBridges[i].corners[3].y)

                self.Model_corners[0] = corner_1
                self.Model_corners[1] = corner_2
                self.Model_corners[2] = corner_3
                self.Model_corners[3] = corner_4

                try:
                    len1_4 = math.sqrt( (corner_1[0]-corner_4[0])**2 + (corner_1[1]-corner_4[1])**2)
                    len1_2 = math.sqrt( (corner_1[0]-corner_2[0])**2 + (corner_1[1]-corner_2[1])**2)

                    #print "1: x[%s] y[%s]"%(str(corner_1[0]), str(corner_1[1]))
                    #print "2: x[%s] y[%s]"%(str(corner_2[0]), str(corner_2[1]))
                    #print "4: x[%s] y[%s]"%(str(corner_4[0]), str(corner_4[1]))

                    #rad = math.atan(int(corner_4[1]-corner_1[1])/int(corner_4[0]-corner_1[0]))
                    #deg = math.degrees(rad)
                    #self.Model_theta = self.euler_to_quaternion(self.degree_to_euler(deg))

                    #print "-"
                    #print "rad:[%s]"%str(rad)
                    #print "deg:[%s]"%str(deg)
                    #print "len1_4:[%s]"%str(len1_4)
                    #print "len1_2:[%s]"%str(len1_2)
                    #print "w:[%s]"%str(self.Model_theta.w)

                    print "---"

                    if self.ArrivedFlag_Z == False:
                        if abs(corner_1[0]-corner_4[0]) < 2.0 and abs(corner_1[1]-corner_2[1]) < 2.0:
                            self.stop_ribbon_bridge("z")
                            self.ArrivedFlag_Z = True
                            print "Arrived Z"

                    else:
                        if abs(corner_1[0]-corner_4[0]) < 2.0 and abs(corner_1[1]-corner_2[1]) < 2.0:
                            pass
                        else:
                            self.ArrivedFlag_Z = False

                except:
                    pass





        """#到着の確認
        if self.ArrivedFlag_X == False:
            if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.linear.x = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_X = True

        else:
            if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
                self.ArrivedFlag_X = False

        if self.ArrivedFlag_Y == False:
            if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.linear.y = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_Y = True

        else:
            if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
                self.ArrivedFlag_Y = False


        if self.ArrivedFlag_Z == False:
            if round(self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z, 2) == round(self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z, 2):
            #if round(self.Model_pose.orientation.z,2) == self.Goal_pose.orientation.z:
                #if round(self.Model_pose.orientation.w,2) == self.Goal_pose.orientation.w:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.angular.z = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_Z = True

        else:
            if round(self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z, 2) != round(self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z, 2):
                self.ArrivedFlag_Z = False"""

    def euler_to_quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    def Add_force_with_param(self, fx, fy, fz, tx, ty, tz):
        apply_body_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
        get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

        target_link = self.Model_name + "::body"
        wrench = Wrench()
        duration = rospy.Duration(self.Duration_time) #単位はsecond

        wrench.force.x = fx
        wrench.force.y = fy
        wrench.force.z = fz
        wrench.torque.x = tx
        wrench.torque.y = ty
        wrench.torque.z = tz

        if self.ArrivedFlag_Z == True:
            wrench.torque.z = 0.0

        try:
            apply_body_wrench(body_name=target_link,
            reference_frame=self.Reference_frame,
            wrench=wrench,
            duration=duration)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/apply_body_wrench] Exception")
            rospy.sleep(5)

    def turn(self):
        if self.ArrivedFlag_Z == True:
            pass

        else:
            dist_z = self.Model_corners[1][1] - self.Model_corners[0][1]

            tz = self.Fz + self.Kp * (dist_z-self.DistZ_1) + self.Ki * dist_z + self.Kd * ((dist_z-self.DistZ_1)-(self.DistZ_1-self.DistZ_2))

            self.Add_force_with_param(0.0,0.0,0.0,0.0,0.0,tz)

            self.Fz = tz
            self.DistZ_2 = self.DistZ_1
            self.DistZ_1 = dist_z

    def move(self):
        if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
            #print "Arrived Goal_position:x[%s], y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
            pass

        else:
            #dist_x = self.Model_pose.position.x - self.Goal_pose.position.x
            dist_x = self.Goal_pose.position.x - self.Model_pose.position.x
            dist_y = self.Model_pose.position.y - self.Goal_pose.position.y

            fx = self.Fx + self.Kp * (dist_x-self.DistX_1) + self.Ki * dist_x + self.Kd * ((dist_x-self.DistX_1)-(self.DistX_1-self.DistX_2))
            fy = self.Fy + self.Kp * (dist_y-self.DistY_1) + self.Ki * dist_y + self.Kd * ((dist_y-self.DistY_1)-(self.DistY_1-self.DistY_2))

            #self.Add_force_with_param(-dist_x,dist_y,0.0,0.0,0.0,dist_z)
            self.Add_force_with_param(fx,fy,0.0,0.0,0.0,0.0)

            self.Fx = fx
            self.Fy = fy
            #self.Fz = tz

            self.DistX_2 = self.DistX_1
            self.DistY_2 = self.DistY_1
            #self.DistZ_2 = self.DistZ_1

            self.DistX_1 = dist_x
            self.DistY_1 = dist_y
            #self.DistZ_1 = dist_z

            print "Goal:x[%s] y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
            print "Now:x[%s] y[%s] "%(str(round(self.Model_pose.position.x, 2)),str(round(self.Model_pose.position.y,2)))
            print "Distance:x[%s] y[%s] "%(str(round(dist_x, 2)), str(round(dist_y, 2)))
            print "Arrived x:[%s] y:[%s] z:[%s]"%(self.ArrivedFlag_X, self.ArrivedFlag_Y, self.ArrivedFlag_Z)
            print "---"



    def main(self):
        self.Goal_pose.position.x = 800#4096/2
        self.Goal_pose.position.y = 450#2160/2

        self.Goal_pose.orientation.z = 0.707
        self.Goal_pose.orientation.w = 0.707

        while not rospy.is_shutdown():
            self.move()
            self.turn()
            #rospy.sleep(0.1)

            #if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                #rospy.loginfo("Stopping Ribbon Bridge")
                #rospy.sleep(3)
                #rospy.loginfo("***** Arrived Goal Position *****")
                #break

        rospy.loginfo("***** Arrived Goal Position *****")





if __name__ == "__main__":
    rospy.init_node("SelfMove", anonymous=True)
    c = SelfMove()
    c.main()
    rospy.spin()
