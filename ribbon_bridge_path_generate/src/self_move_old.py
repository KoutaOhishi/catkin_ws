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
from matplotlib import pyplot as plt

class SelfMove():
    def __init__(self):
        self.Model_name = "tug_boat"
        self.Reference_frame = "world"

        self.Model_pose = Pose()
        self.Goal_pose = Pose()

        self.Duration_time = 0.01 #一回の操作で浮体に力を与える時間

        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_pose", Pose, self.sub_Goal_pose_CB)
        self.sub_Model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_Model_pose_CB)

        self.pub_Model_pose = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

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
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.count = 1

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg

        #self.Goal_pose.orientation.z = 0.707 #横長
        #self.Goal_pose.orientation.z = 0.000 #縦長

    def sub_Model_pose_CB(self, msg):
        """ 指定した浮体の位置をsubscribeする、Goalに到着したらブレーキをかける """
        i = msg.name.index(self.Model_name)
        self.Model_pose = msg.pose[i]

        #到着の確認
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
                self.ArrivedFlag_Z = False

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

        try:
            apply_body_wrench(body_name=target_link,
            reference_frame=self.Reference_frame,
            wrench=wrench,
            duration=duration)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/apply_body_wrench] Exception")

    def pid(self):
        dist_x = self.Goal_pose.position.x - self.Model_pose.position.x
        dist_y = self.Goal_pose.position.y - self.Model_pose.position.y
        #dist_z = self.Goal_pose.orientation.z - self.Model_pose.orientation.z
        dist_z = self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z - self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z

        fx = self.Fx + self.Kp * (dist_x-self.DistX_1) + self.Ki * dist_x + self.Kd * ((dist_x-self.DistX_1)-(self.DistX_1-self.DistX_2))
        fy = self.Fy + self.Kp * (dist_y-self.DistY_1) + self.Ki * dist_y + self.Kd * ((dist_y-self.DistY_1)-(self.DistY_1-self.DistY_2))
        tz = self.Fz + self.Kp * (dist_z-self.DistZ_1) + self.Ki * dist_z + self.Kd * ((dist_z-self.DistZ_1)-(self.DistZ_1-self.DistZ_2))

        self.Add_force_with_param(fx,fy,0.0,0.0,0.0,tz)

        #print "Goal:x[%s], y[%s], z[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)),str(round(self.Goal_pose.orientation.z)))
        #print "Now:x[%s], y[%s], z:[%s]"%(str(round(self.Model_pose.position.x,2)),str(round(self.Model_pose.position.y,2)),str(round(self.Model_pose.orientation.z,2)))
        #print "Distance:x[%s], y[%s], z[%s]"%(str(round(dist_x, 2)), str(round(dist_y, 2)), str(round(dist_z, 2)))
        #print "fx:[%s], fy:[%s], tz:[%s]"%(str(round(fx,2)),str(round(fy,2)), str(round(tz,2)))
        #print "Arrived x:[%s] y:[%s] z:[%s]"%(self.ArrivedFlag_X, self.ArrivedFlag_Y, self.ArrivedFlag_Z)
        #print "Counter:[%s]"%str(self.count)
        #print "-----"

        print "Goal_position:x[%s] y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
        print "Model_position:x[%s] y[%s]"%(str(round(self.Model_pose.position.x)),str(round(self.Model_pose.position.y)))
        print "position_distance:x[%s], y[%s]"%(str(round(dist_x, 2)), str(round(dist_y, 2)))
        print "---"
        print "Goal_rotation:[%s]"%(str(round(self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z, 2)))
        print "Model_rotation:[%s]"%(str(round(self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z, 2)))
        print "Rotation_distance:[%s]"%str(round(dist_z,2))
        print "---"
        print "Arrived x:[%s] y:[%s] z:[%s]"%(self.ArrivedFlag_X, self.ArrivedFlag_Y, self.ArrivedFlag_Z)
        print "Counter:[%s]"%str(self.count)
        print "*****"



        self.Fx = fx
        self.Fy = fy
        self.Fz = tz

        self.DistX_2 = self.DistX_1
        self.DistY_2 = self.DistY_1
        self.DistZ_2 = self.DistZ_1

        self.DistX_1 = dist_x
        self.DistY_1 = dist_y
        self.DistZ_1 = dist_z

        self.count += 1

    def main(self):
        self.Goal_pose.position.x = -10.0
        self.Goal_pose.position.y = 0.0
        self.Goal_pose.orientation.z = 0.707
        self.Goal_pose.orientation.w = 0.707

        while not rospy.is_shutdown():
            self.pid()





if __name__ == "__main__":
    rospy.init_node("SelfMove", anonymous=True)
    c = SelfMove()
    c.main()
    rospy.spin()
