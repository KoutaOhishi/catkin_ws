#!/usr/bin/env python
#coding: utf-8
import rospy
import numpy as np

from matplotlib import pyplot as plt
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

#STEP応答法でパラメータを求める
L = 500.00 #無駄時間（グラフの先が上がり始めるまでの時間）
T = 1000.00 #時定数
K = 160.00 #定常値（最初の計測値と目標値との差）
Kp = (0.70 * T) / (K * L) #0.60~0.95
Ki = 0.60 / (K * L) #0.60~0.70
Kd = (0.30 * T) / K #0.30~0.45
print Kp, Ki, Kd

Goal_pose = Pose()
X_flag = False
Y_flag = False
Fx1 = 0.0
Fy1 = 0.0
dist_x1 = 0.0
dist_y1 = 0.0
dist_x2 = 0.0
dist_y2 = 0.0
list_x = []
list_y = []
count = 0


def main():
    modelname = "tug_boat_4"
    goal_modelname = "tug_boat"

    sub_goal_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Goal_cb, goal_modelname)

    sub_model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Model_cb, modelname)

    rospy.spin()

def Add_force(body_name, fx, fy, fz, tx, ty, tz):
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    reference_frame = "world"
    wrench = Wrench()
    duration = rospy.Duration(1) #単位はsecond

    wrench.force.x = fx
    wrench.force.y = fy
    wrench.force.z = fz
    wrench.torque.x = tx
    wrench.torque.y = ty
    wrench.torque.z = tz

    try:
        apply_body_wrench(body_name=body_name,
        reference_frame=reference_frame,
        wrench=wrench,
        duration=duration)

    except rospy.ServiceException as e:
        rospy.logerr("Service Exception")

def Goal_cb(msg, modelname):
    global Goal_pose
    i = msg.name.index(modelname)
    Goal_pose = msg.pose[i]


def Model_cb(msg, modelname):
    global Goal_pose
    global X_flag, Y_flag
    global Fx1, Fy1 #１つ前に与えたforce値
    global dist_x1, dist_y1 #１つ前の偏差
    global dist_x2, dist_y2 #２つ前の偏差
    global list_x, list_y, count

    i = msg.name.index(modelname)

    body_name = modelname + "::body"

    x = msg.pose[i].position.x
    y = msg.pose[i].position.y

    #dist_x = Goal_pose.position.x - x
    #dist_y = Goal_pose.position.y - y

    #中心をゴールとする
    dist_x = 0.0 - x
    dist_y = 0.0 - y

    fx = Fx1 + Kp * (dist_x - dist_x1) + Ki * dist_x + Kd * ((dist_x - dist_x1) - (dist_x1 - dist_x2))

    fy = Fy1 + Kp * (dist_y - dist_y1) + Ki * dist_y + Kd * ((dist_y - dist_y1) - (dist_y1 - dist_y2))

    Add_force(body_name, 0, fy, 0, 0, 0, 0)

    print "-------------------------------------"
    print ("Goal Location:[%s][%s]"%(str(Goal_pose.position.x), str(Goal_pose.position.y)))
    print ("Present Location:[%s][%s]"%(str(x), str(y)))
    print ("Distance:[%s][%s]"%(str(dist_x), str(dist_y)))
    print ("fx:[%s] fy:[%s]"%(str(fx), str(fy)))

    list_x.append(count)
    list_y.append(dist_y)

    if count % 50000 == 0:
        plt.plot(list_x, list_y, color="blue")
        plt.axhline(0.0, color="red")
        plt.xlim(0,count)
        plt.ylim(-20,20)
        #plt.pause(0.001)  # 引数はsleep時間
        #plt.cla()  # 現在描写されているグラフを消去
        plt.show()

    else:
        pass

    Fx1 = fx
    Fy1 = fy
    dist_x1 = dist_x
    dist_y1 = dist_y
    dist_x2 = dist_x1
    dist_y2 = dist_y1
    count += 1



    #rospy.sleep(1)

    print count


if __name__ == "__main__":
    rospy.init_node("pid_move")
    main()
