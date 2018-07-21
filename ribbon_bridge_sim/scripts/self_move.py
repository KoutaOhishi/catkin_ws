#!/usr/bin/env python
#coding: utf-8
import rospy
import sys

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from matplotlib import pyplot as plt

Goal_pose = Pose()
Goal_area_x = 2.0
Goal_area_y = 0.0
Force_param = 10
Torque_param = 10

x_flag = False
y_flag = False

list_x = []
list_y = []
count = 0

def Force(way):
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)

    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

    body_name = "tug_boat::body"
    reference_frame = "world"
    wrench = Wrench()
    duration = rospy.Duration(0.1) #単位はsecond

    if way == "front":
        wrench.force.x = Force_param
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

    elif way == "back":
        wrench.force.x = -Force_param
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

    elif way == "right":
        wrench.force.x = 0
        wrench.force.y = Force_param
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

    elif way == "left":
        wrench.force.x = 0
        wrench.force.y = -Force_param
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

    elif way == "turn_right":
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = -Torque_param

    elif way == "turn_left":
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = Torque_param

    elif way == "brake":
        res = get_link_state(link_name=body_name)
        if res.link_state.twist.linear.x < -0.001:
            wrench.force.x = Force_param * 100

        elif res.link_state.twist.linear.x > 0.001:
            wrench.force.x = -Force_param * 100

        if res.link_state.twist.linear.y < -0.001:
            wrench.force.y = Force_param * 100

        elif res.link_state.twist.linear.y > 0.001:
            wrench.force.y = -Force_param * 100

        if res.link_state.twist.angular.z < -0.001:
            wrench.torque.z = Torque_param

        elif res.link_state.twist.angular.z > 0.001:
            wrench.torque.z = -Torque_param

        else:
            pass

    else:
        pass

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


def Model_pose_cb(msg, modelname):
    global x_flag, y_flag
    global list_x, list_y, count

    i = msg.name.index(modelname)

    x = msg.pose[i].position.x
    y = msg.pose[i].position.y

    #dist_x = Goal_pose.position.x - x
    #dist_y = Goal_pose.position.y - y

    dist_x = 0.0 - x
    dist_y = 0.0 - y

    if abs(dist_x) > Goal_area_x:
        x_flag = False

    if abs(dist_y) > Goal_area_y:
        y_flag = False

    if x_flag == False or y_flag == False:
        if dist_x < Goal_area_x and dist_x > -1.0 * Goal_area_x:
            #cmd = "brake"
            x_flag = True

        if dist_y < Goal_area_y and dist_y > -1.0 * Goal_area_y:
            #cmd = "brake"
            y_flag = True

        if dist_y > Goal_area_y:
            cmd = "right"
            y_flag = False

        if dist_y < -1.0 * Goal_area_y:
            cmd = "left"
            y_flag = False

        if dist_x > Goal_area_x: #front
            cmd = "front"
            x_flag = False


        if dist_x < -1.0 * Goal_area_x: #back
            cmd = "back"
            x_flag = False

        else:
            #rospy.logwarn("warning")
            #cmd = "brake"
            pass

    if x_flag == True and y_flag == True:
        #Force("brake")
        cmd = "brake"
        rospy.loginfo("Arrived Goal Point")

    else:
        pass

    Force(cmd)

    print "-------------------------------------"
    print ("Goal Location:[%s][%s]"%(str(Goal_pose.position.x), str(Goal_pose.position.y)))
    print ("Present Location:[%s][%s]"%(str(x), str(y)))
    print ("Distance:[%s][%s]"%(str(dist_x), str(dist_y)))
    print ("Arrived:x[%s] y[%s]"%(str(x_flag), str(y_flag)))
    print ("Command:[%s]"%(cmd))
    print count

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

    count += 1


def main():
    modelname = "tug_boat_0"
    target_modelname = "tug_boat"

    sub_model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Model_pose_cb, modelname)

    sub_goal_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Goal_cb, target_modelname)

    rospy.spin()

    #sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, Goal_cb)

if __name__ == "__main__":
    rospy.init_node("move2goal_point")
    main()
    rospy.spin()
