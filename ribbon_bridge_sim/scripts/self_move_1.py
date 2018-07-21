#!/usr/bin/env python
#coding: utf-8
import rospy
import sys

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

Goal_pose = Pose()
Goal_area = 3.0
Force_param = 1.0
Torque_param = 0.1

def Force(way):
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)

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
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        clear_body_wrenches(body_name=body_name)

    else:
        pass

    try:
        apply_body_wrench(body_name=body_name,
        reference_frame=reference_frame,
        wrench=wrench,
        duration=duration)

    except rospy.ServiceException as e:
        rospy.logerror("Service Exception")

def Goal_cb(msg, modelname):
    global Goal_pose

    i = msg.name.index(modelname)
    Goal_pose = msg.pose[i]


def Model_pose_cb(msg, modelname):
    i = msg.name.index(modelname)

    x = msg.pose[i].position.x
    y = msg.pose[i].position.y
    z = msg.pose[i].position.z

    dist_x = Goal_pose.position.x - x
    dist_y = Goal_pose.position.y - y


    if dist_x > Goal_area: #front
        Force("front")

    elif dist_x < -Goal_area: #back
        Force("back")

    elif dist_y > Goal_area:
        Force("right")

    elif dist_y < -Goal_area:
        Force("left")

    elif dist_x < Goal_area and dist_x > -Goal_area:
        if dist_y < Goal_area and dist_y > -Goal_area:
            rospy.loginfo("Arrived Goal Position")
            Force("brake")
        else:
            pass
    else:
        rospy.logwarn("warning")

    print "-------------------------------------"
    print ("Goal Location:[%s][%s]"%(str(Goal_pose.position.x), str(Goal_pose.position.y)))
    print ("Present Location:[%s][%s]"%(str(x), str(y)))
    print ("Distance:[%s][%s]"%(str(dist_x), str(dist_y)))

def main():
    modelname = "tug_boat"
    target_modelname = "tug_boat_0"

    sub_model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Model_pose_cb, modelname)

    sub_goal_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, Goal_cb, target_modelname)

    #sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, Goal_cb)

if __name__ == "__main__":
    rospy.init_node("move2goal_point")
    main()
    rospy.spin()
