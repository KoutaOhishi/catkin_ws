#!/usr/bin/env python
#coding: utf-8

import rospy
import sys

import tf
from ribbon_bridge_sim.msg import *
from gazebo_msgs.msg import *

def handle_boat_pose(msg, modelname):
    i = msg.name.index(modelname)

    x = msg.pose[i].position.x
    y = msg.pose[i].position.y
    z = msg.pose[i].position.z

    qx = msg.pose[i].orientation.x
    qy = msg.pose[i].orientation.y
    qz = msg.pose[i].orientation.z
    qw = msg.pose[i].orientation.w

    br = tf.TransformBroadcaster()
    br.sendTransform((x,y,z),
        (qx, qy, qz, qw),
        rospy.Time.now(),
        modelname,
        "world")

def main(modelname):
    rospy.init_node("boat_tf_broadcaster_" + modelname, anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, handle_boat_pose, modelname)
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "There is no model name."

    else:
        print "Model name: " + str(sys.argv[1])
        main(sys.argv[1])
