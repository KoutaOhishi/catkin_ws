#!/usr/bin/env python
#coding: utf-8
""" modelの名前をプログラム上で指定してからそのｔｆをbroadcastするノード """
import rospy

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


def main():
    print "Input modelname"
    modelname = raw_input()

    rospy.init_node("boat_tf_broadcaster_" + modelname)

    rospy.Subscriber("/gazebo/model_states", ModelStates, handle_boat_pose, modelname)
    rospy.spin()

if __name__ == "__main__":
    main()
