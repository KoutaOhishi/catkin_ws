#!/usr/bin/env python
#coding: utf-8

import rospy

import tf
from ribbon_bridge_sim.msg import *
from gazebo_msgs.msg import *

def handle_boat_pose(msg, modelname):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        rospy.Time.now(),
        modelname,
        "world")

def callback(msg, modelname):
    i = msg.name.index(modelname)

    x = msg.pose[i].position.x
    y = msg.pose[i].position.y
    z = msg.pose[i].position.z

    qx = msg.pose[i].orientation.x
    qy = msg.pose[i].orientation.y
    qz = msg.pose[i].orientation.z
    qw = msg.pose[i].orientation.w

    br = tf.TransformBroadcaster()
    br.sendTransform((x,y,0),
        (qx, qy, qz, qw),
        rospy.Time.now(),
        modelname,
        "world")


def main():
    print "Input modelname"
    modelname = raw_input()
    rospy.init_node("boat_tf_broadcaster_" + modelname)


    #rospy.Subscriber('/%s/pose' % turtlename,
        #turtlesim.msg.Pose,
        #handle_turtle_pose,
        #turtlename)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, modelname)
    rospy.spin()

if __name__ == "__main__":
    main()
