#!/usr/bin/env python
#coding: utf-8

import rospy

import tf
from ribbon_bridge_sim.msg import *
from gazebo_msgs.msg import *

def handle_link_pose(msg):
    link_num = len(msg.name)

    for i in range(link_num):
        link_name = msg.name[i]

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
            link_name,
            "world")


def main():
    rospy.init_node("gazebo_tf_broadcaster_all_links")

    rospy.Subscriber("/gazebo/link_states", LinkStates, handle_link_pose)
    rospy.spin()

if __name__ == "__main__":
    main()
