#!/usr/bin/env python
#coding: utf-8
import rospy
import math
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *


class SetModelState():
    def __init__(self):
        self.Model_name = "tug_boat_3"

        self.pub_ModelState = ModelState()
        self.SetModelStateFlag = False

        self.sub_model_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_model_state_CB)

        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

    def euler_to_quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])



    def sub_model_state_CB(self, msg):
        if self.SetModelStateFlag == True:
            if self.Model_name in msg.name:
                target_index = msg.name.index(self.Model_name)

                self.pub_ModelState.pose = msg.pose[target_index]

                q = self.euler_to_quaternion(Vector3(0.0,0.0,0))

                self.pub_ModelState.pose.orientation.x = q.x
                self.pub_ModelState.pose.orientation.y = q.y
                self.pub_ModelState.pose.orientation.z = q.z
                self.pub_ModelState.pose.orientation.w = q.w

                self.pub.publish(self.pub_ModelState)


            else:
                rospy.logerr("[%s] is NOT exist."%Model_name)



    def main(self):
        self.pub_ModelState.model_name = self.Model_name
        self.pub_ModelState.twist = Twist()
        self.SetModelStateFlag = True

        while not rospy.is_shutdown():
            pass





if __name__ == "__main__":
    rospy.init_node("GetModelState", anonymous=True)
    c = SetModelState()
    c.main()
    rospy.spin()
