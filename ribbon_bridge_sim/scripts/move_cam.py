#!/usr/bin/env python
#coding: utf-8
import rospy
import sys, time
import readchar

from gazebo_msgs.msg import *

def main():
    pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1, latch=True)

    pub_msg = ModelState()

    model_name = "aerial_camera"

    pub_msg.model_name = model_name

    init_x = 0.0 #[m]
    init_y = 0.0 #[m]
    init_z = 5.0 #[m]

    # カメラが/worldの真ん中に来る位置
    pub_msg.pose.position.x = init_x
    pub_msg.pose.position.y = init_y
    pub_msg.pose.position.z = init_z

    # カメラが下を向くposition
    #pub_msg.pose.orientation.x = 0.707
    #pub_msg.pose.orientation.y = 0.0
    #pub_msg.pose.orientation.z = -0.707
    #pub_msg.pose.orientation.w = 0.0
    pub_msg.pose.orientation.x = 0.0
    pub_msg.pose.orientation.y = 0.707
    pub_msg.pose.orientation.z = 0.0
    pub_msg.pose.orientation.w = 0.707

    # カメラの回転
    pub_msg.twist.linear.z = 1.0

    usage = """
    Moving around:
            i
        j   k   l
            ,

    t/b -> up/down camera height

     r  -> reset position
     g  -> quit
    """

    print usage

    while not rospy.is_shutdown():
        key = readchar.readchar()

        if key == "q":
            print "\n" # 改行
            break

        elif key == "i":
            pub_msg.pose.position.x += 1

        elif key == ",":
            pub_msg.pose.position.x -= 1

        elif key == "l":
            pub_msg.pose.position.y += 1

        elif key == "j":
            pub_msg.pose.position.y -= 1

        elif key == "t":
            pub_msg.pose.position.z += 1

        elif key == "b":
            pub_msg.pose.position.z -= 1

        elif key == "r":
            pub_msg.pose.position.x = init_x
            pub_msg.pose.position.y = init_y
            pub_msg.pose.position.z = init_z

        else:
            pass

        pub.publish(pub_msg)

        sys.stdout.write("\r x:[%s]     y:[%s]     z:[%s]" %(str(pub_msg.pose.position.x), str(pub_msg.pose.position.y), str(pub_msg.pose.position.z)))

        sys.stdout.flush()
        time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("move_cam")

    main()
