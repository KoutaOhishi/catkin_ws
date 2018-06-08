#!/usr/bin/env python
#coding: utf-8

from robotx_gazebo.msg import UsvDrive

import readchar
import rospy
import sys, time

usage = """
Moving around:
    u   i   o
    j       l
    m   ,   .

t/b -> up/down max speeds x2

 r  -> reset speed parameters
 g  -> quit
"""

def main():
    rospy.init_node("Teleop_key_gazebo")

    topic_name = "/cmd_drive"

    # publisheの定義
    pub = rospy.Publisher(topic_name, UsvDrive, queue_size=10)

    # msg型の定義
    msg = UsvDrive()

    # 左右のスタスターの初期値
    init_param = 10.0

    left = init_param
    right = init_param

    print usage

    while not rospy.is_shutdown():
        #rospy.loginfo("left:[%s]"%str(left))
        #rospy.loginfo("right:[%s]"%str(right))
        sys.stdout.write("\rleft:[%s]   right:[%s]" %(str(left), str(right)))
        sys.stdout.flush()
        time.sleep(0.01)

        key = readchar.readchar()

        if key == "q":
            print "\n" # 改行
            break

        elif key == "t": # speed up
            left *= 2
            right *= 2

        elif key == "b": # speed down
            left *= 0.5
            right *= 0.5

        elif key == "r": # reset parameters
            left = init_param
            right = init_param

        elif key == "i": # 前進
            msg.left = left
            msg.right = right
            pub.publish(msg)

        elif key == "o": # 右前
            msg.left = left
            msg.right = right/4
            pub.publish(msg)

        elif key == "l": # 右
            msg.left = 0.1
            msg.right = -0.1
            pub.publish(msg)

        elif key == ".": # 右後ろ
            msg.left = -left
            msg.right = -right/4
            pub.publish(msg)

        elif key == ",": # 後進
            msg.left = -left
            msg.right = -right
            pub.publish(msg)

        elif key == "m": # 左後ろ
            msg.left = -left/4
            msg.right = -right
            pub.publish(msg)

        elif key == "j": # 左
            msg.left = -0.1
            msg.right = 0.1
            pub.publish(msg)

        elif key == "u": # 左前
            msg.left = left/4
            msg.right = right
            pub.publish(msg)

        else:
            pass








if __name__ == "__main__":
    main()
