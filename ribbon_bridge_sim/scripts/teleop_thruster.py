#!/usr/bin/env python
#coding: utf-8

from robotx_gazebo.msg import UsvDrive

import readchar
import rospy
import sys, time

usage = """
Moving around:
    u   i   o
    j   k   l
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

    msg.left = init_param
    msg.right = init_param

    print usage

    sys.stdout.write("\rleft:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))
    sys.stdout.flush()
    time.sleep(0.01)

    while not rospy.is_shutdown():
        #rospy.loginfo("left:[%s]"%str(left))
        #rospy.loginfo("right:[%s]"%str(right))
        key = readchar.readchar()


        if left >= 10000.0:
            left = init_param

        if right >= 10000.0:
            right = init_param

        if key == "q":
            print "\n" # 改行
            break

        elif key == "t": # speed up
            left *= 2
            right *= 2
            sys.stdout.write("\rcommand[speed up]   left:[%s]   right:[%s]" %(str(left), str(right)))

        elif key == "b": # speed down
            left *= 0.5
            right *= 0.5
            sys.stdout.write("\rcommand[speed down]   left:[%s]   right:[%s]" %(str(left), str(right)))

        elif key == "k": #ブレーキ
            msg.left = 0.0
            msg.right = 0.0
            pub.publish(msg)
            sys.stdout.write("\rcommand[   stop   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))


        elif key == "r": # reset parameters
            left = init_param
            right = init_param
            sys.stdout.write("\rcommand[reset parameters]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "i": # 前進
            msg.left = left
            msg.right = right
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ↑   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "o": # 右前
            msg.left = left
            msg.right = right/4
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ➚   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "l": # 右
            msg.left = 0.1
            msg.right = -0.1
            pub.publish(msg)
            sys.stdout.write("\rcommand[   →   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == ".": # 右後ろ
            msg.left = -left
            msg.right = -right/4
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ➘   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == ",": # 後進
            msg.left = -left
            msg.right = -right
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ↓   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "m": # 左後ろ
            msg.left = -left/4
            msg.right = -right
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ↙   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "j": # 左
            msg.left = -0.1
            msg.right = 0.1
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ←   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        elif key == "u": # 左前
            msg.left = left/4
            msg.right = right
            pub.publish(msg)
            sys.stdout.write("\rcommand[   ↖   ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        else:
            sys.stdout.write("\rcommand[       ]   left:[%s]   right:[%s]" %(str(msg.left), str(msg.right)))

        sys.stdout.flush()
        time.sleep(0.01)


if __name__ == "__main__":
    main()
