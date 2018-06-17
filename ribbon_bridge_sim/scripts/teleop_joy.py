#!/usr/bin/env python
#coding: utf-8
import rospy
import sys, time

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.srv import *
from sensor_msgs.msg import Joy

Stop_flag = Bool()
Stop_flag = False

Init_param = 10.0
Force_param = Init_param

#buttons[4] = front
#buttons[5] = right
#buttons[6] = back
#buttons[7] = left
#buttons[10] = L1
#buttons[11] = R1
#buttons[12] = △
#buttons[13] = ○
#buttons[14] = ×
#buttons[15] = □
How2Use = """
  ↑     |     x
←   →   |  -y   y
  ↓     |    -x

△ -> Speed Up     × -> Speed Down
□ -> Reset Speed  ○ -> Brake

L1 + R1 -> Quit
"""
def joy_cb(msg):
    global Stop_flag, Force_param
    command = ""

    if msg.buttons[4] == 1:
        command = "front"

    elif msg.buttons[5] == 1:
        command = "right"

    elif msg.buttons[6] == 1:
        command = "back"

    elif msg.buttons[7] == 1:
        command = "left"

    elif msg.buttons[10] == 1 and msg.buttons[11] == 1:
        Stop_flag = True

    elif msg.buttons[12] == 1:
        Force_param = Force_param + 1
        command = "Speed Up"

    elif msg.buttons[13] == 1:
        command = "brake"

    elif msg.buttons[14] == 1:
        Force_param = Force_param - 1
        command = "Speed Down"

    elif msg.buttons[15] == 1:
        Force_param = Init_param
        command = "Reset Speed"

    else:
        pass

    Force(command)
    sys.stdout.write("\rcommand:[%s] speed:[%s]" %(command,str(Force_param)))
    sys.stdout.flush()
    time.sleep(0.01)


def Force(way):
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    body_name = "tug_boat::body"
    reference_frame = "world"
    wrench = Wrench()
    duration = rospy.Duration(1) #単位はsecond

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

    elif way == "brake":
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

    else:
        pass

    try:
        apply_body_wrench(body_name=body_name,
        reference_frame=reference_frame,
        wrench=wrench,
        duration=duration)

    except rospy.ServiceException as e:
        rospy.logerror("Service Exception")


def main():
    sub_joy = rospy.Subscriber("/joy", Joy, joy_cb)

    print How2Use

    while not rospy.is_shutdown():
        if Stop_flag == True:
            print "\n"
            break
        else:
            pass


if __name__ == "__main__":
    rospy.init_node("JoyStickController4TugBoat")
    main()
