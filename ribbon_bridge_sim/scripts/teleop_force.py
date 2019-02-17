#!/usr/bin/env python
#coding: utf-8
import readchar
import rospy
import sys, time

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.srv import *
from robotx_gazebo.msg import UsvDrive

usage = """
Moving:
    u   i   o
    j   k   l
    m   ,   .

parameter:
    t/b → up/down speed
     r  → reset all parameters

else:
     q  → quit
"""
init_param = 10
Force_param = init_param
Torque_param = init_param


def Force(way):
    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    body_name = "tug_boat_6::body"
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

    elif way == "turn":
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = Torque_param

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
    global Force_param, Torque_param

    rospy.wait_for_service('/gazebo/apply_body_wrench')

    print usage

    while not rospy.is_shutdown():
        key = readchar.readchar()

        if key == "q":
            print "\n"
            break

        elif key == "r":
            Force_param = init_param
            Torque_param = init_param
            sys.stdout.write("\rway:[reset params] speed:[%s]" %(str(Force_param)))

        elif key == "t" or key == "b":
            if key == "t":
                Force_param = Force_param * 2
                sys.stdout.write("\rway:[speed up] speed:[%s]" %(str(Force_param)))
            else:
                Force_param = Force_param * 0.5
                sys.stdout.write("\rway:[speed down] speed:[%s]" %(str(Force_param)))

        elif key == "i":
            Force("front")
            sys.stdout.write("\rway:[↑] speed:[%s]" %(str(Force_param)))

        elif key == "l":
            Force("right")
            sys.stdout.write("\rway:[→] speed:[%s]" %(str(Force_param)))

        elif key == ",":
            Force("back")
            sys.stdout.write("\rway:[↓] speed:[%s]" %(str(Force_param)))

        elif key == "j":
            Force("left")
            sys.stdout.write("\rway:[←] speed:[%s]" %(str(Force_param)))

        elif key == "k":
            Force("turn")
            sys.stdout.write("\rway:[↺] speed:[%s]" %(str(Force_param)))

        else:
            pass

        sys.stdout.flush()
        time.sleep(0.01)



if __name__ == "__main__":
    rospy.init_node("Add_force", anonymous=True)
    main()
