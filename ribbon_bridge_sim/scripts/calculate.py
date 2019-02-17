#!/usr/bin/env python
#coding: utf-8
import rospy
import math
from std_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *

""" 計測した結果から浮体の座標を求める(浮体１個だけ) """

class Calculate():
    def __init__(self):
        self.CameraName = "aerial_camera"
        self.CameraState = Point()
        self.subCameraState = rospy.Subscriber("/gazebo/set_model_state", ModelState, self.CameraStateCB)
        self.CameraStateFlag = False

        self.ImageTopic = "/aerial_camera/camera1/image_raw"
        self.ImageHeight = 2160
        self.ImageWidth = 4096
        self.subImageTopic = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, self.ImageTopicCB)
        self.ImageTopicFlag = False

        self.RibbonBridgeCenter_X = 0.0
        self.RibbonBridgeCenter_Y = 0.0
        self.subResultData = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.ResultDataCB)
        self.ResultDataFlag = False

        self.RibbonBridgeSize = [12.0, 8.0] #meter
        self.FovAngle = 90.0 #Field of View angle (視野角)
        self.Height = 0.0
        self.Width = 0.0

    def CameraStateCB(self, msg):
        if msg.model_name == self.CameraName:
            self.CameraState = msg.pose.position
            self.CameraStateFlag = True

    def ImageTopicCB(self, msg):
        self.ImageHeight = msg.height
        self.ImageWidth = msg.width
        self.ImageTopicFlag = True

    def ResultDataCB(self, msg):
        if len(msg.RibbonBridges) == 0:
            #rospy.logwarn("There are NOT any RibbonBridges.")
            self.RibbonBridgeCenter_X = 0.0
            self.RibbonBridgeCenter_Y = 0.0
            self.ResultDataFlag = False

        else:
            #print msg.RibbonBridges[0]
            self.RibbonBridgeCenter_X = msg.RibbonBridges[0].center.x
            self.RibbonBridgeCenter_Y = msg.RibbonBridges[0].center.y
            self.ResultDataFlag = True

    def calculate(self):
        print "w:[%s] h:[%s]"%(str(round(self.Width)),str(round(self.Height)))
        print "Bridge:[%s,%s]"%(str(round(self.RibbonBridgeCenter_X)),str(round(self.RibbonBridgeCenter_Y)))

        imgCenter_X = self.Width/2.0
        imgCenter_Y = self.Height/2.0

        #print imgCenter_X
        #print imgCenter_Y

        w = self.Width * self.RibbonBridgeCenter_X / self.ImageWidth - imgCenter_X
        h = self.Height * self.RibbonBridgeCenter_Y / self.ImageHeight - imgCenter_Y

        print "w(y):[%s]"%str(round(-w,3))
        print "h(x):[%s]"%str(round(-h,3))


        print "-"



    def main(self):
        rospy.loginfo("--- start ---")
        while not rospy.is_shutdown():
            rospy.sleep(1)
            if self.CameraStateFlag == True and self.ImageTopicFlag == True and self.ResultDataFlag == True:
                # 画像における実際の距離
                self.Width = 2.0 * self.CameraState.z * math.tan(math.radians(self.FovAngle/2.0))
                self.Height = self.Width * self.ImageHeight / self.ImageWidth

                #print "w:[%s] h:[%s]"%(str(round(self.Width)),str(round(self.Height)))
                #print "Bridge:[%s,%s]"%(str(round(self.RibbonBridgeCenter_X)),str(round(self.RibbonBridgeCenter_Y)))
                #print "-"
                self.calculate()

            else:
                if self.CameraStateFlag == False:
                    rospy.logwarn("[/gazebo/set_model_state] is NOT subscribed.")

                if self.ImageTopicFlag == False:
                    rospy.logwarn("[%s] is NOT subscribed."%self.ImageTopic)

                if self.ResultDataFlag == False:
                    rospy.logwarn("[/ribbon_bridge_measurement/result_data] is NOT subscribed.")






if __name__ == "__main__":
    rospy.init_node("calculate_boat_position")
    c = Calculate()
    c.main()
    rospy.spin()
