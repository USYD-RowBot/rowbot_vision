#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import *
from sensor_msgs.msg import Image

rospy.init_node("videoFileTest")
broad=rospy.Publisher("front_camera/image_raw",Image)
bridge=CvBridge()
testFile="/home/stevenliu/Videos/boaty_boat.mp4"
rate = rospy.Rate(10)
cap=cv2.VideoCapture(testFile)
print (cap)
while (cap.isOpened() and (not rospy.is_shutdown())):
    ret,frame=cap.read()
    cv2.imshow("rawin",frame)
    broad.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
    cv2.waitKey(100)
    rate.sleep()
    
