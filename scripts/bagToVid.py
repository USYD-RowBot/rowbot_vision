#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    image= bridge.imgmsg_to_cv2(data, "bgr8")
    out.write(image)
    cv2.imshow("out",image)
    k=cv2.waitKey(20)
    if k==102:
        out.release()
        quit()

rospy.init_node("vid_reciever")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))
bridge=CvBridge()
sub=rospy.Subscriber(rospy.get_param("~channel"),Image,callback)
rospy.spin()