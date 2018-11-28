#!/usr/bin/env python
import cv2
import numpy as np
# from matplotlib import pyplot as plt
from rowbot_vision.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import buoy
import lightbcn
import shapes

class ImageServer():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rowbot/image_raw",Image, self.callback)
        self.image=None
        self.last_time = rospy.get_time()
        return

    def classify_buoy(self,bearing=0,objInFront=False):
        if not self.image is None:
            ids=buoy.identify(self.image,bearing)
            types=[]
            confidences=[]
            cv2.waitKey(100)
            for id in ids:
                types.append(id['name'])
                confidences.append(id['confidence'])
            return (types, confidences)
        else:
            return ([],[])

    def callback(self,data):
        #print("Rec image")
        seconds_start = rospy.get_time()
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = self.image.shape[:2]
        new_w = int(width* 1)
        new_h = int(height* 1)
        self.image = cv2.resize(self.image,(new_w, new_h))
        result=shapes.identify(self.image)
        rospy.loginfo(result)
        
        result=lightbcn.identify(self.image)
        rospy.loginfo(result)

        if __name__ == "__main__":
            result=Is.classify_buoy(bearing=None)
            rospy.loginfo(result)

if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is = ImageServer()
    # also run the classify_buoy cos y not
    rospy.spin()

