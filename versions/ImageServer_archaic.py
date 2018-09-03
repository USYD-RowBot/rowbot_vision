#!/usr/bin/env python
#-35 > 0 > 35
import cv2
import numpy as np
from matplotlib import pyplot as plt
from rowbot_vision.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_scripts.shapes import get_shape
from vision_scripts.light_buoy import LightPattern
def nothing(x):
    pass

def overlay(bg,img2):
    img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(img2gray, 1, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    img1_bg = cv2.bitwise_and(bg,bg,mask = mask_inv)
    img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
    dst = cv2.add(img1_bg,img2_fg)
    return dst
class ImageServer():
    def __init__(self):
        #Initalise the image server
        #Initalise image subscriber
        self.pub = rospy.Publisher("image_out",Image)
        self.lp = LightPattern()
        self.cvimage = None
        self.image= None
        #self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rowbot/image_raw",Image, self.callback)
        self.last_time = rospy.get_time()
        pass
    def classify_buoy(self,bearing,objInFront):
        print("classifying")
        try:
            pass
            #rospy.wait_for_service('detect_buoy', timeout = 0.25)
        except rospy.ROSException, e:
            print("Waited too long")
        if self.image is not None:
            try:
                detect_buoy_service = rospy.ServiceProxy('detect_buoy',BuoyDetect)
                req = BuoyDetectRequest()
                req.image = self.image
                req.bearing = bearing
                req.ObjInFront = objInFront
                resp = detect_buoy_service(req)
                types = resp.types
                conf = resp.confidences
                print("Service returned", types,conf)
                return types,conf

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return False
        else:
            return False
        #print(min_water, max_water)
        #height,width,depth = image.shape
        #margin = margin_deg  * width/ 70
        #x_coord = width/70 * bearing + width/2
        #if bearing > fov/2 or bearing  < -fov/2:
        #    print("Out of range")
        #    return False
        #print(x_coord)



    def callback(self,data):
        #print("Rec image")
        seconds_start = rospy.get_time()
        self.image = data
        try:
            self.cvimage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = self.cvimage.shape[:2]
        new_w = int(width* 1)
        new_h = int(height* 1)
        resized = cv2.resize(self.cvimage,(new_w, new_h))
        sd_image, result = get_shape(resized)
        lp_image,_ = self.lp.getPattern(resized)


        detect_buoy_service = rospy.ServiceProxy('detect_buoy',BuoyDetect)
        req = BuoyDetectRequest()
        req.image = self.image
        req.bearing = 0
        req.ObjInFront = 0
        resp = detect_buoy_service(req)
        #self.pub.publish(resp.mask)
        bd_mask = self.bridge.imgmsg_to_cv2(resp.mask, "bgr8")
        #cv2.imshow("mask",mask)
        #print("img")
        combined = resized.copy()
        combined = overlay(combined,bd_mask)

        combined = overlay(combined,lp_image)
        combined = overlay(combined,sd_image)
        #cv2.imshow("shapes", combined)

        print("Running at "+ str(1/(rospy.get_time() -self.last_time)) + " Hz")
        self.last_time = rospy.get_time()
        #cv2.waitKey(1)

        self.pub.publish(self.bridge.cv2_to_imgmsg(combined, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is = ImageServer()
    rospy.spin()
