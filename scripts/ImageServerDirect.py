#!/usr/bin/env python
import cv2
import numpy as np
# from matplotlib import pyplot as plt
from rowbot_vision.srv import *
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import functools
import math
#import buoy_contour_match as buoy
import buoy
import lightPattern as lightbcn 
import shapes
"""
FUNCTIONS
buoy.identify(image,bearing) Returns buoy classification with given bearing
lightbcn.enable()
lightbcn.disable()
lightbcn publishes a message, or runs inside imageServer and sets a variable.

shape.identify(image,bearing)
"""
cameraOffsets={
    "left":-65,
    "right":65,
    "front":5
}

class ImageServer():
    def __init__(self):

        self.params = {
            "cameraAngularRange": 45,
            "roiAngularRange": 4,
            'debugLevel': 0,
            'debugImgserv':0,
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel']=max(self.params['debugLevel'],self.params['debugImgserv'])

        self.bridge = CvBridge()
        self.camera_subs={}
        self.images = {}
        self.last_call = {}
        for c in cameraOffsets:
            self.camera_subs[c]=rospy.Subscriber(
            c+"_camera/image_raw", Image, functools.partial(self.callback,c))
            #rospy.loginfo("lambda is "+c)
            self.images[c]=None
            self.last_call[c]=None
        self.last_time = rospy.get_time()
        self.buoyDetector = buoy.BuoyDetector()
        self.lightDetector = lightbcn.LightDetector()
        self.lightDetectorEnabled=True
        self.tf_listener=tf.TransformListener(True,rospy.Duration(10.0))
        self.shapeDetector = shapes.ShapeDetector()
        return

    def enable_light_buoy(self):
        """Enable the light buoy.
        """
        self.lightDetectorEnabled=True

    def disable_light_buoy(self):
        """Disable the light buoy.
        """
        self.lightDetectorEnabled=False

    def get_light_buoy_info(self):
        """Gets the light buoy info at current time.
        Try to use the service where possible instead of this direct command.

        Returns:
            array of string -- Array containing the current cached pattern.
        """
        return self.lightDetector.recordedPatterns[len(self.lightDetector.recordedPatterns)-1]

    def classify_shape(self, bearing):
        """Classify a shape at a given bearing.

        Arguments:
            bearing {int} -- the bearing of the object.

        Returns:
            array of string: in format ["color shape"], sorted by confidence.
        """
        result = []
        for c in cameraOffsets:
            _result = self.shapeDetector.identify(self.images[c])
            _result = [(i['shape'] + i['color']) for i in _result]
            result.append(_result)
        return result

    def classify_buoy(self, tfName="derp"):
        """Call to classify a given buoy.

        Keyword Arguments:
            bearing {int} -- The angular position in degrees of the target. (default: {0})
            objInFront {bool} -- Whether or not there is another buoy in front of the given buoy. (default: {False})

        Returns:
            (types,confidences) -- an array of object types and confidences.
        """
        types= []
        confidences= []
        for c in cameraOffsets:
            if not self.images[c] is None:
                img=self.images[c]
                (trans,rot) = self.tf_listener.lookupTransformFull(c+"_camera_link",self.last_call[c],tfName,rospy.Time(0),"odom")
                bearing= math.degrees(math.atan2(trans[1], trans[0]))
                roi_start = int(((bearing-self.params['roiAngularRange'])*img.shape[1])/self.params['cameraAngularRange'])+img.shape[1]/2
                roi_start=max(0,min(roi_start,img.shape[1]))
                roi_end = int((bearing+self.params['roiAngularRange'])*img.shape[1]/self.params['cameraAngularRange']+img.shape[1]/2)
                roi_end=max(0,min(roi_end,img.shape[1]))
                roi_center=int(((bearing)*img.shape[1])/self.params['cameraAngularRange'])+img.shape[1]/2
                roi_center=max(0,min(roi_center,img.shape[1]))
                roi_range=min(roi_end-roi_center,roi_center-roi_start)
                roi_end=roi_center+roi_range
                roi_start=roi_center-roi_range
                roi_img = img[:,roi_start:roi_end, :]
                _result = self.buoyDetector.identify(roi_img,tfName+"::"+c)
                if not _result==False:
                    [types.append(i['name']) for i in _result]
                    [confidences.append(i['confidence']) for i in _result]
                    if len(_result)>0:
                        print (_result[0]['confidence'],math.sqrt(trans[0]**2+trans[1]**2))
            else:
                print("nnoooo")
        return (types, confidences)

    def callback(self, name,data):
        #rospy.loginfo(name)
        """Generic callback for storing the image (cached filter)

        Arguments:
            data {Image} -- Image from ROS subscriber.
        """
        try:
            self.images[name]=self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #rescale
        self.last_call[name]=rospy.Time.now()
        #print("lastcall:",str(self.last_call[name]))
        if (self.lightDetectorEnabled):
            self.lightDetector.identify(self.images[name])

        if __name__ == "__main__" and name=="front": #when debugging
            rospy.loginfo(self.classify_shape(0))
            rospy.loginfo(self.classify_buoy(0))
            rospy.loginfo(self.lightDetector.identify(self.images["front"]))


if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is= ImageServer()
    # also run the classify_buoy cos y not
    rospy.spin()
