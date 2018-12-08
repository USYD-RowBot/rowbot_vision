#!/usr/bin/env python
import cv2
import numpy as np
# from matplotlib import pyplot as plt
from rowbot_vision.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import functools

import buoy
import lightbcn
import shapes
import filters
"""
FUNCTIONS
buoy.identify(image,bearing) Returns buoy classification with given bearing
lightbcn.enable()
lightbcn.disable()
lightbcn publishes a message, or runs inside imageServer and sets a variable.

shape.identify(image,bearing)
"""
cameraOffsets={
    #"left":-60,
    #"right":60,
    "front":0
}

class ImageServer():
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_subs={}
        self.filters={}
        self.images = {}
        for c in cameraOffsets:
            self.camera_subs[c]=rospy.Subscriber(
            c+"_camera/image_raw", Image, functools.partial(self.callback,c))
            rospy.loginfo("lambda is "+c)
            self.filters[c]=filters.FilterCacher()
            self.images[c]=None
        self.last_time = rospy.get_time()
        self.buoyDetector = buoy.BuoyDetector()
        self.lightDetector = lightbcn.LightPatternDetector()
        self.lightDetectorEnabled=True
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
        return self.lightDetector.cachedPattern

    def classify_shape(self, bearing):
        """Classify a shape at a given bearing.

        Arguments:
            bearing {int} -- the bearing of the object.

        Returns:
            array of string: in format ["color shape"], sorted by confidence.
        """
        result = []
        for c in cameraOffsets:
            _result = self.shapeDetector.identify(self.images[c], self.filters[c],bearing+cameraOffsets[c])
            _result = [(i['shape'] + i['color']) for i in _result]
            result.append(_result)
        return result

    def classify_buoy(self, bearing=0, objInFront=False):
        """Call to classify a given buoy.

        Keyword Arguments:
            bearing {int} -- The angular position in degrees of the target. (default: {0})
            objInFront {bool} -- Whether or not there is another buoy in front of the given buoy. (default: {False})

        Returns:
            (types,confidences) -- an array of object types and confidences.
        """
        ids=[]
        for c in cameraOffsets:
            _result = self.buoyDetector.identify(self.images[c], self.filters[c],bearing+cameraOffsets[c])
            _result = [(i['shape'] + i['color']) for i in _result]
            ids.append(_result)
        
        types= []
        confidences= []
        for id in ids:
            types.append(id['name'])
            confidences.append(id['confidence'])
        return (types, confidences)

    def callback(self, name,data):
        rospy.loginfo(name)
        """Generic callback for storing the image (cached filter)

        Arguments:
            data {Image} -- Image from ROS subscriber.
        """
        try:
            self.image= self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #rescale
        height, width = self.image.shape[: 2]
        new_w= int(width * 1)
        new_h= int(height * 1)
        self.images[name]= cv2.resize(self.image, (new_w, new_h))
        self.filters[name].preprocess(self.image)
        if (self.lightDetectorEnabled):
            self.lightDetector.identify(self.images[name],self.filters[name])

        if __name__ == "__main__" and name=="front": #when debugging
            rospy.loginfo(self.classify_shape(0))
            rospy.loginfo(self.classify_buoy(0))
            rospy.loginfo(self.lightDetector.identify(self.images["front"],self.filters["front"]))


if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is= ImageServer()
    # also run the classify_buoy cos y not
    rospy.spin()
