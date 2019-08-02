#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import tf
import functools
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import buoy


cameraOffsets = {
    #"left": -55,
    #"right": 65,
    "front": 5
}


class ImageServer():
    def __init__(self):
        self.params = {
            "cameraAngularRange": 45,
            'debugLevel': 0,
            'debugCamserv': 0,
            'buoyTargetRatio': 0.5,
            'debugTarget': 'screen',
            'outputDirectory': 'buoyout//'
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel'] = max(
            self.params['debugLevel'], self.params['debugCamserv'])

        self.bridge = CvBridge()
        self.camera_subs = {}
        self.images = {}
        self.recievedAt={}
        for c in cameraOffsets:
            self.camera_subs[c] = rospy.Subscriber(
                c+"_camera/image_raw", Image, functools.partial(self.callback, c))
            #rospy.loginfo("lambda is "+c)
            self.images[c] = None
            self.recievedAt[c]=0
        self.buoyDetector = buoy.BuoyDetector()
        self.tf_listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.tf_publisher = tf.TransformBroadcaster()

        self.totalCount = 0
        return

    def classify_and_broadcast(self):
        for c in cameraOffsets:
            if not self.images[c] is None:
                try:
                    (ctrans,crot) = self.tf_listener.lookupTransformFull('odom',rospy.Time(0),c+"_camera_link",self.recievedAt[c],"odom")
                except Exception:
                    return
                _results = self.buoyDetector.identify(self.images[c],str(rospy.Time.now()))
                for itm in _results:
                    angle = cameraOffsets[c]+itm['cx']*self.params['cameraAngularRange'] / \
                        self.images[c].shape[1] - \
                        self.params['cameraAngularRange']/2
                    angle =180-angle
                    angle=math.radians(angle)
                    scale=550/math.sqrt(itm['confidence'])
                    """self.tf_publisher.sendTransform(
                        (scale*math.cos(angle), scale*math.sin(angle), 0),
                        (0, 0, 0, 1), rospy.Time.now(
                        ), itm['name']+str(self.totalCount), 'front_camera_link'
                    )"""
                    
                    self.tf_publisher.sendTransform(
                        (ctrans[0]+scale*math.cos(angle), ctrans[1]+scale*math.sin(angle), 0),
                        (0, 0, 0, 1), rospy.Time.now(
                        ), itm['name']+"_"+str(itm['confidence'])+"_"+str(self.totalCount), 'odom'
                    )
                    self.totalCount = self.totalCount+1
    def callback(self, name, data):
        # rospy.loginfo(name)
        """Generic callback for storing the image (cached filter)

        Arguments:
            data {Image} -- Image from ROS subscriber.
        """
        try:
            self.images[name] = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.recievedAt[name]=rospy.Time.now()
        except CvBridgeError as e:
            print(e)
        # rescale


if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is = ImageServer()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        Is.classify_and_broadcast()
        rate.sleep()
