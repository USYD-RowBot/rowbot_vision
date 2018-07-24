#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from shape.shape import identify
from shape_test import get_shape



def getFromFrame(frame):
    equ=np.copy(frame);
    for i in range(0,3):
        equ[:,:,i] = cv2.equalizeHist(equ[:,:,i])
    ret = 0
    hsv=cv2.cvtColor(equ,cv2.COLOR_BGR2HSV)

    overmask=0;

    # get the hue channel
    fullh=(hsv[:,:,0]).astype(int)
    # shift the colours by 180/12 so the red bins end up together
    fullh+=int(180/12);
    # loop the values back to within  0 - 180
    fullh%=180;
    # reinsert into hsv -- for future refinement
    hsv[:,:,0]=fullh*1.0;
    shapeID="none"
    colorID="none"



    #Perform equalisation on frame
    #hsv[:,:,0] = cv2.equalizeHist(hsv[:,:,0])
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    hsv[:,:,0] = clahe.apply(hsv[:,:,0])

    # For each color...
    satTop=255;
    satBottom=67;
    valTop=255;
    valBottom=100;
    colors=[
        (np.array([0,satBottom,valBottom]),np.array([20,satTop,valTop])), # red
        (np.array([50,satBottom,valBottom]),np.array([95,satTop,valTop])),  # green
        (np.array([106,satBottom,valBottom]),np.array([125,satTop,valTop])),  # blue
    ]
    font = cv2.FONT_HERSHEY_SIMPLEX
    for c,i in enumerate(colors):
        if c==0:
            colorID="red"
        elif c==1:
            colorID="Green"
        else:
            colorID="blue"
        # define range of color in HSV
        lowerRange = i[0]
        upperRange = i[1]
        shapeIdentified=False;
        # Threshold the HSV image to get only desired colors
        mask = cv2.inRange(hsv, lowerRange, upperRange)

        # contour detection
        image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        # Get largest contour
        if not (contours is None or len(contours)<1):
            for d,i in enumerate(contours):
                if (len(i)>5):
                    x,y,w,h = cv2.boundingRect(i)
                    rectArea=w*h;
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                    if np.abs((cv2.contourArea(i)-rectArea)/rectArea) < 0.2 and h>w:
                        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                        cv2.putText(frame,colorID,(x,y), font, 0.5,(255,0,0),2,cv2.LINE_AA)
                        shapeID="Rectangle"
                        shapeIdentified=True;
        if shapeIdentified:
            if c==0:
                colorID="red"
            elif c==1:
                colorID="Green"
            else:
                colorID="blue"

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,shapeID,(10,200), font, 2,(255,0,0),2,cv2.LINE_AA)
    cv2.putText(frame,colorID,(10,300), font, 2,(255,0,0),2,cv2.LINE_AA)
    return frame

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image, self.callback)
    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        return_image = get_shape(frame);

        #cv2.imshow("Image window", return_image)
        try:

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(return_image, "bgr8"))
        except CvBridgeError as e:
          print(e)
def main(args):
  rospy.init_node('image_converter', anonymous=True)

  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
