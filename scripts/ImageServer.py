#!/usr/bin/env python
import cv2
import numpy as np
# from matplotlib import pyplot as plt
from rowbot_vision.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visops import runOperation
from vision_scripts import shapes
from vision_scripts import buoy
from vision_scripts import light_buoy





'''
Key issues:
1. some masking techniques will create contours which aren't representative of colour.
Solution: separate contour detection and colour detection.
2. It's difficult to test the algorithm.
Solution: build a feeder which can test the algorithm.
3. We want to return a classified image in a frame, with an option to display it.
Solution: chain the output of operator into fimgs['debug']
'''







max_angle=35
interest_region_width=0.1 ## 2 * percentagewise width of the region that we're going to be interested in 

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

def getContoursFromMask(mask):
    # kernel = np.ones((5,5),np.uint8)
    # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closing=mask
    cnts = cv2.findContours(closing, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1]
    return cnts

class ImageServer():
    def __init__(self):
        #Initalise the image server
        #Initalise image subscriber
        self.pub = rospy.Publisher("image_out",Image)
        self.lp = []
        self.fimgs = {} # filtered images.
        self.masks ={} # masks and respective contours. Masks are single channel. {'img':mask image,'cnts': contours}.
        self.image= None
        #self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rowbot/image_raw",Image, self.callback)
        self.last_time = rospy.get_time()

        # available operations
        # fn is function call, passive is whether or not operation runs in each loop
        self.ops={
            "buoyDetect": {"fn":buoy, "passive":False},
            "shapeDetect": {"fn":shapes, "passive":False},
            "lpDetect": {"fn":light_buoy, "passive":False}
        }
        pass
    def generate_contours(self):
        # hsv: normal HSV.
        self.fimgs['hsv']=cv2.cvtColor(self.fimgs['bgr'],cv2.COLOR_BGR2HSV)
        
        # s_hsv: shifted HSV.
        # convert to integer so we can use modulo
        fullh=(self.fimgs['hsv'][:,:,0]).astype(int)
        # shift the colours by 180/12 so the red bins end up together
        fullh+=int(180/12)
        # loop the values back to within  0 - 180
        fullh%=180
        # reinsert into hsv -- for future refinement
        self.fimgs['s_hsv']=np.copy(self.fimgs['hsv'])
        self.fimgs['s_hsv'][:,:,0]=fullh*1.0

        # brite: relative saturation and value thresholding, to get rid of grey objects and darker objects.
        self.masks['brite']={'img':cv2.inRange(self.fimgs['s_hsv'], (0,np.max(self.fimgs['s_hsv'][:,:,1])*0.3,np.max(self.fimgs['s_hsv'][:,:,2])*0.2), (180,255,255))}
        
        # white and black masks
        # white: high value, low saturation
        self.masks['white']={'img':cv2.inRange(self.fimgs['s_hsv'], (0,0,np.max(self.fimgs['s_hsv'][:,:,2])*0.9), (180,np.max(self.fimgs['s_hsv'][:,:,1])*0.2,255))}
        # black: low values
        self.masks['black']={'img':cv2.inRange(self.fimgs['s_hsv'], (0,0,0),(180,255,np.max(self.fimgs['s_hsv'][:,:,2])*0.2))}

        # saturation + HSV Red, blue and green.
        self.fimgs['brite_hsv'] = cv2.bitwise_and(self.fimgs['s_hsv'],self.fimgs['s_hsv'],mask = self.masks['brite']['img'])
        self.masks['sat_red'] = {'img':cv2.inRange(self.fimgs['brite_hsv'],  np.array([0, 1, 1]),np.array([30, 255, 255]))}
        self.masks['sat_green'] = {'img':cv2.inRange(self.fimgs['brite_hsv'],  np.array([50, 1, 1]),np.array([95, 255, 255]))}
        self.masks['sat_blue'] = {'img':cv2.inRange(self.fimgs['brite_hsv'],  np.array([106, 1, 1]),np.array([140, 255, 255]))}
        
        # sp_green: specifically for detecting green things by making a mask which is not green.
        # use the same theory to detect things which are definitely red and blue?
        sp_colors=['sp_blue','sp_green','sp_red']
        for i,s in enumerate(sp_colors):
            mid=(self.fimgs['bgr'][:, :, i].astype(int)-((self.fimgs['bgr'][:, :, (i+1)%3].astype(int)+self.fimgs['bgr'][:, :, (i+2)%3].astype(int))/3)).astype(np.uint8)
            self.masks[sp_colors[i]]={"img":cv2.inRange(mid,np.max(mid)*0.5,255)}
        # contour finding
        for mask in self.masks:
            self.masks[mask]['cnts']=getContoursFromMask(self.masks[mask]['img'])
        
    def classify_buoy(self,bearing=0,objInFront=False):
        return self.runOp("buoyDetect",bearing,objInFront)
    def runOp(self,opname,bearing=0,objInFront=False):
        print("running operation "+ opname+"...")
        self.generate_contours()
        results=runOperation(self.masks,{
            'sp_blue':'blue',
            'sp_green':'green',
            'sp_red':'red',
            'sat_blue':'blue',
            'sat_green':'green',
            'sat_red':'red',
            "white":"white",
            "black":"black"
            },self.ops[opname]["fn"].identify,self.ops[opname]["fn"].collate) # still going with mask-colours. I hope it works...
        print (results)
        return False
        # img_halfwidth=self.fimgs['bgr'].shape[1]/2
        # ROI_center = (bearing / max_angle * img_halfwidth)+ img_halfwidth 
        # real_irw=interest_region_width*img_halfwidth

        # # rospy.loginfo(result)
        # # do a bit of post processing of the result to get it into the form we want
        # # list comprehension yay
        # print(result)
        # filtered_result=[i for i in result if abs(i['cx']-ROI_center)<real_irw]



        # types = [i['name'] for i in filtered_result]
        # confs = [i['conf'] for i in filtered_result]
        # retval=[types, confs]
        
        # return retval 



    def callback(self,data):
        #print("Rec image")
        seconds_start = rospy.get_time()
        self.image = data
        try:
            self.fimgs['bgr'] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = self.fimgs['bgr'].shape[:2]
        new_w = int(width* 1)
        new_h = int(height* 1)
        self.fimgs['bgr'] = cv2.resize(self.fimgs['bgr'],(new_w, new_h))
        # create filtered images.
        self.generate_contours()
        
        
        # run the shape identifier
        # desired result: a single shape.
        result=runOperation(self.masks,{
            'sp_blue':'blue',
            'sp_green':'green',
            'sp_red':'red',
            'sat_blue':'blue',
            'sat_green':'green',
            'sat_red':'red'
            },shapes.identify,shapes.collate)
        rospy.loginfo(result)
        
        # run the light pattern identifier
        # desired result: the current light pattern.
        self.lp=runOperation(self.masks,{
            'sp_blue':'blue',
            'sp_green':'green',
            'sp_red':'red',
            'sat_blue':'blue',
            'sat_green':'green',
            'sat_red':'red'
            },light_buoy.identify,light_buoy.collate)
        rospy.loginfo(self.lp)
        
        # self.pub.publish(self.bridge.cv2_to_imgmsg(combined, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("Image_test")
    Is = ImageServer()
    rospy.spin()
