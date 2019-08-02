#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import rospy
# load contours from the dictionary

cntdict={}
my_path = os.path.abspath(os.path.dirname(__file__))

for f in os.listdir(os.path.join(my_path,"cntdict")):
    file=open(os.path.join(my_path,"cntdict",f),"r")
    cntdict[f.split(".")[0]]={
        "ccnt":np.load(file),
        "cname":f.split(".")[0].split("_")[0]
    }
    print ("loaded "+f+" with "+str(len(cntdict[f.split(".")[0]]))+" vertices")



class BuoyDetector:
    def __init__(self):
        # ros based parameters
        self.params = {
            "cameraAngularRange": 45,
            "roiAngularRange": 5,
            'debugLevel': 0,
            'debugBuoy':0,
            'buoyTargetRatio':0.5
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel']=max(self.params['debugLevel'],self.params['debugBuoy'])
        pass

    def identify(self, img, bearing,debugName):
        # slice the image to the bearing range
        # print (self.params['debugLevel'])
        #print (rospy.get_param('~debugLevel'))
        self.imshape=img.shape
        if (self.params['debugLevel']<70):
            rar=self.params['roiAngularRange']
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
            #print (((bearing-self.params['roiAngularRange']) /self.params['cameraAngularRange']),roi_end)
            #print (roi_img.shape, img.shape)
            print("bearing:",bearing,roi_start,roi_end)
            #print("params:",rar,self.params['cameraAngularRange'],img.shape)
            #print("bits:",bearing-self.params['roiAngularRange'],((bearing-self.params['roiAngularRange'])*img.shape[1]),int(((bearing-self.params['roiAngularRange'])*img.shape[1])/self.params['cameraAngularRange']))
            #print("bits2:",bearing+self.params['roiAngularRange'],((bearing+self.params['roiAngularRange'])*img.shape[1]),int(((bearing+self.params['roiAngularRange'])*img.shape[1])/self.params['cameraAngularRange']))
        else:
            roi_img=img
        if roi_img.shape[1]==0:
            return False
        else:
            pass
        if (self.params['debugLevel']>0):
            dbg_img=roi_img.copy()
        if (50<self.params['debugLevel']<70):
            cv2.imshow(debugName, roi_img)
            print (roi_img.shape)
            
        # get filters
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        # shifted hsv
        fullh = (hsv[:, :, 0]).astype(int)
        # shift the colours by 180/12 so the red bins end up together
        fullh += int(180/12)
        # loop the values back to within  0 - 180
        fullh %= 180
        # reinsert into hsv -- for future refinement
        s_hsv = hsv.copy()
        s_hsv[:, :, 0] = fullh*1.0
        # red, green, blue, white masks
        masks = {}
        masks['blue'] = cv2.inRange(s_hsv,  np.array(
            [106, 76, 1]), np.array([140, 255, 255]))
        masks['green'] = cv2.inRange(s_hsv,  np.array(
            [45, 76, 1]), np.array([100, 255, 255]))
        masks['red'] = cv2.inRange(s_hsv,  np.array(
            [0, 76, 1]), np.array([30, 255, 255]))
        masks['white'] = cv2.inRange(s_hsv, (0, 0, np.max(s_hsv)*0.7), (180, np.max(s_hsv)*0.1, 255))
        # rbgw contours
        if self.params['debugLevel']>90:
            for m in masks:
                disp = cv2.bitwise_and(
                    img, img, mask=masks[m])
                cv2.imshow(m, disp)
        cnts = {}
        for m in masks:
            kernel = np.ones((10, 10), np.uint8 )
            closing = cv2.morphologyEx(masks[m], cv2.MORPH_OPEN, kernel)
            # closing=mask
            img, cnts[m], hierarchy = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # get buoys
        # for each mask, find the contour that best matches the description of a buoy
        # get the largest of these contours.
        results=[]
        for col in cnts:
            for c in cnts[col]:
                ___result,conf=self.filter(c)
                if ___result != False:
                    results.append({'contour':c,'name':col+___result,'confidence':conf})
            #if (self.params['debugLevel']>90):
            #    cv2.drawContours(roi_img,cnts[col],-1,[0,0,0],3)
            #    cv2.imshow('buoyOut',roi_img)
        # sort results and pick most likely n.
        results=sorted(results,key=lambda i: i['confidence'],reverse=True)
        #print(results)
        if (self.params['debugLevel']>50):
            cv2.drawContours(dbg_img,[c['contour'] for c in results],-1,[0,255,0],3)
            for c in results:
                cv2.putText(dbg_img, c['name']+":"+str(c['confidence']), (int(c['contour'][0][0][0]), int(
                    c['contour'][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX,0.5, [0, 255, 0])
            cv2.imshow(debugName+'out',dbg_img)
        if (self.params['debugLevel']>0):
            cv2.waitKey(10)
        return results
    def filter(self, cnt):
        # remove things with insignificant area
        cA=cv2.contourArea(cnt)
        if cA<0.0001*self.imshape[0]*self.imshape[1]:
            return False,0
        M = cv2.moments(cnt)
        cX = int(M["m10"] / M["m00"])
        for dcnt in cntdict:
            similarity=cv2.matchShapes(cntdict[dcnt]['ccnt'],cnt,1,0.0)
            if (similarity<0.1):
                return cntdict[dcnt]['cname'],1-similarity
        return False,0