#!/usr/bin/env python
import rospy
import cv2
import numpy as np

class BuoyDetector:
    def __init__(self):
        # ros based parameters
        self.params = {
            "cameraAngularRange": 70,
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

    def identify(self, img, bearing):
        # slice the image to the bearing range
        if (self.params['debugLevel']<70):
            roi_start = int((bearing-self.params['roiAngularRange'])*img.shape[0]/self.params['cameraAngularRange']+img.shape[0]/2)
            roi_end = int((bearing+self.params['roiAngularRange'])*img.shape[0]/self.params['cameraAngularRange']+img.shape[0]/2)
            roi_img = img[:,roi_start:roi_end, :]
            #print (((bearing-self.params['roiAngularRange']) /self.params['cameraAngularRange']),roi_end)
            print (roi_img.shape, img.shape)
        else:
            roi_img=img
        if (self.params['debugLevel']>100):
            cv2.imshow('slice', roi_img)

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
            [45, 76, 1]), np.array([95, 255, 255]))
        masks['red'] = cv2.inRange(s_hsv,  np.array(
            [0, 76, 1]), np.array([30, 255, 255]))
        masks['white'] = cv2.inRange(s_hsv, (0, 0, np.max(s_hsv)*0.7), (180, np.max(s_hsv)*0.1, 255))
        # rbgw contours
        cnts = {}
        for m in masks:
            kernel = np.ones((10, 10), np.uint8)
            closing = cv2.morphologyEx(masks[m], cv2.MORPH_CLOSE, kernel)
            # closing=mask
            img, cnts[m], hierarchy = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # get buoys
        # for each mask, find the contour that best matches the description of a buoy
        # get the largest of these contours.
        results=[]
        for col in cnts:
            for c in cnts[col]:
                if self.filter(c)==True:
                    results.append({'contour':c,'name':col+" buoy",'confidence':cv2.contourArea(c)})
            #if (self.params['debugLevel']>90):
            #    cv2.drawContours(roi_img,cnts[col],-1,[0,0,0],3)
            #    cv2.imshow('buoyOut',roi_img)
        # sort results and pick most likely n.
        results=sorted(results,key=lambda i: i['confidence'],reverse=True)
        print(results)
        if (self.params['debugLevel']>50):
            cv2.drawContours(roi_img,[c['contour'] for c in results],-1,[0,255,0],3)
            cv2.imshow('buoyOut',roi_img)
        if (self.params['debugLevel']>0):
            cv2.waitKey(10)
        return results
    def filter(self, cnt):
        x,y,w,h=cv2.boundingRect(cnt)
        if abs((h/w)>1.5):
            return True
        return False

if __name__ == "__main__":
    bd=BuoyDetector()
    print(bd)
