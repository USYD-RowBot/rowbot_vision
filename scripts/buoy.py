#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
debugContourDrawColours={
    "red":(0,0,255),
    "green":(0,255,0),
    "white":(255,255,255)
}
class BuoyDetector:
    def __init__(self):
        # ros based parameters
        self.params = {
            "cameraAngularRange": 45,
            "roiAngularRange": 4,
            'debugLevel': 0,
            'debugBuoy':0,
            'buoyTargetRatio':0.5,
            'debugTarget':'screen',
            'outputDirectory':'buoyout//'
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel']=max(self.params['debugLevel'],self.params['debugBuoy'])
        pass

    def debugImShow(self,name,img):
        #print ("try")
        if self.params['debugTarget']=='file':
            try:
                os.mkdir(self.params['outputDirectory'])
            except Exception:
                pass
            cv2.imwrite(self.params['outputDirectory']+name+"::"+str(rospy.Time.now())+".png",img)
            #print (__file__)
            #print(self.params['outputDirectory']+'buoyout-'+name+str(rospy.Time.now())+".png")
        else:
            cv2.imshow(name,img)

    def identify(self, roi_img,debugName):
        if roi_img.shape[1]==0:
            return False
        else:
            pass
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
        #masks['blue'] = cv2.inRange(s_hsv,  np.array(
        #    [106, 76, 1]), np.array([140, 255, 255]))
        masks['green'] = cv2.inRange(s_hsv,  np.array(
            [45, 56, 1]), np.array([100, 255, 255]))
        masks['red'] = cv2.inRange(s_hsv,  np.array(
            [0, 76, 1]), np.array([30, 255, 255]))
        masks['white'] = cv2.inRange(s_hsv, (0, 0, np.max(s_hsv[:,:,2])*0.7), (180, np.max(s_hsv[:,:,1])*0.2, 255))
        
        # rbgw contours
        
        cnts = {}
        for m in masks:
            kernel = np.ones((10, 10), np.uint8)
            closing = cv2.morphologyEx(masks[m], cv2.MORPH_CLOSE, kernel)
            # closing=mask
            _img, cnts[m], hierarchy = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if self.params['debugLevel']>69:
            flt_dbg_img=roi_img.copy()
            for m in masks:
                cv2.drawContours(flt_dbg_img,cnts[m],-1,debugContourDrawColours[m],2)
            self.debugImShow("filters",flt_dbg_img)
            #cv2.imshow(m, disp)
        # get buoys
        # for each mask, find the contour that best matches the description of a buoy
        # get the largest of these contours.

        results=[]
        for col in cnts:
            for c in cnts[col]:
                x,y,w,h=cv2.boundingRect(c)
                if (w*h>100):
                 if y>0:
                  if abs((h/w)>1.5)==True:
                    if x+w <roi_img.shape[1] and x>0 and y>0:
                            results.append({'contour':c,'name':col+" buoy",'confidence':cv2.contourArea(c),'cx':x+w/2})
        # sort results and pick most likely n.
        results=sorted(results,key=lambda i: i['confidence'],reverse=True)
        #print(results)
        if (self.params['debugLevel']>49):
            dbg_img=roi_img.copy()
            cv2.drawContours(dbg_img,[c['contour'] for c in results],-1,[0,255,0],3)
            for c in results:
                cv2.putText(dbg_img, c['name']+":"+str(c['confidence']), (int(c['contour'][0][0][0]), int(
                    c['contour'][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX,0.5, [0, 255, 0])
            self.debugImShow(debugName,dbg_img)

        if (self.params['debugLevel']>0):
            cv2.waitKey(10)
        return results


