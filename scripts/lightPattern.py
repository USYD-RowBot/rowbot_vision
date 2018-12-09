#!/usr/bin/env python
import rospy
import cv2
import numpy as np

class LightDetector:
    def __init__(self):
        # ros based parameters
        self.params = {
            'debugLevel': 0,
            'debugLightPattern': 0,
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel'] = max(
            self.params['debugLevel'], self.params['debugLightPattern'])
        self.cache = []
        self.recordedPatterns=[]
        pass

    def identify(self, img, ):
        # identify all rectangles

        # get shifted hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # shifted hsv
        fullh = (hsv[:, :, 0]).astype(int)
        # shift the colours by 180/12 so the red bins end up together
        fullh += int(180/12)
        # loop the values back to within  0 - 180
        fullh %= 180
        # reinsert into hsv -- for future refinement
        s_hsv = hsv.copy()
        s_hsv[:, :, 0] = fullh*1.0
        # brite mask
        brite = cv2.inRange(s_hsv, (0, np.max(
            s_hsv[:, :, 1])*0.3, np.max(s_hsv[:, :, 2])*0.2), (180, 255, 255))
        # fetch the contours
        im,cnts,hier=cv2.findContours(brite,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        newRects=[]
        for c in cnts:
            if (cv2.contourArea(c)>0):
                br=cv2.minAreaRect(c)
                if ((br[1][0]*br[1][1])/cv2.contourArea(c))<1.1: # rectangularity check
                    newRects.append({'cx':br[0][0],'cy':br[0][1],'w':br[1][0],'h':br[1][1]})
        # see if they overlap with a previous detection 
        newcache=[]
        retval=False
        if len(self.cache)>0:
            for newr in newRects:
                seen=False
                n={}
                for oldr in self.cache:
                    if (newr['cx']-oldr['cx'])<newr['w']/2 and (newr['cy']-oldr['cy'])<newr['h']/2:
                        n.update({'cx':newr['cx'],'cy':newr['cy'],'w':newr['w'],'h':newr['h'],'prechain': oldr['prechain']})
                        seen=True
                        break
                if seen==False:
                    n.update({'cx':newr['cx'],'cy':newr['cy'],'w':newr['w'],'h':newr['h'],'prechain': []})
                csum=np.sum(brite[n['cx']-n['w']/2:n['cx']+n['w']/2,n['cy']-n['h']/2:n['cy']+n['h']/2])
                col=csum/(n['w']*n['h'])
                color='derp'
                if 0<col<30:
                    color='red'
                elif 45<col<95:
                    color='green'
                elif 106<col<140:
                    color='blue'
                if color !='derp':
                    # see if they change color
                    if len(n['prechain']):
                        if len(n['prechain'])==0 or n['prechain'][len(n['prechain'])-1]!=color:
                            n['prechain'].append(color)
                            if len(n['prechain'])>1 and n['prechain'][0]==color:
                                # report a successful pattern pickup
                                retval=True
                                self.recordedPatterns.append(n['prechain'])
                                n['prechain']=[]
                    newcache.append(n)    
        self.cache=newcache
        return retval

