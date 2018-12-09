#!/usr/bin/env python
import rospy
import cv2
import numpy as np


class ShapeDetector:
    def __init__(self):
        # ros based parameters
        self.params = {
            'debugLevel': 0,
            'debugShape': 0,
        }
        try:
            for i in self.params:
                self.params[i] = rospy.get_param('~'+i, self.params[i])
        except Exception:
            pass
        self.params['debugLevel'] = max(
            self.params['debugLevel'], self.params['debugShape'])
        self.cache = []
        self.recordedPatterns = []
        pass

    def identify(self, img, ):
        # generate notcolor filter

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

        notcolor = cv2.inRange(s_hsv, (0, 0, 0), (180, np.max(s_hsv)*0.15, 255))

        # get contours from notcolor filter

        # fetch the contours
        im,cnts,hier = cv2.findContours(notcolor, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
        IDs=[]
        for c in cnts:
            shape="weird"
            peri=cv2.arcLength(c, True)
            approx=cv2.approxPolyDP(c, 0.03 * peri, True)
            cA=cv2.contourArea(approx)
            if cA > 1 and (cv2.contourArea(c)/cA) < 0.95:
                shape="weird"
            # Classify ellipses
            if len(approx) > 5 and cv2.isContourConvex(approx):
                ellipse=cv2.fitEllipse(approx)
                ellipseArea=ellipse[1][0]*ellipse[1][1]*np.pi/4
                if (ellipseArea):
                    if np.abs((cA-ellipseArea)/ellipseArea) < 1:
                        shape='circle'
            # Classify based on vertex count
            elif len(approx) == 3:
                shape="triangle"
            elif len(approx) == 12:
                shape="cross"
            else:
                shape="weird"
            if shape!='weird':
                # see if they change color
                mask = np.zeros_like(img[:, :, 0])
                mask = cv2.drawContours(mask, [c], 0, 1, -1)
                img2 = s_hsv[:, :, 0] * mask.astype(s_hsv.dtype)
                col= np.sum(img2)/cv2.contourArea(c)
                color='derp'
                if 0<col<30:
                    col='red'
                elif 45<col<95:
                    col='green'
                elif 106<col<140:
                    col='blue'
                if color!='derp':
                    # report a detection
                    IDs.append({'area': cA, 'cnt': c, 'shape': shape,
                        'color': col, 'confidence': cA})
        IDs=sorted(IDs,key=lambda i: i['confidence'],reverse=True)
        return IDs
