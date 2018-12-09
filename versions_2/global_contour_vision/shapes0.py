#!/usr/bin/env python
import cv2
import numpy as np
import filters
import os
import rospy

class ShapeDetector:
    def __init__(self):
        self.max_angle=70
        self.interest_region_width=0.1
        if rospy.has_param("~debug_level") and rospy.get_param("~debug_level") == "full":
            self.debug_level=101
        elif rospy.has_param("~debug_shape"):
            self.debug_level=51
        else:
            self.debug_level=0
        pass

    def identify(self,img,filters,bearing=None):

        if self.debug_level > 0:
            dbg_img = img.copy()
        #rospy.loginfo(rospy.get_param('~debug_level'))
        coldict = {
            'sat_green': 'green',
            'sp_green': 'green',
            'sp_red': 'red',
            'sat_red': 'red',
            'sp_blue': 'blue',
            'sat_blue': 'blue'
        }
        cnts = filters.getContours(
            img, [i for i in coldict])
        # process contours
        IDs = []

        # get max area so we know to ignore smaller contours
        #cnts=sorted(cnts,key=lambda i:cv2.contourArea(i),reverse=True)

        # for i in range(0,min(5,len(cnts))): # pick largest 5 contours or otherwise
        #    cnt=cnts[i]
        for col in cnts:
            _cnt = cnts[col]
            #print ("contour parent was outer level")
            for c in _cnt:
                # compute the center of the contour, then detect the name of the
                # shape using only the contour
                shape = "weird"
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.03 * peri, True)
                cA = cv2.contourArea(approx)
                if cA > 1 and (cv2.contourArea(c)/cA) < 0.95:
                    shape = "weird"
                # Classify ellipses
                if len(approx) > 5 and cv2.isContourConvex(approx):
                    ellipse = cv2.fitEllipse(approx)
                    ellipseArea = ellipse[1][0]*ellipse[1][1]*np.pi/4
                    if (ellipseArea):
                        if np.abs((cA-ellipseArea)/ellipseArea) < 1:
                            shape = 'circle'
                # Classify based on vertex count
                elif len(approx) == 3:
                    shape = "triangle"

                elif len(approx) == 12:
                    shape = "cross"
                else:
                    shape = "weird"
                x, y, w, h = cv2.boundingRect(c)
                rat = float(w)/float(h)
                cA = cv2.contourArea(c)
                # check the shape
                if shape != "weird" and cA > 80 and rat > 0.7 and rat < 1.5:
                    fullname=col+" "+shape
                    IDs.append({'area': cA, 'cnt': c, 'shape': shape,
                                'color': col, 'confidence': cA,'cx':x+w/2,'fullname':fullname})
                    if self.debug_level>50:
                        print(x)
                        cv2.drawContours(dbg_img, [c], -1, [0, 255, 0], 4)
                        cv2.putText(dbg_img, fullname, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2, [0, 255, 0])
        # POST PROCESSSING: confidence based sorting
        IDs = sorted(IDs, key=lambda i: i['area'], reverse=True)

        if not bearing is None:  # filter based on bearing
            img_halfwidth = img.shape[1]/2
            ROI_center = (bearing / self.max_angle *
                          img_halfwidth) + img_halfwidth
            real_irw = self.interest_region_width*img_halfwidth
            _IDs = [i for i in IDs if abs(i['cx']-ROI_center) < real_irw]
            IDs = _IDs
        # get the colours now.
        if self.debug_level>0:
            cv2.imshow('shapes_output',dbg_img)
            cv2.waitKey(10)
        return IDs


#file testing not available atm

"""
my_path = os.path.abspath(os.path.dirname(__file__))

debugMode = "fromFile"
if __name__ == "__main__":
    if debugMode == "fromFile":
        for f in os.listdir(os.path.join(my_path, "shp_tst")):
            print(f)
            testimg = cv2.imread(os.path.join(my_path, "shp_tst", f))
            # print(testimg)
            r = identify(testimg)
            print(r)
            cv2.waitKey(-1)
"""
