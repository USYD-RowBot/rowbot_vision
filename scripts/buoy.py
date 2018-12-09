#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import rospy
# load contours from the dictionary



class BuoyDetector:
    def __init__(self):
        self.max_angle = 35
        # 2 * percentagewise width of the region that we're going to be interested in
        self.interest_region_width = 0.1
        self.toPrintContours = 0
        self.bnu20 = 1
        if rospy.has_param("~debug_level") and rospy.get_param("~debug_level") == "full":
            self.debug_level=101
        elif rospy.has_param("~debug_buoy"):
            if rospy.get_param("~debug_buoy") == "full":
                self.debug_level=101
            else:
                self.debug_level=51
        else:
             self.debug_level=0

    def identify(self, img, filters, bearing=None):
        if self.debug_level > 0:
            dbg_img = img.copy()
        # apply a bunch of relevant filters to get contours

        # for mapping colour names
        coldict = {
            'sat_green': 'green',
            'sat_red': 'red',
            'white': 'white',
            'black': 'black',
        }
        cnts = filters.getContours(
            img, [i for i in coldict])
        # ,'black obstacle', 'white lightbuoy']
        realItems = ['green buoy', 'red buoy']
        # process contours
        IDs = []

        # get max area so we know to ignore smaller contours
        #cnts=sorted(cnts,key=lambda i:cv2.contourArea(i),reverse=True)

        # for i in range(0,min(5,len(cnts))): # pick largest 5 contours or otherwise
        #    cnt=cnts[i]
        for col in cnts:
            _cnt = cnts[col]
            #av=[cv2.contourArea(i) for i in _cnt]
            # print(av)
            if self.debug_level>0:
                _cnt = sorted(
                    _cnt, key=lambda i: cv2.contourArea(i), reverse=True)
            #av=[cv2.contourArea(i) for i in _cnt]
            # print(av)
            for cnt in _cnt:
                if rospy.has_param('~debug_level') and rospy.get_param('~debug_level') == 'full':
                    cv2.drawContours(dbg_img, [cnt], -1, [0, 0, 0])

                cA = cv2.contourArea(cnt)
                if cA < 10:
                    continue
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                if len(cnt) > 5:
                    try:
                        elps = cv2.fitEllipse(cnt)[1]
                        elpsa = elps[0]*elps[1]
                        if (abs(1-cv2.contourArea(cnt)/elpsa) < 0.01):  # shape is elliptical
                            fullname = coldict[col]+' obstacle'
                            if fullname in realItems:  # filter out obstacles that shouldnt exist
                                IDs.append(
                                    {"name": fullname, 'cx': cX, 'cnt': cnt, 'confidence': cv2.contourArea(cnt)})
                    except:
                        pass


                # parameters
                recta = rect[1][0]*rect[1][1]

                ratio = 0.2
                ratioError = 1.0/3.0
                #rectangularity = 0.25
                #angleLowerBound = -80
                #angleUpperBound = -10
                # https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned
                if self.debug_level>100:
                    if (abs(rect[1][0]/rect[1][1]-ratio) < ratioError):
                        cv2.drawContours(
                            dbg_img, [cnt], -1, [255, 255, 0], 6)  # cyan
                        rospy.loginfo(rect[2])
                    if rect[2]<angleLowerBound or rect[2]>angleUpperBound:
                       cv2.drawContours(dbg_img, [cnt], -1, [255, 0, 0], 4) # blue
                    #if (abs(1-recta/cv2.contourArea(cnt)) < rectangularity):
                    #    cv2.drawContours(
                    #        dbg_img, [cnt], -1, [255, 0, 255], 2)  # purple
                if abs(rect[1][0]/rect[1][1]-ratio) < ratioError: # structured like this so we can easily toggle conditions
                 #if abs(1-recta/cv2.contourArea(cnt)) < rectangularity:
                   if rect[2]<angleLowerBound or rect[2]>angleUpperBound:
                    fullname = coldict[col]+' buoy'
                    if self.debug_level>50:
                        cv2.drawContours(dbg_img, [cnt], -1, [0, 255, 0], 4)
                        cv2.putText(dbg_img, str(rect), (int(rect[0][0]), int(
                            rect[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.2, [0, 255, 0])
                    if fullname in realItems:  # filter out obstacles that shouldnt exist
                        # filter based on bearing
                        IDs.append(
                            {"name": fullname, 'cx': cX, 'cnt': cnt, 'confidence': cv2.contourArea(cnt)})

                """
                recta = (rect[0][1]-rect[1][1])*(rect[0][0]-rect[1][0])
                if (  # abs(1-cv2.contourArea(cnt)/recta)<0.1 and
                        abs((rect[0][1]-rect[1][1])/(rect[0][0]-rect[1][0])-4) < 1.9):  # and abs(rect[2])<20):
                    fullname = coldict[col]+' buoy'
                    print ('ok')
                    if fullname in realItems:  # filter out obstacles that shouldnt exist
                        IDs.append(
                            {"name": fullname, 'cx': cX, 'cnt': cnt, 'confidence': cv2.contourArea(cnt)})
                        if rospy.has_param('~debug_level') and rospy.get_param('~debug_level') == 'full':
                            cv2.drawContours(
                                dbg_img, [cnt], -1, [255, 255, 0], 5)
                """
                if (self.toPrintContours != -1):
                    # ,1-cv2.contourArea(cnt)/recta)
                    print ((rect[0][1]-rect[1][1]) /
                           (rect[0][0]-rect[1][0])-0.25, rect[2]+90, cA)
                    """
                    __rect=np.array((
                        ((rect[0][0],rect[0][1])),
                        ((rect[1][0],rect[0][1])),
                        ((rect[1][0],rect[1][1])),
                        ((rect[0][0],rect[1][1]))
                    ), np.int32)
                    print(cnt)
                    print(__rect)
                    cv2.fillConvexPoly(dbg_img,__rect,[255, 0, 255])
                    """
        IDs = sorted(IDs, key=lambda i: i["confidence"], reverse=True)
        # print(IDs)
        if not bearing is None:  # filter based on bearing
            img_halfwidth = img.shape[1]/2
            ROI_center = (bearing / self.max_angle *
                          img_halfwidth) + img_halfwidth
            real_irw = self.interest_region_width*img_halfwidth
            _IDs = [i for i in IDs if abs(i['cx']-ROI_center) < real_irw]
            IDs = _IDs
        if self.debug_level>0:
            cv2.imshow('buoy_output', dbg_img)
            self.toPrintContours = cv2.waitKey(10)

        return IDs


""" file only debugging still needs work
import filters
my_path = os.path.abspath(os.path.dirname(__file__))
debugMode="fromFile"
if __name__=="__main__":
    if debugMode=="fromFile":
        flt=filters.FilterCacher()
        bd=BuoyDetector()
        for f in os.listdir(os.path.join(my_path,"cntdict_tst")):

            testimg=cv2.imread(os.path.join(my_path,"cntdict_tst",f))
            flt.preprocess(testimg)
            #print(testimg)
            r=bd.identify(testimg,flt)
            print(r)
            for id in r:
                testimg=cv2.drawContours(testimg,[id['cnt']],0,(255,255,0),2)
                cv2.putText(testimg,id['name'],(id['cnt'][0,0,0],id['cnt'][0,0,1]),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            cv2.imshow('result',testimg)
            cv2.waitKey(-1)
"""
