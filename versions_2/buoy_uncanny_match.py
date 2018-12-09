#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import rospy
# load contours from the dictionary


my_path = os.path.abspath(os.path.dirname(__file__))


class BuoyDetector:
    def __init__(self):
        self.max_angle = 35
        # 2 * percentagewise width of the region that we're going to be interested in
        self.interest_region_width = 0.1
        self.toPrintContours = 0
        self.bnu20 = 1

        self.cntdict = {}

        for f in os.listdir(os.path.join(my_path, "cntdict")):
            file = open(os.path.join(my_path, "cntdict", f), "r")
            self.cntdict[f.split(".")[0]] = np.load(file)
            print ("loaded "+f+" with " +
                   str(len(self.cntdict[f.split(".")[0]]))+" vertices")

        if rospy.has_param("~debug_level") and rospy.get_param("~debug_level") == "full":
            self.debug_level = 101
        elif rospy.has_param("~debug_buoy"):
            self.debug_level = 51
        else:
            self.debug_level = 0

    def identify(self, img, filters, bearing=None):
        if self.debug_level > 0:
            dbg_img = img.copy()
        # for mapping colour names
        coldict = {
            'sat_green': 'green',
            'sat_red': 'red',
            'sp_green': 'green',
            'sp_red': 'red',
            'white': 'white',
            'black': 'black',
        }
        # Use canny on the masks
        for i in coldict:
            canny=filters.masks['black']
            
        cnts = filters.getContours(
            img, [i for i in coldict])
        # ,'black obstacle', 'white lightbuoy']
        realItems = ['green buoy', 'red buoy']
        # process contours
        IDs = []

        for col in cnts:
            _cnt = cnts[col]
            # get max area so we know to ignore smaller contours
            _cnts = sorted(_cnt, key=lambda i: cv2.contourArea(i), reverse=True)
            # pick larger half of contours to remove noise contours or otherwise
            _cnt=[]
            for i in range(0, min(max(5, len(_cnts)/2), len(_cnts))):
                _cnt.append(_cnts[i])

            for cnt in _cnt:
                # general noise removal shenanigans
                # remove things with insignificant area
                cA = cv2.contourArea(cnt)
                if cA < 10:
                    continue
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                for dcnt in self.cntdict:
                    similarity = 1 - \
                        cv2.matchShapes(self.cntdict[dcnt], cnt, 1, 0.0)
                    if (similarity > 0):
                        fullname = coldict[col]+' '+dcnt
                        if fullname in realItems:  # filter out obstacles that shouldnt exist
                            IDs.append({"name": fullname, "similarity": similarity, 'cx': cX,
                                        'cnt': cnt, 'confidence': similarity*100+cv2.contourArea(cnt)})
                            if self.debug_level > 50:
                                cv2.drawContours(
                                    dbg_img, [cnt], -1, [0, 255, 0], 4)
                                cv2.putText(dbg_img, fullname, (int(cnt[0][0][0]), int(
                                    cnt[0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 0])
        IDs = sorted(IDs, key=lambda i: i["confidence"], reverse=True)
        # print(IDs)
        if not bearing is None:  # filter based on bearing
            img_halfwidth = img.shape[1]/2
            ROI_center = (bearing / self.max_angle *
                          img_halfwidth) + img_halfwidth
            real_irw = self.interest_region_width*img_halfwidth
            _IDs = [i for i in IDs if abs(i['cx']-ROI_center) < real_irw]
            IDs = _IDs
        if self.debug_level > 0:
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
