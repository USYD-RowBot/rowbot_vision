#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import filters
import rospy 

my_path = os.path.abspath(os.path.dirname(__file__))

class LightPatternDetector:
    def __init__(self,file_based=False):
        if not file_based:
            if rospy.has_param("~debug_level") and rospy.get_param("~debug_level") == "full":
                self.debug_level=101
            elif rospy.has_param("~debug_pattern"):
                self.debug_level=51
            else:
                self.debug_level=0
        else:
            self.debug_level=0
        self.cachedPattern=[]
        self.cachedContourRect=[]
        # todo: value and saturation detection.
    def clearAndPush(self):
        _cachedPattern=self.cachedPattern
        self.cachedPattern=[]
        return _cachedPattern

    def identify(self, img, filters):
        # apply a bunch of relevant filters to get contours 
        cnts=filters.getContours(img,['white'],True)
        # find 2 box contours nested inside each other

        nextCnt=[]
        rectIndices=[]
        for c,cnt in enumerate(cnts['white'][0]):
            # check if they are rectangles
            ## will not work because the contours will be hollow!
            r=cv2.boundingRect(cnt)
            nextCnt.append((cnt,r,cnts['white'][1][c]))
            if (cnts['white'][1][c][3]==-1):rectIndices.append(c)# append outermost white rectangles
            # if cv2.contourArea(cnt[0])/(r[2]*r[3])<0.9:
                
        cnts=nextCnt
        '''cnt
            [0]:contour
            [1]:bounding rect
            [2]:hierarchy
        '''
        if self.debug_level>0:
            dbg_img=img.copy()
        nextCnt=[]
        for cnt in cnts:
            if self.debug_level>50:
                cv2.drawContours(dbg_img,[cnt[0]],-1,[0,0,0])
            # pick out 2nd level contours which overlap the previous detection
            if cnt[2][3] in rectIndices:
                #print ("contour parent was outer level")
                if self.cachedContourRect==[] or (self.cachedContourRect[0]<cnt[1][0]+cnt[1][2]/2<self.cachedContourRect[0]+self.cachedContourRect[2] and
                    self.cachedContourRect[1]<cnt[1][1]+cnt[1][3]/2<self.cachedContourRect[1]+self.cachedContourRect[3]):
                    #print("contour was in cached contour rect")
                    if cv2.contourArea(cnt[0])/(cnt[1][2]*cnt[1][3])>0.8:
                        #print("contour was rectangular")
                        nextCnt.append((cnt[0],cnt[1]))
        '''cnt
            [0]:contour
            [1]:bounding rect
        '''
        if self.debug_level>0:
            cv2.imshow('lightbcn_output',dbg_img)
            cv2.waitKey(10)
        try:
            mostProbable=max(nextCnt,key=lambda x: cv2.contourArea(x[0]))
            self.cachedContourRect=mostProbable[1]
            col=filters.getContourColor(img,mostProbable[0])
            if 0<col<60:
                nextCol="red"
            if 60<col<120:
                nextCol="green"
            if 120<col<180:
                nextCol="blue"
            if len(self.cachedPattern)>1 and nextCol==self.cachedPattern[0]:
                return self.clearAndPush()
            elif len(self.cachedPattern)>0 and self.cachedPattern[len(self.cachedPattern)-1]==nextCol:
                print("Light beacon: Pattern persists")
                return 1 # pattern persists
            else:
                self.cachedPattern.append(nextCol)
                print("Light beacon: Pattern logged")
                return 2 # return code for pattern logged
        except ValueError:
            # out of range; return sequence
            if (self.cachedPattern!=[]):
                return self.clearAndPush()
            # Otherwise return a no-show
            rospy.loginfo("Light beacon: No Show")
            return 0

debugMode="fromFile"
if __name__=="__main__":
    import filters
    draw_colors={
        "green":[0,255,0],
        "blue":[255,0,0],
        "red":[0,0,255]
    }
    ft=filters.FilterCacher()
    if debugMode=="fromFile":
        lp=LightPatternDetector(True)
        for f in os.listdir(os.path.join(my_path,"lb_tst")):
            print(f)
            testimg=cv2.imread(os.path.join(my_path,"lb_tst",f))
            ft.preprocess(testimg)
            lp.identify(testimg,ft)
            r=lp.cachedPattern
            h,w,chan=testimg.shape
            print(r)
            for (c,id) in enumerate(r):
                cv2.rectangle(testimg,(c*w/10,h*9/10),((c+1)*w/10,h),draw_colors[id],-1)
                #cv2.putText(testimg,id['name'],(id['cnt'][0,0,0],id['cnt'][0,0,1]),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            cv2.imshow('result',testimg)
            cv2.waitKey(-1)