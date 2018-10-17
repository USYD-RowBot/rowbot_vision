#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import filters

my_path = os.path.abspath(os.path.dirname(__file__))

cachedPattern=[]
cachedContourRect=[]

# todo: value and saturation detection.

def clearAndPush():
    global cachedPattern
    _cachedPattern=cachedPattern
    cachedPattern=[]
    return _cachedPattern

def identify(img):
    global cachedPattern
    global cachedContourRect
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
    nextCnt=[]
    for cnt in cnts:
        # pick out 2nd level contours which overlap the previous detection
        if cnt[2][3] in rectIndices:
            #print ("contour parent was outer level")
            if cachedContourRect==[] or (cachedContourRect[0]<cnt[1][0]+cnt[1][2]/2<cachedContourRect[0]+cachedContourRect[2] and
                cachedContourRect[1]<cnt[1][1]+cnt[1][3]/2<cachedContourRect[1]+cachedContourRect[3]):
                #print("contour was in cached contour rect")
                if cv2.contourArea(cnt[0])/(cnt[1][2]*cnt[1][3])>0.8:
                    #print("contour was rectangular")
                    nextCnt.append((cnt[0],cnt[1]))
    '''cnt
        [0]:contour
        [1]:bounding rect
    '''
    try:
        mostProbable=max(nextCnt,key=lambda x: cv2.contourArea(x[0]))
        cachedContourRect=mostProbable[1]
        col=filters.getContourColor(img,mostProbable[0])
        if 0<col<60:
            nextCol="red"
        if 60<col<120:
            nextCol="green"
        if 120<col<180:
            nextCol="blue"
        if len(cachedPattern)>1 and nextCol==cachedPattern[0]:
            return clearAndPush()
        elif len(cachedPattern)>0 and cachedPattern[len(cachedPattern)-1]==nextCol:
            print("Light beacon: Pattern persists")
            return 1 # pattern persists
        else:
            cachedPattern.append(nextCol)
            print("Light beacon: Pattern logged")
            return 2 # return code for pattern logged
    except ValueError:
        # out of range; return sequence
        if (cachedPattern!=[]):
            return clearAndPush();
        # Otherwise return a no-show
        print("Light beacon: No Show")
        return 0

debugMode="fromFile"
if __name__=="__main__":
    if debugMode=="fromFile":
        for f in os.listdir(os.path.join(my_path,"lb_tst")):
            print(f)
            testimg=cv2.imread(os.path.join(my_path,"lb_tst",f))
            #print(testimg)
            r=identify(testimg)
            print(r)
            # for id in r:
            #     testimg=cv2.drawContours(testimg,[id['cnt']],0,(255,255,0),2)
            #     cv2.putText(testimg,id['name'],(id['cnt'][0,0,0],id['cnt'][0,0,1]),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            # cv2.imshow('result',testimg)
            cv2.waitKey(-1)
            
    