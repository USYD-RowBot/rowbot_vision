#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
import filters
# load contours from the dictionary

cntdict={}
my_path = os.path.abspath(os.path.dirname(__file__))

for f in os.listdir(os.path.join(my_path,"cntdict")):
    file=open(os.path.join(my_path,"cntdict",f),"r")
    cntdict[f.split(".")[0]]=np.load(file)
    print ("loaded "+f+" with "+str(len(cntdict[f.split(".")[0]]))+" vertices")





max_angle=35
interest_region_width=0.1 ## 2 * percentagewise width of the region that we're going to be interested in 



def identify(img, bearing = None):
    # apply a bunch of relevant filters to get contours
    cnts=filters.getContours(img,['sat_green','sat_red','sp_green','sp_red','white'])
    # for mapping colour names
    coldict={
        'sat_green':'green',
        'sat_red':'red',
        'sp_green':'green',
        'sp_red':'red',
        'white':'white'
    }

    realItems=['green buoy','red buoy','white buoy','black obstacle', 'white lightbuoy']
    # process contours
    IDs=[]

    # get max area so we know to ignore smaller contours
    #cnts=sorted(cnts,key=lambda i:cv2.contourArea(i),reverse=True)

    #for i in range(0,min(5,len(cnts))): # pick largest 5 contours or otherwise
    #    cnt=cnts[i]
    for col in cnts:
        _cnt=cnts[col]
        for cnt in _cnt:
            # general noise removal shenanigans
            # remove things with insignificant area
            cA=cv2.contourArea(cnt)
            if cA<10:
                continue
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            for dcnt in cntdict:
                similarity=1-cv2.matchShapes(cntdict[dcnt],cnt,1,0.0)
                if (similarity>0):
                    fullname=coldict[col]+' '+dcnt
                    if fullname in realItems: # filter out obstacles that shouldnt exist
                        IDs.append({"name":fullname,"similarity":similarity,'cx':cX,'cnt':cnt})
    IDs=sorted(IDs,key=lambda i:i["similarity"],reverse=True)
    if not bearing is None:# filter based on bearing
        img_halfwidth=img.shape[1]/2
        ROI_center = (bearing / max_angle * img_halfwidth)+ img_halfwidth 
        real_irw=interest_region_width*img_halfwidth
        _IDs=[i for i in IDs if abs(i['cx']-ROI_center)<real_irw]
        IDs=_IDs
    return IDs











# clipping, for later
# img_halfwidth=chan.shape[0]/2
# ROI_center = (bearing / max_angle * image_halfwidth)+ image_halfwidth 
# clipped_image=chan[ROI_center-interest_region_width*img_halfwidth:ROI_center+interest_region_width*img_halfwidth]
# #cv2.imshow("baii",clipped_image)
# #cv2.waitKey(1)
# res=BuoyDetect(clipped_image)
debugMode="fromFile"
if __name__=="__main__":
    if debugMode=="fromFile":
        for f in os.listdir(os.path.join(my_path,"cntdict_tst")):
            
            testimg=cv2.imread(os.path.join(my_path,"cntdict_tst",f))
            #print(testimg)
            r=identify(testimg)
            print(r)
            for id in r:
                testimg=cv2.drawContours(testimg,[id['cnt']],0,(255,255,0),2)
                cv2.putText(testimg,id['name'],(id['cnt'][0,0,0],id['cnt'][0,0,1]),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            cv2.imshow('result',testimg)
            cv2.waitKey(-1)
            
    