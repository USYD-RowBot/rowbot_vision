#!/usr/bin/env python
import cv2
import numpy as np
import os
import json
# load contours from the dictionary

# saved_cnts=open()
cntdict={}
my_path = os.path.abspath(os.path.dirname(__file__))

for f in os.listdir(os.path.join(my_path,"cntdict")):
    file=open(os.path.join(my_path,"cntdict",f),"r")
    cntdict[f.split(".")[0]]=np.load(file)
    print ("loaded "+f+" with "+str(len(cntdict[f.split(".")[0]]))+" vertices")

# Load contours into the dictionary.

# Things we need to look out for:
    # buoys
    # light beacon
    # obstacle beacons
    # thats litreally it

def identify(cnts):
    IDs=[]
    for cnt in cnts:
        # general noise removal shenanigans
        # remove things with insignificant area
        cA=cv2.contourArea(cnt)
        if cA<3:
            continue
        M = cv2.moments(cnt)
        cX = int(M["m10"] / M["m00"])
        for dcnt in cntdict:
            similarity=1-cv2.matchShapes(cntdict[dcnt],cnt,1,0.0)
            if (similarity>0):
                IDs.append({"obj":dcnt,"similarity":similarity,'cx':cX})
            
    IDs=sorted(IDs,key=lambda i:i["similarity"],reverse=True)
    return IDs
# https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_contours/py_contours_more_functions/py_contours_more_functions.html#contours-more-functions

realItems=['green buoy','red buoy','white buoy','black obstacle', 'white lightbuoy']

def collate(results):
    retval=[]
    for color in results:
        for id in results[color]:
            if (color+" "+id['obj']) in realItems: # if this item/object combination is valid
                retval.append({'name':color+" "+id['obj'],'conf':id['similarity'], 'cx': id['cx']})
    retval=sorted(retval,key=lambda i: i['conf'],reverse=True)
    return retval

# clipping, for later
# img_halfwidth=chan.shape[0]/2
# ROI_center = (bearing / max_angle * image_halfwidth)+ image_halfwidth 
# clipped_image=chan[ROI_center-interest_region_width*img_halfwidth:ROI_center+interest_region_width*img_halfwidth]
# #cv2.imshow("baii",clipped_image)
# #cv2.waitKey(1)
# res=BuoyDetect(clipped_image)