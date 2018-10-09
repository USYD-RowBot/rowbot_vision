#!/usr/bin/env python
import cv2
import numpy as np
import os
# load contours from the dictionary

def getContoursFromMask(mask):
    # kernel = np.ones((5,5),np.uint8)
    # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closing=mask
    cnts = cv2.findContours(closing, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1]
    return cnts

# saved_cnts=open()
cntdict={}
for f in os.listdir("cntdict_img"):
    img=cv2.imread(os.path.join("cntdict_img",f))
    print (os.path.join("cntdict_img",f))
    img=img.astype(int)
    filename=os.path.join("cntdict",f).split(".")[0]+".npy"
    cnts=getContoursFromMask(cv2.inRange(img,(10,10,10),(255,255,255))) # basically black and white
    cnts = sorted(cnts, key=lambda i:cv2.contourArea(i),reverse=True)
    cnt=cnts[0]
    fl=open(filename,"w")
    np.save(fl,cnt)
