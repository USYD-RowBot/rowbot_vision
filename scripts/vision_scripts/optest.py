import numpy as np
import cv2
import os
import buoy as toTest

def getContoursFromMask(mask):
    # kernel = np.ones((5,5),np.uint8)
    # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closing=mask
    _cnts = cv2.findContours(closing, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    _cnts = _cnts[1]
    return _cnts

cnts=[]
for f in os.listdir("cntdict_tst"):
    img=cv2.imread(os.path.join("cntdict_tst",f))
    img=img.astype(int)
    mask=255-np.clip(img[:,:,1]-((img[:,:,0]+img[:,:,2])/2),0,255)# green mask
    cnt=getContoursFromMask(cv2.inRange(img,(10,10,10),(255,255,255))) # basically black and white
    result=toTest.identify(cnt)
    print(f+":"+"".join(str(result)))