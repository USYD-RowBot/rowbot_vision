import cv2
import numpy as np

def bgr(img):
    return img

def hsv(img):
    return 
filters={
    "bgr":{"result":None,"procedure":lambda i: return i},
    "hsv":{"result":None,"procedure":lambda i: return i},

}


cacheSum=0
def preprocess(img):
    if np.sum(img)==cacheSum:
        # we have already processed this image- do nothing
        return
    else:
        for 

def getContours