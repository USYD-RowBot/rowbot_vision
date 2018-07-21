# Generate the full RGB spectrum and apply a mask to try and get the mask we want.
import cv2
import numpy as np


h=np.array([np.arange(0,255,dtype=np.uint8)]*255)
s=np.transpose([np.arange(0,255,dtype=np.uint8)]*255)
v=np.ones((255,255),dtype=np.uint8)*255;
screen=np.stack((h,s,v),2);
bgr=cv2.cvtColor(screen,cv2.COLOR_HSV2BGR_FULL)
cv2.imshow('meep',bgr);

h=np.array([np.arange(0,255,dtype=np.uint8)]*255)
s=np.ones((255,255),dtype=np.uint8)*255;
v=np.transpose([np.arange(0,255,dtype=np.uint8)]*255)
screen=np.stack((h,s,v),2);
bgr=cv2.cvtColor(screen,cv2.COLOR_HSV2BGR_FULL)
cv2.imshow('beep',bgr);

h=np.ones((255,255),dtype=np.uint8)*255;
s=np.transpose([np.arange(0,255,dtype=np.uint8)]*255)
v=np.transpose([np.arange(0,255,dtype=np.uint8)]*255)
screen=np.stack((h,s,v),2);
bgr=cv2.cvtColor(screen,cv2.COLOR_HSV2BGR_FULL)
cv2.imshow('jeep',bgr);

h=np.array([np.arange(0,255,dtype=np.uint8)]*255)
s=np.transpose([np.arange(0,255,dtype=np.uint8)]*255)
v=np.ones((255,255),dtype=np.uint8)*255;
screen=np.stack((h,s,v),2);
bgr=cv2.cvtColor(screen,cv2.COLOR_HSV2BGR_FULL)
cv2.imshow('meep',bgr);






while 1:
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()