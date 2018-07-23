import cv2
import numpy as np
import os



lowerRange=np.array([0,0,0])
upperRange=np.array([180,255,255])

def reRender(x):
    global lowerRange;
    global upperRange;
    lowerRange=np.array([cv2.getTrackbarPos('Hmin','image'),cv2.getTrackbarPos('Smin','image'),cv2.getTrackbarPos('Vmin','image')])
    upperRange=np.array([cv2.getTrackbarPos('Hmax','image'),cv2.getTrackbarPos('Smax','image'),cv2.getTrackbarPos('Vmax','image')])

# Create a black image, a window
cap = cv2.VideoCapture(0)

cv2.namedWindow('image')
cv2.createTrackbar('Hmin','image',0,255,reRender)
cv2.createTrackbar('Hmax','image',255,255,reRender)
cv2.createTrackbar('Smin','image',0,180,reRender)
cv2.createTrackbar('Smax','image',180,180,reRender)
cv2.createTrackbar('Vmin','image',0,255,reRender)
cv2.createTrackbar('Vmax','image',255,255,reRender)




while(1):

    # Take each frame
    _, frame = cap.read()
    
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    

    # create trackbars for color change
    mask = cv2.inRange(hsv, lowerRange, upperRange)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow('mask',mask)
    cv2.imshow('frame',frame)
    cv2.imshow('res',res)
    
    
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of four trackbars
    

cv2.destroyAllWindows()
