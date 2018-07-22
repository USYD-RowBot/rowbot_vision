import cv2
import numpy as np
import os




def reRender(x):
    lowerRange=np.array([cv2.getTrackbarPos('Hmin','image'),cv2.getTrackbarPos('Smin','image'),cv2.getTrackbarPos('Vmin','image')])
    upperRange=np.array([cv2.getTrackbarPos('Hmax','image'),cv2.getTrackbarPos('Smax','image'),cv2.getTrackbarPos('Vmax','image')])
    mask = cv2.inRange(hsv, lowerRange, upperRange)
    cv2.imshow('image',mask)

# Create a black image, a window
frame = cv2.imread(os.path.join(os.path.dirname(__file__), 'yinput.png'))
hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Hmin','image',0,255,reRender)
cv2.createTrackbar('Smin','image',0,255,reRender)
cv2.createTrackbar('Vmin','image',0,255,reRender)
cv2.createTrackbar('Hmax','image',0,255,reRender)
cv2.createTrackbar('Smax','image',0,255,reRender)
cv2.createTrackbar('Vmax','image',0,255,reRender)
lowerRange=np.array([0,100,0])
upperRange=np.array([180,255,255])
mask = cv2.inRange(hsv, lowerRange, upperRange)
cv2.imshow('image',mask)
while(1):
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of four trackbars
    

cv2.destroyAllWindows()
