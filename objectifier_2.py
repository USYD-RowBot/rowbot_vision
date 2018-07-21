import cv2
import numpy as np
import knn #Not actually KNN but does the same kinda thing.

frame = cv2.imread('C:/Users/Steven Liu/Documents/Projects/Development/Python/opencv/line detection/yinput.jpg')
hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
overmask=0;

# For each color...
colors=[
(np.array([95,100,60]),np.array([140,255,255])), # blue
(np.array([45,100,60]),np.array([80,255,255])),  # green
(np.array([165,100,60]),np.array([180,255,255])),  # red 1
(np.array([0,100,60]),np.array([15,255,255]))  # red 2
]

shapeID="none"
colorID="none"
for c,i in enumerate(colors):
    # define range of color in HSV
    lowerRange = i[0]
    upperRange = i[1]
    shapeIdentified=False;
    # Threshold the HSV image to get only desired colors
    mask = cv2.inRange(hsv, lowerRange, upperRange)
    
    ## Line detection method
    edges = cv2.Canny(mask,50,150,apertureSize = 3)
    # cv2.imshow('edj',edges)
    minLineLength = 30
    maxLineGap = 50
    lines = cv2.HoughLinesP(edges,1,np.pi/1000,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
    
    if not lines is None:
        # print (lines)
        intersections=[];
        for a1,k1 in enumerate(lines):
            x1=k1[0][0]
            y1=k1[0][1]
            x2=k1[0][2]
            y2=k1[0][3]
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,0),2)
            # Matrix math to find intersections with all other lines
            for a2,k2 in enumerate(lines):
                if not a1==a2:
                    bb=np.array([[k1[0][2]-k1[0][0],k2[0][2]-k2[0][0]],[[k1[0][3]-k1[0][1],k2[0][3]-k2[0][1]]])
                    bb=np.linalg.inv(bb)
                    aa=np.array([k1[0][0]-k2[0][0],k1[0][1]-k2[0][1]])
                    kk=np.matmul(bb,aa); # 2x2 coefficients
                    intersections.append(np.array([k1[0][0]+kk[0]*bb[0][0],k1[0][1]+kk[0]*bb[1][0]]))
        # (fake) KNN the intersections
        
        
        # Use KNN to find how many regions there are. 
        
        # merge similar lines? 
        
        
        
        
        # print (lines.size)
        if lines.size/4<4:
            shapeID="Triangle"
            shapeIdentified=True;
        elif lines.size/4<14:
            shapeID="Cross"
            shapeIdentified=True;
    
    
    
    
    
    # check if there is a circle
    image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    # Get largest contour
    if not (contours is None or len(contours)<1):
        lcont=None;
        lcontA=0;
        for d,i in enumerate(contours):
            cA=cv2.contourArea(i)
            if lcontA<cA:
                lcontA=cA;
                lcont=i
        # is the countour roughly circular/elliptical? Calculate dA/A from bounding ellipse
        if (len(lcont)>5):
            ellipse = cv2.fitEllipse(lcont)
            ellipseArea=ellipse[1][0]*ellipse[1][1]*np.pi/4
            if np.abs((lcontA-ellipseArea)/ellipseArea) < 0.02:
                shapeID="Circle"
                shapeIdentified=True;
                print(lcontA)
    
    # Color filtering
    if shapeIdentified:
        if c==0:
            colorID="Blue"    
        elif c==1:
            colorID="Green"
        else:
            colorID="Red"

font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(frame,shapeID,(10,200), font, 2,(0,0,0),2,cv2.LINE_AA)
cv2.putText(frame,colorID,(10,300), font, 2,(0,0,0),2,cv2.LINE_AA)

cv2.imshow('frame',frame)

while 1:
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()