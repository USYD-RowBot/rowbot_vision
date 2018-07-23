import cv2
import numpy as np
import os




# For each color...
colors=[
(np.array([95,100,60]),np.array([140,255,255])), # blue
(np.array([25,100,60]),np.array([85,255,255])),  # green
(np.array([165,100,60]),np.array([180,255,255])),  # red 1
(np.array([0,100,60]),np.array([15,255,255]))  # red 2
]
cap=cv2.VideoCapture(0)
while (1):
    shapeID="none"
    colorID="none"
    r, frame = cap.read()
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    overmask=0;
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
        
        #Should put in a confidence rating and maximise the confidence rating.
        lineCount=0;
        if not lines is None:
            # print (lines)
            intersections=[];
            for a1,k1 in enumerate(lines):
                x1=k1[0][0]
                y1=k1[0][1]
                x2=k1[0][2]
                y2=k1[0][3]
                ignoreThis=False
                for a2,k2 in enumerate(lines):
                    if (a2>a1):
                        # merge similar lines
                        # Gradients similar?
                        dx1=k1[0][0]-k1[0][2]
                        dy1=k1[0][1]-k1[0][3]
                        dx2=k2[0][0]-k2[0][2]
                        dy2=k2[0][1]-k2[0][3]
                        gradientSimilarityCoefficient=20
                        startingPointSimilarityCoefficient=40 #THESE SHOULDNT BE STATIC
                        if np.abs(dx2*dy1-dx1*dy2)<=np.abs(gradientSimilarityCoefficient*dx1*dx2) and \
                            np.abs(dy2*dx1-dy1*dx2)<=np.abs(gradientSimilarityCoefficient*dy1*dy2) and \
                            (np.abs((k1[0][0]-k2[0][0])**2+(k1[0][1]-k2[0][1])**2)<startingPointSimilarityCoefficient or \
                            np.abs((k1[0][2]-k2[0][0])**2+(k1[0][3]-k2[0][1])**2)<startingPointSimilarityCoefficient):
                            # Lines are similar, ignore one.
                            ignoreThis=True;
                        else:
                            print(np.abs(dx2*dx1-dx1*dy2),gradientSimilarityCoefficient*dx1*dx2)
                        # Starting positions similar?
                if not ignoreThis:
                    lineCount+=1;
                    cv2.line(frame,(x1,y1),(x2,y2),(0,0,0),2)
                else:
                    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
            
            
            # print (lines.size)
            if lineCount<4:
                shapeID="Triangle"
                shapeIdentified=True;
            elif lineCount<14:
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
        
        # Color filtering
        if shapeIdentified:
            if c==0:
                colorID="Blue"    
            elif c==1:
                colorID="Green"
            else:
                colorID="Red"

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,shapeID,(10,200), font, 2,(255,0,0),2,cv2.LINE_AA)
    cv2.putText(frame,colorID,(10,300), font, 2,(255,0,0),2,cv2.LINE_AA)

    cv2.imshow('frame',frame)


    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()