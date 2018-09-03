#!/usr/bin/env python
import cv2
import numpy as np

def identify(cnts):
    IDs=[]
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        shape = "weird"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.03 * peri, True)
        cA=cv2.contourArea(approx)
        if cA>1 and (cv2.contourArea(c)/cA)<0.95: shape="weird";
        # Classify ellipses
        if len(approx)>5 and cv2.isContourConvex(approx):
            ellipse = cv2.fitEllipse(approx)
            ellipseArea=ellipse[1][0]*ellipse[1][1]*np.pi/4
            if (ellipseArea):
                if np.abs((cA-ellipseArea)/ellipseArea) < 1:
                    shape = 'circle'
        # Classify based on vertex count
        elif len(approx) == 3:
            shape = "triangle"

        elif len(approx) == 12:
            shape = "cross"
        else:
            shape = "weird"
        x,y,w,h = cv2.boundingRect(c)
        rat = float(w)/float(h)
        cA=cv2.contourArea(c)
        # check the shape
        if shape != "weird" and cA > 80 and rat > 0.7 and rat < 1.5 :
            IDs.append({'area':cA,'cnt':c,'shape':shape})
    #####POST PROCESSSING: confidence based sorting
    IDs=sorted(IDs,key= lambda i: i['area'],reverse=True)
    return IDs

def collate(results):
    allIDs=[]
    for color in results: # confidence = area
        for id in results[color]:
            # append every item with its color and shape, and area as confidence.
            allIDs.append({'name':color+id['shape'],'conf':id['area']})
    # sort items by confidence
    if len(allIDs):
        allIDs=sorted(allIDs,key= lambda i: i['conf'],reverse=True)
        # return highest confidence shape
        return allIDs[0]['name']
    else:
        return "nothing"
