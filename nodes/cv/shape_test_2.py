#!/usr/bin/env python
import cv2
import numpy as np

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
                    
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.03 * peri, True)
        cA=cv2.contourArea(approx)
        if cA>1 and (cv2.contourArea(c)/cA)<0.95: return "weird";
        
        
        if len(approx)>5 and cv2.isContourConvex(approx):
            ellipse = cv2.fitEllipse(approx)
            ellipseArea=ellipse[1][0]*ellipse[1][1]*np.pi/4      
            if (ellipseArea):
                if np.abs((cA-ellipseArea)/ellipseArea) < 0.005:
                    shape = 'circle';
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"

        elif len(approx) == 12:
            shape = "cross"

        # if the shape has 4 vertices, it is either a square or
        # a rectangle

        # if the shape is a pentagon, it will have 5 vertices
        # otherwise, we assume the shape is a circle
        else:
            shape = "weird"
        # return the name of the shape
        return shape


def getAverageColor(c,hsv):
    mask = np.zeros(hsv.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [c], -1, 255, -1)
    
    
    mean = cv2.mean(hsv, mask=mask)[0] # just the hue value
    return mean

def inRange(colour,lower,upper):
    if colour >= lower and colour <= upper:
        return True
    else:
        return False

        
        
        
def getContoursFromMask(mask):
    # kernel = np.ones((5,5),np.uint8)
    # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closing=mask
    cnts = cv2.findContours(closing, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1]
    return cnts;
        
        
def get_shape(image):
    
    height, width = image.shape[:2]
    resized = cv2.resize(image,(width, height))
    ratio = image.shape[0] / float(resized.shape[0])
    
    
    ###### INITIAL PROCCESSING
    hsv=cv2.cvtColor(resized,cv2.COLOR_BGR2HSV)
    fullh=(hsv[:,:,0]).astype(int)
    # shift the colours by 180/12 so the red bins end up together
    fullh+=int(180/12);
    # loop the values back to within  0 - 180
    fullh%=180;
    # reinsert into hsv -- for future refinement
    hsv[:,:,0]=fullh*1.0;
    allIDs=[]
    cnts=[]
    
    
    # ###### Saturation thresholding
    # colourful=(hsv[:,:,1])
    # mask = cv2.inRange(colourful, np.max(colourful)*0.3, 255)
    # cnts+=getContoursFromMask(mask)
    # cv2.imshow('sat',mask);
    
    
    ##### Not black or white thresholding
    mask = cv2.inRange(hsv, (0,np.max(hsv[:,:,1])*0.3,np.max(hsv[:,:,2])*0.2), (180,255,255))
    cnts+=getContoursFromMask(mask)
    cv2.imshow('nbw',mask);

    ##### NOTGREEN THRESHOLDING
    # From testing, it seems that red and blue work fine; green is the key issue. So do contour detection on the notGreen channel.
    floatResized=resized.astype(float);
    notGreen=np.clip(floatResized[:,:,1]-((floatResized[:,:,0]+floatResized[:,:,2])/2),0,255)
    #cv2.imshow('ngr2',notGreen);
    #notGreen=notGreen.astype(np.uint8);
    #print (np.max(notGreen))
    mask = cv2.inRange(notGreen, np.max(notGreen)*0.3, 255)
    cv2.imshow('notgr',mask);
    cnts+=getContoursFromMask(mask)
    
    sd = ShapeDetector()
    IDs=[]
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        shape = sd.detect(c)
        x,y,w,h = cv2.boundingRect(c)
        rat = float(w)/float(h)
        cA=cv2.contourArea(c)
        if shape != "weird" and cA > 50 and rat > 0.7 and rat < 1.5 :
                colour = getAverageColor(c,hsv)
                color_label = "u"
                if inRange(colour,106,126):
                    color_label = "blue "
                if inRange(colour,0,20):
                    color_label = "red "
                if inRange(colour,50,95):
                    color_label = "green "
                IDs.append({'area':cA,'cnt':c,'color':color_label,'shape':shape})
    #####POST PROCESSSING
    IDs=sorted(IDs,key= lambda i: i['area'],reverse=True)
    
    ##### PRINTING
    amt_to_print=1;
    result="nothing"
    for id in IDs: # print out top 5 only
        if id['color'] == "u":
            pass
        else:
            # if cv2.inRange()
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image\
            c=id['cnt'];
            M = cv2.moments(c)
            cX = int((M["m10"] / (M["m00"]+0.1)) * ratio)
            cY = int((M["m01"] / (M["m00"]+0.1)) * ratio)
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 0, 0), 2)
            cv2.putText(image, id['color'] + id['shape'], (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 0), 2)
            result=id['color'] + id['shape']
            amt_to_print-=1;
            if amt_to_print==0: break;
    return (image,result);
        
        


    #blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    
