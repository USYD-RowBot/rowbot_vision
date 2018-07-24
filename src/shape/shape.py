import cv2
import numpy as np
import os


def drawlbl(fr,x,y,c,s,co):
    font = cv2.FONT_HERSHEY_SIMPLEX
    x=int(x)
    y=int(y)
    cv2.putText(fr,s,(x,y), font, 0.5,(255,0,0),2,cv2.LINE_AA)
    cv2.putText(fr,c,(x,y+20), font, 0.5,(255,0,0),2,cv2.LINE_AA)
    cv2.putText(fr,"confidence:"+str(co),(x,y+40), font, 0.5,(255,0,0),2,cv2.LINE_AA)

def getConfidence(contourArea,shapeArea,threshold):
    if shapeArea==0: return 0;
    return (threshold-np.abs((contourArea-shapeArea)/shapeArea))/threshold

def getShapesFromMask(mask):
    image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return getShapes(contours);

## Picks out any shapes, given a set of contours.
def getShapes(cnts):
    results=[];
    # Each element: (x,y,cnt,type,area,confidence,color)
    # confidence is out of 1. 1 is very confident.
    for i in cnts:
        cA=cv2.contourArea(i)
        #ignore really small things
        if not (i is None):
            # circle detection
            if len(i)>5:
                ellipse = cv2.fitEllipse(i)
                ellipseArea=ellipse[1][0]*ellipse[1][1]*np.pi/4
                
                if (ellipseArea):
                    if np.abs((cA-ellipseArea)/ellipseArea) < 0.05:
                        # cv2.ellipse(frame,ellipse,(255,255,255),3)
                        results.append([ellipse[0][0],ellipse[0][1],i,'Circle',cA,getConfidence(cA,ellipseArea,0.05),'none']);
                        continue;
            # Detection for other shapes
            triangleConfidence=0;
            crucifixConfidence=0; # not jesus christ
            # Get simplified shape
            lines=cv2.approxPolyDP(i,(cA**0.7)*0.02,1)
            lineArea=cv2.contourArea(lines) 
            arcLen=cv2.arcLength(lines,True)
            if lines is None or len(lines)<3: continue; # skip lines and dots
            
            ### VERTEX COUNT METHOD
            triangleConfidence+=0.5*(10**-(abs(len(lines)-3))); #triangle means triangle
            crucifixConfidence+=0.5*(3**-(abs(len(lines)-12)));

            
            ### ANGLE SIZE METHOD -- crucifix only
            vertices=[np.array((i[0][0],i[0][1])) for i in lines]
            vertices.append(vertices[0]) # make the vertices loop
            vertices.append(vertices[1]) # make the vertices loop
            tv=0;
            tc=0;
            for v,i in enumerate(vertices):
                if v>len(vertices)-3: break
                a=threePointAngle(vertices[v],vertices[v+1],vertices[v+2])
                # triangleConfidence+=0.3*(1.01**-(abs(a-60)))*(np.linalg.norm(vertices[v]-vertices[v+1])+np.linalg.norm(vertices[v+2]-vertices[v+1]))/(arcLen*2); # A lot of effort to make sure confidences sum up to 1.
                crucifixConfidence+=0.3*(10**-(abs(a-90)))/(len(vertices)-2);
                # tv+=0.3*(1.01**-(abs(a-90)))*(np.linalg.norm(vertices[v]-vertices[v+1])+np.linalg.norm(vertices[v+2]-vertices[v+1]))/(arcLen*2);
                # tc+=0.3*(1.01**-(abs(a-60)))*(np.linalg.norm(vertices[v]-vertices[v+1])+np.linalg.norm(vertices[v+2]-vertices[v+1]))/(arcLen*2);
                # if tc>0.3: print ("over 0.3 aaaa");
                
            #### RELATIVE AREA METHOD
            rect = cv2.minAreaRect(lines)
            relArea=cA/(rect[1][0]*rect[1][1]);
            triangleConfidence+=0.5*(3**-(abs(relArea-0.5)));
            crucifixConfidence+=0.2*(5**-(abs(relArea-(5.0/9.0))));
            
            if (max(triangleConfidence,crucifixConfidence)>0.5):
                if (triangleConfidence>crucifixConfidence):
                    results.append([lines[0][0][0],lines[0][0][1],lines,'Triangle',lineArea,triangleConfidence,'none']);
                else:
                    results.append([lines[0][0][0],lines[0][0][1],lines,'Crucifix',lineArea,crucifixConfidence,'none']);
                    #print (relArea)
                    
    return results;

def threePointAngle(a,b,c):
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)
    
    
def identify(frame):
    ###### INITIAL PROCCESSING
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    fullh=(hsv[:,:,0]).astype(int)
    # shift the colours by 180/12 so the red bins end up together
    fullh+=int(180/12);
    # loop the values back to within  0 - 180
    fullh%=180;
    # reinsert into hsv -- for future refinement
    hsv[:,:,0]=fullh*1.0;
    allIDs=[]
    
    ###### Saturation thresholding
    colourful=(hsv[:,:,1])
    mask = cv2.inRange(colourful, 100, 255)
    #cv2.imshow('sat',mask);
    # cv2.imshow('m',mask);
    allIDs+=getShapesFromMask(mask);
    
    ###### Greyscale THRESHOLDING
    # gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # mask = cv2.inRange(gray, 0, 100)
    # allIDs+=getShapesFromMask(mask);
    
    
    ##### NOTGREEN THRESHOLDING
    # From testing, it seems that red and blue work fine; green is the key issue. So do contour detection on the notGreen channel.
    notGreen=frame[:,:,1]-(frame[:,:,0]+frame[:,:,2])/2
    mask = cv2.inRange(notGreen, 0, 100)
    #cv2.imshow('notgr',mask);
    allIDs+=getShapesFromMask(mask);
    
    # ###### COLOUR 
    # satTop=255;
    # satBottom=100;
    # valTop=255;
    # valBottom=107;
    # colors=[
        # (np.array([0,satBottom,valBottom]),np.array([20,satTop,valTop])), # red
        # (np.array([50,satBottom,valBottom]),np.array([95,satTop,valTop])),  # green
        # (np.array([106,satBottom,valBottom]),np.array([125,satTop,valTop])),  # blue
    # ]
    # overmask=0;
    
    # for c,i in enumerate(colors):
        # # define range of color in HSV
        # lowerRange = i[0]
        # upperRange = i[1]
        # shapeIdentified=False;
        # # Threshold the HSV image to get only desired colors
        # mask = cv2.inRange(hsv, lowerRange, upperRange)      
        # # check if there is a circle
        # allIDs+=getShapesFromMask(mask);
    
    
    
    
    
    
    
    
    ##### FILTERING: only top 20 otherwise the fps drops to 0.1
    allIDs=sorted(allIDs,key=lambda i: (i[5]**0.01)*(i[4]**0.5), reverse=True)
    
    numProcessed=0;
    ci=0;
    satWeight=0.9;
    valWeight=0.9;
    #print("#####");
    while numProcessed<min(len(allIDs),20) and ci<len(allIDs):
        i=allIDs[ci];
        mask = np.zeros(frame.shape[0:2]);
        cv2.fillPoly(mask,[i[2]],255);
        if i[4]<0.0001: break; # all the contours are bad lol dont even bother
        sumMask=np.sum(mask)
        avgHue=np.sum(hsv[:,:,0]*mask)/sumMask;
        avgSat=np.sum(hsv[:,:,1]*mask)/sumMask;
        avgVal=np.sum(hsv[:,:,2]*mask)/sumMask;
        i[5]*=(avgSat/255)*satWeight+(1-satWeight);
        i[5]*=(avgVal/255)*valWeight+(1-valWeight);
        # Determine the colour.
        if 0 <= avgHue <= 20:
            i[6]='red';
        elif 50 <= avgHue <= 95:
            i[6]='green';
            #Since we seem to hate green things so much lets give us a confidence boost. 
            # lmao this doesnt work
            # i[5]=i[5]**0.2;
        elif 106 <= avgHue <= 126:
            i[6]='blue';
        else:
            i[5]=0; # no nones
        ci+=1;
        numProcessed+=1;
        # #print(avgHue);
        #if numProcessed==1: cv2.imshow('m',mask);
    allIDs=sorted(allIDs,key=lambda i: (i[5]**0.01)*(i[4]**0.5), reverse=True)
    for i in allIDs:
            cv2.polylines(frame,[i[2]],1,(0,0,255),2);
            drawlbl(frame,i[0],i[1],i[6],i[3],i[5]);
            print(i[4],i[5])
            break; # just draw the first one
    cv2.imshow('frame',frame)
    return frame
        
def wrapper():
    
    # For each color...
    cap=cv2.VideoCapture(0)
    while (1):
        r, frame = cap.read()
        #frame=cv2.imread(os.path.join(os.path.dirname(__file__),"crossifix.jpg"))
        IDlist=identify(frame);
        for i in IDlist:
            cv2.polylines(frame,[i[2]],1,(0,0,255),2);
            drawlbl(frame,i[0],i[1],i[6],i[3],i[5]);
            print(i[4],i[5])
            break; # just draw the first one
        #cv2.imshow('frame',frame)
        # if cv2.waitKey(0): break;
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
    

