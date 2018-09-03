import cv2
import numpy as np
import os


cap = cv2.VideoCapture(os.path.join(os.path.dirname(__file__), 'vtest.avi'))
p=0;# previous frame
preAbsSum=10000;
t=0; # frame number
while(cap.isOpened):
    t+=1;
    ret, frame = cap.read()
    # hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Isolate and subtract the colour channel
    
    h=hsv[:,:,0]/180
    cv2.imshow('frame',frame);
    # cv2.imshow('hue',h);
    cv2.imshow('delta',np.abs(p-h));
    if np.sum(np.abs(p-h))>preAbsSum*10:
        print(np.sum(np.abs(p-h)),preAbsSum,t)
        # get the hue channel again without dividing
        fullh=(hsv[:,:,0]).astype(int)
        # shift the colours by 180/12 so the red bins end up together
        fullh+=int(180/12);
        # loop the values back to within  0 - 180
        fullh%=180;
        # Only consider areas where the delta is greater than some threshold.
        threshold=np.mean(np.abs(p-h));
        renormalisationFactor=0.05;
        deltaMask=((np.abs(p-h)-np.clip(np.abs(p-h),threshold,None))/renormalisationFactor+renormalisationFactor)/renormalisationFactor # returns negative values if delta is smaller than the threshold; 1 otherwise
        # cv2.imshow('dm',deltaMask);
        fullh=deltaMask*fullh
        print(fullh.max())
        # run a histogram of the colours to determine the dominant colour.
        n,d=np.histogram(fullh,bins=6,range=(0,fullh.max()))
        maxColNum=np.argmax(n)
        maxColCenter=(d[maxColNum]+d[maxColNum+1])/2;
        if (maxColCenter<180/3): print('red');
        elif (maxColCenter<180/2): print('green');
        else: print('blue');
        print(maxColCenter)
    preAbsSum=np.sum(np.abs(p-h))
    p=hsv[:,:,0]/180
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()