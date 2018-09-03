#!/usr/bin/env python

import cv2
import numpy as np
import cv.shape_test_2 as test
import os
import time

speechOK=False;
try:
    import win32com.client as wincl
    import threading;
    from threading import Thread
    def sayStuff(thingToSay):
        speak.Speak(thingToSay)
    speak = wincl.Dispatch("SAPI.SpVoice")
    speechOK=False;
except ImportError:
    print("Speech off ;-;")

testType="shapeDetect";
#testType="lightBuoy";
#testType="beacon";

cap=cv2.VideoCapture(1);
p=None;


    

while (1):
    ret, frame=cap.read();
    (im,res)=test.get_shape(frame)
    cv2.imshow('test',im)
    if speechOK and res!="nothing" and ((p is None) or not p.is_alive()):
        p=Thread(target = sayStuff,args=[res])
        p.start()
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    if k == 32: # space key
        path=os.path.join(os.pardir,'images')
        path=os.path.join(path,testType)
        path=os.path.join(os.path.dirname(__file__),path);
        fileName=time.asctime(time.localtime()).replace(":","-");
        fileName+="@"
        fileName+=testType
        fileName+=res;
        fileName+='.png';
        path=os.path.join(path,fileName);
        cv2.imwrite(path,frame)
        print("Wrote file to "+path)
        # output the file and diagonsis to another directory.
cv2.destroyAllWindows()