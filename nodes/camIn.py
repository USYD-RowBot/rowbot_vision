#!/usr/bin/env python

import cv2
import numpy as np
import opencv_subscriber as test
cap=cv2.VideoCapture(0);

while (1):
    ret, frame=cap.read();
    cv2.imshow('test',test.getFromFrame(frame))
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()