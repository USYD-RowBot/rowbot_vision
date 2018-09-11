#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
fimgs={}
masks={}
while(1):
    ret, fimgs['bgr'] = cap.read()
    
    fimgs['hsv'] = cv2.cvtColor(fimgs['bgr'], cv2.COLOR_BGR2HSV)

    # s_hsv: shifted HSV.
    # convert to integer so we can use modulo
    fullh = (fimgs['hsv'][:, :, 0]).astype(int)
    # shift the colours by 180/12 so the red bins end up together
    fullh += int(180/12)
    # loop the values back to within  0 - 180
    fullh %= 180
    # reinsert into hsv -- for future refinement
    fimgs['s_hsv'] = np.copy(fimgs['hsv'])
    fimgs['s_hsv'][:, :, 0] = fullh*1.0

    # brite: relative saturation and value thresholding, to get rid of grey objects and darker objects.
    masks['brite'] = {'img': cv2.inRange(fimgs['s_hsv'], (0, np.max(
        fimgs['s_hsv'][:, :, 1])*0.3, np.max(fimgs['s_hsv'][:, :, 2])*0.2), (180, 255, 255))}

    # white and black masks
    # white: high value, low saturation
    masks['white'] = {'img': cv2.inRange(fimgs['s_hsv'], (0, 0, np.max(
        fimgs['s_hsv'][:, :, 2])*0.9), (180, np.max(fimgs['s_hsv'][:, :, 1])*0.2, 255))}
    # black: low values
    masks['black'] = {'img': cv2.inRange(
        fimgs['s_hsv'], (0, 0, 0), (180, 255, np.max(fimgs['s_hsv'][:, :, 2])*0.2))}

    # saturation + HSV Red, blue and green.
    # fimgs['brite_hsv'] = cv2.bitwise_and(
    #     fimgs['s_hsv'], fimgs['s_hsv'], mask=masks['brite']['img'])
    # masks['sat_red'] = {'img': cv2.inRange(
    #     fimgs['brite_hsv'],  np.array([0, 1, 1]), np.array([30, 255, 255]))}
    # masks['sat_green'] = {'img': cv2.inRange(
    #     fimgs['brite_hsv'],  np.array([50, 1, 1]), np.array([95, 255, 255]))}
    # masks['sat_blue'] = {'img': cv2.inRange(
    #     fimgs['brite_hsv'],  np.array([106, 1, 1]), np.array([140, 255, 255]))}

    # sp_green: specifically for detecting green things by making a mask which is not green.
    # use the same theory to detect things which are definitely red and blue?
    #masks['sp_blue'] = {'img': 255-np.clip(fimgs['bgr'][:, :, 0].astype(int)-(
    #    (fimgs['bgr'][:, :, 1].astype(int)+fimgs['bgr'][:, :, 2].astype(int))/2), 0, 255)}
    midblue=(fimgs['bgr'][:, :, 0].astype(int)-(
        (fimgs['bgr'][:, :, 1].astype(int)+fimgs['bgr'][:, :, 2].astype(int))/3)).astype(np.uint8)
    masks['wtf_blue']={'img':cv2.inRange(midblue,np.max(midblue)*0.5,255)}
    #print (fimgs['bgr'].dtype)
    #masks['sp_green'] = {'img': 255-np.clip(fimgs['bgr'][:, :, 1]-(
    #    (fimgs['bgr'][:, :, 0]+fimgs['bgr'][:, :, 2])), 0, 255)}
    #masks['sp_red'] = {'img': 255-np.clip(fimgs['bgr'][:, :, 2]-(
    #    (fimgs['bgr'][:, :, 1]+fimgs['bgr'][:, :, 0])), 0, 255)}
    for m in masks:
        cv2.imshow(m,masks[m]['img'])
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
