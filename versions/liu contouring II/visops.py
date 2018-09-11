import cv2
import numpy as np
def getColor(c,colorChannel):
    mask=np.zeros_like(colorChannel)
    cv2.drawContours(mask,[c],0,255,-1)
    maskchannel=colorChannel*mask
    return np.sum(maskchannel)/cv2.contourArea(c)


def runOperation(maskCollection, masks, identifier, colorChannel):
    toit=maskCollection
    if not masks is None:
        toit = masks
    contours=[]
    # Map filter name[s] to output colour name.
    for m in toit:
        contours= contours+maskCollection[m]['cnts']
    results=[]
    # run the identifier on all contours
    for j in contours:
        results=results+identifier(contours[j], colorChannel)
    # get the colour of each contour.
    for c,r in enumerate(results):
        results[c]['color']=getColor(results[c]['c'],colorChannel)
    return results