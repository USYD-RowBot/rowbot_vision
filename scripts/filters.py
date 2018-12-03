import cv2
import numpy as np
import os
fimgs={}
masks={}
my_path = os.path.abspath(os.path.dirname(__file__))

def getContoursFromMask(mask):
    kernel = np.ones((5,5),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # closing=mask
    img, cnts,hierarchy = cv2.findContours(closing, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if not hierarchy is None : return (cnts,hierarchy[0])
    else: return (cnts,None)

cacheSum=0
def preprocess(img):
    global cacheSum
    if np.sum(img)==cacheSum:
        #print('filtering caught optimised')
        # we have already processed this image- do nothing
        return
    else:
        cacheSum=np.sum(img)
        fimgs['bgr']=img
        # hsv: normal HSV.
        fimgs['hsv']=cv2.cvtColor(fimgs['bgr'],cv2.COLOR_BGR2HSV)
        
        # s_hsv: shifted HSV.
        # convert to integer so we can use modulo
        fullh=(fimgs['hsv'][:,:,0]).astype(int)
        # shift the colours by 180/12 so the red bins end up together
        fullh+=int(180/12)
        # loop the values back to within  0 - 180
        fullh%=180
        # reinsert into hsv -- for future refinement
        fimgs['s_hsv']=np.copy(fimgs['hsv'])
        fimgs['s_hsv'][:,:,0]=fullh*1.0

        # brite: relative saturation and value thresholding, to get rid of grey objects and darker objects.
        masks['brite']={'img':cv2.inRange(fimgs['s_hsv'], (0,np.max(fimgs['s_hsv'][:,:,1])*0.3,np.max(fimgs['s_hsv'][:,:,2])*0.2), (180,255,255))}
        
        # white and black masks
        # white: high value, low saturation
        masks['white']={'img':cv2.inRange(fimgs['s_hsv'], (0,0,np.max(fimgs['s_hsv'][:,:,2])*0.9), (180,np.max(fimgs['s_hsv'][:,:,1])*0.2,255))}
        # black: low values
        masks['black']={'img':cv2.inRange(fimgs['s_hsv'], (0,0,0),(180,255,np.max(fimgs['s_hsv'][:,:,2])*0.1))}

        # saturation + HSV Red, blue and green.
        fimgs['brite_hsv'] = cv2.bitwise_and(fimgs['s_hsv'],fimgs['s_hsv'],mask = masks['brite']['img'])
        masks['sat_red'] = {'img':cv2.inRange(fimgs['brite_hsv'],  np.array([0, 1, 1]),np.array([30, 255, 255]))}
        masks['sat_green'] = {'img':cv2.inRange(fimgs['brite_hsv'],  np.array([50, 1, 1]),np.array([95, 255, 255]))}
        masks['sat_blue'] = {'img':cv2.inRange(fimgs['brite_hsv'],  np.array([106, 1, 1]),np.array([140, 255, 255]))}
        
        # sp_green: specifically for detecting green things by making a mask which is not green.
        # use the same theory to detect things which are definitely red and blue?
        sp_colors=['sp_blue','sp_green','sp_red']
        for i,s in enumerate(sp_colors):
            mid=np.clip(fimgs['bgr'][:, :, i].astype(int)-((fimgs['bgr'][:, :, (i+1)%3].astype(int)+fimgs['bgr'][:, :, (i+2)%3].astype(int))/2),0,255).astype(np.uint8)
            # print(np.max(mid))
            masks[sp_colors[i]]={"img":cv2.inRange(mid,np.max(mid)*0.5+1,255)}
        # contour finding
        for mask in masks:
            masks[mask]['cnts']=getContoursFromMask(masks[mask]['img'])

def getContours(img,masknames,includeHierarchy=False):
    preprocess(img)
    cnts={}
    for m in masknames:
        try:
            if includeHierarchy==False:
                cnts[m]=masks[m]['cnts'][0]
            else:
                cnts[m]=masks[m]['cnts']
        except KeyError:
            print(m+" not recognised to be a mask.")
    return cnts

def getContourColor(img,cnt):
    mask=np.zeros_like(img[:,:,0])
    preprocess(img)#,['s_hsv'])# implement second argument to save time for later
    mask=cv2.drawContours(mask,[cnt],0,1,-1)
    img2=fimgs['s_hsv'][:,:,0]*mask.astype(fimgs['s_hsv'].dtype)
    return np.sum(img2)/cv2.contourArea(cnt)


debugMode="fromFile"
todraw=['white']
if __name__=="__main__":
    if debugMode=="fromFile":
        for f in os.listdir(os.path.join(my_path,"cntdict_tst")):
            
            testimg=cv2.imread(os.path.join(my_path,"cntdict_tst",f))
            preprocess(testimg)
            if len(todraw)>0:
                for m in todraw:
                    disp=cv2.bitwise_and(testimg,testimg, mask=masks[m]['img'])
                    cv2.imshow (m,disp)
            else:
                for m in masks:
                    disp=cv2.bitwise_and(testimg,testimg, mask=masks[m]['img'])
                    cv2.imshow (m,disp)
            while (1):
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    break