import cv2
import numpy as np
import os

"""List of filters and masks
filter images:
bgr:        blue green red
hsv:        hue sat value
s_hsv:      shifted hsv
brite_hsv:  only bright shifted hsv

masks:
brite:      non grey i.e. colourful
white:      white
black:      black
sat_red:    hue-based red
sat_green:  hue-based green
sat_blue:   hue-based blue
sp_red:     relative-rgb based red
sp_green:   relative-rgb based green
sp_blue:    relative-rgb based blue

"""




my_path = os.path.abspath(os.path.dirname(__file__))
class FilterCacher:
    def __init__(self):
        self.fimgs = {}
        self.masks = {}
        self.cacheSum = 0
        self.capacitive=None

    def getContoursFromMask(self, mask):
        kernel = np.ones((10, 10), np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # closing=mask
        img, cnts, hierarchy = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not hierarchy is None:
            return (cnts, hierarchy[0])
        else:
            return (cnts, None)

    def preprocess(self, img):
        if np.sum(img) == self.cacheSum:
            #print('filtering caught optimised')
            # we have already processed this image- do nothing
            return
        else:
            #self.clahe = cv2.createCLAHE(clipLimit=2.0,tileGridSize=(20,20))
            self.cacheSum = np.sum(img)
            self.fimgs['bgr'] = img
            # hsv: normal HSV.

            # self.fimgs['lab'] = cv2.cvtColor(self.fimgs['bgr'], cv2.COLOR_BGR2LAB)

            # lab_planes = cv2.split(self.fimgs['lab'])

            # lab_planes[0] = self.clahe.apply(lab_planes[0])

            # self.fimgs['clahe_lab'] = cv2.merge(lab_planes)

            # self.fimgs['clahe_bgr'] = cv2.cvtColor(self.fimgs['clahe_lab'], cv2.COLOR_LAB2BGR)
            self.fimgs['hsv']=cv2.cvtColor(self.fimgs['bgr'], cv2.COLOR_BGR2HSV)
            # s_hsv: shifted HSV.
            # convert to integer so we can use modulo
            fullh = (self.fimgs['hsv'][:, :, 0]).astype(int)
            # shift the colours by 180/12 so the red bins end up together
            fullh += int(180/12)
            # loop the values back to within  0 - 180
            fullh %= 180
            # reinsert into hsv -- for future refinement
            self.fimgs['s_hsv'] = np.copy(self.fimgs['hsv'])
            self.fimgs['s_hsv'][:, :, 0] = fullh*1.0

            # brite: relative saturation and value thresholding, to get rid of grey objects and darker objects.
            self.masks['brite'] = {'img': cv2.inRange(self.fimgs['s_hsv'], (0, np.max(
                self.fimgs['s_hsv'][:, :, 1])*0.3, np.max(self.fimgs['s_hsv'][:, :, 2])*0.2), (180, 255, 255))}
            # brite: relative saturation and value thresholding, to get rid of grey objects and darker objects.
            self.masks['notcolor'] = {'img': cv2.inRange(self.fimgs['s_hsv'], (0, 0,0), (180, np.max(self.fimgs['s_hsv'][:,:,1])*0.15, 255))}
            # white and black self.masks
            # white: high value, low saturation
            self.masks['white'] = {'img': cv2.inRange(self.fimgs['s_hsv'], (0, 0, np.max(
                self.fimgs['s_hsv'][:, :, 2])*0.7), (180, np.max(self.fimgs['s_hsv'][:, :, 1])*0.1, 255))}
            # black: low values
            self.masks['black'] = {'img': cv2.inRange(
                self.fimgs['s_hsv'], (0, 0, 0), (180, 255, np.max(self.fimgs['s_hsv'][:, :, 2])*0.1))}

            # saturation + HSV Red, blue and green.
            self.fimgs['brite_hsv'] = cv2.bitwise_and(
                self.fimgs['s_hsv'], self.fimgs['s_hsv'], mask=self.masks['brite']['img'])
            self.masks['sat_red'] = {'img': cv2.inRange(
                self.fimgs['s_hsv'],  np.array([0, 76, 1]), np.array([30, 255, 255]))}
            self.masks['sat_green'] = {'img': cv2.inRange(
                self.fimgs['s_hsv'],  np.array([45, 76, 1]), np.array([95, 255, 255]))}
            self.masks['sat_blue'] = {'img': cv2.inRange(
                self.fimgs['s_hsv'],  np.array([106, 1, 1]), np.array([140, 255, 255]))}

            # sp_green: specifically for detecting green things by making a mask which is not green.
            # use the same theory to detect things which are definitely red and blue?
            sp_colors = ['sp_blue', 'sp_green', 'sp_red']
            for i, s in enumerate(sp_colors):
                mid = np.clip(self.fimgs['bgr'][:, :, i].astype(int)-((self.fimgs['bgr'][:, :, (i+1) % 3].astype(
                    int)+self.fimgs['bgr'][:, :, (i+2) % 3].astype(int))*5/6), 0, 255).astype(np.uint8)
                # print(np.max(mid))
                self.masks[sp_colors[i]] = {
                    "img": cv2.inRange(mid, np.max(mid)*0.5+1, 255)}
            # contour finding
            for mask in self.masks:
                self.masks[mask]['cnts'] = self.getContoursFromMask(
                    self.masks[mask]['img'])

            # #### capacitive
            # if not self.capacitive is None:

            #     self.capacitive=(self.capacitive/2+img/2)
            # else:
            #     self.capacitive=img
            # cv2.imshow("cap",self.capacitive)
            # cv2.waitKey(100)


    def getContours(self, img, masknames, includeHierarchy=False):
        self.preprocess(img)
        cnts = {}
        for m in masknames:
            try:
                if includeHierarchy == False:
                    cnts[m] = self.masks[m]['cnts'][0]
                else:
                    cnts[m] = self.masks[m]['cnts']
            except KeyError:
                print(m+" not recognised to be a mask.")
        return cnts

    def getContourColor(self, img, cnt):
        mask = np.zeros_like(img[:, :, 0])
        # ,['s_hsv'])# implement second argument to save time for later
        self.preprocess(img)
        mask = cv2.drawContours(mask, [cnt], 0, 1, -1)
        img2 = self.fimgs['s_hsv'][:, :, 0] * \
            mask.astype(self.fimgs['s_hsv'].dtype)
        return np.sum(img2)/cv2.contourArea(cnt)


debugMode = "fromVideo"
todraw = ['notcolor']
todraw_fimgs = ['bgr']
if __name__ == "__main__":
    if debugMode == "fromFile":
        fc=FilterCacher()
        for f in os.listdir(os.path.join(my_path, "cntdict_tst")):
            testimg = cv2.imread(os.path.join(my_path, "cntdict_tst", f))
            fc.preprocess(testimg)
            if len(todraw) > 0:
                for m in todraw:
                    disp = cv2.bitwise_and(
                        testimg, testimg, mask=fc.masks[m]['img'])
                    cv2.imshow(m, disp)
            else:
                for m in fc.masks:
                    disp = cv2.bitwise_and(
                        testimg, testimg, mask=fc.masks[m]['img'])
                    cv2.imshow(m, disp)
            while (1):
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    break
    if debugMode == "fromVideo":
        fc=FilterCacher()
        vc=cv2.VideoCapture("/home/stevenliu/Videos/boaty_boat.mp4")
        while vc.isOpened():
            ret,testimg = vc.read()
            fc.preprocess(testimg)
            if len(todraw) > 0:
                for m in todraw:
                    disp = cv2.bitwise_and(
                        testimg, testimg, mask=fc.masks[m]['img'])
                    cv2.imshow(m, disp)
            else:
                for m in fc.masks:
                    disp = cv2.bitwise_and(
                        testimg, testimg, mask=fc.masks[m]['img'])
                    cv2.imshow(m, disp)
            if len(todraw_fimgs)>0:
                for f in todraw_fimgs:
                    cv2.imshow(f,fc.fimgs[f])
            while (1):
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    break