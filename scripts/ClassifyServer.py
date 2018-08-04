import cv2
import numpy as np
from matplotlib import pyplot as plt
def nothing(x):
    pass


class ClassifyServer():
    def __init__(self):
        #Initalise the image server
        
        pass
    def classify_buoy(self,bearing):
        fov = 70
        margin_deg = 5 #degrees
        print("classifying as red buoy")
        #print(min_water, max_water)
        #height,width,depth = image.shape
        #margin = margin_deg  * width/ 70
        #x_coord = width/70 * bearing + width/2
        #if bearing > fov/2 or bearing  < -fov/2:
        #    print("Out of range")
        #    return False
        #print(x_coord)
        return ["red_buoy"],[1]
