import cv2
import numpy as np
import time

def identify(cnts):
    # we only care about vertices
    results=[]
    for cnt in cnts:
        peri = cv2.arcLength(cnt, True)
        vertices = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        if len(vertices) == 4:
            # Using the boundingRect method to calculate the width and height
            # of the quadrilateral and use this to calculate the aspect ratio
            x, y, width, height = cv2.boundingRect(vertices)
            aspectRatio = float(width) / height

            # If the aspectRatio is approximately equal, the shape is a square
            # Otherwise, the shape is a rectangle
            if aspectRatio > 1:
                pass
            #if aspectRatio >= 0.95 and aspectRatio <= 1.05:
            #shape = "rectangle"
            else:
                results.append({'area':cv2.contourArea(vertices)})
    return results

def collate(results):
    # update the light pattern with the dominant rectangle.
    allIDs=[]
    for color in results: # confidence = area
        for id in results[color]:
            # append every item with its color and shape, and area as confidence.
            allIDs.append({'color':color,'conf':id['area']})
    # sort items by confidence
    result=""
    if len(allIDs):
        allIDs=sorted(allIDs,key= lambda i: i['conf'],reverse=True)
        # set result to highest confidence color
        result=allIDs[0]['color']
    return lp.update(result)

class LightPattern():
    def __init__(self):
        self.sequence = []
        self.position = 0
        self.timeout=10
        self.timeout_counter = 0
        self.last_sequence = []

    def update(self,color):
        if color == "":
            self.timeout_counter = self.timeout_counter  + 1
            print(self.timeout_counter)
            if self.timeout_counter > self.timeout:
                self.position = 0
                self.last_sequence = self.sequence
                self.sequence = []
                print("Detected sequence:" + self.last_sequence)
        else:
            self.timeout_counter =0
            if len(self.sequence) and self.sequence[len(self.sequence)-1] == color:
                pass
            else:
                self.sequence.append(color)
        print("Current sequence:" + " ".join(self.sequence))
        return self.last_sequence

lp=LightPattern()