#!/usr/bin/env python
import cv2
import numpy as np
import colorsys

class ShapeDetector:
	def __init__(self):
		pass

	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.03 * peri, True)
		# if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"

		elif len(approx) == 12:
			shape = "cross"

		# if the shape has 4 vertices, it is either a square or
		# a rectangle

		# if the shape is a pentagon, it will have 5 vertices


		# otherwise, we assume the shape is a circle
		elif cv2.isContourConvex(approx):
			shape = "circle"
		else:
			shape = "weird"
		# return the name of the shape
		return shape
def getAverageColor(c,image):
	mask = np.zeros(image.shape[:2], dtype="uint8")
	cv2.drawContours(mask, [c], -1, 255, -1)
	mask = cv2.erode(mask, None, iterations=2)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mean = cv2.mean(hsv, mask=mask)[:3]
	#cv2.imshow("mask",mask)
	return mean
def inRange(colour,lower,upper):
	print(colour[0])
	if lower[0] > upper[0]:
		lower[0] = lower[0]-180
	if (colour[0] >= lower [0] and
	colour[1] >= lower [1] and
	colour[2] >= lower [2] and
	colour[0] <= upper [0] and
	colour[1] <= upper [1] and
	colour[2] <= upper [2]):
		return True
	else:
		return False
def get_shape(image):

	lower_blue = np.array([110,100,100])
	upper_blue = np.array([130,255,255])
	lower_green = np.array([40,50,50])
	upper_green = np.array([80,255,255])
	lower_red = np.array([160,100,100])
	upper_red = np.array([10,255,255])
	height, width = image.shape[:2]
	resized = cv2.resize(image,(width, height))
	ratio = image.shape[0] / float(resized.shape[0])

	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	upper = 255
	lower = 120
	thresh_inv = cv2.threshold(blurred, lower ,upper, cv2.THRESH_BINARY_INV)[1]
	thresh = cv2.threshold(blurred, lower, upper, cv2.THRESH_BINARY)[1]
	thresh_inv_uint8 = 255 * thresh
	#cv2.imshow('gray',gray)
	#cv2.imshow('thresh_inv',thresh_inv)
	#cv2.imshow('thresh',thresh)

	cnts = cv2.findContours(thresh_inv.copy(), cv2.RETR_LIST,
	cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[1]
	sd = ShapeDetector()

	for c in cnts:
		# compute the center of the contour, then detect the name of the
		# shape using only the contour
		M = cv2.moments(c)
		cX = int((M["m10"] / (M["m00"]+0.1)) * ratio)
		cY = int((M["m01"] / (M["m00"]+0.1)) * ratio)
		shape = sd.detect(c)
		x,y,w,h = cv2.boundingRect(c)
		rat = float(w)/float(h)
		if shape != "weird" and cv2.contourArea(c) > 50 and rat > 0.5 and rat < 2 :
				colour = getAverageColor(c,image)
				color_label = "u"
				if inRange(colour,lower_blue,upper_blue):
					color_label = "blue "
				if inRange(colour,lower_red,upper_red):
					color_label = "red "
				if inRange(colour,lower_green,upper_green):
					color_label = "green "
				if color_label == "u":
					pass
				else:

					# if cv2.inRange()
					# multiply the contour (x, y)-coordinates by the resize ratio,
					# then draw the contours and the name of the shape on the image\

					c = c.astype("float")
					c *= ratio
					c = c.astype("int")
					cv2.drawContours(image, [c], -1, (0, 0, 0), 2)
					cv2.putText(image, color_label + shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (0, 0, 0), 2)

		# show the output image
	#cv2.imshow("Image", image)
	#cv2.waitKey(3)
	return image
