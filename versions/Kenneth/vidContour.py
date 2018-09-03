import cv2
import numpy as np
import time

def detectShape(cnt):

    # Initially define the shape as a string 'unknown'
    shape = 'unknown'

    # Calculate the perimeter of the contour
    peri = cv2.arcLength(cnt, True)

    # Apply contour approximation and store the result in vertices
    vertices = cv2.approxPolyDP(cnt, 0.005 * peri, True)

    # If there are 3 vertices, the shape is a triangle
    if len(vertices) == 3:
        shape = 'triangle'

    # If there are 4 vertices, the shape is a quadrilateral
    elif len(vertices) == 4:

        # Using the boundingRect method to calculate the width and height
        # of the quadrilateral and use this to calculate the aspect ratio
        x, y, width, height = cv2.boundingRect(vertices)
        aspectRatio = float(width) / height

        # If the aspectRatio is approximately equal, the shape is a square
        # Otherwise, the shape is a rectangle
        if aspectRatio >= 0.95 and aspectRatio <= 1.05:
            shape = "square"
        else:
            shape = "rectangle"

    # If there are 5 vertices, the shape is a pentagon
    elif len(vertices) == 5:
        shape = "pentagon"

    # Otherwise, assume the shape is a circle
    else:
        shape = "circle"

    # Return the shape string
    return shape

class LightPattern():
    def __init__(self):
        self.sequence = ["_","_","_"]
        self.position = 0
        self.timeout_counter = 0

    def getPattern(self,frame):

        pattern = ""
        counter = 0
        tempPattern = ""
        samecolFlag = 0
        timeStarted = 0
        timeout = 15
        #Code
        # Blur frame to increase accuracy
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert the colours from BGR to HSV format
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        # Define the HSV colour boundaries
        lower_blue = np.array([76, 86, 0])
        upper_blue = np.array([135, 255, 255])

        # Apply a mask to the frame
        maskB = cv2.inRange(hsv, lower_blue, upper_blue)
        flframe=frame.astype(float)
        blue=flframe[:,:,0]-(flframe[:,:,2]+flframe[:,:,1])/2
        np.clip(blue,0,255)
        cv2.inRange(blue,np.max(blue)*0.5,255)
        cv2.imshow('bl',blue)
        # Obtain the contours of the masked image
        _, contoursB, _ = cv2.findContours(maskB, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Define the HSV colour boundaries
        lower_green = np.array([46, 86, 0])
        upper_green = np.array([75, 255, 255])

        # Apply a mask to the frame
        maskG = cv2.inRange(hsv, lower_green, upper_green)

        # Obtain the contours of the masked image
        _, contoursG, _ = cv2.findContours(maskG, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Define the HSV colour boundaries
        lower_red1 = np.array([0, 86, 0])
        upper_red1 = np.array([15, 255, 255])

        # Apply a mask to the frame
        maskR1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # Obtain the contours of the masked image
        _, contoursR1, _ = cv2.findContours(maskR1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


        # Define the HSV colour boundaries
        lower_red2 = np.array([166, 86, 0])
        upper_red2 = np.array([180, 255, 255])

        # Apply a mask to the frame
        maskR2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Obtain the contours of the masked image
        _, contoursR2, _ = cv2.findContours(maskR2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


        # Define the HSV colour boundaries
        lower_yellow = np.array([16, 86, 0])
        upper_yellow = np.array([45, 255, 255])

        # Apply a mask to the frame
        maskY = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Obtain the contours of the masked image
        _, contoursY, _ = cv2.findContours(maskY, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)



        # A function to detect the shape of a given contour

        colour = ""


        # Loop for all the contours
        for contour in contoursB:

            # Calculate the area of the contour
            area = cv2.contourArea(contour)
            # Call a function to find the shape of the contour
            shape = detectShape(contour)

            # If the contour is within the range, it may be our desired object
            if (area > 2000) & (shape == "rectangle"):

                # Calculate the moment of the contour
                M = cv2.moments(contour)

                # Using the moment, calculate the centroid of the contour
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])


                # Label the contour with the shape name
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                colour =  "blue"

                # Draw the contours onto the object
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)

        # Loop for all the contours
        for contour in contoursG:

            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Call a function to find the shape of the contour
            shape = detectShape(contour)

            # If the contour is within the range, it may be our desired object
            if (area > 2000) & (shape == "rectangle"):

                # Calculate the moment of the contour
                M = cv2.moments(contour)

                # Using the moment, calculate the centroid of the contour
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])

                # Label the contour with the shape name
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                # Draw the contours onto the object
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)


                colour =  "green"

        # Loop for all the contours
        for contour in contoursR1:

            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Call a function to find the shape of the contour
            shape = detectShape(contour)

            # If the contour is within the range, it may be our desired object
            if (area > 2000) & (shape == "rectangle"):

                # Calculate the moment of the contour
                M = cv2.moments(contour)

                # Using the moment, calculate the centroid of the contour
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])



                # Label the contour with the shape name
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                # Draw the contours onto the object
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)

                colour =  "red"

        # Loop for all the contours
        for contour in contoursR2:

            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Call a function to find the shape of the contour
            shape = detectShape(contour)

            # If the contour is within the range, it may be our desired object
            if (area > 2000) & (shape == "rectangle"):

                # Calculate the moment of the contour
                M = cv2.moments(contour)

                # Using the moment, calculate the centroid of the contour
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])


                # Label the contour with the shape name
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                # Draw the contours onto the object
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)


                colour =  "red"

        # Loop for all the contours
        for contour in contoursY:

            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Call a function to find the shape of the contour
            shape = detectShape(contour)

            # If the contour is within the range, it may be our desired object
            if (area > 2000) & (shape == "rectangle"):

                # Calculate the moment of the contour
                M = cv2.moments(contour)

                # Using the moment, calculate the centroid of the contour
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])

                # Label the contour with the shape name
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                # Draw the contours onto the object
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)


                colour =  "yellow"


        if colour == "":
            self.timeout_counter = self.timeout_counter  + 1
            print(self.timeout_counter)
            if self.timeout_counter > timeout:
                self.position = 0
                self.sequence = ["_","_","_"]
        else:
            self.timeout_counter =0
            if self.position == 0:
                self.sequence[0] = colour
                self.position = self.position +1
            else:
                if self.sequence[self.position-1] == colour:
                    pass
                else:
                    self.sequence[self.position] = colour
                    self.position = self.position +1
            if self.position > 2:
                self.position = 2
        print(self.sequence)

        cv2.imshow("Red1",maskR1)
        cv2.imshow("Rd2",maskR2)
        cv2.imshow("Green",maskG)
        cv2.imshow("Yellow",maskY)
        cv2.imshow("Blue",maskB)

        cv2.putText(frame, "Sequence: " + pattern, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return frame

if __name__ == "__main__":
    cap = cv2.VideoCapture(1)
    cap.set(3,640)
    cap.set(4,480)

    pattern = ""
    counter = 0
    tempPattern = ""
    samecolFlag = 0
    timeStarted = 0
    lightpattern = LightPattern()

    while True:

        # Capture a frame of the video
        _,frame = cap.read()
        result = lightpattern.getPattern(frame)

        # Display the captured frame
        cv2.imshow("Frame", result)
        tempPattern = pattern



    # Check if the user wants to quit (Press q)
    if cv2.waitKey(1) & 0xFF == ord('q'):

        # Break out of the loop
        break

cap.release()
cv2.destroyAllWindows()
