#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <cv.h>
//#include <highgui.h>
//#include <cxcore.h>

#include <vector>
#include <iostream>
//#include "math.h"

//#include "enums.h"
#include "input.hpp"
#include "ShapeDetector.hpp"
#include "ColourExtractor.hpp"
#include "Classifier.hpp"
#include "Object.hpp"

int main(int argc, char* argv[]) {
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    cv::VideoCapture capture;
    if (argc == 1) {
        capture = getWebcam(0);
        if(!capture.isOpened()) {
            std::cout << "ERROR: Camera could not be opened" << std::endl;
            return -1;
        }
    }

    // grab an image from the capture or image
    cv::Mat image;
    if (argc == 1) {
        capture >> image;
        if (image.empty()) {
            std::cout << "ERROR: Frame empty" << std::endl;
            return -1;
        }
    } else if (argc == 2) {
        std::string fileName = argv[1];
        image = getImage(fileName);
        if (!image.data) {
            std::cout << "ERROR: Could not open or find image" << std::endl;
            return -1;
        }
    } else {
        std::cout << "ERROR: Too many input arguments (" << argc << ")" << std::endl;
        return -1;
    }

    bool loop = true;
    while(loop) {
        // Get one frame if webcam is on
        if (argc == 1) {
            capture >> image;
            if (image.empty()) {
                std::cout << "ERROR: Frame empty" << std::endl;
                return -1;
            }
        } else {
            loop = false;
        }

        ColourExtractor colEx = ColourExtractor();
        Classifier classifier = Classifier(70, image.cols, image.rows); // initialise classifier with 70 FOV camera
        //blur once
        cv::Mat blurred_hsv_orig = colEx.bgr2hsv_withBlur(image);
        for (int colour = 0; colour < NUM_COLOURS; ++colour) {
            // Get filtered image
            cv::Mat thresholded = colEx.extract(blurred_hsv_orig, (Colour)colour, true);

            // Detect shapes
            ShapeDetector sd = ShapeDetector(thresholded, (Colour)colour);
            sd.detectContours();
            sd.detectShapes();
            sd.printObjects();
            sd.drawOutput(image, true);
            //sd.detectHoughCircles(true);
            //sd.drawHoughCircles(image);

            std::vector<std::string> shapeNames = sd.getShapeNames();
            std::vector<cv::Rect> boundRects = sd.getBoundRects();
            std::vector<cv::Point2f> centers = sd.getCenters();
            for (int i = 0; i < sd.getNumberShapes(); ++i) {
                classifier.classifyAndLocate(shapeNames[i], boundRects[i], (Colour)colour, centers[i]);
            }
        }
        classifier.printObjects();

        std::string windowName;
        if (argc == 1) {
            windowName = "Camera";
        } else {
            windowName = "Image";
        }
        cv::imshow(windowName, image); // Original stream with detected ball overlay

        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cv::waitKey(10) & 255) == 27 ) break;
    }

    // Release the capture device housekeeping
    cv::waitKey(0);
    return 0;
}
