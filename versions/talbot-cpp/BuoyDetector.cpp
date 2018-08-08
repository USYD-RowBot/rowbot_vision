#include "BuoyDetector.hpp"
#include "Classifier.hpp"
#include "ColourExtractor.hpp"
#include "enums.h"
#include "ShapeDetector.hpp"

void BuoyDetector::classifyBuoys(cv::Mat image, bool printToScreen) {
    if (image.data) {
        // Initialise colour extractor
        ColourExtractor colEx = ColourExtractor();
        // initialise classifier with 70 FOV camera
        Classifier classifier = Classifier(m_cameraFOV, image.cols, image.rows); // initialise classifier with 70 FOV camera
        //blur once
        cv::Mat blurred_hsv_orig = colEx.bgr2hsv_withBlur(image);
        for (int colour = 0; colour < NUM_COLOURS; ++colour) {
            // Get filtered image
            cv::Mat thresholded = colEx.extract(blurred_hsv_orig, (Colour)colour, false);

            // Detect shapes
            ShapeDetector sd = ShapeDetector(thresholded, (Colour)colour);
            sd.detectContours();
            sd.detectShapes();
            if (printToScreen) {
                sd.printObjects();
            }
            sd.drawOutput(m_mask, false);
            //sd.detectHoughCircles(true);
            //sd.drawHoughCircles(image);

            std::vector<std::string> shapeNames = sd.getShapeNames();
            std::vector<cv::Rect> boundRects = sd.getBoundRects();
            std::vector<cv::Point2f> centers = sd.getCenters();
            for (int i = 0; i < sd.getNumberShapes(); ++i) {
                classifier.classifyAndLocate(shapeNames[i], boundRects[i], (Colour)colour, centers[i]);
            }
        }
        if (printToScreen) {
            classifier.printObjects();
        }
    } else {
        std::cout << "ERROR: Image has no data" << std::endl;
    }
}

cv::Mat BuoyDetector::getMask() const {
    return m_mask;
}

std::vector<Object> BuoyDetector::getObjects() const {
    return m_objects;
}

Object* BuoyDetector::getBestAtBearing(float bearing, int objsInFront) const {
    if (m_objects.size() == 0) {
        return NULL;
    }
    float margin = 0.10*m_cameraFOV; // 10% of camera FOV
    float highestConfidence = -1;
    int index = -1;
    for (int i = 0; i < m_objects.size(); ++i) {
        if (m_objects[i].getMaxConfidence() > highestConfidence && m_objects[i].getBearing() > (bearing - margin/2) && m_objects[i].getBearing() < (bearing + margin/2)) {
            index = i;
            highestConfidence = m_objects[i].getMaxConfidence();
        }
    }
    if (index >= 0) {
        return &m_objects[index];
    }
    return NULL;
}
