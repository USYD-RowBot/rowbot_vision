#ifndef CLASSIFIER
#define CLASSIFIER

#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Object.hpp"
#include "enums.h"

// ideas for classifier:
// use hough circles to increase/decrease confidence
// develop range function based on y coord and distance to horizon
// look at the colour distribution of the object to increase/decrease colour confidence

class Classifier {
private:

    float m_fov;
    float m_imageWidth;
    float m_imageHeight;

public:
    std::vector<Object> m_objects;
    Classifier(float cameraFOV, float imageWidth, float imageHeight)
    : m_fov{cameraFOV}, m_imageWidth{imageWidth}, m_imageHeight{imageHeight}
    {};

    Object classify(std::string shape_name, cv::Rect boundingRect, Colour colour);
    float bearing(cv::Point2f center);
    float range(cv::Point2f center);
    void classifyAndLocate(std::string shape_name, cv::Rect boundingRect, Colour colour, cv::Point2f center);
    void printObjects() const;
};

#endif
