#ifndef BUOY_DETECTOR
#define BUOY_DETECTOR

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Object.hpp"

// This is the overall class wrapper
class BuoyDetector {
private:
    float m_cameraFOV{70.0};
    cv::Mat m_mask;
    std::vector<Object> m_objects;


public:
    BuoyDetector() {
        //m_mask = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0,0,0));
        //classifyBuoys(image, printToScreen);
    }
    void classifyBuoys(cv::Mat image, bool printToScreen);
    cv::Mat getMask() const;
    std::vector<Object> getObjects() const;
    Object* getBestAtBearing(float bearing, int objsInFront) ;
};

#endif
