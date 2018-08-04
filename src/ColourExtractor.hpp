#ifndef COLOUR_EXTRACTOR
#define COLOUR_EXTRACTOR

#include <iostream>
#include <array>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "enums.h"

// Steps to add a new colour:
// 1. add two scalars indicating colour range to ColourExtractor of form:
///// cv::Scalar(hue_min, sat_min, value_min, 0)
///// cv::Scalar(hue_max, sat_max, value_max, 0)
// 2. add a case to the enumeration list in enums.h
// 3. add a case to ColourExtractor::extract()
// 4. add a colour case to ShapeDetector::drawShapes()
// 5. add a colour case to ShapeDetector::getColourStr()
// 6. add array element to Classifer::classify()

class ColourExtractor {
private:
    cv::Scalar m_hsvMinNavy = cv::Scalar(110, 0, 0, 0);
    cv::Scalar m_hsvMaxNavy = cv::Scalar(140, 255, 100, 0);
    cv::Scalar m_hsvMinRed = cv::Scalar(0, 100, 100, 0);
    cv::Scalar m_hsvMaxRed = cv::Scalar(15, 255, 255, 0);
    cv::Scalar m_hsvMinRed2 = cv::Scalar(165, 100, 100, 0);
    cv::Scalar m_hsvMaxRed2 = cv::Scalar(179, 255, 255, 0);
    cv::Scalar m_hsvMinGreen = cv::Scalar(45, 75, 0, 0);
    cv::Scalar m_hsvMaxGreen = cv::Scalar(90, 255, 255, 0);
    cv::Scalar m_hsvMinWhite = cv::Scalar(0, 0, 222, 0);
    cv::Scalar m_hsvMaxWhite = cv::Scalar(179, 10, 255, 0);
    cv::Scalar m_hsvMinBlack = cv::Scalar(0, 0, 0, 0);
    cv::Scalar m_hsvMaxBlack = cv::Scalar(255, 150, 50, 0);
    cv::Scalar m_hsvMinBlue = cv::Scalar(110, 175, 175, 0);
    cv::Scalar m_hsvMaxBlue = cv::Scalar(125, 255, 255, 0);
    cv::Scalar m_hsvMinYellow = cv::Scalar(26, 0, 150, 0);
    cv::Scalar m_hsvMaxYellow = cv::Scalar(33, 255, 255, 0);

public:
    ColourExtractor()
    {};
    cv::Mat bgr2hsv_withBlur(cv::Mat image);
    cv::Mat extract(cv::Mat hsv_image, Colour colour, bool displayALL);
};

#endif
