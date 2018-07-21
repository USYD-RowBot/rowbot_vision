#ifndef SHAPE_DETECTOR
#define SHAPE_DETECTOR

#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "enums.h"

class ShapeDetector {
private:
    cv::Mat m_image;
    Colour m_tag;

    int m_minRad{3};
    int m_minDims{6};

    // Regular shape detection
    std::vector< std::vector<cv::Point> > m_contours;
    std::vector<cv::Vec4i> m_hierarchy;
    std::vector< std::vector<cv::Point> > m_shapes;
    std::vector<std::string> m_shape_names;
    std::vector<cv::Rect> m_boundRects;
    std::vector<cv::Point2f> m_centers;
    std::vector<float> m_radii;
    void drawShapes(cv::Mat underlay);
    void drawContours(cv::Mat underlay);

    // Hough circle detection
    std::vector<cv::Vec3f> m_houghCircles;


public:
    ShapeDetector(cv::Mat image, Colour tag)
    : m_image{image}, m_tag{tag}
    {};

    void detectContours();
    void detectShapes();
    void detectHoughCircles(bool print);
    void drawOutput(bool contours); // this version draws onto black
    void drawOutput(cv::Mat underlay, bool contours);
    void drawHoughCircles(cv::Mat underlay);
    std::string getColourStr() const;
    void removeDuplicateShapes();
    void printObjects() const;
    std::vector<std::string> getShapeNames() const;
    std::vector<cv::Rect> getBoundRects() const;
    std::vector<cv::Point2f> getCenters() const;
    int getNumberShapes() const;
};

#endif
