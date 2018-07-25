#include "ShapeDetector.hpp"

void ShapeDetector::detectContours() {
    // Preprocessing gaussian blur
    int blurKernel = 5; // must be odd and positive, determines size of blur square
    cv::GaussianBlur(m_image, m_image, cv::Size(blurKernel, blurKernel), 0, 0);

    // Detect edges using Canny
    cv::Mat cannyOutput;
    int lowerThreshold = 1, upperThreshold = 3, kernelSize = 5; // kernelSize must be in 3-7 range
    cv::Canny(m_image, cannyOutput, lowerThreshold, upperThreshold, kernelSize);

    // Find contours
    cv::findContours(cannyOutput, m_contours, m_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
}

void ShapeDetector::detectShapes() {
    std::string shape;
    float perimeter;

    for (int i = 0; i < m_contours.size(); ++i) {
        shape = "unidentified";
        perimeter = cv::arcLength(m_contours[i], true);

        // Get the approximate shape
        std::vector<cv::Point> approx;
        cv::approxPolyDP(m_contours[i], approx, 0.01*perimeter, true);

        // Get the bounding box and dimensions
        cv::Rect boundRect = cv::boundingRect(cv::Mat(approx));
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(cv::Mat(approx), center, radius);

        // Identify the approximate shape
        float sidesRatio = boundRect.width/boundRect.height;
        if (approx.size() == 4 || sidesRatio < 0.5 || sidesRatio > 2) {
            if (sidesRatio > 0.90 && sidesRatio < 1.10) {
                shape = "square";
            } else {
                shape = "rectangle";
            }
        } else if (approx.size() > 4) {
            shape = "circle";
        }
        if (radius > m_minRad && (boundRect.width > m_minDims || boundRect.height > m_minDims)) {
            m_shapes.push_back(approx);
            m_shape_names.push_back(shape);
            m_boundRects.push_back(boundRect);
            m_centers.push_back(center);
            m_radii.push_back(radius);
        }
    }

    // Delete any duplicates
    removeDuplicateShapes();
}

void ShapeDetector::detectHoughCircles(bool print) {
    int houghParam1 = 100, houghParam2 = 35;
    int minRadius = 10, maxRadius = 400;
    cv::HoughCircles(m_image, m_houghCircles, CV_HOUGH_GRADIENT, 2, m_image.rows/8, houghParam1, houghParam2, minRadius, maxRadius);

    if (print) {
        std::cout << "Hough Circles:" << std::endl;
        for (int i = 0; i < m_houghCircles.size(); ++i) {
            std::cout << "Circle at x=" << m_houghCircles[i][0] << " y=" << m_houghCircles[i][1] << " z=" << m_houghCircles[i][2] << std::endl;
        }
        if (m_houghCircles.size() == 0) {
            std::cout << "No circles detected using hough method" << std::endl;
        }
    }
}

void ShapeDetector::drawContours(cv::Mat underlay) {
    for (int i = 0; i < m_contours.size(); ++i) {
        cv::drawContours(underlay, m_contours, i, cv::Scalar(255, 0, 0), 1, 8, m_hierarchy, 0, cv::Point());
    }
}

void ShapeDetector::drawShapes(cv::Mat underlay) {
    // select colour
    cv::Scalar colour;
    if (m_tag == NAVY) {
        colour = cv::Scalar(255, 0, 0);
    } else if (m_tag == RED) {
        colour = cv::Scalar(0, 0, 150);
    } else if (m_tag == GREEN) {
        colour = cv::Scalar(0, 175, 0);
    } else if (m_tag == WHITE) {
        colour = cv::Scalar(255, 255, 255);
    } else if (m_tag == BLACK) {
        colour = cv::Scalar(0, 0, 0);
    } else if (m_tag == BLUE) {
        colour = cv::Scalar(0, 0, 255);
    } else if (m_tag == YELLOW) {
        colour = cv::Scalar(0, 255, 255);
    } else {
        colour = cv::Scalar(0, 255, 0);
        std::cout << "Green colour default used" << std::endl;
    }
    // Draw rectangles around everything except circles
    for (int i = 0; i < m_shapes.size(); ++i) {
        if (m_shape_names[i] != "circle") {
            cv::rectangle(underlay, m_boundRects[i], colour, 1, 8, 0);
        } else {
            cv::circle(underlay, m_centers[i], (int)m_radii[i], colour, 1, 8, 0);
        }
        cv::putText(underlay, m_shape_names[i], cv::Point(m_boundRects[i].x, m_boundRects[i].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 1);;
    }
}

void ShapeDetector::drawOutput(bool contours) {
    cv::Mat m_output = cv::Mat::zeros(m_image.rows, m_image.cols, CV_8UC3);
    if (contours) {
        drawContours(m_output);
    }
    drawShapes(m_output);
    cv::namedWindow("Output", CV_WINDOW_AUTOSIZE);
    cv::imshow("Output", m_output);
}

void ShapeDetector::drawOutput(cv::Mat underlay, bool contours) {
    if (contours) {
        drawContours(underlay);
    }
    drawShapes(underlay);
}

void ShapeDetector::drawHoughCircles(cv::Mat underlay) {
    for (int i = 0; i < m_houghCircles.size(); i++) {
        cv::Vec3f circle = m_houghCircles[i];

        // draw a circle with the centre and the radius obtained from the hough transform
        cv::circle(underlay, cv::Point(round(circle[0]), round(circle[1])), 2, CV_RGB(255,255,255), -1, 8, 0);
        cv::circle(underlay, cv::Point(round(circle[0]), round(circle[1])), round(circle[2]), CV_RGB(0,255,0), 2, 8, 0);
    }
}

std::string ShapeDetector::getColourStr() const {
    switch(m_tag) {
        case NAVY:
            return "navy";
            break;
        case RED:
            return "red";
            break;
        case GREEN:
            return "green";
            break;
        case WHITE:
            return "white";
            break;
        case BLACK:
            return "black";
            break;
        case BLUE:
            return "blue";
            break;
        case YELLOW:
            return "yellow";
            break;
        case NUM_COLOURS:
            std::cout << "ERROR: Ran NUM_COLOURS in getColourStr(Colour colour) function in ColourExtractor" << std::endl;
            break;
        default:
            std::cout << "ERROR: Update getColourStr(Colour colour) function in ColourExtractor" << std::endl;
            break;
    }
    return "ERROR";
}

void ShapeDetector::removeDuplicateShapes() {
    cv::Point2f center;
    float radius;
    float radius_ratio;
    float epsilon_position{1.0}; // within 2 pixels
    float epsilon_radius_percentage{0.10}; // 10% either side
    if (m_shapes.size() > 0) {
        for (int i = 0; i < m_shapes.size()-1; ++i) {
            center = m_centers[i];
            radius = m_radii[i];
            for (int j = i+1; j < m_shapes.size(); ++j) {
                radius_ratio = m_radii[j]/radius;
                if (abs(m_centers[j].x - center.x) < epsilon_position
                    && abs(m_centers[j].y - center.y) < epsilon_position
                    && (radius_ratio > 1-epsilon_radius_percentage
                        || radius_ratio < 1+epsilon_radius_percentage)) {
                    m_shapes.erase(m_shapes.begin() + j);
                    m_shape_names.erase(m_shape_names.begin() + j);
                    m_boundRects.erase(m_boundRects.begin() + j);
                    m_centers.erase(m_centers.begin() + j);
                    m_radii.erase(m_radii.begin() + j);
                }
            }
        }
    }
}

void ShapeDetector::printObjects() const {
    std::cout << m_shapes.size() << " shapes extracted from " << m_contours.size() << " contours:" << std::endl;
    for (int i = 0; i < m_shapes.size(); ++i) {
        std::cout << m_shape_names[i] << " (" << getColourStr() << ")";
        std::cout << " detected at (" << m_centers[i].x << ", " << m_centers[i].y;
        std::cout << ")";// << " with " << approx.size() << " sides";
        if (m_shape_names[i] == "circle") {
            std::cout << " (radius=" << m_radii[i] << ")" << std::endl;
        } else {
            std::cout << " (height=" << m_boundRects[i].height;
            std::cout << ", width=" << m_boundRects[i].width << ")" << std::endl;
        }
    }
}

std::vector<std::string> ShapeDetector::getShapeNames() const {
    return m_shape_names;
}

std::vector<cv::Rect> ShapeDetector::getBoundRects() const {
    return m_boundRects;
}

std::vector<cv::Point2f> ShapeDetector::getCenters() const {
    return m_centers;
}

int ShapeDetector::getNumberShapes() const {
    return m_shape_names.size();
}
