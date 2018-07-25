#include <iostream>
#include "Classifier.hpp"

//// Types with their multMatrix position:
// obstacle_buoy (navy or black, circular, small or large) (0, 0) & (0, 4)
// white_post (1, 3)
// white_log (2, 3)
// white_buoy (0, 3)
// red_post (1, 1)
// green_post (1, 2)
// blue_post (1, 5)
// yellow_post (1, 6)

Object Classifier::classify(std::string shape_name, cv::Rect boundingRect, Colour colour) {
    // Establish shape confidence. shape_confs has order of
    // sphere confidence, post confidence, log confidence
    std::array<float, 3> shape_confs = {0.34, 0.33, 0.33};
    float heightToWidth = static_cast<float>(boundingRect.height/boundingRect.width);
    if (shape_name == "circle") {
        if (heightToWidth <= 1.11 && heightToWidth >= 0.9) {
            shape_confs = {0.9, 0.05, 0.05};
        } else if (heightToWidth > 1.11 && heightToWidth <= 1.25) {
            shape_confs = {0.8, 0.2, 0};
        } else if (heightToWidth > 1.25 && heightToWidth <= 1.43) {
            shape_confs = {0.7, 0.3, 0};
        } else if (heightToWidth > 1.43 && heightToWidth <= 1.67) {
            shape_confs = {0.6, 0.4, 0};
        } else if (heightToWidth > 1.67) {
            shape_confs = {0.5, 0.5, 0};
        } else if (heightToWidth < 0.8 && heightToWidth >= 1.25) {
            shape_confs = {0.8, 0, 0.2};
        } else if (heightToWidth < 0.7 && heightToWidth >= 1.43) {
            shape_confs = {0.7, 0, 0.3};
        } else if (heightToWidth < 0.6 && heightToWidth >= 0.5) {
            shape_confs = {0.6, 0, 0.4};
        } else if (heightToWidth < 0.5) {
            shape_confs = {0.5, 0, 0.5};
        }
    } else if (shape_name == "square") {
        if (heightToWidth > 1.053) {
            shape_confs = {0.9, 0.1, 0};
        } else if (heightToWidth < 0.95) {
            shape_confs = {0.9, 0, 0.1};
        } else {
            shape_confs = {0.9, 0.05, 0.05};
        }
    } else if (shape_name == "rectangle") {
        if (heightToWidth >= 4) {
            shape_confs = {0.05, 0.95, 0};
        } else if (heightToWidth >= 3.5) {
            shape_confs = {0.08, 0.92, 0};
        } else if (heightToWidth >= 3) {
            shape_confs = {0.1, 0.90, 0};
        } else if (heightToWidth >= 2.5) {
            shape_confs = {0.15, 0.85, 0};
        } else if (heightToWidth >= 2) {
            shape_confs = {0.2, 0.80, 0};
        } else if (heightToWidth <= 0.5) {
            shape_confs = {0.2, 0, 0.80};
        } else if (heightToWidth <= 0.4) {
            shape_confs = {0.15, 0, 0.85};
        } else if (heightToWidth <= 0.33) {
            shape_confs = {0.1, 0, 0.90};
        } else if (heightToWidth <= 0.285) {
            shape_confs = {0.08, 0, 0.92};
        } else if (heightToWidth <= 0.25) {
            shape_confs = {0.05, 0, 0.95};
        }
    }

    // Establish colour confidence. colour_confs has order:
    // NAVY, RED, GREEN, WHITE, BLACK, BLUE, YELLOW
    std::array<float, 7> colour_confs = {0.15, 0.15, 0.14, 0.14, 0.14, 0.14, 0.14};
    switch (colour) {
        case NAVY:
            colour_confs = {0.75, 0, 0.05, 0, 0.10, 0.10, 0};
            break;
        case RED:
            colour_confs = {0.01, 0.94, 0.01, 0.01, 0.01, 0.01, 0.01};
            break;
        case GREEN:
            colour_confs = {0.08, 0, 0.75, 0, 0.09, 0.08, 0};
            break;
        case WHITE:
            colour_confs = {0, 0, 0, 0.9, 0, 0, 0.1};
            break;
        case BLACK:
            colour_confs = {0.1, 0, 0.1, 0, 0.75, 0.05, 0};
            break;
        case BLUE:
            colour_confs = {0.1, 0, 0.1, 0, 0.05, 0.75, 0};
            break;
        case YELLOW:
            colour_confs = {0, 0, 0, 0.1, 0, 0, 0.9};
            break;
        case NUM_COLOURS:
            // fall through
        default:
            // error
            std::cout << "ERROR: colour " << (int)colour << " not supported" << std::endl;
            break;
    }

    // Create multiplication matrix
    std::array<std::array<float, 7>, 3> multMatrix;
    for (int row = 0; row < multMatrix.size(); ++row) {
        for (int col = 0; col < multMatrix[0].size(); ++col) {
            multMatrix[row][col] = shape_confs[row]*colour_confs[col];
        }
    }

    // Initialise vector of possible types
    std::vector<std::string> types;
    std::vector<float> confidences;

    // Choose those above 5% that are also applicable objects
    if (multMatrix[0][0] >= 0.05) {
        types.push_back("obstacle_buoy");
        confidences.push_back(multMatrix[0][0]);
    } else if (multMatrix[0][4] >= 0.05) {
        types.push_back("obstacle_buoy");
        confidences.push_back(multMatrix[0][4]);
    } else if (multMatrix[1][3] >= 0.05) {
        types.push_back("white_post");
        confidences.push_back(multMatrix[1][3]);
    } else if (multMatrix[2][3] >= 0.05) {
        types.push_back("white_log");
        confidences.push_back(multMatrix[2][3]);
    } else if (multMatrix[0][3] >= 0.05) {
        types.push_back("white_buoy");
        confidences.push_back(multMatrix[0][3]);
    } else if (multMatrix[1][1] >= 0.05) {
        types.push_back("red_post");
        confidences.push_back(multMatrix[1][1]);
    } else if (multMatrix[1][2] >= 0.05) {
        types.push_back("green_post");
        confidences.push_back(multMatrix[1][2]);
    } else if (multMatrix[1][5] >= 0.05) {
        types.push_back("blue_post");
        confidences.push_back(multMatrix[1][5]);
    } else if (multMatrix[1][6] >= 0.05) {
        types.push_back("yellow_post");
        confidences.push_back(multMatrix[1][6]);
    }

    // Return the object
    return Object(types, confidences);
}

float Classifier::bearing(cv::Point2f center) {
    return m_fov*(center.x - m_imageWidth/2)/m_imageWidth;
}

float Classifier::range(cv::Point2f center) {
    return m_imageHeight - center.y;
}

void Classifier::classifyAndLocate(std::string shape_name, cv::Rect boundingRect, Colour colour, cv::Point2f center) {
    Object obj = classify(shape_name, boundingRect, colour);
    obj.setBearing(bearing(center));
    obj.setRange(range(center));
    m_objects.push_back(obj);
}

void Classifier::printObjects() const {
    std::cout << std::endl << m_objects.size() << " objects detected." << std::endl;
    for (int i = 0; i < m_objects.size(); ++i) {
        m_objects[i].print();
    }
}
