#include "ColourExtractor.hpp"

cv::Mat ColourExtractor::bgr2hsv_withBlur(cv::Mat image) {
    // create the hsv mat object
    cv::Mat hsv_image(cv::Size(image.rows, image.cols), CV_16SC3);

    // apply a median blur to the bgr image before conversion to hsv
    cv::medianBlur(image, image, 3);

    // Convert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);

    // return the hsv image
    return hsv_image;
}

cv::Mat ColourExtractor::extract(cv::Mat hsv_image, Colour colour, bool displayALL) {
    // define min and max
    cv::Scalar hsv_min;
    cv::Scalar hsv_max;

    // assign min and max according to colour specified
    switch (colour) {
        case NAVY:
            hsv_min = m_hsvMinNavy;
            hsv_max = m_hsvMaxNavy;
            break;
        case RED:
            hsv_min = m_hsvMinRed;
            hsv_max = m_hsvMaxRed;
            break;
        case GREEN:
            hsv_min = m_hsvMinGreen;
            hsv_max = m_hsvMaxGreen;
            break;
        case WHITE:
            hsv_min = m_hsvMinWhite;
            hsv_max = m_hsvMaxWhite;
            break;
        case BLACK:
            hsv_min = m_hsvMinBlack;
            hsv_max = m_hsvMaxBlack;
            break;
        case BLUE:
            hsv_min = m_hsvMinBlue;
            hsv_max = m_hsvMaxBlue;
            break;
        case YELLOW:
            hsv_min = m_hsvMinYellow;
            hsv_max = m_hsvMaxYellow;
            break;
        case NUM_COLOURS:
            // fall through
        default:
            std::cout << "Error in extracting colour id:" << (int)colour << std::endl;
    }

    // create the hsv mat object
    cv::Size size = cv::Size(hsv_image.rows, hsv_image.cols);
    cv::Mat thresholded(size, CV_16SC3);

    // Filter out colors which are out of range on the hsv_image for thresholded image
    cv::inRange(hsv_image, hsv_min, hsv_max, thresholded);

    // special red case
    if (colour == RED) {
        cv::Mat thresholded2(size, CV_16SC3);
        cv::inRange(hsv_image, m_hsvMinRed2, m_hsvMaxRed2, thresholded2);
        // combine the two
        cv::bitwise_or(thresholded, thresholded2, thresholded);
    }

    // show individual components
    if (displayALL) {
        // create individual mat components
        cv::Mat h_thresholded(size, CV_16SC3);
        cv::Mat s_thresholded(size, CV_16SC3);
        cv::Mat v_thresholded(size, CV_16SC3);
        // filter individual components
        std::vector<cv::Mat> hsv_split;
        cv::split(hsv_image, hsv_split);
        cv::inRange(hsv_split[0], cv::Scalar(hsv_min[0],0,0,0), cv::Scalar(hsv_max[0],0,0,0), h_thresholded);
        cv::inRange(hsv_split[1], cv::Scalar(hsv_min[1],0,0,0), cv::Scalar(hsv_max[1],0,0,0), s_thresholded);
        cv::inRange(hsv_split[2], cv::Scalar(hsv_min[2],0,0,0), cv::Scalar(hsv_max[2],0,0,0), v_thresholded);
        // red case
        if (colour == RED) {
            cv::Mat h_thresholded2(size, CV_16SC3);
            cv::inRange(hsv_split[0], cv::Scalar(m_hsvMinRed2[0],0,0,0), cv::Scalar(m_hsvMaxRed2[0],0,0,0), h_thresholded2);
            cv::bitwise_or(h_thresholded, h_thresholded2, h_thresholded);
        }
        // show individual components
        cv::imshow(                  "HSV", hsv_image    ); // Original stream in the HSV color space
        cv::imshow("Post-Colour Filtering", thresholded  ); // The stream after color filtering
        cv::imshow(                    "H", h_thresholded); // individual filters
        cv::imshow(                    "S", s_thresholded);
        cv::imshow(                    "V", v_thresholded);
    }

    // Threshold the image and return it
    return thresholded;
}
