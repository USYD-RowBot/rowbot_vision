#include "input.hpp"

cv::VideoCapture getWebcam(int index) {
    return cv::VideoCapture(index);
}

cv::Mat getImage(std::string fileName) {
    return cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
}
