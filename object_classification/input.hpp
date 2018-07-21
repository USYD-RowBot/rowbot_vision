#ifndef INPUT
#define INPUT

#include "opencv2/opencv.hpp"
#include <iostream>

cv::VideoCapture getWebcam(int index);

cv::Mat getImage(std::string fileName);

#endif
