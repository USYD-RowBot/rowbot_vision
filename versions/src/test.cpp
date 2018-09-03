#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    cv::VideoCapture capture;
    if (argc == 1) {
        capture = cv::VideoCapture(0);
        if(!capture.isOpened()) {
            std::cout << "ERROR: Camera could not be opened" << std::endl;
            return -1;
        }
    }

    // grab an image from the capture or image
    cv::Mat image;
    if (argc == 1) {
        capture >> image;
        if (image.empty()) {
            std::cout << "ERROR: Frame empty" << std::endl;
            return -1;
        }
    } else {
        std::cout << "ERROR: Too many input arguments (" << argc << ")" << std::endl;
        return -1;
    }

    return 0;
  }
