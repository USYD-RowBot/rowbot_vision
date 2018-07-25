#!/bin/bash
g++ -std=c++11 `pkg-config --cflags --libs opencv` buoy_detection.cpp input.cpp ShapeDetector.cpp ColourExtractor.cpp Classifier.cpp Object.cpp -o buoy_detection.o
