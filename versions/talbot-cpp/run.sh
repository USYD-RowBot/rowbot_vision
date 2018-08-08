#!/bin/bash
g++ -std=c++11  buoy_detection.cpp input.cpp ShapeDetector.cpp ColourExtractor.cpp Classifier.cpp Object.cpp -o buoy_detection.o `pkg-config --cflags --libs opencv`
