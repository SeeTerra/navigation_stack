#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

#include "ArucoDetector.hpp"

ArucoDetector::ArucoDetector() {
  std::cout << "Hello world! I am the ArucoDetector!" << std::endl;
}

ArucoDetector::~ArucoDetector() {
  std::cout << "Goodbye, ol' cruel world." << std::endl;
}
