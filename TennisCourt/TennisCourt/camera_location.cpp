#include "camera_location.h"
#include <opencv2/core/core.hpp>

bool CameraLocation::GetCoordinate(cv::Mat frame, cv::Point3f *coords) {
  return false;
}

bool CameraLocation::GetCourtCorners(cv::Mat frame, std::vector<Corner> *corners) {
  return false;
}
