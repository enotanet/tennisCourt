#ifndef COURT_DETECTOR_H__
#define COURT_DETECTOR_H__
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

std::vector<std::pair<cv::Point2f, cv::Point3d>> mapCourtCornersTo3DCoordinates(cv::Mat &frame, std::vector<cv::Point2f> courtCorners, int camera);
std::vector<cv::Point2f> getCourtCorners(cv::Mat &frame);
cv::Mat getCourtOnly(cv::Mat &frame);
void diplayCourtDetectorResult(cv::Mat &frame, std::string windowName="Result");

struct hash_point {
  long operator()(const cv::Point &point) const{
    int x = point.x;
    int y = point.y;
    return 0.5 * (x + y) * (x + y + 1) + y;
  }
};

#endif
