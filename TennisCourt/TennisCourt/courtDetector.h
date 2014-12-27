#ifndef COURT_DETECTOR_H__
#define COURT_DETECTOR_H__
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

std::vector<cv::Vec2f> getCourt(cv::Mat &frame);
cv::Mat preprocessCourt(cv::Mat &frame);
void diplayCourtDetectorResult(cv::Mat &frame, cv::Mat &preprocesedFrame);

struct hash_point {
  long operator()(const cv::Point &point) const{
    int x = point.x;
    int y = point.y;
    return 0.5 * (x + y) * (x + y + 1) + y;
  }
};

#endif