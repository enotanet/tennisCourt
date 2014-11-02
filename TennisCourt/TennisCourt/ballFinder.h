#ifndef BALL_FINDER_H__
#define BALL_FINDER_H__
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

std::vector<cv::Vec3f> getCircles(cv::Mat img);
bool getCirclesVerify(cv::Mat img);
bool getCirclesTest(cv::Mat *img);

#endif