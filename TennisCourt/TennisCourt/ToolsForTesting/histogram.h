#ifndef HISTOGRAM_H__
#define HISTOGRAM_H__
#include "opencv2/imgproc/imgproc.hpp"

void displayHistogram(cv::Mat &image, std::string name);
cv::Mat drawHistogram(cv::Mat &image);

#endif