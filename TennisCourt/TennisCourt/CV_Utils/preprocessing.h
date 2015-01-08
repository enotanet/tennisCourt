#ifndef PREPROCESSING_H__
#define PREPROCESSING_H__
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

void cutBetweenIntensities(cv::Mat &image, int lowerBound, int upperBound, bool inverse=false);
int* getIntensities(cv::Mat &image);
int getModeIntensity(cv::Mat &image);

#endif