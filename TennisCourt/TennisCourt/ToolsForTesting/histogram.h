#ifndef HISTOGRAM_H__
#define HISTOGRAM_H__

void displayHistogram(cv::Mat &image, std::string name, std::vector<int> &intensitiesToHighlight=std::vector<int>());
cv::Mat drawHistogram(cv::Mat &image);

#endif