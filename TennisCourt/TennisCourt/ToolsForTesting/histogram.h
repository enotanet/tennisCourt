#ifndef HISTOGRAM_H__
#define HISTOGRAM_H__

#ifdef WIN32
void displayHistogram(cv::Mat &image, std::string name, std::vector<int> &intensitiesToHighlight=std::vector<int>());
#else
void displayHistogram(cv::Mat &image, std::string name, std::vector<int> &intensitiesToHighlight);
#endif
cv::Mat drawHistogram(cv::Mat &image);

#endif
