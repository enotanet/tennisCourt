#include "court_display.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

CourtDisplay::CourtDisplay(std::string filename) :
    image(cv::imread(filename)) {
  centre = image.size();
  centre.x /= 2;
  centre.y /= 2;
  court_size = 2.0 / 3 * image.cols;
}

bool CourtDisplay::display(cv::Point2d ball, std::vector<cv::Point2d> players, cv::Mat *res) {
  *res = image.clone();
  cv::circle(*res, convertCoordinate(ball), court_size / 60, cv::Scalar(44, 237, 198), -1, 8);
  for (auto p : players) {
    cv::circle(*res, convertCoordinate(p), court_size / 50, cv::Scalar(0, 0, 255), -1, 8);
  }
  return true;
}

void CourtDisplay::calibrate() {
  cv::namedWindow("Calibrate Court", CV_WINDOW_NORMAL);
  std::vector<cv::Point2d> corners;
  corners.push_back(cv::Point2d(-39.0 * k_foot, 18 * k_foot));
  corners.push_back(cv::Point2d(39.0 * k_foot, 18 * k_foot));
  corners.push_back(cv::Point2d(-39.0 * k_foot, -18 * k_foot));
  corners.push_back(cv::Point2d(39.0 * k_foot, -18 * k_foot));
  char last_key = 't';
  while (last_key != 'o' && last_key != 'x' && last_key != 27) {
    switch (last_key) {
    case 'h':
      centre.x -= 1;
      break;
    case 'j':
      centre.y -= 1;
      break;
    case 'k':
      centre.y += 1;
      break;
    case 'l':
      centre.x += 1;
      break;
    case '+':
    case 'a':
      ++court_size;
      break;
    case '-':
    case 'd':
      --court_size;
      break;
    }
    cv::Mat img;
    display(cv::Point2d(0, 0), corners, &img);
    cv::imshow("Calibrate Court", img);
    last_key = cv::waitKey(0);
  }
}

cv::Point2i CourtDisplay::convertCoordinate(cv::Point2d point) {
  // court_size is 78 feet. point is in meters based on centre.
  // meters to pixels.
  //
  cv::Point2i scaled = point * (1.0 / k_foot) * (court_size / 78.0);
  return scaled + centre;
}

