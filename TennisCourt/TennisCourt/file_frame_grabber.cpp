#include "file_frame_grabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>

FileFrameGrabber::FileFrameGrabber(const std::string &filename) : vc(filename) {
  fprintf(stdout, "Opening file: %s\n", filename.c_str());
}

bool FileFrameGrabber::getNextFrame(cv::Mat *res) {
  if (!vc.isOpened())
    return false;
  cv::Mat col;
  bool ok = vc.read(col);
  if (!ok) {
    return false;
  }
  cv::cvtColor(col, *res, CV_BGR2GRAY);
  return true;
}

cv::Mat FileFrameGrabber::getMat() {
  cv::Mat frame;
  vc.read(frame);
  return frame;
}
