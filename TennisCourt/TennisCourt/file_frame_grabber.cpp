#include "file_frame_grabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>

FileFrameGrabber::FileFrameGrabber(const std::string &filename) : vc(filename) {
}

bool FileFrameGrabber::getNextFrame(cv::Mat *res) {
  if (!vc.isOpened())
    return false;
  return vc.read(*res);
}

cv::Mat FileFrameGrabber::getMat() {
  cv::Mat frame;
  vc.read(frame);
  return frame;
}
