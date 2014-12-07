#include "utils.h"
#include "file_frame_grabber.h"
#include "sys_file_frame_grabber.h"
#include <cassert>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

SystemFileFrameGrabber::SystemFileFrameGrabber(const std::vector<std::string> &filenames) {
  INFO("Initialising grabber with " << filenames.size() << " fiels");
  for (const auto &filename : filenames) {
    grabbers.emplace_back(FileFrameGrabber(filename));
  }
}

bool SystemFileFrameGrabber::getNextFrames(std::vector<cv::Mat> *res) {
  assert(res->size() == grabbers.size());
  for (size_t i = 0; i < grabbers.size(); ++i) {
    if (!grabbers[i].getNextFrame(&(*res)[i])) {
      return false;
    }
  }
  return true;
}

std::vector<cv::Mat> SystemFileFrameGrabber::getContainer() {
  std::vector<cv::Mat> cont;
  INFO("Grabber with " << grabbers.size() << " items");
  for (auto &fg : grabbers) {
    cont.push_back(fg.getMat());
  }
  return cont;
}
