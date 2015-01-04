#include "utils.h"
#include "file_frame_grabber.h"
#include "sys_file_frame_grabber.h"
#include <cassert>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

SystemFileFrameGrabber::SystemFileFrameGrabber(const std::vector<std::string> &filenames) {
  INFO("Initialising grabber with " << filenames.size() << " files");
  for (const auto &filename : filenames) {
    grabbers.emplace_back(FileFrameGrabber(filename));
  }
  offset.assign(filenames.size(), 0);
  for (size_t i = 0; i < grabbers.size() && i < g_args["skip"].size(); ++i) {
    int frames_to_skip;
    sscanf(g_args["skip"][i].c_str(), "%d", &frames_to_skip);
    DEBUG("Skipping " << frames_to_skip << " frames from video #" << i);
    for (int j = 0; j < frames_to_skip; ++j) {
      cv::Mat m;
      grabbers[i].getNextFrame(&m);
      ++offset[i];
    }
  }
}

bool SystemFileFrameGrabber::getNextFrames(std::vector<cv::Mat> *res) {
  assert(res->size() == grabbers.size());
  for (size_t i = 0; i < grabbers.size(); ++i) {
    ++offset[i];
    if (!grabbers[i].getNextFrame(&(*res)[i])) {
      return false;
    }
    char buf[32];
    sprintf(buf, "Frames: %d\n", offset[i]);
    cv::putText((*res)[i], buf, cv::Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 3, cv::Scalar(0, 0, 0), 5, 8);
  }
  return true;
}

bool SystemFileFrameGrabber::getNextFrame(size_t stream, std::vector<cv::Mat> *res) {
  assert(stream < grabbers.size());
  ++offset[stream];
  bool ret = grabbers[stream].getNextFrame(&(*res)[stream]);
  char buf[32];
  sprintf(buf, "Frames: %d\n", offset[stream]);
  cv::putText((*res)[stream], buf, cv::Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 3, cv::Scalar(0, 0, 0), 5, 8);
  return ret;
}

std::vector<cv::Mat> SystemFileFrameGrabber::getContainer() {
  std::vector<cv::Mat> cont;
  INFO("Grabber with " << grabbers.size() << " items");
  for (auto &fg : grabbers) {
    cont.push_back(fg.getMat());
  }
  return cont;
}
