#ifndef FILE_FRAME_GRABBER_H__
#define FILE_FRAME_GRABBER_H__

#include "frame_grabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>

class FileFrameGrabber : public FrameGrabber {
public:
  FileFrameGrabber(const std::string &filename);
  bool getNextFrame(cv::Mat *res);
  cv::Mat getMat();
private:
  cv::VideoCapture vc;
};

#endif  // FILE_FRAME_GRABBER_H__
