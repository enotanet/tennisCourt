#ifndef SYS_FILE_FRAME_GRABBER_H__
#define SYS_FILE_FRAME_GRABBER_H__

#include "file_frame_grabber.h"
#include "sys_frame_grabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

class SystemFileFrameGrabber : public SystemFrameGrabber {
public:
  SystemFileFrameGrabber(const std::vector<std::string> &filename);
  bool getNextFrames(std::vector<cv::Mat> *res);
  std::vector<cv::Mat> getContainer();
private:
  std::vector<FileFrameGrabber> grabbers;
};

#endif  // SYS_FILE_FRAME_GRABBER_H__
