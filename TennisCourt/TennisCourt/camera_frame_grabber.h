// Concrete class that can grab frames from camera.
// Not thread-safe.

// Do we need a lock for CTlFactory?? Probably!

#ifndef CAMERA_FRAME_GRABBER_H__
#define CAMERA_FRAME_GRABBER_H__

#include "frame_grabber.h"
#include <opencv2/core/core.hpp>
#include <mutex>
#include <thread>

#ifdef PYLON_INSTALLED
#include <pylon/PylonIncludes.h>

// Behaves differently from grabbing from file.
// There, we would expect consecutive frames. Here, this behaviour
// would require an extra buffer which might get too big.
// Also, threading problems? Maybe there is a better function
// to simulate files in Pylon.
class CameraFrameGrabber : public FrameGrabber {
public:
  // Might want to add new params for setting camera options.
  CameraFrameGrabber(int camera_id);
  ~CameraFrameGrabber();
  bool getNextFrame(cv::Mat *res);
  cv::Mat getMat();
private:
  Pylon::CInstantCamera* camera;
  GenApi::CIntegerPtr width;
  GenApi::CIntegerPtr height;
  Pylon::CGrabResultPtr ptrGrabResult;
  Pylon::CImageFormatConverter fc;
  Pylon::CPylonImage image;
  // Lock acquired during construction
  static std::mutex pylonFactoryLock;
};

#else

class CameraFrameGrabber : public FrameGrabber {
public:
  // Might want to add new params for setting camera options.
  CameraFrameGrabber(int camera_id) {}
  ~CameraFrameGrabber() {}
  bool getNextFrame(cv::Mat *res) {
    return true;
  }
  cv::Mat getMat() {
    return cv::Mat();
  }
#endif  // PYLON_INSTALLED

#endif  // CAMERA_FRAME_GRABBER_H__
