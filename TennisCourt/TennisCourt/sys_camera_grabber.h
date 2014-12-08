// Same as frame_grabber, just gives 4 frames at a time.

// Interface for grabbing frames from file or camera.
// Let's put it into a class and test performance.
// Classes not thread-safe. Just a single thread reads
// from the frame source.

#ifndef SYS_CAMERA_GRABBER_H__
#define SYS_CAMERA_GRABBER_H__

#include "sys_frame_grabber.h"
#include <opencv2/core/core.hpp>
#include <vector>

#ifdef PYLON_INSTALLED

#include <pylon/PylonIncludes.h>

class SystemCameraGrabber : public SystemFrameGrabber {
public:
  ~SystemCameraGrabber();
  bool startGrabbing();  // Put into constructor?
  bool getNextFrames(std::vector<cv::Mat> *res);
  // Returns a Mat, capable of storing the frames grabbed.
  //
  std::vector<cv::Mat> getContainer();
private:

  // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  // Careful when introducing other Pylon things! Order of AutoInitTerm is very important
  //
  Pylon::PylonAutoInitTerm autoInitTerm;

  Pylon::CInstantCameraArray cameras;
  Pylon::CImageFormatConverter fc;
  Pylon::CPylonImage img;

  static const int c_maxCamerasToUse = 4;
  static const int MAX_FRAMES_TO_KEEP = 5720;
};

#else  // NO PYLON - empty implementation

class SystemCameraGrabber : public SystemFrameGrabber {
public:
  SystemCameraGrabber() {}
  bool startGrabbing() {
    return true;
  }
  bool getNextFrames(std::vector<cv::Mat> *res) {
    return true;
  }
  // Returns a Mat, capable of storing the frames grabbed.
  //
  std::vector<cv::Mat> getContainer() {
    return std::vector<cv::Mat>(c_maxCamerasToUse);
  }
private:
  static const int c_maxCamerasToUse = 4;
};

#endif  // PYLON INSTALLED

#endif  // SYS_CAMERA_GRABBER_H__