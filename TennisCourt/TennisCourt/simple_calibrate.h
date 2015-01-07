#ifndef SIMPLE_CALIBRATE_H__
#define SIMPLE_CALIBRATE_H__

#include "sys_frame_grabber.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

bool ExtractCorners(std::string corners_file, SystemFrameGrabber *grabber,
                    std::string output_file);

bool EvaluateCalib(std::string corn_out, SystemFrameGrabber *grabber);

std::vector<std::vector<std::pair<cv::Point2d, cv::Point3d>>> CalibReadFile(
    std::string corn_out);

class CalibratedCamera {
public:
  CalibratedCamera() : calibrated(false) {}
  CalibratedCamera(std::vector<std::vector<std::pair<cv::Point2d, cv::Point3d>>> pairs);
  bool GetRay(size_t cam, cv::Point2d frame_pos, cv::Point3d *a, cv::Point3d *b);
  cv::Point2d Project(size_t cam, cv::Point3d X);
private:
  void Calibrate(std::vector<std::vector<std::pair<cv::Point2d, cv::Point3d>>> pairs);

  bool calibrated;
  std::vector<cv::Mat> P;
  std::vector<cv::Point3d> C;
  std::vector<cv::Mat> Pinv;
};

#endif  // SIMPLE_CALIBRATE_H__
