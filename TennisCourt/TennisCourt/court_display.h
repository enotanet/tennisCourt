#ifndef COURT_DISPLAY_H__
#define COURT_DISPLAY_H__

#include "utils.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

const double k_foot = 0.3048;

class CourtDisplay {
public:
  // Constructs a CourtDisplay based on the image provided in filename.
  //
  CourtDisplay(std::string filename);
  // Returns a frame with the marked position of the ball and players.
  // Coordinates are provided in meters based on the centre of the tennis court.
  //
  bool display(cv::Point2d ball, std::vector<cv::Point2d> players, cv::Mat *res);
  // Runs a simple calibration program, with which the user can align the
  // corners of the tennis court. If we are always using the same image,
  // extract parameters as a config file.
  //
  void calibrate();
private:
  cv::Point2i convertCoordinate(cv::Point2d point);
  cv::Mat image;
  // Doubles?
  // Court size in pixels inside the image. This is the longer side.
  //
  int court_size;
  // Centre of the tennis court in the image.
  //
  cv::Point2i centre;
};

#endif  // COURT_DISPLAY_H__
