#ifndef ANALYSIS_SYSTEM_H__
#define ANALYSIS_SYSTEM_H__

#include "camera_location.h"
#include "sys_frame_grabber.h"
#include "ballFinder.h"
#include "utils.h"
#include "simple_calibrate.h"
#include "court_display.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <set>

// This is getting clumsy and memory-heavy.
//
struct ParabolaTraj2d {
  double xa;
  double xb;
  double xc;

  double ya;
  double yb;
  double yc;

  int first;
  int last;
  int initial;

  cv::Point2d evaluate(double t) const {
    return cv::Point2d(xa * t * t + xb * t + xc, ya * t * t + yb * t + yc);
  }
};

class LinkedParabolaTrajectory2d {
public:
  LinkedParabolaTrajectory2d() {}
  LinkedParabolaTrajectory2d(
      const std::vector<std::pair<int, cv::Point2f>> &points,
      const std::vector<ParabolaTraj2d> &trajectories);
  bool EvaluateAt(double t, cv::Point2d *p);
private:
  double ParabolaDistance(const std::vector<ParabolaTraj2d> &trajectories, int a, int b);

  // Satisfied sets for all trajectories. Expensive!
  //
  std::vector<std::set<int>> satisfied;
  std::vector<ParabolaTraj2d> path;
};

struct ParabolaTraj {
  double a;
  double b;
  double c;

  // First and last point indexes used to approximate the parabola.
  //
  int first;
  int last;
  
  // Index of the initial point used to start the trajectory calculation.
  //
  int initial;
};

void RunOnlineSystem(SystemFrameGrabber *grabber);

void RunOfflineSystem(SystemFrameGrabber *grabber);

struct OutputResult {
  std::vector<cv::Mat> images;
};

// Buffer that holds everything produced when processing frames.
// TODO(perf): Will end up consuming too much memory. Keep a limited #frames,
// and recalculate some frames.
class OutputBuffer {
public:
  OutputBuffer() : lastFrame(-1), results(max_frames_to_keep) {};

  OutputResult get(long long timestamp);
  void put(OutputResult result);
  long long lastFrame;

private:
  // An overkill.
  //
  static const int max_frames_to_keep = (1 << 8);
  std::vector<OutputResult> results;
};

class FrameProcessor {
public:
  FrameProcessor(size_t frame_number) : ballFinders(frame_number),
                                        cameraLocations(frame_number),
                                        ballPositions2d(frame_number),
                                        origBallPositions2d(frame_number),
                                        ballCandidates2d(frame_number),
                                        count(0),
                                        oa(-1),
                                        ytime(frame_number),
                                        ytimeTent(frame_number),
                                        xtime(frame_number),
                                        xtimeTent(frame_number),
                                        traj(frame_number) {}
  // Maybe bools?
  // Needs state! Wrap in a class.
  //
  void ProcessFrames(std::vector<cv::Mat> frames,
                     OutputResult *outputResult,
                     CalibratedCamera *calib = nullptr,
                     CourtDisplay *displ = nullptr,
                     CourtDisplay *displ2 = nullptr);

private:
  bool ComputeTrajectories(int i, std::vector<ParabolaTraj2d> *trajectories);

  std::vector<BallFinder> ballFinders;
  std::vector<CameraLocation> cameraLocations;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> ballPositions2d;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> origBallPositions2d;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> ballCandidates2d;
  int count;
  double oa, ob, oc;
  ParabolaTraj2d parab;
  std::set<int> framesWithBalls;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> ytime;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> ytimeTent;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> xtime;
  std::vector<std::vector<std::pair<int, cv::Point2f>>> xtimeTent;
  std::vector<LinkedParabolaTrajectory2d> traj;
};

bool MatchXYTrajectories(const std::vector<std::pair<int, cv::Point2f>> &xtime,
                         const std::vector<ParabolaTraj> &xtrajectories,
                         const std::vector<std::pair<int, cv::Point2f>> &ytime,
                         const std::vector<ParabolaTraj> &ytrajectories,
                         std::vector<ParabolaTraj2d> *trajectories);

bool ParabolaSatisfiedSet(const std::vector<std::pair<int, cv::Point2f>> &ballPositions,
                          const int min_index,
                          const int max_index,
                          const ParabolaTraj &parab,
                          const double trajectory_eps,
                          std::vector<int> *satisfied,
                          double *dist_sum);

double PointToParabolaDistance(cv::Point2d p, const ParabolaTraj &parab);

bool getParabola(std::vector<cv::Point2d> points, ParabolaTraj* parab);

// Approximates the trajectory of a point moving in 2d space with a parabola.
// This could be a projection of the ball inside a plane.
//
void BestTrajectories(const std::vector<std::pair<int, cv::Point2f>> &ballPositions,
                      const std::vector<std::pair<int, cv::Point2f>> &tentative,
                      std::vector<ParabolaTraj> *trajectories,
                      int max_window_size = 20,
                      int good_trajectory_threshold = 12,
                      double trajectory_eps = 1.2);

void InitialiseOutput(size_t windowCount);

void DisplayOutput(OutputResult output);

#endif  // ANALYSIS_SYSTEM_H__
