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
#include <queue>
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
      const std::deque<std::pair<int, cv::Point2f>> &points,
      const std::vector<ParabolaTraj2d> &trajectories);
  bool EvaluateAt(double t, cv::Point2d *p);
  double ConfidenceAt(double t);
private:
  double ParabolaDistance(const std::vector<ParabolaTraj2d> &trajectories, int a, int b);

  // Satisfied sets for all trajectories. Expensive!
  //
  std::vector<std::set<int>> satisfied;
  std::vector<ParabolaTraj2d> path;
  std::vector<int> goodPoints;
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

class SingleCameraProcessor {
public:
  SingleCameraProcessor(size_t id, const CalibratedCamera &calib);
  // Main function that is responsible for aggregating results.
  //
  bool addFrame(cv::Mat frame);

  // How certain are we that this camera can approximate the
  // ball at the given time.
  //
  double trajectoryConfidence(double time);

  bool vectorToBall(double time, cv::Point3d *A, cv::Point3d *B);

  bool vectorToPlayer(double time, cv::Point3d *A, cv::Point3d *B);

  bool NoFutureBalls(double time);

  void DrawBallPositions(int time, cv::Mat frame);

private:
  bool BallPositionAt(double time, cv::Point2d *ballpos);

  bool PlayerPositionAt(double time, cv::Point2d *playerpos);

  void ComputeTrajectories();

  bool GetRay(cv::Point2d frame_pos, cv::Point3d *a, cv::Point3d *b);

  bool AddBallPosition(int frame_no, cv::Point2f ballpos);

  bool AddBallCandidate(int frame_no, cv::Point2f ballcand);

  bool RemoveBallCandidate(int frame_no);

  void normalise();

  static const int frames_to_keep = 128;
  
  static const int recompute_steps = 32;
  // Current moment in time.
  // Keep information for about 128 frames?
  //
  int count;
  
  cv::Mat P;
  cv::Point3d C;
  cv::Mat Pinv;

  BallFinder ballFinder;
  std::deque<std::pair<int, cv::Point2f>> ballPositionsOrig2d;
  std::deque<std::pair<int, cv::Point2f>> ballPositions2d;
  std::deque<std::pair<int, cv::Point2f>> ballCandidates2d;
  // Only use the centre?
  std::deque<std::pair<int, object>> playerPosition2d;
  // std::deque<std::pair<int, cv::Point2f>> playerPosition2d;

  std::set<int> framesWithBalls;
  std::deque<std::pair<int, cv::Point2f>> ytime;
  std::deque<std::pair<int, cv::Point2f>> ytimeTent;
  std::deque<std::pair<int, cv::Point2f>> xtime;
  std::deque<std::pair<int, cv::Point2f>> xtimeTent;
  std::vector<ParabolaTraj2d> tempTrajectories;
  LinkedParabolaTrajectory2d traj;
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
  static const int max_frames_to_keep = (1 << 7);
  std::vector<OutputResult> results;
};

class FrameProcessor {
public:
  FrameProcessor(size_t frame_number) {}
  FrameProcessor(size_t frame_number, const CalibratedCamera &calib) {
    for (size_t i = 0; i < frame_number; ++i) {
      processors.emplace_back(i, calib);
    }
  }

  // Maybe bools?
  // Needs state! Wrap in a class.
  //
  void ProcessFrames(std::vector<cv::Mat> frames,
                     OutputResult *outputResult);

  bool GetBallPosition(double time, cv::Point3d *ballpos);

  bool NoFutureBalls(double time);

  bool GetPlayersPositions(double time, std::vector<cv::Point3d> *players);

  int GetFrameCount() {
    return count;
  }

  void DrawBallPositions(int time, std::vector<cv::Mat> frames);
private:
  bool ComputeTrajectories(int i, std::vector<ParabolaTraj2d> *trajectories);

  std::vector<SingleCameraProcessor> processors;

  int count;
};

bool MatchXYTrajectories(const std::vector<std::pair<int, cv::Point2f>> &xtime,
                         const std::vector<ParabolaTraj> &xtrajectories,
                         const std::vector<std::pair<int, cv::Point2f>> &ytime,
                         const std::vector<ParabolaTraj> &ytrajectories,
                         std::vector<ParabolaTraj2d> *trajectories);

bool ParabolaSatisfiedSet(const std::deque<std::pair<int, cv::Point2f>> &ballPositions,
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
void BestTrajectories(const std::deque<std::pair<int, cv::Point2f>> &ballPositions,
                      const std::deque<std::pair<int, cv::Point2f>> &tentative,
                      std::vector<ParabolaTraj> *trajectories,
                      int max_window_size = 20,
                      int good_trajectory_threshold = 12,
                      double trajectory_eps = 1.2);

void InitialiseOutput(size_t windowCount);

void DisplayOutput(OutputResult output);

// fp should be const. Requires some code changes.
//
double BallSpeedAt(FrameProcessor &fp, double time);

#endif  // ANALYSIS_SYSTEM_H__
