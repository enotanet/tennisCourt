#include "analysis_system.h"
#include "camera_location.h"
#include "sys_frame_grabber.h"
#include "utils.h"
#include "simple_calibrate.h"
#include "court_display.h"
#include "courtDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <set>
#include <thread>

#include <fstream>
#include <ctime>

const double k_fps = 60;
const double comp_eps = 1e-6;

template<typename T>
bool myComp(const std::pair<int, T> &a, const std::pair<int, T> &b) {
  return a.first < b.first;
};

double BallSpeedAt(FrameProcessor &fp, double time) {
  cv::Point3d cur, prev;
  if (!fp.GetBallPosition(time, &cur)) {
    return nan("");
  }
  if (!fp.GetBallPosition(time - 1, &prev)) {
    return nan("");
  }
  double dist = cv::norm(cur - prev);
  return dist * k_fps;
}

bool is_bounce(const std::deque<std::pair<int, cv::Point3d>> &ballpos,
               std::pair<double, cv::Point3d> cand) {
  if (cand.second.z > 0.18) {
    // Not a bounce. Over 18 cm
    return false;
  }
  auto beg = std::lower_bound(ballpos.begin(), ballpos.end(),
                std::make_pair((int) cand.first - 5, cv::Point3d()),
                myComp<cv::Point3d>);
  auto end = std::upper_bound(ballpos.begin(), ballpos.end(),
                std::make_pair((int) cand.first + 5, cv::Point3d()),
                myComp<cv::Point3d>);
  for (auto it = beg; it != end; ++it) {
    if (cand.second.z > it->second.z + comp_eps) {
      return false;
    }
  }
  return true;
}

bool is_hit(const std::deque<std::pair<int, cv::Point3d>> &ballpos,
               std::pair<double, cv::Point3d> cand) {
  auto beg = std::lower_bound(ballpos.begin(), ballpos.end(),
                std::make_pair((int) cand.first - 5, cv::Point3d()),
                myComp<cv::Point3d>);
  auto end = std::upper_bound(ballpos.begin(), ballpos.end(),
                std::make_pair((int) cand.first + 5, cv::Point3d()),
                myComp<cv::Point3d>);
  bool t1 = true, t2 = true;
  // Can be improved taking into account side of court. I.e. sign of x.
  //
  for (auto it = beg; it != end; ++it) {
    if (it->second.x + comp_eps < cand.second.x) {
      t1 = false;
    }
    if (it->second.x > cand.second.x + comp_eps) {
      t2 = false;
    }
  }

  return t1 | t2;
}

// Intercepts key presses and does stuff. Needs to remember previous state
// for everything that we output. Introduce MyImageShow that has a frame buffer 
// for each named window that we have.
//
void RunOfflineSystem(SystemFrameGrabber *grabber) {
  INFO("Running Offline System");
  // What about ownership of grabber? Should be system-wide.
  //
  OutputBuffer outputBuffer;

  std::vector<cv::Mat> frames = grabber->getContainer();
  InitialiseOutput(frames.size() + 1);
  grabber->getNextFrames(&frames);

  std::vector<std::vector<std::pair<cv::Point2f, cv::Point3d>>> corn(frames.size());
  if (g_args["calibrate"].size() < 1) {
    for (size_t i = 0; i < frames.size(); ++i) {
      cv::Mat grayFrame = frames[i].clone();
      cv::cvtColor(frames[i], grayFrame, CV_BGR2GRAY);
      // grayFrame = grayFrame.clone();
      diplayCourtDetectorResult(grayFrame);
      corn[i] = mapCourtCornersTo3DCoordinates(grayFrame, getCourtCorners(grayFrame), 1);
      for (auto y : corn[i]) {
        DEBUG("MATCH " << y.first << " " << y.second);
      }
      DEBUG("\n\n");
      cv::imwrite("wtf.png", grayFrame);
    }
  } else {
    corn = CalibReadFile(g_args["calibrate"][0]);
  }

  CalibratedCamera calib(corn);
  FrameProcessor proc(frames.size(), calib);
  if (g_args["cd"].size() < 1) {
    g_args["cd"].push_back("");
  }
  CourtDisplay displ("TennisCourt/TestData/Court/courtFromTop.jpg", g_args["cd"][0]);

  INFO("Found " << frames.size() << " things");

  // TODO(GUI): Introduce some nice slider and stuff!
  //
  char last_key = 'n';
  long long frame_position = -1;
  long long last_ballpos = 0;
  std::deque<std::pair<int, cv::Point3d>> ballpositions;
  while (last_key != 'q' && last_key != 27) {
    switch (last_key) {
    case 'p':
      if (frame_position > 0) {
        --frame_position;
      }
      break;
    case 's':
      frame_position += 50;
      break;
    default:
      // Default behaviour is advance one frame.
      //
      ++frame_position;
      break;
    }

    while (frame_position + 64 > outputBuffer.lastFrame) {
      if (!grabber->getNextFrames(&frames)) {
        DEBUG("Couldn't grab frames?!?");
        break;
      }
      OutputResult res;
      proc.ProcessFrames(frames, &res);
      outputBuffer.put(res);
      DEBUG("Putting " << res.images.size() << " images at position "
            << outputBuffer.lastFrame);
    }
    while (frame_position + 5 > last_ballpos) {
      cv::Point3d ballpos;
      if (proc.GetBallPosition(last_ballpos, &ballpos)) {
        ballpositions.emplace_back(last_ballpos, ballpos);
      }
      ++last_ballpos;
    }
    while (ballpositions.size() > 1024) {
      ballpositions.pop_front();
    }
    cv::Point3d ballpos(-200, -200, -200);
    auto it = std::lower_bound(ballpositions.begin(), ballpositions.end(),
                  std::make_pair(frame_position, cv::Point3d()), myComp<cv::Point3d>);
    if (it != ballpositions.end() && it->first == frame_position) {
      ballpos = it->second;
    }
    bool bounce;
    bool hit;
    bool net;
    if (ballpos.x < -100) {
      bounce = hit = false;
      net = false;
    } else {
      bounce = is_bounce(ballpositions,
                std::make_pair(frame_position, ballpos));
      hit = is_hit(ballpositions,
                std::make_pair(frame_position, ballpos));
      net = proc.NoFutureBalls(frame_position);
      // Also add checks for proximity to the net.
      // NoFutureBalls returns true too often.
      //
      if (std::abs(ballpos.x) > 1) {
        net = false;
      }
      if (std::abs(ballpos.z) > 1.2) {
        net = false;
      }
    }

    DEBUG("Frame " << frame_position << " Bounce: " << bounce <<
          " Hit: " << hit << " Net: " << net);

    std::vector<cv::Point3d> players;
    proc.GetPlayersPositions(frame_position, &players);
    if (g_args.count("display")) {
      OutputResult res;
      res.images.resize(outputBuffer.get(frame_position).images.size());
      cv::Scalar ballcol(255 * bounce, 255, 255 * hit);
      for (size_t i = 0; i < outputBuffer.get(frame_position).images.size(); ++i) {
        cv::cvtColor(outputBuffer.get(frame_position).images[i], res.images[i],
            CV_GRAY2BGR);
        if (ballpos.x > -100) {
          cv::circle(res.images[i], calib.Project(i, ballpos),
              2, ballcol, -1, 8);
        }
        for (auto pl : players) {
          cv::circle(res.images[i], calib.Project(i, pl),
              4, cv::Scalar(0, 255, 255), -1, 8);
        }
      }
      DEBUG("I have " << outputBuffer.get(frame_position).images.size() << " images "
            << "at step " << frame_position << " " << outputBuffer.lastFrame
            << bounce << " " << hit);
      proc.DrawBallPositions(frame_position, res.images);

      res.images.push_back(cv::Mat::zeros(1, 1, CV_8U));
      std::vector<cv::Point2d> players2d;
      for (auto pl : players) {
        players2d.emplace_back(pl.x, pl.y);
      }
      displ.display(cv::Point2d(ballpos.x, ballpos.y), players2d, &res.images.back());

      DisplayOutput(res);
      last_key = cv::waitKey(0);
    }
  }
  const double comp_eps = 1e-9;
  const double frames_per_second = 60;
  int last_player_hit = -1000;
  std::ofstream fout("annotation.txt");
  for (size_t i = 0; i < ballpositions.size(); ++i) {
    // fout << ballpositions[i].first << " " << ballpositions[i].second << std::endl;
    if (i > 0 && i + 1 < ballpositions.size()) {
      if (ballpositions[i].second.z + comp_eps < ballpositions[i - 1].second.z
          && ballpositions[i].second.z + comp_eps < ballpositions[i + 1].second.z
          && ballpositions[i].second.z < 0.18) {
        // Report ball bounce.
        //
        DEBUG("Ball bounced at position " << ballpositions[i].second);
        
        fout << "bounce " << (double) ballpositions[i].first / frames_per_second
            << " at " << ballpositions[i].second << " ; " << ballpositions[i].first
            << std::endl;
      }
      if ((ballpositions[i].second.x + comp_eps < ballpositions[i - 1].second.x
          && ballpositions[i].second.x + comp_eps < ballpositions[i + 1].second.x)
        || (ballpositions[i].second.x < ballpositions[i - 1].second.x + comp_eps
          && ballpositions[i].second.x < ballpositions[i + 1].second.x + comp_eps)) {
          
        if (ballpositions[i].first - last_player_hit < 20) {
          // Report ball bounce.
          //
          DEBUG("Ball hit by player " << ballpositions[i].second);
          
          fout << "player hit " << (double) ballpositions[i].first / frames_per_second
              << " at " << ballpositions[i].second << " ; " << ballpositions[i].first
              << std::endl;
          last_player_hit = ballpositions[i].first;
        }
      }
      if (i % 120 == 0) {
        auto trans = ballpositions[i].second - ballpositions[i - 1].second;
        double speed = cv::norm(trans) * frames_per_second;
        fout << "ball speed at " << (double) ballpositions[i].first / frames_per_second
             << " is " << speed << " at pos " << ballpositions[i].second << std::endl;
      }
    }
  }
}

void InitialiseOutput(size_t windowCount) {
  for (size_t i = 0; i < windowCount; ++i) {
    char name[32];
    sprintf(name, "Window %lu", i);
    cv::namedWindow(name, CV_WINDOW_NORMAL);
  }
}

void DisplayOutput(OutputResult output) {
  for (size_t i = 0; i < output.images.size(); ++i) {
    char name[32];
    sprintf(name, "Window %lu", i);
    cv::imshow(name, output.images[i]);
  }
}

OutputResult OutputBuffer::get(long long timestamp) {
  return results[timestamp % max_frames_to_keep];
}

// A lot of copying going around. Mat make break things.
//
void OutputBuffer::put(OutputResult res) {
  ++lastFrame;
  results[lastFrame % max_frames_to_keep] = res;
}

bool IntersectIn3d(size_t c1, cv::Point2d p1, size_t c2, cv::Point2d p2,
    CalibratedCamera *calib, cv::Point3d *res) {
  cv::Point3d A1, B1, A2, B2;
  if (!calib->GetRay(c1, p1, &A1, &B1)) {
    return false;
  }
  if (!calib->GetRay(c2, p2, &A2, &B2)) {
    return false;
  }
  *res = LineIntersect(A1, (B1 - A1), A2, (B2 - A2));
  return true;
}

void dummy(SingleCameraProcessor *proc, cv::Mat *frame) {
  proc->addFrame(*frame);
}

void FrameProcessor::DrawBallPositions(int time, std::vector<cv::Mat> frames) {
  for (size_t i = 0; i < frames.size(); ++i) {
    processors[i].DrawBallPositions(time, frames[i]);
  }
}

// TODO: Main function that deals with processing each set of frames.
// Put the results in outputResult so it can be used to display now or later
//
void FrameProcessor::ProcessFrames(std::vector<cv::Mat> frames,
    OutputResult *outputResult) {
  std::vector<std::thread> exec;
  for (size_t i = 0; i < frames.size(); ++i) {
    exec.emplace_back(dummy, &processors[i], &frames[i]);
  }
  for (size_t i = 0; i < exec.size(); ++i) {
    exec[i].join();
  }
  if (g_args.count("display")) {
    for (size_t i = 0; i < frames.size(); ++i) {
      outputResult->images.push_back(frames[i].clone());
    }
  }
  // DEBUG("Put " << outputResult->images.size() << " images in result set at count "
  //       << count);
  ++count;
  return;
}

bool FrameProcessor::GetBallPosition(double time, cv::Point3d *ballpos) {
  if (processors.size() < 2) {
    return false;
  }
  // std::vector<std::pair<double, size_t>> confidences;
  std::vector<double> confidences(processors.size());
  for (size_t i = 0; i < processors.size(); ++i) {
    confidences[i] = processors[i].trajectoryConfidence(time);
    // confidences.emplace_back(processors[i].trajectoryConfidence(time), i);
  }
  // sort(confidences.begin(), confidences.end());
  int cam0, cam1;
  cam0 = 0;
  cam1 = 1;
  if (processors.size() >= 4) {
    if (confidences[0] * confidences[1] < confidences[2] * confidences[3]) {
      cam0 = 2;
      cam1 = 3;
    }
  }
  cv::Point3d A1, B1, A2, B2;
  int iter = 0, recalc = 1;
  bool ok = false;
  while (iter < 3 && recalc) {
    recalc = false;
    ++iter;
    bool succ = true;
    if (!processors[cam0].vectorToBall(time, &A1, &B1)) {
      succ = false;
    }
    if (!processors[cam1].vectorToBall(time, &A2, &B2)) {
      succ = false;
    }
    if (succ) {
      *ballpos = LineIntersect(A1, (B1 - A1), A2, (B2 - A2));
    }
    if (processors.size() == 4) {
      if ((!succ || ballpos->x < 0) && cam0 + cam1 == 5) {
        cam0 = 0;
        cam1 = 1;
        recalc = true;
      } else if ((!succ || ballpos->x > 0) && cam0 + cam1 == 1) {
        cam0 = 2;
        cam1 = 3;
        recalc = true;
      }
    }
    ok |= succ;
  }
  return ok;
}

bool FrameProcessor::NoFutureBalls(double time) {
  for (size_t i = 0; i < processors.size(); ++i) {
    if (!processors[i].NoFutureBalls(time)) {
      return false;
    }
  }
  return true;
}
// Two player detection requires both cameras.
//
bool FrameProcessor::GetPlayersPositions(double time, std::vector<cv::Point3d> *players) {
  if (processors.size() < 2) {
    return false;
  }
  players->resize(processors.size() / 2);
  for (size_t i = 0; i + 1 < processors.size(); i += 2) {
    cv::Point3d A1, B1, A2, B2;
    if (!processors[i].vectorToPlayer(time, &A1, &B1)) {
      return false;
    }
    if (!processors[i + 1].vectorToPlayer(time, &A2, &B2)) {
      return false;
    }
    (*players)[i / 2] = LineIntersect(A1, (B1 - A1), A2, (B2 - A2));
  }
  return true;
}

bool MatchXYTrajectories(const std::deque<std::pair<int, cv::Point2f>> &xtime,
                         const std::vector<ParabolaTraj> &xtrajectories,
                         const std::deque<std::pair<int, cv::Point2f>> &ytime,
                         const std::vector<ParabolaTraj> &ytrajectories,
                         std::vector<ParabolaTraj2d> *trajectories) {
  const int min_common_points = 8;
  const double trajectory_eps = 2.2;
  trajectories->clear();
  for (size_t i = 0; i < xtrajectories.size(); ++i) {
    // Optimise second loop to do a lookup instead of linear search.
    //
    for (size_t j = 0; j < ytrajectories.size(); ++j) {
      // I might need to relax this to some [first, last] intersection
      //
      if (xtrajectories[i].initial == ytrajectories[j].initial) {
        double dist_sum;
        std::vector<int> xsat;
        ParabolaSatisfiedSet(xtime, xtrajectories[i].first, xtrajectories[i].last,
                             xtrajectories[i], trajectory_eps, &xsat, &dist_sum);
        std::vector<int> ysat;
        ParabolaSatisfiedSet(ytime, ytrajectories[j].first, ytrajectories[j].last,
                             ytrajectories[j], trajectory_eps, &ysat, &dist_sum);
        // Determine set satisfied by both.
        //
        std::set<int> xFrames;
        for (int s : xsat) {
          xFrames.insert(xtime[s].first);
        }

        std::set<int> yFrames;
        for (int s : ysat) {
          yFrames.insert(ytime[s].first);
        }

        int common = 0;
        int first;
        int last;
        for (int s : xFrames) {
          if (yFrames.find(s) != yFrames.end()) {
            if (!common) {
              first = s;
            }
            ++common;
            last = s;
          }
        }
        
        if (common >= min_common_points) {
          ParabolaTraj2d parab;
          parab.xa = xtrajectories[i].a;
          parab.xb = xtrajectories[i].b;
          parab.xc = xtrajectories[i].c;

          parab.ya = ytrajectories[j].a;
          parab.yb = ytrajectories[j].b;
          parab.yc = ytrajectories[j].c;
          
          parab.first = first;
          parab.last = last;
          parab.initial = xtrajectories[i].initial;

          trajectories->push_back(parab);
        }
      }
    }
  }
  return true;
}

// Returns the set of points within min_index, max_index that are within trajectory_eps
// of the parabola.
//
bool ParabolaSatisfiedSet(const std::deque<std::pair<int, cv::Point2f>> &ballPositions,
                          const int min_index,
                          const int max_index,
                          const ParabolaTraj &parab,
                          const double trajectory_eps,
                          std::vector<int> *satisfied,
                          double *dist_sum) {
  *dist_sum = 0;
  for (size_t i = 0; i < ballPositions.size(); ++i) {
    if (ballPositions[i].first < min_index) {
      continue;
    }
    if (ballPositions[i].first > max_index) {
      continue;
    }
    double trajectory_dist = PointToParabolaDistance(ballPositions[i].second, parab);
    if (trajectory_dist < trajectory_eps) {
      satisfied->push_back(i);
      *dist_sum += trajectory_dist;
    } else {
      *dist_sum += trajectory_eps;
    }
  }
  return true;
}

// Approximates the trajectory of a point moving in 2d space with a parabola.
// This could be a projection of the ball inside a plane.
// Currently running in the projection plane of the camera, ignoring distortion.
// Should lead to better results if run inside the plane in which we assume the ball
// flies. That is, the plane defined by 3 points of the ball path, which best fits
// a range of ball positions.
//
// Assumes perfect ball detection. On the other hand, a modification of this algorithm
// could be used to detect a ball-like trajectory from many candidates.
// TODO: test if this gives good results for camera projection plane, can improve ball
// detection.
// TODO: After projecting the points inside the plane where the ball flies, we can
// assume a constant acceleration moving object as suggested in a paper by Fei.
//
void BestTrajectories(const std::deque<std::pair<int, cv::Point2f>> &ballPositions,
                      const std::deque<std::pair<int, cv::Point2f>> &tentative,
                      std::vector<ParabolaTraj> *trajectories,
                      int max_window_size,
                      int good_trajectory_threshold,
                      double trajectory_eps) {
  // Tries to fit a parabola that approximates all points inside the window.
  //
  trajectories->clear();
  for (size_t mid = 1; mid + 1 < ballPositions.size(); ++mid) {
    ParabolaTraj cur;
    ParabolaTraj best;
    std::vector<cv::Point2d> points;
    
    // Get initial parabala around midpoint.
    //
    points.push_back(ballPositions[mid - 1].second);
    points.push_back(ballPositions[mid].second);
    points.push_back(ballPositions[mid + 1].second);
    if (!getParabola(points, &cur)) {
      DEBUG("WTF points " << mid << " : "
          << points[0] << " " << points[1] << " " << points[2]);
      continue;
    }
    cur.initial = ballPositions[mid].first;
    cur.first = ballPositions[mid - 1].first;
    cur.last = ballPositions[mid + 1].first;

    double best_satisfied = 0;
    double best_dist = 1e100;
    double cur_satisfied = 0;
    double dist_sum = 1e100;
    do {
      best_satisfied = cur_satisfied;
      best_dist = dist_sum;
      // Get the set of satisfied ball positions.
      //
      std::vector<int> satisfied;
      ParabolaSatisfiedSet(ballPositions,
                           ballPositions[mid].first - max_window_size,
                           ballPositions[mid].first + max_window_size,
                           cur,
                           trajectory_eps,
                           &satisfied,
                           &dist_sum);
      
      if (satisfied.size() < 3) {
        break;
      }
      cur_satisfied = satisfied.size();
      auto beg = std::lower_bound(tentative.begin(), tentative.end(),
          std::make_pair(ballPositions[mid].first - max_window_size, cv::Point2f(0, 0)),
          myComp<cv::Point2f>);
      auto end = std::upper_bound(tentative.begin(), tentative.end(),
          std::make_pair(ballPositions[mid].first + max_window_size, cv::Point2f(0, 0)),
          myComp<cv::Point2f>);
      for (auto it = beg; it != end; ++it) {
        if (PointToParabolaDistance(it->second, cur) < trajectory_eps) {
          cur_satisfied += 0.7;
        }
      }
      if (cur_satisfied > best_satisfied && dist_sum < best_dist) {
        best = cur;
        // Update.
        assert(points.size() == 3);
        int fi = ballPositions[satisfied[0]].first;
        int la = ballPositions[satisfied.back()].first;
        // Find a candidate for the middle of the parabola that is about
        // as far from both endpoints in the satisfied set.
        //
        int mid_cand = satisfied[0];
        int cost = abs(abs(ballPositions[mid_cand].first - fi)
                       - abs(ballPositions[mid_cand].first - la));
        for (size_t i = 0; i < satisfied.size(); ++i) {
          int c_cost = abs(abs(ballPositions[satisfied[i]].first - fi)
                           - abs(ballPositions[satisfied[i]].first - la));
          if (c_cost < cost) {
            cost = c_cost;
            mid_cand = satisfied[i];
          }
        }
        points[0] = ballPositions[satisfied[0]].second;
        points[1] = ballPositions[mid_cand].second;
        points[2] = ballPositions[satisfied.back()].second;
        getParabola(points, &cur);

        cur.first = ballPositions[satisfied[0]].first;
        cur.last = ballPositions[satisfied.back()].first;
      }
    } while (cur_satisfied > best_satisfied && dist_sum < best_dist);
    // DEBUG("Trajectory " << best.a << " " << best.b << " " << best.c
    //       << " satisfies " << best_satisfied << " points");
    if (best_satisfied >= good_trajectory_threshold) {
      trajectories->push_back(best);
    }
  }
}

double PointToParabolaDistance(cv::Point2d p, const ParabolaTraj &parab) {
  return PointToParabolaDistance(p, parab.a, parab.b, parab.c);
}

bool getParabola(std::vector<cv::Point2d> points, ParabolaTraj* parab) {
  return getParabola(points, &parab->a, &parab->b, &parab->c);
}

void RunOnlineSystem(SystemFrameGrabber *grabber) {
  INFO("Running Online System");
  // What about ownership of grabber? Should be system-wide.
  //
  OutputBuffer outputBuffer;

  std::vector<cv::Mat> frames = grabber->getContainer();
  // InitialiseOutput(frames.size());
  FrameProcessor proc(frames.size());

  INFO("Found " << frames.size() << " things");

  // TODO(GUI): Introduce some nice slider and stuff!
  //
  long long start = clock();
  long long frame_position = -1;
  while (clock() - start < 5LL * CLOCKS_PER_SEC) {
    ++frame_position;

    if (outputBuffer.lastFrame < frame_position) {
      grabber->getNextFrames(&frames);
      OutputResult res;
      res.images = frames;
      // proc.ProcessFrames(frames, res);
      outputBuffer.put(res);
    }
    // DisplayOutput(outputBuffer.get(frame_position));
    // last_key = cv::waitKey(0);
  }
  DEBUG(frame_position << " frames recorded!");

  InitialiseOutput(frames.size());

  char last_key = 'n';
  frame_position = -1;
  while (last_key != 'q' && last_key != 27) {
    switch (last_key) {
    case 'p':
      if (frame_position > 0) {
        --frame_position;
      }
      break;
    default:
      // Default behaviour is advance one frame.
      //
      ++frame_position;
      break;
    }

    DisplayOutput(outputBuffer.get(frame_position));
    last_key = cv::waitKey(0);
  }

  cv::waitKey(0);
}

LinkedParabolaTrajectory2d::LinkedParabolaTrajectory2d(
    const std::deque<std::pair<int, cv::Point2f>> &points,
    const std::vector<ParabolaTraj2d> &trajectories) {
  DEBUG("Doing dejkstra with " << trajectories.size() << " trajectories");
  std::vector<double> dist(trajectories.size(), 1e100);
  dist[0] = 0;
  std::vector<char> used(trajectories.size(), 0);
  std::vector<int> prev(trajectories.size(), -1);
  satisfied.resize(trajectories.size());
  const double trajectory_eps = 2.4;

  // Satisfied array ready.
  //
  for (size_t i = 0; i < trajectories.size(); ++i) {
    for (auto p : points) {
      if (p.first >= trajectories[i].first && p.first <= trajectories[i].last) {
        if (cv::norm(trajectories[i].evaluate(p.first) -
                  cv::Point2d(p.second.x, p.second.y))
              < trajectory_eps) {
          satisfied[i].insert(p.first);
        }
      }
    }
  }

  // 2d dejkstra.
  while (!used[trajectories.size() - 1]) {
    int best = -1;
    for (size_t i = 0; i < trajectories.size(); ++i) {
      if (used[i]) {
        continue;
      }
      if (best == -1 || dist[i] < dist[best]) {
        best = i;
      }
    }
    if (best == -1) {
      break;
    }

    used[best] = 1;
    for (size_t i = 0; i < trajectories.size(); ++i) {
      if (used[i]) {
        continue;
      }
      double cand_dist = dist[best] + ParabolaDistance(trajectories, best, i);
      if (dist[i] > cand_dist) {
        dist[i] = cand_dist;
        prev[i] = best;
      }
    }
  }

  int c = trajectories.size() - 1;
  while (!used[c] && c >= 0) {
    --c;
  }
  while (c >= 0) {
    path.push_back(trajectories[c]);
    c = prev[c];
  }

  // points is sorted by time.
  //
  for (auto p : points) {
    cv::Point2d q;
    if (EvaluateAt(p.first, &q)) {
      if (cv::norm(cv::Point2d(p.second.x, p.second.y) - q) < trajectory_eps) {
        goodPoints.push_back(p.first);
      }
    }
  }
}

bool LinkedParabolaTrajectory2d::EvaluateAt(double t, cv::Point2d *p) {
  // Can be replaced with binary search.
  //
  double min_dist = 1e100;
  size_t best = path.size() - 1;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i].first <= t && t <= path[i].last) {
      *p = path[i].evaluate(t);
      return true;
    }
    if (std::abs(t - path[i].first) < min_dist) {
      min_dist = std::abs(t - path[i].first);
      best = i;
    }
    if (std::abs(t - path[i].last) < min_dist) {
      min_dist = std::abs(t - path[i].last);
      best = i;
    }
  }
  if (path.size()) {
    *p = path[best].evaluate(t);
    return true;
  }
  return false;
}

double LinkedParabolaTrajectory2d::ConfidenceAt(double t) {
  if (!goodPoints.size()) {
    return 1e100;
  }
  // Can be replaced with binary search.
  //
  auto lb = std::lower_bound(goodPoints.begin(), goodPoints.end(), t);
  auto ub = std::upper_bound(goodPoints.begin(), goodPoints.end(), t);
  double best = 1e100;
  if (lb != goodPoints.end()) {
    best = std::min(best, std::abs(t - *lb));
  }
  if (ub != goodPoints.end()) {
    best = std::min(best, std::abs(t - *ub));
  }
  return best;
}

double LinkedParabolaTrajectory2d::ParabolaDistance(
    const std::vector<ParabolaTraj2d> &trajectories, int a, int b) {
  if (trajectories[a].first > trajectories[b].first) {
    std::swap(a, b);
  }
  if (trajectories[a].last > trajectories[b].first) {
    // Trajectories intersecting. Evaluate compatibility.
    //
    for (int i = trajectories[b].first;
         i <= trajectories[a].last && i <= trajectories[b].last; ++i) {
      bool ina = (satisfied[a].find(i) != satisfied[a].end());
      bool inb = (satisfied[b].find(i) != satisfied[b].end());
      if (ina ^ inb) {
        // Trajectories disagree on a point. Better to ignore.
        //
        return 1e100;
      }
    }
    // Trajectories compatible. Cost 0.
    //
    return 0;
  } else {
    // Trajectories disjoint. Find a point which minimises distance.
    //
    double dist = 1e100;
    for (int i = trajectories[a].last; i <= trajectories[b].first; ++i) {
      dist = std::min(dist,
          cv::norm(trajectories[a].evaluate(i) - trajectories[b].evaluate(i)));
    }
    return dist + 3.0 * (trajectories[b].first - trajectories[a].last);
  }
}

SingleCameraProcessor::SingleCameraProcessor(size_t id, const CalibratedCamera &calib) :
    count(0) {
  calib.GetParams(id, &P, &C, &Pinv);
}

bool SingleCameraProcessor::addFrame(cv::Mat frame) {
  std::vector<cv::Point2f> newBallPositions;
  std::vector<object> newPlayers;
  std::vector<cv::Point2f> newBallCandidates;
  if (ballFinder.addFrameNew(frame, newBallPositions, newPlayers, newBallCandidates)) {
    for (auto ballpos : newBallPositions) {
      ballPositionsOrig2d.emplace_back(count, ballpos);
      AddBallPosition(count, ballpos);
    }
    if (newBallPositions.size() > 1) {
      DEBUG("Multiple ball positions at step " << count);
      for (auto ballpos : newBallPositions) {
        DEBUG("Ball at " << ballpos);
      }
      DEBUG(std::endl);
    }
  } else {
    for (auto ballcand : newBallCandidates) {
      AddBallCandidate(count, ballcand);
    }
  }
  const double player_perim_threshold = 75;
  for (auto player : newPlayers) {
    double p = cv::norm(player.top - player.left)
             + cv::norm(player.left - player.bottom)
             + cv::norm(player.bottom - player.right)
             + cv::norm(player.right - player.top);
    DEBUG("Player " << count << " " << p << " " << player.bottom);
    // Only want to detect our player
    //
    if (p > player_perim_threshold) {
      playerPosition2d.push_back({count, player});
    }
  }

  const double point_on_trajectory_eps = 3.3;

  if (count % recompute_steps == 0) {
    ComputeTrajectories();
    if (tempTrajectories.size()) {
      bool change = false;
      for (auto cand : ballCandidates2d) {
        // No need to check the set. Should be consistent.
        //
        for (auto traj : tempTrajectories) {
          cv::Point2d p(cand.second.x, cand.second.y);
          if (cv::norm(traj.evaluate(cand.first) - p) < point_on_trajectory_eps) {
            if (AddBallPosition(cand.first, cand.second)) {
              change = true;
            }
            break;
          }
        }
      }
      if (change) {
        std::sort(ballPositions2d.begin(), ballPositions2d.end(), myComp<cv::Point2f>);
        // ComputeTrajectories();
      }
    }
    // Another 'if' just in case something changed.
    //
    // if (tempTrajectories.size()) {
    //   traj = LinkedParabolaTrajectory2d(ballPositions2d, tempTrajectories);
    // }
  }

  ++count;
  normalise();

  return true;
}

bool SingleCameraProcessor::NoFutureBalls(double time) {
  cv::Point2d ballpos;
  BallPositionAt(time, &ballpos);
  auto it = std::lower_bound(ballPositions2d.begin(), ballPositions2d.end(),
        std::make_pair((int) time + 1, cv::Point2f()), myComp<cv::Point2f>);
  int match = 0;
  int iter = 0;
  for (; it != ballPositions2d.end() && iter < 12; ++iter, ++it) {
    if (it->first > time + 12) {
      break;
    }
    cv::Point2d p(it->second.x, it->second.y);
    if (cv::norm(p - ballpos) < 50) {
      ++match;
    }
  }
  return match < 3;
}

double SingleCameraProcessor::trajectoryConfidence(double time) {
  std::pair<int, cv::Point2f> cand = {(int) time - 6, {}};
  auto a = std::lower_bound(ballPositions2d.begin(),
      ballPositions2d.end(), cand, myComp<cv::Point2f>);
  cand.first = (int) time + 6;
  auto b = std::upper_bound(ballPositions2d.begin(),
      ballPositions2d.end(), cand, myComp<cv::Point2f>);
  return -(b - a);
  return traj.ConfidenceAt(time);
}

bool SingleCameraProcessor::BallPositionAt(double time, cv::Point2d *ballpos) {
  if (ballPositions2d.size() < 2) {
    return false;
  }
  std::pair<int, cv::Point2f> cand = {(int) time, {}};
  auto a = std::lower_bound(ballPositions2d.begin(),
      ballPositions2d.end(), cand, myComp<cv::Point2f>);
  auto b = a - 1;
  if (a == ballPositions2d.begin()) {
    b = a + 1;
  }
  if (a == ballPositions2d.end()) {
    --a;
    b = a - 1;
  } else if (b == ballPositions2d.end()) {
    b = a - 1;
  }
  if (a->first == b->first) {
    DEBUG("Ball positions with equal times!");
    return false;
  }
  double best = std::abs(time - a->first) + std::abs(time - b->first);
  if (best > 15) {
    return false;
  }
  if (best > 10) {
    auto s = a;
    auto x = s;
    int iter = 0;
    while (x != ballPositions2d.begin() && iter < 5) {
      --x;
      ++iter;
    }
    for (iter = 0; x != ballPositions2d.end() && iter < 10; ++x, ++iter) {
      auto y = x + 1;
      if (x != ballPositions2d.end() && y != ballPositions2d.end()) {
        if (std::abs(time - x->first) + std::abs(time - y->first) < best) {
          best = std::abs(time - x->first) + std::abs(time - y->first);
          a = x;
          b = y;
        }
      }
    }
  }
  double A = (a->second.x - b->second.x) / (a->first - b->first);
  double B = a->second.x - A * a->first;
  ballpos->x = A * time + B;

  A = (a->second.y - b->second.y) / (a->first - b->first);
  B = a->second.y - A * a->first;
  ballpos->y = A * time + B;

  return true;
}

bool SingleCameraProcessor::vectorToBall(double time, cv::Point3d *A, cv::Point3d *B) {
  cv::Point2d frame_pos;
  if (!BallPositionAt(time, &frame_pos)) {
    return false;
  }
  return GetRay(frame_pos, A, B);
  // if (!traj.EvaluateAt(time, &frame_pos)) {
  //   return false;
  // }
}

bool SingleCameraProcessor::PlayerPositionAt(double time, cv::Point2d *playerpos) {
  if (!playerPosition2d.size()) {
    return false;
  }
  std::pair<int, object> cand = {(int) time, object()};
  auto a = std::lower_bound(playerPosition2d.begin(), playerPosition2d.end(),
          cand, myComp<object>);
  if (a != playerPosition2d.begin()) {
    --a;
  }
  // Experiment with returning a->second.bottom and midpoint
  //
  // *playerpos = 0.25 * (a->second.top + a->second.bottom +
  //       a->second.right + a->second.left);
  *playerpos = cv::Point2d(a->second.bottom.x, a->second.bottom.y);
  return true;
}

bool SingleCameraProcessor::vectorToPlayer(double time, cv::Point3d *A, cv::Point3d *B) {
  cv::Point2d frame_pos;
  if (!PlayerPositionAt(time, &frame_pos)) {
    return false;
  }
  return GetRay(frame_pos, A, B);
  // if (!traj.EvaluateAt(time, &frame_pos)) {
  //   return false;
  // }
}

void SingleCameraProcessor::DrawBallPositions(int time, cv::Mat frame) {
  for (auto ball : ballPositions2d) {
    if (ball.first < time - 16 || ball.first > time + 16) {
      continue;
    }
    cv::circle(frame, ball.second, 4, cv::Scalar(0, 0, 255), 1, 8);
  }
  for (auto ball : ballPositionsOrig2d) {
    if (ball.first < time - 16 || ball.first > time + 16) {
      continue;
    }
    cv::circle(frame, ball.second, 2, cv::Scalar(255, 0, 0), -1, 8);
  }
}

void SingleCameraProcessor::ComputeTrajectories() {
  tempTrajectories.clear();
  std::vector<ParabolaTraj> ytrajectories;
  BestTrajectories(ytime, ytimeTent, &ytrajectories);

  std::vector<ParabolaTraj> xtrajectories;
  BestTrajectories(xtime, xtimeTent, &xtrajectories);

  MatchXYTrajectories(xtime, xtrajectories, ytime, ytrajectories, &tempTrajectories);
}

bool SingleCameraProcessor::AddBallPosition(int frame_no, cv::Point2f ballpos) {
  if (framesWithBalls.find(frame_no) != framesWithBalls.end()) {
    DEBUG("Adding another point at frame " << frame_no << " : " << ballpos);
    return false;
  }
  ballPositions2d.push_back({frame_no, ballpos});
  framesWithBalls.insert(frame_no);
  if (!RemoveBallCandidate(frame_no)) {
    return false;
  }
  ytime.push_back({frame_no, cv::Point2f(frame_no, ballpos.y)});
  xtime.push_back({frame_no, cv::Point2f(frame_no, ballpos.x)});
  return true;
}

bool SingleCameraProcessor::AddBallCandidate(int frame_no, cv::Point2f ballcand) {
  ballCandidates2d.push_back({frame_no, ballcand});
  ytimeTent.push_back({frame_no, cv::Point2f(frame_no, ballcand.y)});
  xtimeTent.push_back({frame_no, cv::Point2f(frame_no, ballcand.x)});
  return true;
}

void removeFromDeque(int frame_no, std::deque<std::pair<int, cv::Point2f>> &d) {
  std::pair<int, cv::Point2f> cand = {frame_no, cv::Point2f(0, 0)};
  auto beg = std::lower_bound(d.begin(), d.end(), cand, myComp<cv::Point2f>);
  auto end = std::upper_bound(d.begin(), d.end(), cand, myComp<cv::Point2f>);
  d.erase(beg, end);
}

bool SingleCameraProcessor::RemoveBallCandidate(int frame_no) {
  removeFromDeque(frame_no, ballCandidates2d);
  removeFromDeque(frame_no, ytimeTent);
  removeFromDeque(frame_no, xtimeTent);
  return true;
}

template<typename T>
void normaliseDeque(int minval, std::deque<std::pair<int, T>> &d) {
  while (d.size() && d[0].first < minval) {
    d.pop_front();
  }
}

void SingleCameraProcessor::normalise() {
  int minval = count - frames_to_keep;
  normaliseDeque(minval, ballPositionsOrig2d);
  normaliseDeque(minval, ballPositions2d);
  normaliseDeque(minval, ballCandidates2d);
  normaliseDeque(minval, playerPosition2d);
  normaliseDeque(minval, ytime);
  normaliseDeque(minval, ytimeTent);
  normaliseDeque(minval, xtime);
  normaliseDeque(minval, xtimeTent);
}

bool SingleCameraProcessor::GetRay(cv::Point2d frame_pos, cv::Point3d *a, cv::Point3d *b) {
  *a = C;
  cv::Mat x = cv::Mat::zeros(3, 1, CV_64F);
  x.at<double>(0) = frame_pos.x;
  x.at<double>(1) = frame_pos.y;
  x.at<double>(2) = 1;
  cv::Mat X = Pinv * x;
  double w = X.at<double>(3);
  *b = cv::Point3d(X.at<double>(0) / w,
                   X.at<double>(1) / w,
                   X.at<double>(2) / w);
 
  return true;
}
