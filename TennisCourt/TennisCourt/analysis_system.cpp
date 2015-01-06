#include "analysis_system.h"
#include "camera_location.h"
#include "sys_frame_grabber.h"
#include "utils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <set>

#include <ctime>

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
  InitialiseOutput(frames.size());
  FrameProcessor proc(frames.size());

  INFO("Found " << frames.size() << " things");
//   if (g_args["skip"].size()) {
//     int frames_to_skip;
//     sscanf(g_args["skip"][0].c_str(), "%d", &frames_to_skip);
//     for (int i = 0; i < frames_to_skip; ++i) {
//       grabber->getNextFrames(&frames);
//     }
//   }

  // TODO(GUI): Introduce some nice slider and stuff!
  //
  char last_key = 'n';
  long long frame_position = -1;
  while (last_key != 'q' && last_key != 27) {
    switch (last_key) {
    case 'p':
      if (frame_position > 0) {
        --frame_position;
      }
      break;
    case 's':
      frame_position += 50;
      while (outputBuffer.lastFrame + 1 < frame_position) {
        grabber->getNextFrames(&frames);
        OutputResult res;
        proc.ProcessFrames(frames, &res, 1);
        outputBuffer.put(res);
      }
      break;
    default:
      // Default behaviour is advance one frame.
      //
      ++frame_position;
      break;
    }

    while (outputBuffer.lastFrame < frame_position) {
      grabber->getNextFrames(&frames);
      OutputResult res;
      proc.ProcessFrames(frames, &res);
      outputBuffer.put(res);
    }
    DisplayOutput(outputBuffer.get(frame_position));
    last_key = cv::waitKey(0);
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

// TODO: Main function that deals with processing each set of frames.
// Put the results in outputResult so it can be used to display now or later
//
void FrameProcessor::ProcessFrames(std::vector<cv::Mat> frames,
    OutputResult *outputResult,
    bool skip_clone) {
  // Multithread this & reading maybe. Reduce the number of clones!
  for (size_t i = 0; i < frames.size(); ++i) {
    cv::Point2f ballPosition(-1, -1);
    std::vector<object> players;
    if (ballFinders[i].addFrame(frames[i], ballPosition, players)) {
      cv::circle(frames[i], ballPosition, 4, cv::Scalar(0, 255, 0), -1, 8);

      // Let's ignore players for a bit..
      //
      for (object player : players) {
        cv::circle(frames[i], player.top, 2, cv::Scalar(255, 0, 0), -1, 8);
        cv::circle(frames[i], player.bottom, 2, cv::Scalar(255, 0, 0), -1, 8);
        cv::circle(frames[i], player.right, 2, cv::Scalar(255, 0, 0), -1, 8);
        cv::circle(frames[i], player.left, 2, cv::Scalar(255, 0, 0), -1, 8);
      }

      if (ballPosition.x >= 0 && ballPosition.y >= 0) {
        ballPositions2d[i].push_back(std::make_pair(count, ballPosition));
      }
    }
    if (players.size() > 0) {
      // do something with the players positions
    }
    if (count % 10 == 0) {
      // DEBUG("Doing trajectory analysis with " << ballPositions2d[i].size()
      //       << " ball positions");
      // std::vector<cv::Point3d> trajectories;
      // BestTrajectories(ballPositions2d[i], &trajectories, 24, 18, 1.0);
      // cv::RNG rng(42);
      // for (cv::Point3d p : trajectories) {
      //   drawParabola(frames[i], p.x, p.y, p.z, cv::Scalar(rng.uniform(0, 255),
      //                                                     rng.uniform(0, 255),
      //                                                     rng.uniform(0, 255)));
      // }
      // if (trajectories.size()) {
      //   oa = trajectories.back().x;
      //   ob = trajectories.back().y;
      //   oc = trajectories.back().z;
      // }
      // for (auto p : ballPositions2d[i]) {
      //   cv::circle(frames[i], p.second, 2, cv::Scalar(0, 255, 0), -1, 8);
      // }

      DEBUG("Doing analysis with y/time and x/time parabolas");
      // Very dodgy. We want to guarantee that the same set of points was used to
      // approximate both y/time and x/time parabolas.
      //
      std::vector<ParabolaTraj> ytrajectories;
      std::vector<std::pair<int, cv::Point2f>> ytime;
      for (auto p : ballPositions2d[i]) {
        ytime.push_back(std::make_pair(p.first, cv::Point2f(p.first, p.second.y)));
      }
      BestTrajectories(ytime, &ytrajectories);

      std::vector<ParabolaTraj> xtrajectories;
      std::vector<std::pair<int, cv::Point2f>> xtime;
      for (auto p : ballPositions2d[i]) {
        xtime.push_back(std::make_pair(p.first, cv::Point2f(p.first, p.second.x)));
      }
      BestTrajectories(xtime, &xtrajectories);

      // Match up y-t and x-t trajectories.
      //
      std::vector<ParabolaTraj2d> trajectories;
      MatchXYTrajectories(xtime, xtrajectories, ytime, ytrajectories, &trajectories);
      
      DEBUG(xtrajectories.size() << " " << ytrajectories.size()
            << " " << trajectories.size());
      if (trajectories.size()) {
        parab = trajectories.back();
        // Find a "shortest" path between trajectories[0] and trajectories.back()
        //
        LinkedParabolaTrajectory2d linkedTrajectories(ballPositions2d[i], trajectories);

        for (int t = 0; t <= count; ++t) {
          cv::Point2d p;
          if (linkedTrajectories.EvaluateAt(t, &p)) {
            double ps = (double) t / count * 255.0;
            cv::circle(frames[i], p, 4, cv::Scalar(255 - ps, 0, ps), 1, 8);
          } else {
            DEBUG("Failed to find suitable parabola for " << t);
          }
        }

        for (auto p : ballPositions2d[i]) {
          cv::circle(frames[i], p.second, 1, cv::Scalar(0, 255, 0), -1, 8);
        }
      }
    }
    if (oa >= -1e-7) {
      drawParabola(frames[i], oa, ob, oc, cv::Scalar(255, 0, 128));
      for (auto p : ballPositions2d[i]) {
        if (PointToParabolaDistance(p.second, oa, ob, oc) < 1.0) {
          cv::circle(frames[i], p.second, 2, cv::Scalar(0, 255, 0), -1, 8);
        }
      }
    }
    cv::Point3f cameraCoords;
    if (!cameraLocations[i].GetCoordinate(frames[i], &cameraCoords)) {
      // Oops. Don't continue with analysis
    }
    if (!skip_clone)
      outputResult->images.push_back(frames[i].clone());
  }
  ++count;
}

LinkedParabolaTrajectory2d::LinkedParabolaTrajectory2d(
    const std::vector<std::pair<int, cv::Point2f>> &points,
    const std::vector<ParabolaTraj2d> &trajectories) {
  std::vector<double> dist(trajectories.size(), 1e100);
  dist[0] = 0;
  std::vector<char> used(trajectories.size(), 0);
  std::vector<int> prev(trajectories.size(), -1);
  satisfied.resize(trajectories.size());
  const double trajectory_eps = 2;

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
    return dist + trajectories[b].first - trajectories[a].last;
  }
}

bool MatchXYTrajectories(const std::vector<std::pair<int, cv::Point2f>> &xtime,
                         const std::vector<ParabolaTraj> &xtrajectories,
                         const std::vector<std::pair<int, cv::Point2f>> &ytime,
                         const std::vector<ParabolaTraj> &ytrajectories,
                         std::vector<ParabolaTraj2d> *trajectories) {
  const int min_common_points = 10;
  const double trajectory_eps = 1.0;
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
bool ParabolaSatisfiedSet(const std::vector<std::pair<int, cv::Point2f>> &ballPositions,
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
void BestTrajectories(const std::vector<std::pair<int, cv::Point2f>> &ballPositions,
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
    getParabola(points, &cur);
    cur.initial = ballPositions[mid].first;
    cur.first = ballPositions[mid - 1].first;
    cur.last = ballPositions[mid + 1].first;

    DEBUG("Parabola defined by " << points[0] << " ; " << points[1] << " ; " << points[2]);
    for (int i = mid - 1; i <= mid + 1; ++i) {
      double trajectory_dist = PointToParabolaDistance(ballPositions[i].second, cur);
      DEBUG("Dist " << ballPositions[i].second << " -> " << trajectory_dist);
    }

    int best_satisfied = 0;
    double best_dist = 1e100;
    int cur_satisfied = 0;
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
      
      cur_satisfied = satisfied.size();
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
        if (mid_cand != (int) mid) {
          DEBUG("Picked another mid candidate " << mid << " " << mid_cand);
        }
        points[1] = ballPositions[mid_cand].second;
        points[2] = ballPositions[satisfied.back()].second;
        getParabola(points, &cur);

        cur.first = ballPositions[satisfied[0]].first;
        cur.last = ballPositions[satisfied.back()].first;
      }
    } while (cur_satisfied > best_satisfied && dist_sum < best_dist);
    DEBUG("Trajectory " << best.a << " " << best.b << " " << best.c
          << " satisfies " << best_satisfied << " points");
    if (best_satisfied >= good_trajectory_threshold) {
      trajectories->push_back(best);
    }
  }
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

