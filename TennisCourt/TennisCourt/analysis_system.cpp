#include "analysis_system.h"
#include "camera_location.h"
#include "sys_frame_grabber.h"
#include "utils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

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
        ballPositions.push_back(std::make_pair(count, ballPosition));
      }
    }
    if (players.size() > 0) {
      // do something with the players positions
    }
    // if (ballPositions.size() >= 3) {
    //   std::vector<cv::Point2d> points;
    //   for (int j = ballPositions.size() - 1; j >= (int) ballPositions.size() - 3; --j) {
    //     points.push_back(ballPositions[j].second);
    //     if (j != (int) ballPositions.size() - 1) { 
    //       cv::circle(frames[i], ballPositions[j].second, 4,
    //           cv::Scalar(255, 255, 0), -1, 8);
    //     }
    //   }
    //   double a, b, c;
    //   if (getParabola(points, &a, &b, &c)) {
    //     drawParabola(frames[i], a, b, c, cv::Scalar(0, 255, 255));
    //   }
    //   if (oa >= -1e-7) {
    //     drawParabola(frames[i], oa, ob, oc, cv::Scalar(0, 255, 128));
    //   }
    //   oa = a;
    //   ob = b;
    //   oc = c;
    // }
    if (count % 10 == 0) {
      DEBUG("Doing trajectory analysis with " << ballPositions.size()
            << " ball positions");
      std::vector<cv::Point3d> trajectories;
      BestTrajectories(ballPositions, &trajectories);
      cv::RNG rng(42);
      for (cv::Point3d p : trajectories) {
        drawParabola(frames[i], p.x, p.y, p.z, cv::Scalar(rng.uniform(0, 255),
                                                          rng.uniform(0, 255),
                                                          rng.uniform(0, 255)));
      }
      if (trajectories.size()) {
        oa = trajectories.back().x;
        ob = trajectories.back().y;
        oc = trajectories.back().z;
      }
      for (auto p : ballPositions) {
        cv::circle(frames[i], p.second, 2, cv::Scalar(0, 255, 0), -1, 8);
      }
    }
    if (oa >= -1e-7) {
      drawParabola(frames[i], oa, ob, oc, cv::Scalar(255, 0, 128));
      for (auto p : ballPositions) {
        if (PointToParabolaDistance(p.second, oa, ob, oc) < 1.0) {
          cv::circle(frames[i], p.second, 2, cv::Scalar(0, 255, 0), -1, 8);
        }
      }
    }
    if (players.size() > 0) {
      // do something with the players positions
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
//
void BestTrajectories(const std::vector<std::pair<int, cv::Point2f>> &ballPositions,
                      std::vector<cv::Point3d> *trajectories) {
  // Tries to fit a parabola that approximates all points inside the window.
  // Do comparisons based on point times!
  //
  const int max_window_size = 24;
  const int good_trajectory_threshold = 18;
  // In the camera projection plane case, this distance is in pixels.
  //
  const double trajectory_eps = 1;
  trajectories->clear();
  for (size_t mid = 1; mid + 1 < ballPositions.size(); ++mid) {
    double a, b, c;
    double ba, bb, bc;
    std::vector<cv::Point2d> points;
    
    // Get initial parabala around midpoint.
    //
    points.push_back(ballPositions[mid - 1].second);
    points.push_back(ballPositions[mid].second);
    points.push_back(ballPositions[mid + 1].second);
    getParabola(points, &a, &b, &c);
    DEBUG("Parabola defined by " << points[0] << " ; " << points[1] << " ; " << points[2]);
    DEBUG("Parabola equation " << a << " " << b << " " << c);
    for (int i = mid - 1; i <= mid + 1; ++i) {
      double trajectory_dist = PointToParabolaDistance(ballPositions[i].second, a, b, c);
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
      dist_sum = 0;
      for (size_t i = 0; i < ballPositions.size(); ++i) {
        if (ballPositions[i].first < ballPositions[mid].first - max_window_size) {
          continue;
        }
        if (ballPositions[i].first > ballPositions[mid].first + max_window_size) {
          continue;
        }
        double trajectory_dist = PointToParabolaDistance(ballPositions[i].second, a, b, c);
        if (trajectory_dist < trajectory_eps) {
          satisfied.push_back(i);
          dist_sum += trajectory_dist;
        } else {
          dist_sum += trajectory_eps;
        }
      }
      
      cur_satisfied = satisfied.size();
      if (cur_satisfied > best_satisfied && dist_sum < best_dist) {
        ba = a;
        bb = b;
        bc = c;
        // Update.
        assert(points.size() == 3);
        points[0] = ballPositions[satisfied[0]].second;
        points[2] = ballPositions[satisfied.back()].second;
        getParabola(points, &a, &b, &c);
      }
    } while (cur_satisfied > best_satisfied && dist_sum < best_dist);
    DEBUG("Trajectory " << ba << " " << bb << " " << bc
          << " satisfies " << best_satisfied << " points");
    if (best_satisfied >= good_trajectory_threshold) {
      trajectories->push_back(cv::Point3d(ba, bb, bc));
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

