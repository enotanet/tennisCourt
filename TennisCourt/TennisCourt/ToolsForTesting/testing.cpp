// Primary entry point for testing
//

#include "testing.h"
#include "../court_display.h"
#include "../sys_frame_grabber.h"
#include "../sys_camera_grabber.h"
#include "../sys_file_frame_grabber.h"
#include "../sys_video_writer.h"
#include "../utils.h"

#include <cassert>
#include <cmath>
#include <ctime>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>

void run_tests() {
  INFO("Running tests!");
  if (g_args["test"].size() && g_args["test"][0] == "line") {
    std::vector<cv::Point3d> points;
    points.push_back(cv::Point3d(0, 0, 0));
    points.push_back(cv::Point3d(1, 0, 0));
    points.push_back(cv::Point3d(0, 1, 0));
    points.push_back(cv::Point3d(0, 0, 1));
    points.push_back(cv::Point3d(0, 1, 1));
    points.push_back(cv::Point3d(1, 0, 1));
    points.push_back(cv::Point3d(1, 1, 0));
    points.push_back(cv::Point3d(1, 1, 1));
    for (size_t i = 0; i < points.size(); ++i) {
      for (size_t j = 0; j < points.size(); ++j) {
        for (size_t k = 0; k < points.size(); ++k) {
          for (size_t l = 0; l < points.size(); ++l) {
            cv::Point3d res = LineIntersect(points[i], points[j], points[k], points[l]);
            INFO("Intersection of lines: " << points[i] << " " << points[j] <<
                 " ; " << points[k] << " " << points[l] << " -> " << res);
          }
        }
      }
    }
  } else if (g_args["test"].size() && g_args["test"][0] == "pparab") {
    std::vector<cv::Point2d> points;
    points.push_back(cv::Point2d(0, 0));
    points.push_back(cv::Point2d(5, 0));
    points.push_back(cv::Point2d(0, 5));
    points.push_back(cv::Point2d(5, 5));

    std::vector<cv::Point3d> parab;
    parab.push_back(cv::Point3d(1, 0, 0));
    parab.push_back(cv::Point3d(1, 2, 1));
    for (cv::Point2d p : points) {
      for (cv::Point3d q : parab) {
        double d = PointToParabolaDistance(p, q.x, q.y, q.z);
        INFO("Point to parab distance " << p << " ; " << q << " -> " << d);
      }
    }
  } else if (g_args["test"].size() && g_args["test"][0] == "gpard") {
    std::vector<cv::Point2d> points;
    points.push_back(cv::Point2d(0, 0));
    points.push_back(cv::Point2d(3, 3));
    points.push_back(cv::Point2d(8, 0));
    double a, b, c;
    getParabola(points, &a, &b, &c);
    DEBUG("Parabola defined by " << points[0] << " ; " << points[1] << " ; " << points[2]);
    DEBUG("Parabola equation " << a << " " << b << " " << c);
    for (auto p : points) {
      double trajectory_dist = PointToParabolaDistance(p, a, b, c);
      DEBUG("Dist " << p << " -> " << trajectory_dist);
      assert(std::abs(trajectory_dist) < 1);
    }
  } else if (g_args.count("court_calibrate")) {
    INFO("Running court calibration");
    CourtDisplay displ(g_args["court_calibrate"][0]);
    displ.calibrate();
  } else if (g_args.count("fout")) {
    INFO("Testing reading and writing of video");
    SystemFrameGrabber *grabber;
    if (g_args.count("fin")) {
      assert(g_args["fin"].size() == g_args["fout"].size());
      assert(g_args["fin"].size());
      grabber = new SystemFileFrameGrabber(g_args["fin"]);
    } else {
      DEBUG("Opening system cameras");
      SystemCameraGrabber *camgrab = new SystemCameraGrabber();
      camgrab->startGrabbing();
      grabber = camgrab;
    }
    auto container = grabber->getContainer();

    assert(container.size() == g_args["fout"].size());
    SystemVideoWriter videoWrite(g_args["fout"], container[0].size());
    
    long long start = clock();
    // 10 minutes!
    long long slice = start;
    long long frames = 0;
    while (clock() - start < 60LL* CLOCKS_PER_SEC) {
      if (!grabber->getNextFrames(&container)) {
        if (g_args.count("fin")) {
          INFO("Finished files. Reopen them");
          grabber = new SystemFileFrameGrabber(g_args["fin"]);
          grabber->getNextFrames(&container);
        }
      }

      videoWrite.WriteFrames(container);
      ++frames;
      if (clock() - slice > 10 * CLOCKS_PER_SEC) {
        INFO("Written " << frames << " for the past 10 sec @ " << (double) frames / (double) (clock() - slice) * CLOCKS_PER_SEC);
        slice = clock();
        frames = 0;
      }
    }
    delete grabber;
  }
}
