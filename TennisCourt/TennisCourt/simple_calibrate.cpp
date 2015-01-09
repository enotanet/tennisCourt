#include "utils.h"
#include "simple_calibrate.h"
#include "sys_frame_grabber.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <fstream>
#include <string>

const double k_FOOT_TO_M = 0.3048;

struct Click {
  std::ofstream *out;
  size_t cam;
  size_t cur_frame;
  SystemFrameGrabber *grabber;
  std::vector<cv::Mat> frames;
  std::vector<std::vector<cv::Mat>> hist;
  size_t cur_corner;
  std::vector<cv::Point3d> corners;
};

void drawImage(Click &c) {
  while (c.hist.size() <= c.cur_frame) {
    c.grabber->getNextFrames(&c.frames);
    std::vector<cv::Mat> fcopy(c.frames.size());
    for (size_t i = 0; i < c.frames.size(); ++i) {
      fcopy[i] = c.frames[i].clone();
    }
    c.hist.push_back(fcopy);
  }
  cv::imshow("Click me", c.hist[c.cur_frame][c.cam]);
}

void awaitClick(Click &c) {
  INFO("Please click the corner at " << c.corners[c.cur_corner]);
}

void ImgClick(int event, int x, int y, int flags, void* u) {
  Click *c = (Click*) u;
  if (event == cv::EVENT_LBUTTONDOWN && c->cam < c->frames.size()) {
    DEBUG("Click registered at " << x << " " << y);
    *(c->out) << c->cam << " " << x << " " << y << " "
              << c->corners[c->cur_corner].x << " "
              << c->corners[c->cur_corner].y << " "
              << c->corners[c->cur_corner].z << std::endl;
    bool h = true;
    ++c->cur_corner;
    if (c->cur_corner > c->corners.size()) {
      // No more corners.
      ++c->cam;
      c->cur_corner = 0;
      if (c->cam < c->frames.size()) {
        drawImage(*c);
      } else {
        INFO("Nothing to do. Press q to exit");
        h = false;
      }
    }
    if (h) {
      awaitClick(*c);
    }
  }
}

bool ExtractCorners(std::string corners_file, SystemFrameGrabber *grabber,
                    std::string output_file) {
  Click c;
  c.cam = 0;
  c.cur_frame = 0;
  c.frames = grabber->getContainer();
  c.grabber = grabber;
  // Read corners.
  //
  std::ifstream cfile(corners_file);
  if (!cfile.is_open()) {
    DEBUG("ExtractCorners can't open corners file");
    return false;
  }
  double x, y, z;
  while (cfile >> x >> y >> z) {
    c.corners.push_back(cv::Point3d(x, y, z));
  }
  cfile.close();

  std::ofstream out(output_file);
  if (!out.is_open()) {
    DEBUG("ExtractCorners can't open output file");
    return false;
  }

  c.out = &out;

  // Use autosize to get the correct click position.
  //
  cv::namedWindow("Click me", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Click me", ImgClick, &c);
  
  c.cur_corner = 0;
  awaitClick(c);
  char last_key = 'n';
  while (last_key != 'q' && last_key != 27) {
    switch (last_key) {
    case 'p':
      if (c.cur_frame > 0) {
        --c.cur_frame;
      }
      break;
    default:
      // Default behaviour is advance one frame.
      //
      ++c.cur_frame;
      break;
    }

    if (c.cam < c.frames.size()) {
      drawImage(c);
      last_key = cv::waitKey(0);
    } else {
      break;
    }
  }
  return true;
}

cv::Mat matFromPair(const std::pair<cv::Point2f, cv::Point3d> &p) {
  cv::Mat a = cv::Mat::zeros(2, 12, CV_64F);
  a.at<double>(0, 4) = -p.second.x;
  a.at<double>(0, 5) = -p.second.y;
  a.at<double>(0, 6) = -p.second.z;
  a.at<double>(0, 7) = -1;
  a.at<double>(0, 8) =  p.first.y * p.second.x;
  a.at<double>(0, 9) =  p.first.y * p.second.y;
  a.at<double>(0, 10) = p.first.y * p.second.z;
  a.at<double>(0, 11) = p.first.y * 1;
 
  a.at<double>(1, 0) = p.second.x;
  a.at<double>(1, 1) = p.second.y;
  a.at<double>(1, 2) = p.second.z;
  a.at<double>(1, 3) = 1;
  a.at<double>(1, 8) =  - p.first.x * p.second.x;
  a.at<double>(1, 9) =  - p.first.x * p.second.y;
  a.at<double>(1, 10) = - p.first.x * p.second.z;
  a.at<double>(1, 11) = - p.first.x * 1;
  return a;
}

bool EvaluateCalib(std::string corn_out, SystemFrameGrabber *grabber) {
  INFO("Reading from " << corn_out);
  std::vector<cv::Mat> frames = grabber->getContainer();
  grabber->getNextFrames(&frames);
  cv::namedWindow("cir", CV_WINDOW_NORMAL);

  std::ifstream cfile(corn_out);
  if (!cfile.is_open()) {
    DEBUG("ExtractCorners can't open corners file");
    return false;
  }
  int c, xx, yy;
  double x, y, z;
  std::vector<std::pair<cv::Point2f, cv::Point3d>> cor;
  while (cfile >> c >> xx >> yy >> x >> y >> z) {
    DEBUG("READ " << c << " " << xx << " " << yy << " " << x << " " << y << " " << z);
    cv::circle(frames[c], cv::Point(xx, yy), 2, cv::Scalar(0, 255, 0), -1, 8);
    cor.push_back(std::make_pair(cv::Point2f(xx, yy), cv::Point3d(x, y, z)));
  }
  cfile.close();

  // Attempt to compute matrix P without normalisation.
  //
  cv::Mat A = matFromPair(cor[0]);
  for (size_t i = 1; i < cor.size(); ++i) {
    A.push_back(matFromPair(cor[i]));
  }

  INFO(A);
  
  cv::Mat sln;
  cv::SVD::solveZ(A, sln);
  INFO(sln);
  INFO(cv::norm(sln));
  
  cv::Mat P = cv::Mat::zeros(3, 4, CV_64F);
  for (int i = 0; i < 12; ++i) {
    P.at<double>(i / 4, i % 4) = sln.at<double>(i);
  }
  INFO(P);
  cv::Mat C;
  cv::SVD::solveZ(P, C);
  INFO(C);
  double w = C.at<double>(3);
  INFO("Coords " << C.at<double>(0) / w << " " << C.at<double>(1) / w << " "
        << C.at<double>(2) / w);

  cv::Mat tpoint = cv::Mat::zeros(4, 1, CV_64F);
  tpoint.at<double>(0) = -39;
  tpoint.at<double>(1) = 13.5;
  tpoint.at<double>(2) = 0;
  tpoint.at<double>(3) = 1;
  cv::Mat conv = P * tpoint;
  cv::Point2d pr(conv.at<double>(0) / conv.at<double>(2),
                 conv.at<double>(1) / conv.at<double>(2));
  cv::circle(frames[0], pr, 2, cv::Scalar(0, 0, 255), -1, 8);
  imshow("cir", frames[0]);
  cv::waitKey(0);
  return true;
}

std::vector<std::vector<std::pair<cv::Point2f, cv::Point3d>>> CalibReadFile(
    std::string corn_out) {
  std::vector<std::vector<std::pair<cv::Point2f, cv::Point3d>>> cor;
  std::ifstream cfile(corn_out);
  if (!cfile.is_open()) {
    DEBUG("ExtractCorners can't open corners file");
    return cor;
  }
  int c, xx, yy;
  double x, y, z;
  while (cfile >> c >> xx >> yy >> x >> y >> z) {
    DEBUG("READ " << c << " " << xx << " " << yy << " " << x << " " << y << " " << z);
    while ((int) cor.size() <= c) {
      cor.push_back(std::vector<std::pair<cv::Point2f, cv::Point3d>>());
    }
    x = x * k_FOOT_TO_M;
    y = y * k_FOOT_TO_M;
    z = z * k_FOOT_TO_M;
    cor[c].push_back(std::make_pair(cv::Point2f(xx, yy), cv::Point3d(x, y, z)));
  }
  cfile.close();
  return cor;
}

CalibratedCamera::CalibratedCamera(
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point3d>>> pairs) {
  Calibrate(pairs);
  calibrated = true;
}

void CalibratedCamera::Calibrate(
    std::vector<std::vector<std::pair<cv::Point2f, cv::Point3d>>> pairs) {
  P.resize(pairs.size());
  C.resize(pairs.size());
  Pinv.resize(pairs.size());
  for (size_t cam = 0; cam < pairs.size(); ++cam) {
    // Attempt to compute matrix P without normalisation.
    //
    cv::Mat A = matFromPair(pairs[cam][0]);
    for (size_t i = 1; i < pairs[cam].size(); ++i) {
      A.push_back(matFromPair(pairs[cam][i]));
    }

    cv::Mat sln;
    cv::SVD::solveZ(A, sln);
    DEBUG(cv::norm(sln));
    
    P[cam] = cv::Mat::zeros(3, 4, CV_64F);
    for (int i = 0; i < 12; ++i) {
      P[cam].at<double>(i / 4, i % 4) = sln.at<double>(i);
    }
    cv::Mat C3;
    cv::SVD::solveZ(P[cam], C3);
    double w = C3.at<double>(3);
    C[cam] = cv::Point3d(C3.at<double>(0) / w, C3.at<double>(1) / w, C3.at<double>(2) / w);
    INFO("Coordinates of camera in m " << cam << " " << C[cam]);
    INFO("Coordinates of camera in ft " << cam << " " << C[cam].x / k_FOOT_TO_M << ", "
          << C[cam].y / k_FOOT_TO_M << ", " << C[cam].z / k_FOOT_TO_M);

    cv::invert(P[cam], Pinv[cam], cv::DECOMP_SVD);
    DEBUG(P[cam] * Pinv[cam]);
  }

  DEBUG("asdasdasdsad");
  for (size_t cam = 0; cam < pairs.size(); ++cam) {
    DEBUG(P[cam]);
  }
  DEBUG("asdasdasdsad");
  calibrated = true;
}

bool CalibratedCamera::GetRay(size_t cam, cv::Point2d frame_pos,
    cv::Point3d *a, cv::Point3d *b) {
  if (!calibrated) {
    return false;
  }
  if (cam > C.size()) {
    return false;
  }
  *a = C[cam];
  cv::Mat x = cv::Mat::zeros(3, 1, CV_64F);
  x.at<double>(0) = frame_pos.x;
  x.at<double>(1) = frame_pos.y;
  x.at<double>(2) = 1;
  cv::Mat X = Pinv[cam] * x;
  double w = X.at<double>(3);
  *b = cv::Point3d(X.at<double>(0) / w,
                   X.at<double>(1) / w,
                   X.at<double>(2) / w);
 
  return true;
}

cv::Point2d CalibratedCamera::Project(size_t cam, cv::Point3d X) {
  cv::Mat M = cv::Mat::zeros(4, 1, CV_64F);
  M.at<double>(0) = X.x;
  M.at<double>(1) = X.y;
  M.at<double>(2) = X.z;
  M.at<double>(3) = 1;

  cv::Mat m = P[cam] * M;
  return cv::Point2d(m.at<double>(0) / m.at<double>(2), m.at<double>(1) / m.at<double>(2));
}

bool CalibratedCamera::GetParams(size_t id, cv::Mat *p, cv::Point3d *c, cv::Mat *pinv)
    const {
  *p = P[id];
  *c = C[id];
  *pinv = Pinv[id];
  return true;
}
