#include "utils.h"
#include <cmath>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>

std::map<std::string, std::vector<std::string>> g_args;

// Very hacky and unsafe. Tests?
//
void ParseArguments(int argc, char *argv[]) {
  for (int i = 0; i < argc; ++i) {
    std::stringstream sin(argv[i]);
    std::string key;
    std::getline(sin, key, '=');
    if (!sin.eof()) {
      if (key.size() > 2 && key.substr(0, 2) == "--") {
        key = key.substr(2);
        std::string value;
        while (!sin.eof()) {
          getline(sin, value, ';');
          g_args[key].push_back(value);
        }
      }
    }
  }
}

// Intersect two 3d lines given in the form Pi + s * Vi.
// If the lines don't intersect return a midpoint of a shortest-distance vector
// connecting the lines.
// Source: http://geomalgorithms.com/a07-_distance.html
//
cv::Point3d LineIntersect(cv::Point3d p0, cv::Point3d v0, cv::Point3d p1, cv::Point3d v1) {
  cv::Point3d w0 = p0 - p1;
  double a = v0.dot(v0);
  double b = v0.dot(v1);
  double c = v1.dot(v1);
  double d = v0.dot(w0);
  double e = v1.dot(w0);
  if (std::abs(a * c - b * b) < 1e-7) {
    double t = d / b;
    return (p0 + p1 + t * v1) * 0.5;
  }
  double s = (b * e - c * d) / (a * c - b * b);
  double t = (a * e - b * d) / (a * c - b * b);
  DEBUG("Line intersection distance: " << norm(p0 + s * v0 - p1 - t * v1));
  return (p0 + s * v0 + p1 + t * v1) * 0.5;
}

