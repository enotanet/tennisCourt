#ifndef CAMERA_LOCATION_H__
#define CAMERA_LOCATION_H__

#include <opencv2/core/core.hpp>
#include <vector>

struct Corner {
  // Which corner is it. TODO: Enumerate all the corners on the tennis court.
  // For each corner type we will know the real-life 3d coordinates (fixed).
  // In this way, we can return any set of corners that we manage to recover
  // from the image and use those for the coordinate reconstruction.
  //
  int type;
  // Equivalently, we could return any point on the tennis court with known coordinates
  // Just fill in one of type, position_on_court.
  //
  cv::Point3f position_on_court;
  // The location where we see this corner in the field of view of the camera.
  //
  cv::Point2f in_frame_position;
};

class CameraLocation {
public:
  // Returns false if we weren't able to get the camera location.
  // Only recomputes (uses frame) in rare occasions, when it needs to!
  //
  bool GetCoordinate(cv::Mat frame, cv::Point3f *location);
private:
  // Given a frame, return a set of corners that were reconstructed.
  //
  bool GetCourtCorners(cv::Mat frame, std::vector<Corner> *corners);

  // Define a dictionary
  cv::Point3f camera_loc;
};

#endif  // CAMERA_LOCATION_H__