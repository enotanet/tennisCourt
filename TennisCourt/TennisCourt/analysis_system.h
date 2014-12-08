#ifndef ANALYSIS_SYSTEM_H__
#define ANALYSIS_SYSTEM_H__

#include "sys_frame_grabber.h"
#include "ballFinder.h"
#include <opencv2/core/core.hpp>
#include <vector>

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
  static const int max_frames_to_keep = (1 << 10);
  std::vector<OutputResult> results;
};

class FrameProcessor {
public:
  FrameProcessor(size_t frame_number) : ballFinders(frame_number) {}
  // Maybe bools?
  // Needs state! Wrap in a class.
  //
  void ProcessFrames(std::vector<cv::Mat> frames, OutputResult *outputResult);

private:
  std::vector<BallFinder> ballFinders;
};

void InitialiseOutput(size_t windowCount);

void DisplayOutput(OutputResult output);

#endif