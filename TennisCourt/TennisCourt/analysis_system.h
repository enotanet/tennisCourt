#ifndef ANALYSIS_SYSTEM_H__
#define ANALYSIS_SYSTEM_H__

#include "sys_frame_grabber.h"
#include <opencv2/core/core.hpp>
#include <vector>

void RunOnlineSystem();

void RunOfflineSystem(SystemFrameGrabber *grabber);

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

struct OutputResult {
  std::vector<cv::Mat> images;
};

// Maybe bools?
//
void ProcessFrames(std::vector<cv::Mat> frames, OutputResult& ouputResult);

void InitialiseOutput(int windowCount);

void DisplayOutput(OutputResult output);

#endif