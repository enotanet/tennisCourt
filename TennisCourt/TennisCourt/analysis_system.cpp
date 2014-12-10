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
    default:
      // Default behaviour is advance one frame.
      //
      ++frame_position;
      break;
    }

    if (outputBuffer.lastFrame < frame_position) {
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
    sprintf(name, "Window %d", i);
    cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
  }
}

void DisplayOutput(OutputResult output) {
  for (size_t i = 0; i < output.images.size(); ++i) {
    char name[32];
    sprintf(name, "Window %u", i);
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
void FrameProcessor::ProcessFrames(std::vector<cv::Mat> frames, OutputResult *outputResult) {
  // Multithread this & reading maybe. Reduce the number of clones!
  for (size_t i = 0; i < frames.size(); ++i) {
    cv::Point2f ballPosition;
    if (ballFinders[i].addFrame(frames[i], ballPosition)) {
      cv::circle(frames[i], ballPosition, 4, cv::Scalar(0, 255, 0), -1, 8);
    }
    cv::Point3f cameraCoords;
    if (!cameraLocations[i].GetCoordinate(frames[i], &cameraCoords)) {
      // Oops. Don't continue with analysis
    }
    outputResult->images.push_back(frames[i].clone());
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

