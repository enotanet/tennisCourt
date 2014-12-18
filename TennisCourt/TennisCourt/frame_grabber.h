// Interface for grabbing frames from file or camera.
// Let's put it into a class and test performance.
// Classes not thread-safe. Just a single thread reads
// from the frame source.

#ifndef FRAME_GRABBER_H__
#define FRAME_GRABBER_H__

#include <opencv2/core/core.hpp>

class FrameGrabber {
 public:
   virtual bool getNextFrame(cv::Mat *res) = 0;
   // Returns a Mat, capable of storing the frames grabbed.
   virtual cv::Mat getMat() = 0;
};

#endif  // FRAME_GRABBER_H__