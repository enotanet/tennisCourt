// Same as frame_grabber, just gives 4 frames at a time.

// Interface for grabbing frames from file or camera.
// Let's put it into a class and test performance.
// Classes not thread-safe. Just a single thread reads
// from the frame source.

#ifndef SYS_FRAME_GRABBER_H__
#define SYS_FRAME_GRABBER_H__

#include <opencv2/core/core.hpp>
#include <vector>

class SystemFrameGrabber {
 public:
   virtual bool getNextFrames(std::vector<cv::Mat> *res) = 0;
   // Returns a Mat, capable of storing the frames grabbed.
   virtual std::vector<cv::Mat> getContainer() = 0;
};

#endif  // SYS_FRAME_GRABBER_H__