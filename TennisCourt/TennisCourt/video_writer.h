#ifndef VIDEO_WRITER_H__
#define VIDEO_WRITER_H__

#include "frame_grabber.h"
#include <string>

bool writeVideoToFile(std::string filepath, FrameGrabber* fg, int time);

#endif