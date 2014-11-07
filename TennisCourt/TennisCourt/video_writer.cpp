#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include "video_writer.h"

// Experiment with lossless codecs from http://www.fourcc.org/codecs.php
//
bool writeVideoToFile(const std::string &filepath, FrameGrabber* fg, int time) {
  cv::Mat cv_img(fg->getMat());
  cv::VideoWriter vw(filepath, CV_FOURCC_PROMPT, 60, cv_img.size(), /*isColor =*/ false);
  // cv::CvVideoWriter* vw;
  //vw = cvCreateVideoWriter(filepath.c_str(), -1, 25, cv_img.size());
  // cv::VideoWriter vw(filepath, CV_FOURCC('Y','U','Y','V'), 60, cv_img.size(), /*isColor =*/ false);
  if (!vw.isOpened())
  {
     std::cout << "!!! Output video could not be opened" << std::endl;
       return 0;
  }
  
  clock_t start = clock();
  int frames = 0;
  std::cout << "FF " << frames << std::endl;
  while (fg->getNextFrame(&cv_img)) {
    ++frames;
    //std::cout << "FF " << frames << std::endl;
    vw.write(cv_img);
    if (clock() - start > CLOCKS_PER_SEC * time) {
      std::cout << "FF " << frames << std::endl;
      return 1;
    }
  }
  std::cout << "FF " << frames << std::endl;
  return 0;
}
