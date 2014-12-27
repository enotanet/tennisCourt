#include "opencv2/highgui/highgui.hpp"

#include <algorithm>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <unordered_map>

#include "courtDetector.h"
#include "CV_Utils/preprocessing.h"
#include "CV_Utils/detecting.h"

#include "ToolsForTesting/histogram.h"
#include "utils.h"

using namespace std;
using namespace cv;

// Uncomment this for manual testing
int main(int argc, char *argv[])
{
  for (int i = 1; i < argc; i++) {
    Mat image = imread(argv[i], CV_LOAD_IMAGE_GRAYSCALE);
  
    cout << "TestCourt Image location: " << argv[1] << endl;

    getCourt(image);

    waitKey(0);
  }

  return EXIT_SUCCESS;
}

/**
 * Returns x-y coordinates of the court corner position on the image.
 * Parameters:
 * Mat frame - frame (image) from the camera.
 */
vector<Vec2f> getCourt(Mat &frame)
{
  vector<Vec2f> coordinates;
  
  Mat out(frame.clone());
  preprocessCourt(frame);
  
  diplayCourtDetectorResult(out, frame);

  //coordinates = detectCorners(preprocesedFrame);

  return coordinates;
}

Mat preprocessCourt(Mat &frame)
{
  vector<int> intensitiesToHighlight;

  // Draw histogram
  namedWindow("Initial", CV_WINDOW_AUTOSIZE);
  imshow("Initial", frame);
  displayHistogram(frame, "InitialHistogram");
  
  Rect cropArea(0, 0.5 * frame.rows, frame.cols - 1, 0.5 * frame.rows - 1);
  Mat croppedImage(frame.clone());
  croppedImage = croppedImage(cropArea);
  GaussianBlur(croppedImage, croppedImage, Size(5,5), 0, 0);
  int* croppedImageIntensities = getIntensities(croppedImage);
  int upperBoundIntensity = 255;
  int lowerBoundIntensity = 0;

  for (int i = 255; i >= 0; i--) {
    if (croppedImageIntensities[i] > 0.0005 * croppedImage.cols * croppedImage.rows) {
      cout << "Thresh 1 " << 0.0005 * croppedImage.cols * croppedImage.rows << endl;
      upperBoundIntensity = i;
      break;
    }
  }
  
  for (int i = upperBoundIntensity; i >= 0; i--) {
    if (croppedImageIntensities[i] - croppedImageIntensities[upperBoundIntensity] > 0.001 * croppedImage.cols * croppedImage.rows) {
      cout << "Thresh 2 " << 0.001 * croppedImage.cols * croppedImage.rows << endl;
      lowerBoundIntensity = i;
      break;
    } 
  }

  cout << "Upperbound " << upperBoundIntensity << " Lowerbound " << lowerBoundIntensity << endl;

  namedWindow("CroppedImage", CV_WINDOW_AUTOSIZE);
  imshow("CroppedImage", croppedImage);
  displayHistogram(croppedImage, "CroppedImageHistogram", intensitiesToHighlight);
  
  intensitiesToHighlight.push_back(upperBoundIntensity);
  intensitiesToHighlight.push_back(lowerBoundIntensity);

  cutBetweenIntensities(frame, lowerBoundIntensity, upperBoundIntensity);

  equalizeHist(frame, frame);
  //GaussianBlur(frame, frame, Size(11,11), 0, 0);

  namedWindow("GetCourtOnly", CV_WINDOW_AUTOSIZE);
  imshow("GetCourtOnly", frame);
  displayHistogram(frame, "GetCourtOnlyHistogram", intensitiesToHighlight);

  vector<Point2f> candidates;
  unordered_map<Point, double, hash_point> probabilities;

  for (int y = 0; y < frame.rows; y++) {
    for (int x = 0; x < frame.cols; x++) {
      if (frame.at<uchar>(Point(x,y))) {
        double probability = y / frame.rows;
        if (probability > 0.0) {
          candidates.push_back(Point(x,y));
          probabilities.insert(make_pair(Point(x,y), probability));
        }
      }
    }
  }

  for (Point candidate : candidates) {
    frame.at<uchar>(candidate) = 255 * probabilities[candidate];
  }

  namedWindow("ManualPreprocessing", CV_WINDOW_AUTOSIZE);
  imshow("ManualPreprocessing", frame);

  return frame;
}

void diplayCourtDetectorResult(Mat &frame, Mat &preprocesedFrame) {
  
  Mat displayImage;
  cvtColor(frame, displayImage, CV_GRAY2RGBA);
  
  /*vector<Vec4i> lines;
  HoughLinesP(preprocesedFrame, lines, 1, CV_PI/180, 50, 30, 10);

  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( displayImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }*/

  double qualityLevel = 0.5;
  double minDistance = 0.5 * (preprocesedFrame.cols + preprocesedFrame.rows) * 0.05;
  int blockSize = 10;
  bool useHarrisDetector = true;
  double k = 0.01;
  int maxCorners = 100;

  vector<Point2f> corners;

  // Apply corner detection
  goodFeaturesToTrack(preprocesedFrame,
                      corners,
                      maxCorners,  //maximum number of corners to detect
                      qualityLevel, //minimal accepted quality of image corners
                      minDistance, //
                      Mat(),
                      blockSize,
                      useHarrisDetector,
                      k);

  std::sort(corners.begin(), corners.end(), [] (Point2f p, Point2f q) {
      return p.y > q.y;
  }); 
  
  vector<Point2f> bottomCorners(corners.begin(), corners.begin() + 10);
  
  //Draw circles on corners
  for ( int i = 0; i < bottomCorners.size(); i++ ){ 
    circle( displayImage, bottomCorners[i], 5, Scalar(255,255,0), -1, 8, 0); 
  }

  cout << "Number of corners " << corners.size() << endl;

  imshow("Result", displayImage);

}