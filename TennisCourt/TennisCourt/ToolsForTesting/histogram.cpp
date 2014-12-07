#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdlib>

#include "histogram.h"

using namespace std;
using namespace cv;

int histogramWidth = 512; 
int histogramHeight = 400;
int lineThickness = cvRound((double) histogramWidth/256);

/**
 * This method displays the histogram for the given image. 
 * Parameters:
 * Mat &image - reference to an image for which histogram is displayed.
 * string windowName - name of the window in which histogram will be displayed.
 * (optional) vector<int> intensitiesToHighlight - values of intensities at which to draw a vertical red line
 */
void displayHistogram(Mat &image, string windowName, vector<int> &intensitiesToHighlight) {
  // Display histogram in a new window
  Mat histogram = drawHistogram(image);
  cvtColor(histogram, histogram, CV_GRAY2RGBA);

  if (!intensitiesToHighlight.empty()) {
    for (int intensity : intensitiesToHighlight) {
      line(histogram, 
           Point(lineThickness * intensity, histogramHeight), 
           Point(lineThickness * intensity, 0), 
           Scalar(0,0,255), 1, 8, 0);
    }
  }

  namedWindow(windowName, CV_WINDOW_AUTOSIZE); 
  imshow(windowName, histogram);
}

/**
 * This method returns the reference to intensities histogram image (Mat&) for the given image.
 * Parameters:
 * Mat image - reference to an image for which histogram is drawn.
 */
Mat drawHistogram(Mat &image) {
  // Array to hold a number of pixels for each intensity value (0-255)
  int intensities[256];
  for(int i = 0; i < 255; i++) intensities[i] = 0;
  
  // Calculate the number of pixels for each intensity value
  for(int y = 0; y < image.rows; y++)
  {
    for(int x = 0; x < image.cols; x++)
    {
      intensities[(int)image.at<uchar>(y,x)]++;
    }
  }
  // Histogram to be displayed
  Mat histogram(histogramHeight, histogramWidth, CV_8UC1, Scalar(255, 255, 255));

  // Find the maximum intensity element from histogram
  int max = intensities[0];
  for(int i = 1; i < 256; i++)
  {
    if(max < intensities[i])
    {
      max = intensities[i];
    }
  }

  // Normalize the histogram to fit vertically
  for(int i = 0; i < 255; i++){
    intensities[i] = ( (double) intensities[i] / max) * histogramHeight;
  }

  // Draw the intensity lines for histogram
  for(int i = 0; i < 255; i++)
  {
    line(histogram, Point(lineThickness * i, histogramHeight), Point(lineThickness * i, histogramHeight - intensities[i]), Scalar(0,0,0), 1, 8, 0);
  }
  
  return histogram;
}

