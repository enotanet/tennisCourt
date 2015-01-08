#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <stdio.h>
#include <cstdlib>

using namespace std;
using namespace cv;

/**
 * Changes pixels intensity of an image if not inversed then outside of the range to 0, if inversed then within the range to 0.
 * Parameters:
 * Mat &image - reference to an image
 * int lowerBoundIntensity - lower bound of the intensities range
 * int upperBoundIntensity - uppwe bound of the intensities range
 * bool inverse
 */
void cutBetweenIntensities(Mat &image, int lowerBoundIntensity, int upperBoundIntensity, bool inverse=false) {
  if (inverse) {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        if ( ((int) image.at<uchar>(i, j)) > lowerBoundIntensity || ((int) image.at<uchar>(i, j)) < upperBoundIntensity) {
          image.at<uchar>(i, j) = 0;
        }
      }
    }
  } else {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        if ( ((int) image.at<uchar>(i, j)) < lowerBoundIntensity || ((int) image.at<uchar>(i, j)) > upperBoundIntensity) {
          image.at<uchar>(i,j) = 0;
        }
      }
    }
  }
}

/**
 * Returns an array that holds a number of pixels for each intensity value;
 * Parameters:
 * Mat &image - reference to an image
 */
int* getIntensities(Mat &image) {
  // Array to hold a number of pixels for each intensity value (0-255)
  int *intensities = new int[256];
  for(int i = 0; i < 255; i++) intensities[i] = 0;
  
  // Calculate the number of pixels for each intensity value
  for(int y = 0; y < image.rows; y++)
  {
    for(int x = 0; x < image.cols; x++)
    {
      intensities[(int)image.at<uchar>(y,x)]++;
    }
  }

  return intensities;
}


/**
 * Returns an intensity value that occurs the most in the image
 * Parameters:
 * Mat &image - reference to an image
 */
int getModeIntensity(Mat &image) {
  
  int *intensities = getIntensities(image);
  int maxNumberOfPixels = intensities[0];
  int modeIntensity = 0;

  for (int i = 1; i < 255; i++) {
    if (intensities[i] > maxNumberOfPixels) {
      maxNumberOfPixels = intensities[i];
      modeIntensity = i;
    }
  }
  
  return modeIntensity;
}