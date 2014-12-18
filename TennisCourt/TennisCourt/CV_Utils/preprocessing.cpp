#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <stdio.h>
#include <cstdlib>

using namespace std;
using namespace cv;

/**
 * Changes pixels intensity of an image outside of the range if not inversed to 0 or if inversed to 255.
 * Parameters:
 * Mat &image - reference to an image
 * int lowerBoundIntensity - lower bound of the intensities range
 * int upperBoundIntensity - uppwe bound of the intensities range
 * bool inverse
 */
void cutBetweenIntensities(Mat &image, int lowerBoundIntensity, int upperBoundIntensity, bool inverse=false) {
  if (inverse)
  {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        if ( ((int) image.at<uchar>(i, j)) > lowerBoundIntensity || ((int) image.at<uchar>(i, j)) < upperBoundIntensity) {
          image.at<uchar>(i,j) = 0;
        }
      }
    }
  } else 
  {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        if ( ((int) image.at<uchar>(i, j)) < lowerBoundIntensity || ((int) image.at<uchar>(i, j)) > upperBoundIntensity) {
          image.at<uchar>(i,j) = 0;
        }
      }
    }
  }
}

