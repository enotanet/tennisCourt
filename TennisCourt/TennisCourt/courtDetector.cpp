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

const double k_foot = 0.3048;

// Uncomment this for manual testing
/*int main(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    Mat image = imread(argv[i], CV_LOAD_IMAGE_GRAYSCALE);

    cout << "TestCourt Image location: " << argv[1] << endl;

    diplayCourtDetectorResult(image, "Result");

    waitKey(0);
  }

  return EXIT_SUCCESS;
}*/



/**
 *
 */
vector<pair<Point2f, Point3d>> mapCourtCornersTo3DCoordinates(Mat &frame, vector<Point2f> courtCorners, int camera) {
  
  vector<pair<Point2f, Point3d>> result;
  int xCoefficient = 1;
  int yCoefficient = 1;
  if (camera == 0 || camera == 2) xCoefficient = -1;
  if (camera == 2 || camera == 3) yCoefficient = -1;

  Point2f lowestCorner = courtCorners[0];
  for (Point corner : courtCorners) {
    if (corner.y > lowestCorner.y) {
      lowestCorner = corner;
    }
  }
  result.push_back(make_pair(lowestCorner, Point3d(xCoefficient * 18, yCoefficient * 39, 0)));

  std::sort(courtCorners.begin(), courtCorners.end(), [] (Point2f p, Point2f q) {    
    return p.x > q.x;
  });;

  vector<Point2f> shorterSideCorners;
  vector<Point2f> longerSideCorners;

  for (int i = 0; i < courtCorners.size(); i++) {
    double dx = lowestCorner.x - courtCorners[i].x;
    double dy = lowestCorner.y - courtCorners[i].y;
    if (xCoefficient * dx < 0
        && dy < 0.2 * frame.rows) {
      shorterSideCorners.push_back(courtCorners[i]);
    } else if (xCoefficient * dx > 0
               && dy < 0.3 * frame.rows) {
      longerSideCorners.push_back(courtCorners[i]);
    }
  }
  
  if (shorterSideCorners.size() > 0) {
    result.push_back(make_pair(shorterSideCorners[0], Point3d(xCoefficient * 18, yCoefficient * 21, 0)));
    if (shorterSideCorners.size() > 1) {
      result.push_back(make_pair(shorterSideCorners[1], Point3d(xCoefficient * 13.5, yCoefficient * 21, 0)));
    }
  }  

  for (Point corner : longerSideCorners) {
    double dx = abs(lowestCorner.x - corner.x);
    double dy = lowestCorner.y - corner.y;
    if (dx < 0.1 * frame.cols
        && dy < 0.25 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 13.5, yCoefficient * 21, 0)));
    } else if (dx < 0.15 * frame.cols
               && dy < 0.1 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 13.5, yCoefficient * 39, 0)));
    } else if (dx < 0.175 * frame.cols
               && dy < 0.25 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(0, yCoefficient * 21, 0)));
    } else if (dx < 0.325 * frame.cols
               && dx > 0.225 * frame.cols
               && dy < 0.2 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(0, yCoefficient * 39, 0)));
    } else if (dx < 0.35 * frame.cols
               && dx > 0.325 * frame.cols
               && dy < 0.3 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 13.5, yCoefficient * 21, 0)));
    } else if (dx < 0.425 * frame.cols
               && dx > 0.35 * frame.cols
               && dy < 0.2 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 13.5, yCoefficient * 39, 0)));
    } else if (dx < 0.425 * frame.cols
               && dx > 0.325 * frame.cols
               && dy < 0.5 * frame.rows
               && dy > 0.2 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 18, yCoefficient * 21, 0)));
    } else if (dx < 0.55 * frame.cols
               && dx > 0.25 * frame.cols
               && dy < 0.2 * frame.rows) {
      result.push_back(make_pair(corner, Point3d(xCoefficient * 18, yCoefficient * 39, 0)));
    } 
  }

  return result;
}

/**
 * Returns x-y coordinates of the court corner position on the image.
 * Parameters:
 * Mat frame - frame (image) from the camera.
 */
vector<Point2f> getCourtCorners(Mat &frame) {
  
  Mat preprocesedFrame = getCourtOnly(frame);
    
  // Apply Hough Transform to find lines
  vector<Vec4i> lines;
  HoughLinesP(preprocesedFrame, 
              lines,
              1, 
              CV_PI/180, 
              0.0001 * frame.cols * frame.rows, 
              min(0.1 * frame.cols, 0.1 * frame.rows), 
              min(0.005 * frame.cols, 0.005 * frame.rows));

  // Draw lines
  for( size_t i = 0; i < lines.size(); i++ ) {
    Vec4i l = lines[i];
    //line(out, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 2, CV_AA);
  }
  
  // Find intersections of all lines
  vector<Point2f> intersections;
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l1 = lines[i];
    int a1 = l1[1] - l1[3];
    int b1 = l1[2] - l1[0];
    int c1 = a1*l1[0] + b1*l1[1];
    for (size_t j = i + 1; j < lines.size(); j++) {
      Vec4i l2 = lines[j];
      int a2 = l2[1] - l2[3];
      int b2 = l2[2] - l2[0];
      int c2 = a2*l2[0] + b2*l2[1];
      double det = a1*b2 - a2*b1;
      if (det != 0) {
        double x = (b2*c1 - b1*c2) / det;
        double y = (a1*c2 - a2*c1) / det;
        if ( x < frame.cols && y < frame.rows) { 
          intersections.push_back(Point(x,y));
        }
      }
    }
  }

  // Vector with points that correspoind to corners
  vector<Point2f> courtCorners;
  // Vector with booleans that correspond whether the intersection was considered or not
  vector<bool> consideredIntersection(intersections.size());

  // Find an average point of intersections that fall within same 2.5% of cols x 2.5% of rows bucket.
  for (int i = 0; i < intersections.size(); i++) { 
    vector<Point2f> nearByIntersections;
    nearByIntersections.push_back(intersections[i]);
    consideredIntersection[i] = true;
    for (int j = 0; j < intersections.size(); j++) { 
      if (!consideredIntersection[j]
          && abs(intersections[i].x - intersections[j].x) < 0.025 * frame.cols
          && abs(intersections[i].y - intersections[j].y) < 0.025 * frame.rows) {
       nearByIntersections.push_back(intersections[j]);            
       consideredIntersection[j] = true;
     }
    }
    // Take average only if number of nearby intersections is > 1.5% of all intersections
    if (nearByIntersections.size() > 0.015 * intersections.size()) {
      double averageX = 0;
      double averageY = 0;
      for (Point intersection : nearByIntersections) {
        averageX += intersection.x;
        averageY += intersection.y;
      }
      averageX = averageX / nearByIntersections.size();
      averageY = averageY / nearByIntersections.size();
      courtCorners.push_back(Point(averageX,averageY));
    }
  }

  return courtCorners;
}


/**
 * This method takes a frame and preprocesses it to remove everything that is not the court from it.
 */
Mat getCourtOnly(Mat &frame) {

  // Get lower 50% of the image
  Rect cropArea(0, 0.5 * frame.rows, frame.cols - 1, 0.5 * frame.rows - 1);
  Mat croppedImage(frame.clone());
  croppedImage = croppedImage(cropArea);

  // Get number of pixels by intensity of the lower 50% of the image.
  int* croppedImageIntensities = getIntensities(croppedImage);

  // Initialise the bounds for the intensities in which the court lines will fall. 
  int upperBoundIntensity = 255;
  int lowerBoundIntensity = 0;


  // Find the upped bound. The first value at which at least 0.01% of all pixels falls.
  for (int i = 255; i >= 0; i--) {
    if (croppedImageIntensities[i] > 0.0001 * croppedImage.cols * croppedImage.rows) {
      upperBoundIntensity = i;
      break;
    }
  }
  
  /* Find the lower bound. The intensity value for which at least 0.1% 
   * of all pixels more pixels correspond to than to the previous higher intensity value.
   */
  for (int i = upperBoundIntensity; i >= 0; i--) {
    if (croppedImageIntensities[i] - croppedImageIntensities[upperBoundIntensity] > 0.001 * croppedImage.cols * croppedImage.rows) {
      lowerBoundIntensity = i;
      break;
    } 
  }

  int maxCourtLineWidth = max(0.01 * frame.cols, 0.01 * frame.rows);
  
  Mat out(frame.clone());

  int thresh = (lowerBoundIntensity - getModeIntensity(croppedImage));

  for (int y = 0; y < out.rows; y++) {
    for (int x = 0; x < out.cols; x++) {
      if ( out.at<uchar>(Point(x,y)) < upperBoundIntensity
           && out.at<uchar>(Point(x,y)) > lowerBoundIntensity) {
        if ( x - maxCourtLineWidth >= 0 
             && x + maxCourtLineWidth < out.cols
             && out.at<uchar>(Point(x,y)) - out.at<uchar>(Point(x - maxCourtLineWidth, y)) > thresh
             && out.at<uchar>(Point(x,y)) - out.at<uchar>(Point(x + maxCourtLineWidth, y)) > thresh) {
          out.at<uchar>(Point(x,y)) = 255;  
          continue;
        } else if (y - maxCourtLineWidth >= 0 
                   && y + maxCourtLineWidth < out.rows
                   && out.at<uchar>(Point(x,y)) - out.at<uchar>(Point(x, y - maxCourtLineWidth)) > thresh 
                   && out.at<uchar>(Point(x,y)) - out.at<uchar>(Point(x, y + maxCourtLineWidth)) > thresh) {
          out.at<uchar>(Point(x,y)) = 255;  
          continue;
        }
      }
      out.at<uchar>(Point(x,y)) = 0;
    }
  }

  //namedWindow("Preprocessed Frame", CV_WINDOW_AUTOSIZE);
  //imshow("Preprocessed Frame", out);

  return out;
}

void diplayCourtDetectorResult(Mat &frame, string windowName) {

  Mat out;
  cvtColor(frame, out, CV_GRAY2RGBA);

  vector<Point2f> courtCorners = getCourtCorners(frame);

  // Draw circles on corners
  for (Point2f corner : courtCorners){ 
    circle(out, corner, 3, Scalar(0,0,255), -1, 8, 0); 
  }

  vector<pair<Point2f, Point3d>> courtCornersTo3DCoordinates = mapCourtCornersTo3DCoordinates(frame, courtCorners, 1);
  for (pair<Point2f, Point3d> p : courtCornersTo3DCoordinates) {
    Point2f corner = p.first;
    Point3d coordinateIn3D = p.second;
    Mat out;
    cvtColor(frame, out, CV_GRAY2RGBA);
    char buf[128];
    sprintf(buf, "(%lf, %lf, %lf)", coordinateIn3D.x, coordinateIn3D.y, coordinateIn3D.z);
    putText(out, buf, Point(50,50), CV_FONT_NORMAL, 0.75, Scalar(255,0,0), 2);
    circle(out, corner, 3, Scalar(255,0,0), -1, 8, 0);
    namedWindow("3D Coor", CV_WINDOW_AUTOSIZE);
    imshow("3D Coor", out);
    waitKey(0);
  }

  namedWindow(windowName, CV_WINDOW_AUTOSIZE);
  imshow(windowName, out);
}
