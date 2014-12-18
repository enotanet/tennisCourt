#include "ballFinder.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>

using namespace cv;
using namespace std;

const double eps = 1e-7;

void processVideo(char*);
void writeFrameNumber(Mat, VideoCapture);

void BallFinder::updateFrames(const Mat &frame) {
  frames.insert(frames.begin(), frame.clone());
  if (frames.size() > numberOfFrames) {
    frames.pop_back();
  }
}

void BallFinder::updatesetsOfIsolatedPoints(vector<Point2f> isolatedPts) {
  setsOfIsolatedPoints.insert(setsOfIsolatedPoints.begin(), isolatedPts);
  if (setsOfIsolatedPoints.size() > numberOfFrames) {
    setsOfIsolatedPoints.pop_back();
  }
}

bool BallFinder::oppositeSigns(double a, double b) {
  return (a * b < 0);
//  return (a > 0 && b < 0) || (a < 0 && b > 0);
}

bool BallFinder::isSimilar(double a, double b) {
  if (abs(a) < 4 && abs(b) < 4) return true;
  if (oppositeSigns(a, b)) return false;
  // need something smarter here for the method to work...
  return abs(b / a) < 1.35 && abs(b / a) > 0.65;
}

bool BallFinder::matchesCurrentPath(struct ballCandidate candidate, Point2f point, int frameDifference) {
  //cout << "check if " << point << " matches.." << endl;
  Point2f diff = point - candidate.lastPosition;
  if (!isSimilar(diff.x , frameDifference * candidate.xDiff)) return false;
  if (!isSimilar(diff.y , frameDifference * candidate.yDiff)) return false;
  //candidate.xdiff = diff.x;
  //candidate.ydiff = diff.y;
  //candidate.lastPosition = point;
  //cout << "found match: " << point << endl;
  return true;
}

Point2f BallFinder::findCurrentPosition(ballCandidate candidate, int frameDifference) {
  for (Point2f p : setsOfIsolatedPoints[0]) {
    if (matchesCurrentPath(candidate, p, frameDifference)) return p;
  }
  return Point2f(0, 0);
}

void BallFinder::updateCurrentPosition(ballCandidate *candidate, Point2f currentPosition, int frameDifference) {
  (*candidate).xDiff = (currentPosition.x - (*candidate).lastPosition.x) / frameDifference;
  (*candidate).yDiff = (currentPosition.y - (*candidate).lastPosition.y) / frameDifference;
  (*candidate).lastPosition = currentPosition;
}

vector<ballCandidate> BallFinder::findBallCandidates(Point2f p) {
  int xDiff, yDiff;
  ballCandidate first;
  first.lastPosition = p;
  first.xDiff = 0;
  first.yDiff = 0;
  vector<Point2f> isolatedPoints;
  vector<ballCandidate> currentBallCandidates;
  vector<ballCandidate> nextBallCandidates;
  currentBallCandidates.push_back(first);
  for (int i = numberOfFrames - 2; i >= 0; --i) {
    isolatedPoints = setsOfIsolatedPoints[i];
    for (Point2f next : isolatedPoints) {
      for (ballCandidate prev : currentBallCandidates) {
        // check if next maches the path of prev
        if (norm(prev.lastPosition - next) < maxDistanceBetweenFrames 
                && norm(prev.lastPosition - next) > minDistanceBetweenFrames) {
          //cout << next << " matched!" << endl;
          if ((prev.xDiff < eps && prev.yDiff < eps) || matchesCurrentPath(prev, next, 1)) { 
            ballCandidate c;
            c.xDiff = next.x - prev.lastPosition.x;
            c.yDiff = next.y - prev.lastPosition.y;
            c.lastPosition = next;
            nextBallCandidates.push_back(c);
          } 
        }
      }
    }
    if (nextBallCandidates.size() == 0) return nextBallCandidates;
    currentBallCandidates = nextBallCandidates;
    nextBallCandidates.clear();
  }
  return currentBallCandidates;
}

bool BallFinder::hasRightTrajectory(Point2f p) {
  //cout << "check point " << p << endl;
  double xdiff = 0;
  double ydiff = 0;
  Point2f cur = p;
  bool found;
  for (int i = numberOfFrames - 2; i >= 0; --i) {
    vector<Point2f> isolatedPoints = setsOfIsolatedPoints[i];
    found = false;
    for (Point2f next : isolatedPoints) {
      if (norm(cur - next) < maxDistanceBetweenFrames && norm(cur - next) > minDistanceBetweenFrames) {
        //cout << next << " matched!" << endl;
        if (abs(xdiff) < eps) xdiff = next.x - cur.x;
        else {
          if (!isSimilar(next.x - cur.x, xdiff)) return false;
          xdiff = next.x - cur.x;
        }
        if (abs(ydiff) < eps) ydiff = next.y - cur.y;
        else {
          if (!isSimilar(next.y - cur.y, ydiff)) return false;
          ydiff = next.y - cur.y;
        }
        cur = next;
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

ballCandidate BallFinder::recoverBallCandidate(Point2f p) {
  Point2f cur = p;
  vector<Point2f> isolatedPoints;
  double xdiff, ydiff;
  for (int i = numberOfFrames - 2; i >= 0; --i) {
    isolatedPoints = setsOfIsolatedPoints[i];
     for (Point2f next : isolatedPoints) {
       if (norm(cur - next) < maxDistanceBetweenFrames) {
         xdiff = next.x - cur.x;
         ydiff = next.y - cur.y;
         cur = next;
         break;
       }
     }
  }
  ballCandidate candidate;
  candidate.lastPosition = cur;
  candidate.xDiff = xdiff;
  candidate.yDiff = ydiff;
  return candidate;
}

vector<Point2f> BallFinder::getCentres(vector<vector<Point> > contours) {
  vector<Point2f> centres;
  for (vector<Point> contour : contours) {
      // for each controur take one point 'representing that contour' 
      Point2f center(0,0);
      size_t numberOfPoints = contour.size();
      for (Point point : contour) {
        center.x += point.x;
        center.y += point.y;
      }
      center.x = center.x / numberOfPoints;
      center.y = center.y / numberOfPoints;
      centres.push_back(center);
      //circle(frame, center, 4, Scalar(0, 255, 0), -1, 8);
  }
  return centres;
}

vector<Point2f> BallFinder::getIsolatedPoints(vector<vector<Point> > contours, vector<Point2f> centres) {
  vector<Point2f> isolatedPoints;
  for (size_t i = 0; i < contours.size(); ++i) {
      Point2f point = centres[i];
      size_t contourSize = contours[i].size();
      int j = 0;
      for (; j < centres.size(); ++j) {
        if (i != j) {
          if (norm(point - centres[j]) < 100) {
            break;
          }
        }
      }
    if (j == centres.size()) {
      isolatedPoints.push_back(point);
    }
  }
  return isolatedPoints;
}

bool BallFinder::findBall(vector< vector<Point> > &contours, Point2f &ballpos) {
  vector<Point2f> centres = getCentres(contours);

  // check for isolated points
  vector<Point2f> isolatedPts = getIsolatedPoints(contours, centres);
  updatesetsOfIsolatedPoints(isolatedPts);

  size_t size = ballCandidates.size();
  
  bool found = false;
  // check ballCandidates...
  if (ballCandidates.size() == 1) {
    ballCandidate *candidate = &ballCandidates[0];
    //cout << "ball at position: " << (*candidate).lastPosition << endl;
    Point2f currentPosition = findCurrentPosition(*candidate, frameDifference);
    //cout << "cur pos: " << currentPosition << endl;
    if (norm(currentPosition) != 0) {
      //cout << "draw a circle " << endl;
      ballpos = currentPosition;
      found = true;
      updateCurrentPosition(candidate, currentPosition, frameDifference);
      frameDifference = 1;
    } else if (frameDifference > 3) {
      ballCandidates.clear();
      frameDifference = 1;
    } else {
      ++frameDifference;
    }
  } else if (ballCandidates.size() > 1) {
    for (int i = 0; i < ballCandidates.size(); ++i) {
      ballCandidate *candidate = &ballCandidates[i];
      Point2f currentPosition = findCurrentPosition(*candidate, frameDifference);
      if (norm(currentPosition) != 0)
        updateCurrentPosition(candidate, currentPosition, frameDifference);
      else { // if not found then remove this ball candidate
        ballCandidates.erase(ballCandidates.begin() + i);
        --i; // not sure if this is gonna work (erasing elements from a vector while iterating through it
      }
    }
  } else {
    // no ball candidates so far
    if (setsOfIsolatedPoints.size() == numberOfFrames) {
      vector<Point2f> startPoints = setsOfIsolatedPoints[numberOfFrames - 1];
      vector<ballCandidate> candidates;
      for (Point2f p : startPoints) {
        // new way of doing that
        candidates = findBallCandidates(p);
        if (candidates.size() > 0) 
          ballCandidates.insert(ballCandidates.begin() + ballCandidates.size(), candidates.begin(),
            candidates.end());
        /*
        if (hasRightTrajectory(p)) {
          //cout << "right trajectory found!" << endl;
          ballCandidate candidate = recoverBallCandidate(p);
          ballCandidates.push_back(candidate);
        }
        */
      }
    }
  }

  // prepare args for a call to calcOpticalFlow - VERY wrong comment I guess.
  //vector<uchar> status;
  //vector<float> err;
  //TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03);
  //Size subPixWinSize(10,10), winSize(31,31);
  //vector<Point2f> nextPoints;


  //int size1 = centres.size();
  //int size2 = nextPoints.size();

  //vector<Point2f> ballCandidates;

  return found;
}

void BallFinder::findPlayerCandidates(vector< vector<Point> > &contours) {

}

bool BallFinder::updatePlayerCandidate(person &candidate, vector< vector<Point> > &contours) {

}

void BallFinder::findPlayers(vector< vector<Point> > &contours, vector<person> &players) {
  if (playerCandidates.size() == 0) 
    findPlayerCandidates(contours);
  else {
    bool modified = false;;
    for (int i = 0; i < playerCandidates.size(); ++i) {
      if (!updatePlayerCandidate(playerCandidates[i], contours)) {
        playerCandidates.erase(playerCandidates.begin() + i);
        --i;
        modified = true;
      }
    }
    if (modified) playersConsistency = 0;
    else if (playerCandidates.size() == 2) ++playersConsistency;
    if (playersConsistency > playersConsistencyThreshold)
      players = playerCandidates;
  }
}

/* returns true if it found the ball. Also tries to return positions of both players -
 * if the players vector is nonempty then it found the positions of players */
bool BallFinder::addFrame(const Mat &frame, Point2f &ballpos, vector<person> &players) {
  vector< vector<Point> > contours;

  updateFrames(frame);

  //update the background model
  pMOG2.operator()(frame, fgMask, 0.01);

  // do an opening (erosion and dilation) on the mask
  erode(fgMask, fgMask, Mat());
  dilate(fgMask, fgMask, Mat()); 

  findContours(fgMask,  contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 0.1);

  findPlayers(contours, players);
  return findBall(contours, ballpos);
  
}


int BallFinder::mymain(int argc, char *argv[]) 
{
  if (argc < 2) {
    cerr << "please specify filename" << endl;
    exit(EXIT_FAILURE);
  }

  char* filename = argv[1];
  cout << "filename: " << filename << endl;
  VideoCapture capture(filename);
  
  if (!capture.isOpened()) {
    cerr << "Unable to open video file " << endl;
    exit(EXIT_FAILURE);
  }

  cout << capture.get(CV_CAP_PROP_FPS) << endl;

  cv::Mat frame;
  bool success = capture.read(frame);
  capture.read(frame);
  capture.read(frame);
  cout << capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;

  int keyboard = 0;
  Point2f ballpos;

  while ((char) keyboard != 'q' && keyboard != 27) {
    if ((char) keyboard == 's') {
      // skip 10 frames
      for (int i = 0; i < 10; ++i) {
        if (capture.read(frame))
          updateFrames(frame);
      }
    }
    cout << "before" << endl;
    capture.read(frame);
    cout << "after" << endl;

    if (!capture.read(frame)) {
      cerr << "unable to read frame, exiting... " << endl;
      exit(EXIT_FAILURE);
    }

    cout << "call add frame" << endl;
    vector<person> players;

    if (addFrame(frame, ballpos, players)) {
      cout << "have the ball..." << endl;
    }

    imshow("Frame", frame);
    keyboard = waitKey(0);
  }

  capture.release();
  
}

