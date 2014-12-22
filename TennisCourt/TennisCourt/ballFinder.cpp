#include "ballFinder.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <string>

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
  //cout << "in isSimilar..." << endl;
  if (abs(a) < 4 && abs(b) < 4) return true;
  if (oppositeSigns(a, b)) return false;
  // need something smarter here for the method to work...
  //cout << "check if within bound..." << endl;
  return abs(b / a) < 1.35 && abs(b / a) > 0.65;
}

bool BallFinder::matchesCurrentPath(struct ballCandidate candidate, Point2f point, int frameDifference) {
  //cout << "check if " << point << " matches.." << endl;
  Point2f diff = point - candidate.lastPosition;
  //cout << "is xdiff similar? " << endl;
  if (!isSimilar(diff.x , frameDifference * candidate.xDiff)) return false;
  //cout << "is ydiff sim? " << endl;
  if (!isSimilar(diff.y , frameDifference * candidate.yDiff)) return false;
  //cout << "both are" << endl;
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
  //int xDiff, yDiff;
  //cout << "in findBallCandidates with point" << p << endl;
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
      //cout << "check " << next << endl;
      for (ballCandidate prev : currentBallCandidates) {
        // check if next maches the path of prev
        if (norm(prev.lastPosition - next) < maxDistanceBetweenFrames 
                && norm(prev.lastPosition - next) > minDistanceBetweenFrames) {
          //cout << next << " matched!" << endl;
          if ((prev.xDiff < eps && prev.yDiff < eps) || matchesCurrentPath(prev, next, 1)) { 
            //cout << "match" << endl;
            ballCandidate c;
            c.xDiff = next.x - prev.lastPosition.x;
            c.yDiff = next.y - prev.lastPosition.y;
            c.lastPosition = next;
            nextBallCandidates.push_back(c);
          } 
        }
      }
    }
    if (nextBallCandidates.size() == 0) {
      //cout << "early returnn" <<endl;
      return nextBallCandidates;
    }
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

vector<Point2f> BallFinder::getCentres() {
  vector<Point2f> centres;
  int i = 1;
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
      putText(frame, std::to_string(i), center, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0),1,8,false );
      ++i;
      //circle(frame, center, 4, Scalar(0, 255, 0), -1, 8);
  }
  return centres;
}


/* return centres of small, isolated contours (expect such a contour to be a ball) */
vector<Point2f> BallFinder::getIsolatedPoints() {
  vector<Point2f> isolatedPoints;
  //vector<vector<Point> > isolatedContours;
  vector<bool> notIsolated;
  size_t i, j; 
  for (i = 0; i < contours.size(); ++i)
    notIsolated.push_back(false);
  for (i = 0; i < contours.size(); ++i) {
    if (notIsolated[i]) continue;
    for (j = 0; j < contours.size(); ++j) {
      if (i != j && lowDistanceBetweenContours(i, j)) {
        notIsolated[i] = true;
        notIsolated[j] = true;
        break;
      }
    }
  }
  for (i = 0; i < contours.size(); ++i) {
    if (!notIsolated[i] && contours[i].size() <= ballContourSizeThreshold) {
      //isolatedContours.push_back(contours[i]);
      isolatedPoints.push_back(centres[i]);
      circle(frame, centres[i], 4, Scalar(0,255,0), -1, 8);
    }
  }
  //line(frame, Point(100, 100), Point(100, 100 + isolationThreshold), Scalar(255, 0, 0), 2, 8, 0);
  //drawContours(frame, isolatedContours, -1, cv::Scalar(0, 255, 0), 0.1);
  return isolatedPoints;
}

int BallFinder::lowDistanceBetweenContours(int i, int j) {
  vector<Point> rep1 = representatives[i];
  vector<Point> rep2 = representatives[j];
  for (Point p1 : rep1) 
    for (Point p2 : rep2) 
      if (norm(p1 - p2) < isolationThreshold) return true;
  return false;
}

vector< vector<Point> > BallFinder::getRepresentatives() {
  vector< vector<Point> > representatives;
  vector<Point> cur;
  for (int i = 0; i < contours.size(); ++i) {
    int j = 0;
    while (j < contours[i].size()) {
      cur.push_back(contours[i][j]);
      circle(frame, contours[i][j], 1, Scalar(255,0,0), -1, 8);
      j += representativeFrequency;
    }
    representatives.push_back(cur);
    cur.clear();
  }
  return representatives;
}

void BallFinder::printContour(vector<Point> contour) {
  for (Point p : contour) cout << p << ", ";
  cout << endl;
}

bool BallFinder::findBall(Point2f &ballpos) {
  size_t size = ballCandidates.size();
  
  bool found = false;
  // check ballCandidates...
  if (ballCandidates.size() == 1) {
    //cout << "one ball candidate" << endl;
    ballCandidate *candidate = &ballCandidates[0];
    //cout << "at position: " << (*candidate).lastPosition << endl;
    Point2f currentPosition = findCurrentPosition(*candidate, frameDifference);
    //cout << "cur pos: " << currentPosition << endl;
    if (norm(currentPosition) != 0) {
      //cout << "current position found" << endl;
      circle(frame, currentPosition, 4, Scalar(255,0,0), -1, 8);
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
    //cout << "multiple ball candidates" << endl;
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
      //cout << "look for ball candidates" << endl;
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

bool BallFinder::isClose(object &obj, Point2f &p) {
  return isInside(obj, p) || isNearby(obj, p);
}

bool BallFinder::isNearby(object &obj, Point2f &p) {
  return norm(obj.top - p) <= 2 * isolationThreshold ||
         norm(obj.bottom - p) <= 2 * isolationThreshold ||
         norm(obj.right - p) <= 2 * isolationThreshold ||
         norm(obj.left - p) <= 2 * isolationThreshold; 
}

bool BallFinder::isInside(object &obj, Point2f &p) {
  return p.x <= obj.right.x && p.x >= obj.left.x &&
         p.y >= obj.top.y && p.y <= obj.bottom.y;
}

void BallFinder::updateObject(object &obj, Point2f &p) {
  if (obj.top.y > p.y) obj.top = p;
  if (obj.bottom.y < p.y) obj.bottom = p;
  if (obj.right.x < p.x) obj.right = p;
  if (obj.left.x > p.x) obj.left = p;
}

/* try to find a lot of contours close to each other or big contours */
vector<object> BallFinder::findPlayerCandidates() {
  vector<object> result;
  object cur;
  //maxRight = maxLeft = maxTop = maxBottom = 0;
  int size = 0;
  int i = 0;
  //vector< vector<Point> >::iterator it = contours.begin();
  while (i < contours.size()) {
    if (size == 0) { 
      //cur.right = cur.left = centres[i].x;
      //cur.top = cur.bottom = centres[i].y;
      cur.top = cur.bottom = cur.right = cur.left = centres[i];
      ++size;
      ++i;
    } else if (isClose(cur, centres[i])) {
      updateObject(cur, centres[i]);
      ++size;
      ++i;
    } else {
      if (size >= minPlayerSize) {
        cout << "found player candidate!" << endl;
        result.push_back(cur);
      }
      //cout << "size: " << size << endl;
      size = 0;
    }
  }
  cout << "num of player candidates: " << result.size() << endl;
  return result;
}

bool BallFinder::updatePlayerCandidate(object &candidate) {
  return false;
}

void BallFinder::findPlayers(vector<object> &players) {
  players = findPlayerCandidates();
  /*
  if (playerCandidates.size() == 0) 
    findPlayerCandidates();
  else {
    bool modified = false;
    for (int i = 0; i < playerCandidates.size(); ++i) {
      if (!updatePlayerCandidate(playerCandidates[i])) {
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
  */
}

/* returns true if it found the ball. Also tries to return positions of both players -
 * if the players vector is nonempty then it found the positions of players */
bool BallFinder::addFrame(const Mat &frame, Point2f &ballpos, vector<object> &players) {
  updateFrames(frame);

  //update the background model
  pMOG2.operator()(frame, fgMask, 0.01);

  // do an opening (erosion and dilation) on the mask
  erode(fgMask, fgMask, Mat());
  dilate(fgMask, fgMask, Mat()); 

  findContours(fgMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
  drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 0.1);

  representatives = getRepresentatives();
  centres = getCentres();
  vector<Point2f> isolatedPts = getIsolatedPoints();
  updatesetsOfIsolatedPoints(isolatedPts);
  
  /*
  Mat dst, detected_edges;

  int edgeThresh = 1;
  int lowThreshold = 10;
  int ratio = 3;
  int kernel_size = 3;
  const char* window_name = "Edge Map";

  dst.create( frame.size(), frame.type() );

  /// Reduce noise with a kernel 3x3
  blur( frame, detected_edges, Size(3,3) );
  
  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  
  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);
  
  frame.copyTo( dst, detected_edges);
  imshow( window_name, dst );
  */

  findPlayers(players);
  return findBall(ballpos);
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

  int keyboard = 0;
  int k = 0;

  while ((char) keyboard != 'q' && keyboard != 27) {
    if ((char) keyboard == 's') {
      // skip 10 frames
      for (int i = 0; i < 10; ++i) {
        if (capture.read(frame))
          updateFrames(frame);
      }
    }

    while (k < 770) {
      capture.read(frame);
      updateFrames(frame);
      ++k;
    }

    if (!capture.read(frame)) {
      cerr << "unable to read frame, exiting... " << endl;
      exit(EXIT_FAILURE);
    }

    vector<object> players;
    Point2f ballpos;

    if (addFrame(frame, ballpos, players)) {
      cout << "have the ball..." << endl;
    }

    imshow("Frame", frame);
    keyboard = waitKey(0);
  }

  capture.release();
  return 0;
}

