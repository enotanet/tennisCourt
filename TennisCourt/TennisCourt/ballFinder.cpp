#include "ballFinder.h"
#include "utils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <cmath>
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
}

bool BallFinder::isSimilar(double a, double b) {
  //cout << "in isSimilar..." << endl;
  if (abs(a) < 4 && abs(b) < 4) return true;
  if (oppositeSigns(a, b)) return false;
  // need something smarter here for the method to work...
  //cout << "check if within bound..." << endl;
  return abs(b / a) < 1.53 && abs(b / a) > 0.65;
}

bool BallFinder::matchesCurrentPath(struct ballCandidate candidate, Point2f point, int frameDifference) {
  Point2f diff = point - candidate.lastPosition;
  if (oppositeSigns(diff.x, frameDifference * candidate.xDiff) &&
      oppositeSigns(diff.y, frameDifference * candidate.yDiff)) return false; 
  if (!isSimilar(diff.x , frameDifference * candidate.xDiff)) return false;
  if (!isSimilar(diff.y , frameDifference * candidate.yDiff)) return false;
  return true;
}

Point2f BallFinder::findCurrentPosition(ballCandidate candidate, int frameDifference) {
  for (Point2f p : setsOfIsolatedPoints[0]) 
    if (matchesCurrentPath(candidate, p, frameDifference)) return p;
  return Point2f(0, 0);
}

void BallFinder::updateCurrentPosition(ballCandidate *candidate, Point2f currentPosition, int frameDifference) {
  (*candidate).xDiff = (currentPosition.x - (*candidate).lastPosition.x) / frameDifference;
  (*candidate).yDiff = (currentPosition.y - (*candidate).lastPosition.y) / frameDifference;
  (*candidate).lastPosition = currentPosition;
}

vector<ballCandidate> BallFinder::findBallCandidates(Point2f p) {
  // cout << "start from " << p << endl;
  ballCandidate first;
  first.lastPosition = p;
  first.xDiff = 0;
  first.yDiff = 0;
  vector<Point2f> isolatedPoints;
  vector<ballCandidate> currentBallCandidates;
  vector<ballCandidate> nextBallCandidates;
  currentBallCandidates.push_back(first);
  for (int i = numberOfFrames - 2; i >= 0; --i) {
    // cout << "next it" << endl;
    isolatedPoints = setsOfIsolatedPoints[i];
    for (Point2f next : isolatedPoints) {
      // cout << "check " << next << endl;
      for (ballCandidate prev : currentBallCandidates) {
        if (norm(prev.lastPosition - next) < maxDistanceBetweenFrames 
                && norm(prev.lastPosition - next) > minDistanceBetweenFrames) {
          if ((prev.xDiff < eps && prev.yDiff < eps) || matchesCurrentPath(prev, next, 1)) { 
            ballCandidate c;
            c.xDiff = next.x - prev.lastPosition.x;
            c.yDiff = next.y - prev.lastPosition.y;
            c.lastPosition = next;
            nextBallCandidates.push_back(c);
            // cout << "match" << endl;
          } 
        }
      }
    }
    if (nextBallCandidates.size() == 0) {
      // cout << "no match, ret early" << endl;
      return nextBallCandidates;
    }
    currentBallCandidates = nextBallCandidates;
    nextBallCandidates.clear();
  }
  DEBUG(endl << endl << endl << "got some ball candidates, size " << currentBallCandidates.size() << endl << endl << endl);
  for (ballCandidate bc : currentBallCandidates) {
    circle(frame, bc.lastPosition, 2, Scalar(40, 0, 40), -1, 8);
  }
  return currentBallCandidates;
}

/* METHOD NOT USED RIGHT NOW */
bool BallFinder::hasRightTrajectory(Point2f p) {
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

/* METHOD NOT USED RIGHT NOW */
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

/* get centres of each contour */
vector<Point2f> BallFinder::getCentres() {
  vector<Point2f> centres;
  //int i = 1;
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
      //putText(frame, std::to_string(i), center, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0),1,8,false );
      
      //++i;
      //circle(frame, center, 4, Scalar(0, 255, 0), -1, 8);
  }
  return centres;
}


/* return centres of small, isolated contours (expect such a contour to be a ball) */
vector<Point2f> BallFinder::getIsolatedPoints() {
  vector<Point2f> isolatedPoints;
  //vector<vector<Point> > isolatedContours;
  size_t i, j; 
  notIsolated.clear();
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
  //drawContours(frame, isolatedContours, -1, cv::Scalar(0, 255, 0), 0.1);
  return isolatedPoints;
}

/* check if two contours are close to each other */
int BallFinder::lowDistanceBetweenContours(int i, int j) {
  vector<Point> rep1 = representatives[i];
  vector<Point> rep2 = representatives[j];
  for (Point p1 : rep1) 
    for (Point p2 : rep2) 
      if (norm(p1 - p2) < isolationThreshold) return true;
  return false;
}

/* get representative points from each contour */
vector< vector<Point> > BallFinder::getRepresentatives() {
  vector< vector<Point> > representatives;
  vector<Point> cur;
  for (int i = 0; i < contours.size(); ++i) {
    int j = 0;
    while (j < contours[i].size()) {
      cur.push_back(contours[i][j]);
      j += representativeFrequency;
    }
    representatives.push_back(cur);
    cur.clear();
  }
  return representatives;
}

void BallFinder::printIsolatedPoints() {
  for (auto isolatedPoints : setsOfIsolatedPoints) {
    cout <<  "[";
    for (auto next : isolatedPoints) {
      cout << next << ", ";
    }
    cout << "]" << endl;
  }
}

void BallFinder::printContour(vector<Point> contour) {
  for (Point p : contour) cout << p << ", ";
  cout << endl;
}

void BallFinder::findPossibleBallPositions(int ballNotSeen, vector<Point2f> &candidates) {
  for (int i = 0; i < contours.size(); ++i) {
    if (contours[i].size() <= ballContourSizeThreshold && 
        norm(centres[i] - lastBallPosition) < sqrt(ballNotSeen) * maxDistanceBetweenFrames) {
      candidates.push_back(centres[i]);
      //circle(frame, centres[i], 4, Scalar(50, 100, 150), -1, 8);
      circle(frame, centres[i], 2, Scalar(50, 100, 150), -1, 8);
    }
  }
}

bool BallFinder::findBallNew(vector<Point2f> &ballpos, vector<Point2f> &candidates) {
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG("new frame");
  //line(frame, Point(100,100), Point(100,200),Scalar(0,0,255),1,8,0);
  size_t size = ballCandidates.size();
  bool found = false;

  //cout << "ballcand size " << size << endl;
  if (size == 1) {
    ballCandidate *candidate = &ballCandidates[0];
    Point2f currentPosition = findCurrentPosition(*candidate, frameDifference);
    // check if current position has been found
    if (norm(currentPosition) != 0) {
      circle(frame, currentPosition, 4, Scalar(255,0,0), -1, 8);
      ballpos.push_back(currentPosition);
      lastBallPosition = currentPosition;
      found = true;
      updateCurrentPosition(candidate, currentPosition, frameDifference);
      frameDifference = 1;
      ballNotSeen = 0;
    } else {
      ++ballNotSeen;
      findPossibleBallPositions(ballNotSeen, candidates);
      if (frameDifference > 3) {
        ballCandidates.clear();
        frameDifference = 1;
      } else 
        ++frameDifference;
    }
  } else if (size > 1) {
    DEBUG(endl << endl << "MORE THAN 1 CANDIDATE, size " << size << endl << endl << endl);
    frameDifference = 1;
    vector<ballCandidate> newBallCandidates;
    Point2f currentPosition;
    for (int i = 0; i < size; ++i) {
      //ballCandidate *candidate = &ballCandidates[i];
      DEBUG("next candidate" << endl);
      ballCandidate *candidate = &ballCandidates[i];
      DEBUG("its last positon: " << candidate->lastPosition << endl);
      circle(frame, candidate->lastPosition, 8, Scalar(0, 150, 50), -1, 8);
      //ballCandidates.erase(ballCandidates.begin());
      currentPosition = findCurrentPosition(*candidate, 1);
      if (norm(currentPosition) != 0) {
        DEBUG("found new point, cur position: " << currentPosition << endl);
        ballpos.push_back(currentPosition);
        updateCurrentPosition(candidate, currentPosition, 1);
        newBallCandidates.push_back(*candidate);
        circle(frame, candidate->lastPosition, 4, Scalar(250, 150, 50), -1, 8);
        found = true;
      } /*else { // if not found then remove this ball candidate
        //DEBUG("continuation not found, erase " << endl);
        ballCandidates.erase(ballCandidates.begin() + i);
        --i; // not sure if this is gonna work (erasing elements from a vector while iterating through it
      }*/
    }
    ballCandidates = newBallCandidates;
    DEBUG("ballcandidates size now: " << ballCandidates.size() << endl);
    for (ballCandidate bc : ballCandidates) {
      DEBUG("ballCand, lastPos: " << bc.lastPosition << endl);
    }
  } else {
    // no ball candidates so far
    DEBUG("no candidates" << endl);
    if (ballNotSeen > 0) {
      DEBUG("ballNotSeen = " << ballNotSeen);
      findPossibleBallPositions(ballNotSeen, candidates);
      ++ballNotSeen;
    }
    if (setsOfIsolatedPoints.size() == numberOfFrames) {
      vector<Point2f> startPoints = setsOfIsolatedPoints[numberOfFrames - 1];
      vector<ballCandidate> candidates;
      //DEBUG("have " << startPoints.size() << " start points" << endl);
      for (Point2f p : startPoints) {
        // new way of doing that
        candidates = findBallCandidates(p);
        if (candidates.size() > 0) 
          ballCandidates.insert(ballCandidates.begin() + ballCandidates.size(), candidates.begin(),
            candidates.end());
        /*
        if (hasRightTrajectory(p)) {
          //DEBUG("right trajectory found!" << endl);
          ballCandidate candidate = recoverBallCandidate(p);
          ballCandidates.push_back(candidate);
        }
        */
      }
    }
    if (ballCandidates.size() > 0) {
      DEBUG("have some ballcands now\n");
      for (ballCandidate bc : ballCandidates) {
        DEBUG("candidate: last position ");
        DEBUG(bc.lastPosition);
        DEBUG(", xdiff ");
        DEBUG(bc.xDiff);
        DEBUG(", ydiff ");
        DEBUG(bc.yDiff);
      }
    }
  }

  // prepare args for a call to calcOpticalFlow - VERY wrong comment I guess.
  //vector<uchar> status;
  //vector<float> err;
  //TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03);
  //Size subPixWinSize(10,10), winSize(31,31);
  //vector<Point2f> nextPoints;

  return found;
}


bool BallFinder::findBall(Point2f &ballpos, vector<Point2f> &candidates) {
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG(endl);
  DEBUG("new frame");
  //line(frame, Point(100,100), Point(100,200),Scalar(0,0,255),1,8,0);
  size_t size = ballCandidates.size();
  bool found = false;
  DEBUG("ballCand size " << size);

  if (size == 1) {
    ballCandidate *candidate = &ballCandidates[0];
    Point2f currentPosition = findCurrentPosition(*candidate, frameDifference);
    // check if current position has been found
    if (norm(currentPosition) != 0) {
      circle(frame, currentPosition, 4, Scalar(255,0,0), -1, 8);
      ballpos = currentPosition;
      lastBallPosition = currentPosition;
      found = true;
      updateCurrentPosition(candidate, currentPosition, frameDifference);
      frameDifference = 1;
      ballNotSeen = 0;
      DEBUG("found cur position: " << currentPosition << endl << endl << endl);
    } else {
      ++ballNotSeen;
      findPossibleBallPositions(ballNotSeen, candidates);
      if (frameDifference > 3) {
        ballCandidates.clear();
        frameDifference = 1;
      } else 
        ++frameDifference;
    }

    DEBUG("look for coexisting ball candidates" << endl);
    vector<Point2f> startPoints = setsOfIsolatedPoints[numberOfFrames - 1];
    vector<ballCandidate> candidates;
    //DEBUG("have " << startPoints.size() << " start points" << endl);
    for (Point2f p : startPoints) {
      // new way of doing that
      candidates = findBallCandidates(p);
      for (ballCandidate bc : candidates) {
        DEBUG("found sth" << endl);
        DEBUG("lastPos: " << bc.lastPosition << endl);
        if (norm(bc.lastPosition - currentPosition) > eps) {
          DEBUG("insert " << endl);
          ballCandidates.insert(ballCandidates.begin() + ballCandidates.size(), bc);
        }
      }
      //if (candidates.size() > 0) 
      //  ballCandidates.insert(ballCandidates.begin() + ballCandidates.size(), candidates.begin(),
       //   candidates.end());
    }
  } else if (size > 1) {
    DEBUG(endl << endl << "MORE THAN 1 CANDIDATE, size " << size << endl << endl << endl);
    frameDifference = 1;
    vector<ballCandidate> newBallCandidates;
    Point2f currentPosition;
    for (size_t i = 0; i < size; ++i) {
      //ballCandidate *candidate = &ballCandidates[i];
      DEBUG("next candidate" << endl);
      ballCandidate *candidate = &ballCandidates[i];
      DEBUG("its last positon: " << candidate->lastPosition << endl);
      circle(frame, candidate->lastPosition, 8, Scalar(0, 150, 50), -1, 8);
      //ballCandidates.erase(ballCandidates.begin());
      currentPosition = findCurrentPosition(*candidate, 1);
      if (norm(currentPosition) != 0) {
        DEBUG("found new point, cur position: " << currentPosition << endl);
        ballpos = currentPosition;
        updateCurrentPosition(candidate, currentPosition, 1);
        newBallCandidates.push_back(*candidate);
        circle(frame, candidate->lastPosition, 4, Scalar(250, 150, 50), -1, 8);
        found = true;
      } /*else { // if not found then remove this ball candidate
        //cout << "continuation not found, erase " << endl;
        ballCandidates.erase(ballCandidates.begin() + i);
        --i; // not sure if this is gonna work (erasing elements from a vector while iterating through it
      }*/
    }
  } else {
    // no ball candidates so far
    // cout << "no candidates" << endl;
    if (ballNotSeen > 0) {
      // cout << "ballNotSeen > 0 " << endl;
      findPossibleBallPositions(ballNotSeen, candidates);
      ++ballNotSeen;
    }
    if (setsOfIsolatedPoints.size() == numberOfFrames) {
      vector<Point2f> startPoints = setsOfIsolatedPoints[numberOfFrames - 1];
      vector<ballCandidate> candidates;
      //cout << "have " << startPoints.size() << " start points" << endl;
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

  return found;
}

void BallFinder::printObject(object &obj) {
  cout << "Top: " << obj.top << ", bottom: " << obj.bottom;
  cout << ", left: " << obj.left << ", right: " << obj.right << endl;
}

bool BallFinder::isClose(object &obj, Point2f &p) {
  //cout << "have object ";
  //printObject(obj);
  //cout << " chec if " << p << " is close to that object" << endl;
  return isInside(obj, p) || isNearby(obj, p);
}

bool BallFinder::isNearby(object &obj, Point2f &p) {
  //return norm(obj.top - p) <= playerHeight ||
  //       norm(obj.bottom - p) <= playerHeight ||
  //       norm(obj.right - p) <= playerWidth ||
  //       norm(obj.left - p) <= playerWidth; 
  //cout << "length " << norm(Point2f(obj.right.x, obj.top.y) - p)  << endl;
  //cout << "length " << norm(Point2f(obj.right.x, obj.bottom.y) - p) << endl;
  //cout << "length " << norm(Point2f(obj.left.x, obj.bottom.y) - p) << endl;
  //cout << "length " << norm(Point2f(obj.left.x, obj.top.y) - p) << endl;
  float yratio = obj.centre.y / 900;
  //cout << "yratio " << yratio << endl;
  float xratio = abs(obj.centre.x - 640) / 1280;
  //cout << "xratio " << xratio << endl;
  float ratio = sqrt(yratio) * yratio * yratio - xratio * xratio * xratio * xratio;
  //cout << "length: " << ratio * playerLength << endl;
  return norm(Point2f(obj.right.x, obj.top.y) - p) <= ratio * playerLength &&
         norm(Point2f(obj.right.x, obj.bottom.y) - p) <= ratio * playerLength &&
         norm(Point2f(obj.left.x, obj.bottom.y) - p) <= ratio * playerLength &&
         norm(Point2f(obj.left.x, obj.top.y) - p) <= ratio * playerLength; 
  //cout << "length " << abs(obj.right.x - p.x) << endl;
  //cout << "length " << abs(obj.left.x - p.x) << endl;
  //cout << "length " << abs(obj.top.y - p.y) << endl; 
  //cout << "length " << abs(obj.bottom.y - p.y) << endl;
  //return abs(obj.right.x - p.x) <= playerWidth &&
  //       abs(obj.left.x - p.x) <= playerWidth &&
  //       abs(obj.top.y - p.y) <= playerHeight &&
  //       abs(obj.bottom.y - p.y) <= playerHeight; 
}

bool BallFinder::isInside(object &obj, Point2f &p) {
  return p.x <= obj.right.x && p.x >= obj.left.x &&
         p.y >= obj.top.y && p.y <= obj.bottom.y;
}

void BallFinder::updateObject(object &obj, int i) {
  bool needToInit = obj.madeUpOf.size() == 0;
  // check if the new point is a 'radical' point
  if (obj.top.y > centres[i].y || needToInit) obj.top = centres[i];
  if (obj.bottom.y < centres[i].y || needToInit) obj.bottom = centres[i];
  if (obj.right.x < centres[i].x || needToInit) obj.right = centres[i];
  if (obj.left.x > centres[i].x || needToInit) obj.left = centres[i];
  // update the center
  obj.centre.x = obj.madeUpOf.size() * obj.centre.x + centres[i].x;
  obj.centre.x /= obj.madeUpOf.size() + 1;
  obj.centre.y = obj.madeUpOf.size() * obj.centre.y + centres[i].y;
  obj.centre.y /= obj.madeUpOf.size() + 1;
  obj.madeUpOf.push_back(i);
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
    if (!notIsolated[i]) {
      ++i;
    } else if (size == 0 || isClose(cur, centres[i])) { 
      updateObject(cur, i);
      ++size;
      ++i;
    } else {
      //cout << "is not close" << endl;
      if (size >= minNumberOfContoursRepresentingPlayer) {
        //cout << "found player candidate!" << endl;
        result.push_back(cur);
      }
      //cout << "size: " << size << endl;
      size = 0;
    }
  }
  if (size >= minNumberOfContoursRepresentingPlayer) 
    result.push_back(cur);
  //cout << "num of player candidates: " << result.size() << endl;
  return result;
}

bool BallFinder::updatePlayerCandidate(object &candidate) {
  return false;
}

void BallFinder::findPlayers(vector<object> &players) {
  players = findPlayerCandidates();
  for (object player : players) {
    //float x = (player.top.x + player.bottom.x + player.left.x + player.right.x) / 4;
    //float y = (player.top.y + player.bottom.y + player.left.y + player.right.y) / 4;
    //circle(frame, Point2f(x, y), 4, Scalar(0, 255, 0), -1, 8);
    circle(frame, player.centre, 4, Scalar(100, 100, 100), -1, 8);
    //DEBUG("centre " << player.centre << endl);
  }

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
bool BallFinder::addFrameNew(const Mat &frame, vector<Point2f> &ballpos, vector<object> &players,
                                          vector<Point2f> &candidates) {

  Mat m = Mat();
  cout << "Mat(): " << m.size() << endl;
  updateFrames(frame);

  //update the background model
  pMOG2.operator()(frame, fgMask, 0.01);

  //Mat back;
  //pMOG2.getBackgroundImage(back);
  //imshow("back", back);

  //imshow("mask before opening", fgMask);

  // do an opening (erosion and dilation) on the mask
  erode(fgMask, fgMask, Mat());
  dilate(fgMask, fgMask, Mat()); 

  //imshow("mask after opening", fgMask);

  findContours(fgMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
  drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 0.1);

  representatives = getRepresentatives();
  centres = getCentres();
  vector<Point2f> isolatedPts = getIsolatedPoints();
  updatesetsOfIsolatedPoints(isolatedPts);
  printIsolatedPoints();
  
  /* 
  Mat dst, detected_edges;

  int edgeThresh = 1;
  int lowThreshold = 16;
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
  //return true;
  return findBallNew(ballpos, candidates);
}

/* returns true if it found the ball. Also tries to return positions of both players -
 * if the players vector is nonempty then it found the positions of players */
bool BallFinder::addFrame(const Mat &frame, Point2f &ballpos, vector<object> &players, 
                              vector<Point2f> &candidates) {
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

  // Shape analysis
  //
  const double diam_threshold = 20;
  const double ellipse_ratio = 4.9;
  for (size_t i = 0; i < contours.size(); ++i) {
    double diam = 0.0;
    // Find diameter. Speed-up: convex hull + rotating calipers.
    //
    for (auto p : contours[i]) {
      for (auto q : contours[i]) {
        diam = std::max(diam, norm(p - q));
      }
    }
    if (diam > diam_threshold) {
      continue;
    }

    double min_c_dist = 1e100;
    double max_c_dist = 0;
    for (auto p : contours[i]) {
      min_c_dist = std::min(min_c_dist, norm(cv::Point2f(p.x, p.y) - centres[i]));
      max_c_dist = std::max(max_c_dist, norm(cv::Point2f(p.x, p.y) - centres[i]));
    }
    if (max_c_dist / min_c_dist > ellipse_ratio) {
      continue;
    }
    candidates.push_back(centres[i]);
  }
  vector<Point2f> isolatedPts = getIsolatedPoints();
  updatesetsOfIsolatedPoints(isolatedPts);
  
  /* 
  Mat dst, detected_edges;

  int edgeThresh = 1;
  int lowThreshold = 16;
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
  //return true;
  std::vector<cv::Point2f> tmp;
  return findBall(ballpos, tmp);
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

  cout << capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
  cout << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

  int keyboard = 0;

  while ((char) keyboard != 'q' && keyboard != 27) {
    if ((char) keyboard == 's') {
      // skip 10 frames
      for (int i = 0; i < 10; ++i) {
        if (capture.read(frame))
          updateFrames(frame);
      }
    }

    /*while (k < 500) {
      capture.read(frame);
      updateFrames(frame);
      ++k;
    }*/

    if (!capture.read(frame)) {
      cerr << "unable to read frame, exiting... " << endl;
      exit(EXIT_FAILURE);
    }

    vector<object> players;
    vector<Point2f> ballpos;
    vector<Point2f> candidates;

    if (addFrameNew(frame, ballpos, players, candidates)) {
      DEBUG("have the ball...");
    }

    imshow("Frame", frame);
    keyboard = waitKey(0);
  }

  capture.release();
  return 0;
}

