#include "scoreTracker.h"
#include "utils.h"
#include <string>
#include <algorithm>

using namespace std;
using namespace cv;

void ScoreTracker::startGame(string s1, string s2, short firstToServe) {
  serving = firstToServe;
  firstServe = true;
  players[0].name = s1;
  players[1].name = s2;
  initScoreBoard();
  INFO("A match starts between " << s1 << " and " << s2 << ".");
  INFO(players[firstToServe].name << " to serve");
}

void ScoreTracker::serve(Point2d pos, double speed) {
  if (lastShotWho >= 0) DEBUG("Point is underway, serve shouldn't have occured");
  else {
    // TODO: check if the serve was taken from right place on the court
    lastShotWho = serving;
    lastShotKind = SERVE;
    serveSpeed = speed;
    firstServe ? ++players[serving].firstServesTotal : ++players[serving].secondServesTotal;
  }
}

void ScoreTracker::ballHitsGround(Point2d pos) {
  if (isIn(pos, courtSide(~lastShotWho), lastShotKind)) {
    if (lastShotKind == SERVE) {
      (firstServe) ? ++players[lastShotWho].firstServesIn : 
                     ++players[lastShotWho].secondServesIn;
      firstServe = true;
      players[lastShotWho].fastestServe = max(players[lastShotWho].fastestServe, serveSpeed);
    }
    ballHitGround = true;
  } else { // ball was out
    if (lastShotKind == SERVE)  failedServe();
    else {
      pointFor(~lastShotWho);
    }
    lastShotWho = -1;
    printScore();
  }
}

// returns +1 or -1, denoting the side of the court on which player is
short ScoreTracker::courtSide(short player) {
  // TODO: make sure that works correctly
  return (player ^ serving) ? -servingSide : servingSide;
}

void ScoreTracker::ballHitsNet() {
  INFO(players[lastShotWho].name << " hits the net!");
  if (lastShotKind == SERVE) {
    failedServe();
  } else {
    // if not a serve than whoever hit the net loses the point
    pointFor(~lastShotWho);
  }
  lastShotWho = -1; // means finished rally
  printScore();
}

void ScoreTracker::failedServe() {
  if (firstServe) firstServe = false; 
  else {
    // its a double fault
    pointFor(~lastShotWho);
    ++players[serving].doubleFaults;
    INFO(players[serving].name << " hits a double fault!");
  }
}

bool ScoreTracker::isIn(Point2d pos, short playerSide, shot s) {
  if (s == SERVE) {
    // playerSide tells us the sign of y coordinates
    int remainder = (scores[0].points + scores[1].points) % 2;
    remainder *= 2;
    remainder -= 1;
    short xSign = remainder * playerSide;
    short ySign = playerSide;
    return isInServiceArea(pos, xSign, ySign);
  } 
  return isInTheSideOfTheCourt(pos, playerSide);
}

bool ScoreTracker::isInServiceArea(Point2d pos, short xSign, short ySign) {
  return false; 
}

bool ScoreTracker::isInTheSideOfTheCourt(Point2d pos, short playerSide) {
  return false;
}

void ScoreTracker::pointFor(short player) {

}

void ScoreTracker::printScore() {
  
}

void ScoreTracker::initScoreBoard() {
  for (short i = 0; i < 2; ++i) {
    scores[i].games = 0;
    scores[i].points = 0;
  }
}
