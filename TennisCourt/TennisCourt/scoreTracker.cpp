#include "scoreTracker.h"
#include "utils.h"
#include <string>
#include <algorithm>

using namespace std;
using namespace cv;

void ScoreTracker::startGame(string s1, string s2, player firstToServe) {
  serving = firstToServe;
  firstServe = true;
  tiebreak = false;
  game = true;
  set = 1;
  initPlayer(PLAYER0, s1);
  initPlayer(PLAYER1, s2);
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
    firstServe ? ++players[serving].firstServesTotal[set - 1] : 
                 ++players[serving].secondServesTotal[set - 1];
  }
}

void ScoreTracker::ballHitsGround(Point2d pos) {
  if (isIn(pos, courtSide(other(lastShotWho)), lastShotKind)) {
    if (lastShotKind == SERVE) {
      (firstServe) ? ++players[lastShotWho].firstServesIn[set - 1] : 
                     ++players[lastShotWho].secondServesIn[set - 1];
      firstServe = true;
      players[lastShotWho].fastestServe = max(players[lastShotWho].fastestServe, serveSpeed);
    }
    ballHitGround = true;
  } else { // ball was out
    if (lastShotKind == SERVE)  failedServe();
    else {
      pointFor(other(lastShotWho));
    }
    lastShotWho = NONE;
    printScore();
  }
}

// returns +1 or -1, denoting the side of the court on which player is
short ScoreTracker::courtSide(player p) {
  // TODO: make sure that works correctly
  return (p ^ serving) ? -servingSide : servingSide;
}

void ScoreTracker::ballHitsNet() {
  INFO(players[lastShotWho].name << " hits the net!");
  if (lastShotKind == SERVE) {
    failedServe();
  } else {
    // if not a serve than whoever hit the net loses the point
    pointFor(other(lastShotWho));
  }
  lastShotWho = NONE; // means finished rally
  printScore();
}

void ScoreTracker::failedServe() {
  if (firstServe) firstServe = false; 
  else {
    // its a double fault
    pointFor(other(lastShotWho));
    ++players[serving].doubleFaults[set-1];
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

void ScoreTracker::pointFor(player p) {
  ++scores[p].points;
  size_t p1 = scores[p].points;
  size_t p2 = scores[other(p)].points;
  size_t pointsToWin = pointsToWinGame;
  // TODO: make this work!
  //size_t pointsToWin = tiebreak ? pointsToWinTiebreak : pointsToWinGame;
  if (p1 >= pointsToWin && p1 - p2 == differenceToWinGameOnAdvantages)  // player won the game
    winGame(p);
  else if (tiebreak) {
    if (p1 + p2 % 6 == 0) {
      INFO("Players change sides");
      servingSide = -servingSide;
    } else if (p1 + p2 % 2 == 1) {
      INFO("Change of serve in a tiebreak");
      serving = other(serving);
    }
  }
}

void ScoreTracker::newGame() {
  scores[PLAYER0].points = 0;
  scores[PLAYER1].points = 0;
  serving = other(serving);
  if (scores[PLAYER0].games + scores[PLAYER1].games % 2 == 1) {
    INFO("Players change sides");
    servingSide = -servingSide;
  }
}

void ScoreTracker::winGame(player p) {
  INFO(players[p].name << " won the game!");
  ++scores[p].games;  
  size_t g1 = scores[p].games;
  size_t g2 = scores[other(p)].games;
  bool set = false;
  if (g1 >= gamesToWinSet) {
    if (g1 - g2 >= differenceToWinSetOnAdvantages || g1 == maxNumberOfGamesWonByaPlayerInaSet) {
      winSet(p);
      set = true;
    } else if (g1 == g2 && g1 + 1 == maxNumberOfGamesWonByaPlayerInaSet) {
      tiebreak = true;
      INFO("We are going to have a tiebreak!");
    }
  } 
  if (!set) {
    printScore();
  }
  newGame();
}

void ScoreTracker::winSet(player p) {
  INFO(players[p].name << " won a set");
  ++scores[p].sets;
  if (scores[p].sets == setsToWinMatch) 
    winMatch(p);
  else {
    newSet(PLAYER0);
    newSet(PLAYER1);
  }
}

void ScoreTracker::winMatch(player p) {
  INFO(players[p].name << " won a match!");
}

void ScoreTracker::newSet(player p) {
  ++set;
  scores[p].gamesWonInSets.push_back(scores[p].games);
  scores[p].games = 0;
  
}

void ScoreTracker::printScore() {
  
}

void ScoreTracker::initPlayer(player p, string name) {
  players[p].name = name;
  players[p].doubleFaults.push_back(0);
  players[p].winners.push_back(0);
  players[p].aces.push_back(0);
  players[p].firstServesIn.push_back(0);
  players[p].firstServesTotal.push_back(0);
  players[p].secondServesIn.push_back(0);
  players[p].secondServesTotal.push_back(0);
}

void ScoreTracker::initScoreBoard() {
  for (short i = 0; i < 2; ++i) {
    scores[i].games = 0;
    scores[i].points = 0;
  }
}

player ScoreTracker::other(player p) {
  if (p == NONE) {
    DEBUG("Something possibly went wrong");
    return p;
  }
  return (p == PLAYER0) ? PLAYER1 : PLAYER0;
}
