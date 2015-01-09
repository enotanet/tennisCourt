#ifndef SCORE_TRACKER_H__
#define SCORE_TRACKER_H__
#include <opencv2/core/core.hpp>
#include <string>

struct Score {
  size_t games, points; // in current set
  std::vector<size_t> gamesWonInSets;
};

enum shot { SERVE, SMASH, FOREHAND, BACKHAND, VOLLEY };

struct Player {
  std::string name;
  double fastestServe;
  size_t doubleFaults;
  size_t winners;
  size_t aces;
  size_t firstServesIn;
  size_t firstServesTotal;
  size_t secondServesIn;
  size_t secondServesTotal;
};

class ScoreTracker {
public:
  ScoreTracker() : lastShotWho(-1) {}

  void ballHitsGround(cv::Point2d pos);
  void ballHitsNet();
  void serve(cv::Point2d pos, double speed);
  void forehand(cv::Point2d pos, double speed);
  void backhand(cv::Point2d pos, double speed);
  void smash(cv::Point2d, double speed);
  void rallyFinished();
  void startGame(std::string s1 = "Player 1", std::string s2 = "Player2", 
                      short firstToServe = 0); // player s1 is player 0
                                               // player s2 is player 1

private:
  void initScoreBoard();
  void printScore();
  void pointFor(short player);
  short courtSide(short player); 
  void failedServe();
  bool isIn(cv::Point2d pos, short playerSide, shot s);
  bool isInServiceArea(cv::Point2d pos, short xSign, short ySign); 
  bool isInTheSideOfTheCourt(cv::Point2d pos, short playerSide); 

  Score scores[2];
  Player players[2];
  short serving; // denotes the player who is serving, can be 0 or 1
  short servingSide; // denotes the side on which the serving player is, can be -1 or +1
  short lastShotWho; // denotes who hit the last shot 
  shot lastShotKind; // denotes what kind of shot it was
  double serveSpeed;
  bool firstServe;
  bool ballHitGround;

  static const size_t howManySetsToWin = 2;
};

#endif
