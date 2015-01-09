#ifndef SCORE_TRACKER_H__
#define SCORE_TRACKER_H__
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

struct Score {
  size_t sets, games, points; // in current set
  std::vector<size_t> gamesWonInSets;
};

enum shot { SERVE, SMASH, FOREHAND, BACKHAND, VOLLEY };
enum player { PLAYER0, PLAYER1, NONE };

struct Player {
  std::string name;
  double fastestServe;
  std::vector<size_t> doubleFaults;
  std::vector<size_t> winners;
  std::vector<size_t> aces;
  std::vector<size_t> firstServesIn;
  std::vector<size_t> firstServesTotal;
  std::vector<size_t> secondServesIn;
  std::vector<size_t> secondServesTotal;
};

class ScoreTracker {
public:
  ScoreTracker() : lastShotWho(NONE) {}

  void ballHitsGround(cv::Point2d pos);
  void ballHitsNet();
  void serve(cv::Point2d pos, double speed);
  void forehand(cv::Point2d pos, double speed);
  void backhand(cv::Point2d pos, double speed);
  void smash(cv::Point2d, double speed);
  void rallyFinished();
  void startGame(std::string s1 = "Player 1", std::string s2 = "Player2", 
                      player firstToServe = PLAYER0); // player s1 is player 0
                                               // player s2 is player 1

private:
  void initPlayer(player p, std::string name);
  void initScoreBoard();
  void printScore();
  void pointFor(player p);
  void newGame();
  void winGame(player p);
  void winSet(player p);
  void winMatch(player p);
  void newSet(player p);
  short courtSide(player p); 
  void failedServe();
  bool isIn(cv::Point2d pos, short playerSide, shot s);
  bool isInServiceArea(cv::Point2d pos, short xSign, short ySign); 
  bool isInTheSideOfTheCourt(cv::Point2d pos, short playerSide); 
  player other(player p);

  Score scores[2];
  Player players[2];
  size_t set;
  player serving; // denotes the player who is serving
  short servingSide; // denotes the side on which the serving player is, can be -1 or +1
  player lastShotWho; // denotes who hit the last shot 
  shot lastShotKind; // denotes what kind of shot it was
  double serveSpeed;
  bool firstServe;
  bool ballHitGround;
  bool tiebreak;
  bool game;

  static const size_t howManySetsToWin = 2;
  static const size_t pointsToWinGame = 4;
  static const size_t pointsToWinTiebreak = 7;
  static const size_t gamesToWinSet = 6;
  static const size_t setsToWinMatch = 2;
  static const size_t differenceToWinGameOnAdvantages = 2;
  static const size_t differenceToWinSetOnAdvantages  = 2;
  static const size_t maxNumberOfGamesWonByaPlayerInaSet = 7;
};

#endif
