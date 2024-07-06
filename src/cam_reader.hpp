#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include "utilities.hpp"
#include "logger.hpp"
#include <string>

class BallPos: public Vector2 {
  using Vector2::Vector2;
};

class MyGoalPos: public Vector2 {
  using Vector2::Vector2;
};

class EnemyGoalPos: public Vector2 {
  using Vector2::Vector2;
};

// TODO: temporary
struct CamInfosGlue {
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
};

CamInfosGlue getCamInfos(Radians angleFrontGoalLidar, Radians angleRearGoalLidar);
std::string extractLastCompleteSequence(const char* buffer);

#endif