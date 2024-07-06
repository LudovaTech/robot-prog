#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include <string>

#include "logger.hpp"
#include "utilities.hpp"

class BallPos : public Vector2 {
  using Vector2::Vector2;
};

class MyGoalPos : public Vector2 {
  using Vector2::Vector2;
};

class EnemyGoalPos : public Vector2 {
  using Vector2::Vector2;
};

// TODO: temporary
struct CamInfosGlue {
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
};

Optional<Vector2> interpret(
    Optional<Radians> angleFrontGoalLidar,
    Optional<Radians> angleRearGoalLidar,
    int *Xs,
    int *Ys);

String readFromCam(int bytesAvailable);

bool sequenceToValues(
    String lastCompleteSequence,
    int *ballX,
    int *ballY,
    int *myGoalsX,
    int *myGoalsY,
    int *enemyGoalsX,
    int *enemyGoalsY);

template <typename T>
Optional<T> convertTo(Optional<Vector2> from);

Optional<BallPos> readAndUpdateCache(Optional<BallPos> ballPos);
Optional<MyGoalPos> readAndUpdateCache(Optional<MyGoalPos> myGoalPos);
Optional<EnemyGoalPos> readAndUpdateCache(Optional<EnemyGoalPos> enemyGoalPos);

CamInfosGlue getCamInfos(Optional<Radians> angleFrontGoalLidar, Optional<Radians> angleRearGoalLidar);
String extractLastCompleteSequence(String buffer);

#endif