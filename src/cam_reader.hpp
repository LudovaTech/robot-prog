#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include <string>

#include "logger.hpp"
#include "utilities.hpp"

const Radians angleMargin = 0.3;
const int minBytesAvailable = 57;
const int timeCache = 50;

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

void _test_set_cacheBallPos(Optional<BallPos> _cacheBallPos);
void _test_set_timeCacheBallPos(int _timeCacheBallPos);
void _test_set_cacheEnemyGoalPos(Optional<EnemyGoalPos> _cacheEnemyGoalPos);
void _test_set_timeCacheEnemyGoalPos(int _timeCacheEnemyGoalPos);
void _test_set_cacheMyGoalPos(Optional<MyGoalPos> _cacheMyGoalPos);
void _test_set_timeCacheMyGoalPos(int _timeCacheMyGoalPos);

#endif