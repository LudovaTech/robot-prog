#include "cam_reader.hpp"

const Radians angleMargin = 0.3;
const int minBytesAvailable = 60 * 2;
const int timeCache = 50;

Optional<BallPos> cacheBallPos;
int timeCacheBallPos = millis();
Optional<MyGoalPos> cacheMyGoalPos;
int timeCacheMyGoalPos = millis();
Optional<EnemyGoalPos> cacheEnemyGoalPos;
int timeCacheEnemyGoalPos = millis();

String extractLastCompleteSequence(String str) {
  int lastE = str.lastIndexOf('e');
  if (lastE != -1) {
    int prevB = str.lastIndexOf('b', lastE);
    if (prevB != -1) {
      return str.substring(prevB, lastE + 1);
    }
  }
  return "";
}

Optional<Vector2> interpret(
    Optional<Radians> angleFrontGoalLidar,
    Optional<Radians> angleRearGoalLidar,
    int *Xs,
    int *Ys) {
  if (!(angleFrontGoalLidar.hasValue() && angleRearGoalLidar.hasValue())) {
    return Vector2(Xs[0], Ys[0]);
  }

  for (int i = 0; i <= 2; i++) {
    Vector2 vector2(Xs[i], Ys[i]);
    Radians angle = vector2.angle();
    if (abs(angle - angleFrontGoalLidar.value()) < angleMargin ||
        abs(angle - angleRearGoalLidar.value()) < angleMargin) {
      return vector2;
    }
  }
  return Optional<Vector2>();
}

String readFromCam(int bytesAvailable) {
  String data;
  for (unsigned int i = 0; i < bytesAvailable; i++) {
    int receive = SerialCam.read();
    if (receive == -1) {
      log_a(CriticalLevel, "getCamInfos", "strange, -1");
    }
    data += static_cast<char>(receive);
  }
}

bool sequenceToValues(
    String lastCompleteSequence,
    int *ballX,
    int *ballY,
    int *myGoalsX,
    int *myGoalsY,
    int *enemyGoalsX,
    int *enemyGoalsY) {
  return sscanf(lastCompleteSequence.c_str(), "b%d%d%d%d%d%d%d%d%d%d%d%d%d%de",
                ballX, ballY, &myGoalsX[0], &myGoalsY[0], &myGoalsX[1], &myGoalsY[1], &myGoalsX[2], &myGoalsY[2],
                &enemyGoalsX[0], &enemyGoalsY[0], &enemyGoalsX[1], &enemyGoalsY[1], &enemyGoalsX[2], &enemyGoalsY[2]) == 14;
}

template <typename T>
Optional<T> convertTo(Optional<Vector2> from) {
  if (!from.hasValue()) {
    return Optional<T>();
  }
  float x = from.value().x();
  float y = from.value().y();
  if (x == 0 && y == 0) {
    return Optional<T>();
  } else {
    return T(x, y);
  }
}

Optional<BallPos> readAndUpdateCache(Optional<BallPos> ballPos) {
  if (ballPos.hasValue()) {
    cacheBallPos = ballPos;
    timeCacheBallPos = millis();
    return ballPos;
  } else {
    if (millis() <= timeCacheBallPos + timeCache) {
      // cache valide
      return cacheBallPos;
    }
  }
}

Optional<MyGoalPos> readAndUpdateCache(Optional<MyGoalPos> myGoalPos) {
  if (myGoalPos.hasValue()) {
    cacheMyGoalPos = myGoalPos;
    timeCacheMyGoalPos = millis();
    return myGoalPos;
  } else {
    if (millis() <= timeCacheMyGoalPos + timeCache) {
      // cache valide
      return cacheMyGoalPos;
    }
  }
}

Optional<EnemyGoalPos> readAndUpdateCache(Optional<EnemyGoalPos> enemyGoalPos) {
  if (enemyGoalPos.hasValue()) {
    cacheEnemyGoalPos = enemyGoalPos;
    timeCacheEnemyGoalPos = millis();
    return enemyGoalPos;
  } else {
    if (millis() <= timeCacheEnemyGoalPos + timeCache) {
      // cache valide
      return cacheEnemyGoalPos;
    }
  }
}

CamInfosGlue getCamInfos(Optional<Radians> angleFrontGoalLidar, Optional<Radians> angleRearGoalLidar) {
  size_t bytesAvailable = SerialCam.available();
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
  if (bytesAvailable >= minBytesAvailable) {
    String data = readFromCam(bytesAvailable);
    String lastCompleteSequence = extractLastCompleteSequence(data);
    if (lastCompleteSequence != "") {
      int ballX, ballY, myGoalsX[3], myGoalsY[3], enemyGoalsX[3], enemyGoalsY[3];
      if (sequenceToValues(lastCompleteSequence, &ballX, &ballY, myGoalsX, myGoalsY, enemyGoalsX, enemyGoalsY)) {
        Optional<Vector2> optionalMyGoalPos = interpret(angleFrontGoalLidar, angleRearGoalLidar, myGoalsX, myGoalsY);
        Optional<Vector2> optionalEnemyGoalPos = interpret(angleFrontGoalLidar, angleRearGoalLidar, enemyGoalsX, enemyGoalsY);
        ballPos = convertTo<BallPos>(Vector2(ballX, ballY));
        myGoalPos = convertTo<MyGoalPos>(optionalMyGoalPos);
        enemyGoalPos = convertTo<EnemyGoalPos>(optionalEnemyGoalPos);
      } else {
      }
    } else {
    }
  } else {
  }
  Optional<BallPos> afterCacheBallPos = readAndUpdateCache(ballPos);
  Optional<MyGoalPos> afterCacheMyGoalPos = readAndUpdateCache(myGoalPos);
  Optional<EnemyGoalPos> afterCacheEnemyGoalPos = readAndUpdateCache(enemyGoalPos);
  return CamInfosGlue{
      afterCacheBallPos,
      afterCacheMyGoalPos,
      afterCacheEnemyGoalPos};
}