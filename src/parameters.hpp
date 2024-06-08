#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "utilities.hpp"

class FieldProperties {
 public:
  FieldProperties(
      float fieldLength,
      float fieldWidth,
      float spaceBeforeLineSide,
      float goalWidth,
      Vector2 myGoalPos,
      Vector2 enemyGoalPos,
      float robotRadius,
      float ballRadius)
      : _fieldLength(fieldLength),
        _fieldWidth(fieldWidth),
        _spaceBeforeLineSide(spaceBeforeLineSide),
        _goalWidth(goalWidth),
        _myGoalPos(myGoalPos),
        _enemyGoalPos(enemyGoalPos),
        _robotRadius(robotRadius),
        _ballRadius(ballRadius) {}

  inline float fieldLength() const { return _fieldLength; }
  inline float fieldWidth() const { return _fieldWidth; }
  inline float spaceBeforeLineSide() const { return _spaceBeforeLineSide; }
  inline float goalWidth() const { return _goalWidth; }
  inline Vector2 myGoalPos() const { return _myGoalPos; }
  inline Vector2 enemyGoalPos() const { return _enemyGoalPos; }
  inline float robotRadius() const { return _robotRadius; }
  inline float ballRadius() const { return _ballRadius; }

 private:
  const float _fieldLength;
  const float _fieldWidth;
  const float _spaceBeforeLineSide;
  const float _goalWidth;
  const Vector2 _myGoalPos;
  const Vector2 _enemyGoalPos;
  const float _robotRadius;
  const float _ballRadius;
};

#endif