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
      float distanceYGoalFromCenter,
      float robotRadius,
      float ballRadius,
      float maxDribblerSpeed)
      : _fieldLength(fieldLength),
        _fieldWidth(fieldWidth),
        _spaceBeforeLineSide(spaceBeforeLineSide),
        _goalWidth(goalWidth),
        _distanceYGoalFromCenter(distanceYGoalFromCenter),
        _robotRadius(robotRadius),
        _ballRadius(ballRadius),
        _maxDribblerSpeed(maxDribblerSpeed) {}

  inline float fieldLength() const { return _fieldLength; }
  inline float fieldWidth() const { return _fieldWidth; }
  inline float spaceBeforeLineSide() const { return _spaceBeforeLineSide; }
  inline float goalWidth() const { return _goalWidth; }
  inline float distanceYGoalFromCenter() const { return _distanceYGoalFromCenter; }
  inline float robotRadius() const { return _robotRadius; }
  inline float ballRadius() const { return _ballRadius; }
  inline float maxDribblerSpeed() const { return _maxDribblerSpeed; }

 private:
  const float _fieldLength;
  const float _fieldWidth;
  const float _spaceBeforeLineSide;
  const float _goalWidth;
  const float _distanceYGoalFromCenter;
  const float _robotRadius;
  const float _ballRadius;
  const float _maxDribblerSpeed;
};

#endif