#ifndef STATES_H
#define STATES_H

#include <Arduino.h>

#include "utilities.h"

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
      float ballRadius);

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

class ReadingData {
 private:
  char _typeState = 'x';
  String _xReadingState = "";
  String _yReadingState = "";
  bool _writingInXState = true;

 public:
  ReadingData();
  String toString() const;

  inline char typeState() const { return _typeState; }
  inline String xReadingState() const { return _xReadingState; }
  inline String yReadingState() const { return _yReadingState; }
  inline bool writingInXState() const { return _writingInXState; }

  void nowWriteInYState();
  void addToActiveReadingState(char newChar);
  void reinitWith(char newChar);
};

class RobotState {
 public:
  RobotState(
      Vector2 ballPos,
      Vector2 myPos,
      Vector2 partnerPos,
      Vector2 myGoalPos,
      Vector2 enemyGoalPos,
      Vector2 nearestWall,
      double orientation);

  bool updateFromString(ReadingData readingData, char newChar);

  inline Vector2 ballPos() const { return _ballPos.toVector2(); }
  inline Vector2 myPos() const { return _myPos.toVector2(); }
  inline Vector2 partnerPos() const { return _partnerPos.toVector2(); }
  inline Vector2 myGoalPos() const { return _myGoalPos.toVector2(); }
  inline Vector2 enemyGoalPos() const { return _enemyGoalPos.toVector2(); }
  inline Vector2 nearestWall() const { return _nearestWall.toVector2(); }
  inline double orientation() const { return _orientation; }

  String toString() const;

 private:
  MutableVector2 _ballPos, _myPos, _partnerPos, _myGoalPos, _enemyGoalPos, _nearestWall;
  double _orientation;
};

#endif