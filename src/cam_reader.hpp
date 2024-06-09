#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include "utilities.hpp"

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

class CamInfos {
 public:
  CamInfos(
      Vector2 ballPos,
      Vector2 myGoalPos,
      Vector2 enemyGoalPos);

  bool updateFromString(ReadingData readingData, char newChar);

  inline Vector2 ballPos() const { return _ballPos.toVector2(); }
  inline Vector2 myGoalPos() const { return _myGoalPos.toVector2(); }
  inline Vector2 enemyGoalPos() const { return _enemyGoalPos.toVector2(); }

  String toString() const;

 private:
  MutableVector2 _ballPos, _myGoalPos, _enemyGoalPos;
};

#endif