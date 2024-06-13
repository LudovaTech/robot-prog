#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include "utilities.hpp"

class BallPos: public Vector2 {
  using Vector2::Vector2;
};

class MyGoalPos: public Vector2 {
  using Vector2::Vector2;
};

class EnnemyGoalPos: public Vector2 {
  using Vector2::Vector2;
};

#endif