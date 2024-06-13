#ifndef CAM_READER_H
#define CAM_READER_H

#include <Arduino.h>

#include "utilities.hpp"

class BallPos: Vector2 {
  using Vector2::Vector2;
};

class MyGoalPos: Vector2 {
  using Vector2::Vector2;
};

class EnnemyGoalPos: Vector2 {
  using Vector2::Vector2;
};

#endif