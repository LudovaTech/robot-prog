#include "utilities.hpp"

///////VECTOR2

Vector2::Vector2(float x, float y)
    : _x(x), _y(y) {}

String Vector2::toString() const {
  return "(" + String(_x) + ", " + String(_y) + ")";
}

Vector2 Vector2::distanceRef(Vector2 other) const {
  return Vector2(
      other.x() - x(),
      other.y() - y());
}

float Vector2::distance(Vector2 other) const {
  return sqrt(sq(other.x() - x()) + sq(other.y() - y()));
}

float Vector2::norm() const {
  return sqrt(sq(x()) + sq(y()));
}

Radians Vector2::angle() const {
  float angle;

  // If the y value is zero, the angle is 90°.
  if (y() == 0) {
    angle = PI / 2;
  } else {
    angle = atan2(abs(x()), abs(y()));
  }

  // Change the angle according to the corner in which the destination point is located
  if (x() >= 0 && y() >= 0) {
    angle *= -1;
  } else if (x() <= 0 && y() <= 0) {
    angle = PI - angle;
  } else if (x() >= 0 && y() <= 0) {
    angle -= PI;
  }

  // NoneVector case
  if (x() == 0 && y() == 0) {
    angle = 0;
  }

  return Radians(angle);
}

Vector2 Vector2::rotate(Radians rad) {
  return Vector2(
      x() * cos(rad) - y() * sin(rad),
      x() * sin(rad) + y() * cos(rad));
}

///////MUTABLEVECTOR2

MutableVector2::MutableVector2(Vector2 vector2)
    : _x(vector2.x()), _y(vector2.y()) {}

MutableVector2::MutableVector2(float x, float y)
    : _x(x), _y(y) {}

MutableVector2::MutableVector2()
    : _x(0), _y(0) {}

String MutableVector2::toString() const {
  return "(Mutable," + String(_x) + ", " + String(_y) + ")";
}
Vector2 MutableVector2::toVector2() const {
  return Vector2(x(), y());
}

///////DEGREES AND RADIANS

Degree::Degree(float angle) : _angle(angle) {}
Degree::Degree(Radians angle) : _angle(angle * RAD_TO_DEG) {}

Radians::Radians(float angle) : _angle(angle) {}
Radians::Radians(Degree angle) : _angle(angle * DEG_TO_RAD) {}
