#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

#define SerialDebug Serial8
#define SerialTest Serial
#define SerialCam Serial2
#define SerialLidar Serial6
#define SerialBlue Serial1

class Radians;
class Degree;

class Vector2 {
 public:
  Vector2(float x, float y);
  inline float x() const { return _x; };
  inline float y() const { return _y; };
  String toString() const;

  inline bool operator==(const Vector2 &other) const {
    return (_x == other._x && _y == other._y);
  }
  inline bool operator!=(const Vector2 &other) const {
    return (_x != other._x || _y != other._y);
  }

  Vector2 distanceRef(Vector2 other) const;
  float distance(Vector2 other) const;
  float norm() const;
  Radians angle() const;
  Vector2 rotate(Radians rad);

 private:
  const float _x, _y;
};

class MutableVector2 {
 public:
  MutableVector2(Vector2 vector2);
  MutableVector2(float x, float y);
  MutableVector2();

  inline float x() const { return _x; }
  inline float y() const { return _y; }

  String toString() const;
  Vector2 toVector2() const;

  inline bool operator==(const Vector2 &other) {
    return (_x == other.x() && _y == other.y());
  }
  inline bool operator!=(const Vector2 &other) {
    return (_x != other.x() || _y != other.y());
  }
  inline bool operator==(const MutableVector2 &other) {
    return (_x == other._x && _y == other._y);
  }
  inline bool operator!=(const MutableVector2 &other) {
    return (_x != other._x || _y != other._y);
  }

 private:
  float _x, _y;
};

class Degree {
 private:
  float _angle;

 public:
  Degree(float angle);
  Degree(Radians angle);
  inline operator float() const { return _angle; }
};

class Radians {
 private:
  float _angle;

 public:
  Radians(float angle);
  Radians(Degree angle);
  inline operator float() const { return _angle; }
};

template <typename T>
class ResultOrError {
 private:
  T _value;
  const String _errorMessage;
  const bool _isError;

 public:
  ResultOrError(T value) : _value(value), _isError(false) {}
  ResultOrError(String errorMessage) : _errorMessage(errorMessage), _isError(true) {}

  inline bool hasError() const { return _isError; };
  inline T value() const { return _value; };
  String errorMessage() const { return _errorMessage; };
};

template <typename T>
class Optional {
 private:
  T *_value;
  bool _hasValue;

 public:
  Optional() : _value(nullptr), _hasValue(false) {}

  Optional(const T &value) : _hasValue(true) {
    _value = new T(value);
  }

  Optional(const Optional &other) : _hasValue(other._hasValue) {
    if (_hasValue) {
      _value = new T(*other._value);
    } else {
      _value = nullptr;
    }
  }

  Optional(Optional &&other) noexcept : _value(other._value), _hasValue(other._hasValue) {
    other._value = nullptr;
    other._hasValue = false;
  }

  Optional &operator=(const Optional &other) {
    if (this != &other) {
      delete _value;
      _hasValue = other._hasValue;
      if (_hasValue) {
        _value = new T(*other._value);
      } else {
        _value = nullptr;
      }
    }
    return *this;
  }

  Optional &operator=(Optional &&other) noexcept {
    if (this != &other) {
      delete _value;
      _value = other._value;
      _hasValue = other._hasValue;
      other._value = nullptr;
      other._hasValue = false;
    }
    return *this;
  }

  ~Optional() {
    delete _value;
  }

  bool hasValue() const { return _hasValue; }

  T &value() {
    if (!_hasValue) {
      SerialDebug.println("Error: Attempted to access value when none is present.");
    }
    return *_value;
  }

  const T &value() const {
    if (!_hasValue) {
      SerialDebug.println("Error: Attempted to access value when none is present.");
    }
    return *_value;
  }
};


template <typename T>
class Cache {
 private:
  Optional<T> _value;
  unsigned int _birth = millis();
  const unsigned int _lifetime;

 public:
  Cache(unsigned int lifetime) : _lifetime(lifetime) {}

  bool valid();
  Optional<T> cache();
  void update(Optional<T> newValue);
  Optional<T> readAndUpdate(Optional<T> newValue);
};

//////CACHE

template <typename T>
bool Cache<T>::valid() {
  return millis() < _birth + _lifetime;
}

template <typename T>
Optional<T> Cache<T>::cache() {
  if (valid()) {
    return _value;
  } else {
    return Optional<T>();
  }
}

template <typename T>
void Cache<T>::update(Optional<T> newValue) {
  _value = newValue;
  _birth = millis();
}

template <typename T>
Optional<T> Cache<T>::readAndUpdate(Optional<T> newValue) {
  if (newValue.hasValue()) {
    update(newValue);
    return newValue;
  } else {
    return cache();
  }
}

#endif