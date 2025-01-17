#ifndef MOUVEMENTS_H
#define MOUVEMENTS_H

#include <Arduino.h> 

#include "utilities.hpp"

// TODO supprimer la mémorisation de sens en la remplaçant par une détection en temps réel grâce à FG
enum class Direction { forward,
                       backward,
                       stopped };

class MotorMov {
 public:
  MotorMov(
      uint8_t pinPWM,
      uint8_t pinCWCCW,
      uint8_t pinFG,
      Radians angleAxisKicker);
  void stop();
  void move(int value);
  inline Radians angleAxisKicker() const { return _angleAxisKicker; }
  Radians anglePowerAxisKicker() const;

  bool isLeft() const { return _angleAxisKicker < 0; }

 private:
  const uint8_t _pinPWM;
  const uint8_t _pinCWCCW;
  const uint8_t _pinFG;
  const Radians _angleAxisKicker;  // radians

  Direction _direction;

  void _pwm(int value) const;
  void _cwccw(uint8_t value) const;
  uint8_t _fg() const;
};

class Motors {
 private:
  const MotorMov _frontRight;
  const MotorMov _frontLeft;
  const MotorMov _backRight;
  const MotorMov _backLeft;

 public:
  Motors(
      MotorMov frontRight,
      MotorMov frontLeft,
      MotorMov backRight,
      MotorMov backLeft);

  inline MotorMov frontRight() const { return _frontRight; }
  inline MotorMov frontLeft() const { return _frontLeft; }
  inline MotorMov backRight() const { return _backRight; }
  inline MotorMov backLeft() const { return _backLeft; }

  void fullStop() const;

  void goTo(Vector2 distances, int celerity, Radians orientation) const;
};

class DribblerKicker {
 private:
  MotorMov _dribbler;
  const uint8_t _pinKicker1;
  const uint8_t _pinKicker2;

  const unsigned long _timeKick = 40;
  unsigned long _timeWhenKick = 0;

 public:
  DribblerKicker(
    MotorMov dribbler,
    uint8_t pinKicker1,
    uint8_t pinKicker2
  );

  void dribble(int value);
  void kick();
};

#endif