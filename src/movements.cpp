#include "movements.hpp"

///////MOTORMOV
MotorMov::MotorMov(
    uint8_t pinPWM,
    uint8_t pinCWCCW,
    uint8_t pinFG,
    Radians angleAxisKicker)
    : _pinPWM(pinPWM),
      _pinCWCCW(pinCWCCW),
      _pinFG(pinFG),
      _angleAxisKicker(angleAxisKicker - (PI / 2)) {
  pinMode(_pinPWM, OUTPUT);
  pinMode(_pinCWCCW, OUTPUT);
  pinMode(_pinFG, INPUT);
  stop();
  _direction = Direction::stopped;
}

void MotorMov::stop() {
  _pwm(0);
  _direction = Direction::stopped;
}

void MotorMov::move(int value) {
  if (value == 0) {
    stop();
  } else if (value > 0) {
    // forward
    if (_direction == Direction::backward) {
      stop();
    }
    _cwccw(HIGH);
    _pwm(value);
    _direction = Direction::forward;
  } else {
    // backward
    if (_direction == Direction::forward) {
      stop();
    }
    _cwccw(LOW);
    _pwm(-value);
    _direction = Direction::backward;
  }
}

Radians MotorMov::anglePowerAxisKicker() const {
  return _angleAxisKicker - (PI / 2);
}

void MotorMov::_pwm(int value) const {
  analogWrite(_pinPWM, 255 - value);
}

void MotorMov::_cwccw(uint8_t value) const {
  digitalWrite(_pinCWCCW, value);
}

uint8_t MotorMov::_fg() const {
  return digitalRead(_pinFG);
}

///////MOTORS
Motors::Motors(
    MotorMov frontRight,
    MotorMov frontLeft,
    MotorMov backRight,
    MotorMov backLeft)
    : _frontRight(frontRight),
      _frontLeft(frontLeft),
      _backRight(backRight),
      _backLeft(backLeft) {}

void Motors::fullStop() const {
  frontRight().stop();
  frontLeft().stop();
  backRight().stop();
  backLeft().stop();
}

void Motors::goTo(Vector2 vector, int celerity, Radians orientation) const {
  // If the distance to the destination is less than x, stop the motors
  if (vector.norm() < 0) {  // TODO faire de 3 un parametre global
    fullStop();
  } else {
    // The speed to be sent to the motors is calculated
    float MFRcelerity = cos(vector.angle() - frontRight().angleAxisKicker());
    float MFLcelerity = cos(vector.angle() - frontLeft().angleAxisKicker());
    float MBRcelerity = cos(vector.angle() - backRight().angleAxisKicker());
    float MBLcelerity = cos(vector.angle() - backLeft().angleAxisKicker());

    // The ratio to be used to calculate the speeds to be sent to the motors is calculated, taking into account the desired speed.
    float maximum = max(abs(MFRcelerity), max(abs(MFLcelerity), max(abs(MBRcelerity), abs(MBLcelerity))));
    float rapport = celerity / maximum;

    // Speeds are recalculated taking into account the desired speed and
    // Sends speeds to motors
    float speedFR = MFRcelerity * rapport;
    float speedFL = MFLcelerity * rapport;
    float speedBR = MBRcelerity * rapport;
    float speedBL = MBLcelerity * rapport;

    float minimumSpeed = min(speedFR, min(speedFL, min(speedBR, speedBL)));
    Radians rotation = orientation * celerity * 0.6;

    if (minimumSpeed - rotation < -255) {
      float rapport = (-255 + rotation) / minimumSpeed;
      frontRight().move(speedFR * rapport - rotation);
      frontLeft().move(speedFL * rapport - rotation);
      backRight().move(speedBR * rapport - rotation);
      backLeft().move(speedBL * rapport - rotation);

    } else {
      frontRight().move(speedFR - rotation);
      frontLeft().move(speedFL - rotation);
      backRight().move(speedBR - rotation);
      backLeft().move(speedBL - rotation);
    }
  }
}


///// DribblerKicker

DribblerKicker::DribblerKicker(
    MotorMov dribbler,
    uint8_t pinKicker1,
    uint8_t pinKicker2)
    : _dribbler(dribbler),
      _pinKicker1(pinKicker1),
      _pinKicker2(pinKicker2) {
  pinMode(_pinKicker1, OUTPUT);
  pinMode(_pinKicker2, OUTPUT);
}

void DribblerKicker::dribble(int value) {
  _dribbler.move(value);
}

void DribblerKicker::kick() {
  if (millis() - _timeWhenKick > 2000) {
    _dribbler.move(0);
    delay(10);

    digitalWrite(_pinKicker1, HIGH);
    digitalWrite(_pinKicker2, HIGH);
    delay(_timeKick);
    
    digitalWrite(_pinKicker1, LOW);
    digitalWrite(_pinKicker2, LOW);

    _timeWhenKick = millis();
  }
}
