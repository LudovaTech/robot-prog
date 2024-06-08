#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "states.h"
#include "utilities.h"

class FutureAction {
 private:
  Optional<MutableVector2> _target;
  int _celerity;
  Radians _rotation;
  bool _activeKicker;

 public:
  FutureAction(
      Vector2 target,
      int celerity,
      Radians rotation,
      bool activeKicker);

  inline Vector2 target() const { return _target.value().toVector2(); }
  inline int celerity() const { return _celerity; }
  inline Radians rotation() const { return _rotation; }
  inline bool activeKicker() const { return _activeKicker; }
};

FutureAction chooseStrategy(FieldProperties fP, RobotState cS, FutureAction lA);

bool robotIsLost(FieldProperties fP, RobotState cS);
bool leavingField(FieldProperties fP, RobotState cS);
bool targetInFrontOfRobotFromFront(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetInFrontOfRobotFromMiddle(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetCenterOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustInFrontOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool targetJustBehindOfRobot(FieldProperties fP, RobotState cS, Vector2 tL);
bool goalIsDetected(FieldProperties fP, RobotState cS);
bool ballIsDetected(FieldProperties fP, RobotState cS);
bool ballIsCaught(FieldProperties fP, RobotState cS);

FutureAction refrainFromLeavingStrategy(FieldProperties fP, RobotState cS);
FutureAction goToBallStrategy(FieldProperties fP, RobotState cS);
FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, RobotState cS);
FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, RobotState cS);
FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, RobotState cS);
FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, RobotState cS);
FutureAction slalowingBackwardsStrategy(FieldProperties fP, RobotState cS, FutureAction lA);
FutureAction shootStrategy(FieldProperties fP, RobotState cS);

#endif