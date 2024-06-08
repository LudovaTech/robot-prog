#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "cam_reader.h"
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

  inline static FutureAction stopRobot();
};

FutureAction chooseStrategy(FieldProperties fP, CamInfos cS, FutureAction lA);

bool robotIsLost(FieldProperties fP, CamInfos cS);
bool leavingField(FieldProperties fP, CamInfos cS);
bool targetInFrontOfRobotFromFront(FieldProperties fP, CamInfos cS, Vector2 tL);
bool targetInFrontOfRobotFromMiddle(FieldProperties fP, CamInfos cS, Vector2 tL);
bool targetCenterOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL);
bool targetJustInFrontOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL);
bool targetJustBehindOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL);
bool goalIsDetected(FieldProperties fP, CamInfos cS);
bool ballIsDetected(FieldProperties fP, CamInfos cS);
bool ballIsCaught(FieldProperties fP, CamInfos cS);

FutureAction refrainFromLeavingStrategy(FieldProperties fP, CamInfos cS);
FutureAction goToBallStrategy(FieldProperties fP, CamInfos cS);
FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, CamInfos cS);
FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, CamInfos cS);
FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, CamInfos cS);
FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, CamInfos cS);
FutureAction slalowingBackwardsStrategy(FieldProperties fP, CamInfos cS, FutureAction lA);
FutureAction shootStrategy(FieldProperties fP, CamInfos cS);

#endif