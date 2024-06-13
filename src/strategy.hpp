#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "cam_reader.hpp"
#include "lidar_analyzer_anc.hpp"
#include "parameters.hpp"
#include "utilities.hpp"

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

  FutureAction(
      int celerity,
      Radians rotation,
      bool activeKicker);

  inline Vector2 target() const { return _target.value().toVector2(); }
  inline int celerity() const { return _celerity; }
  inline Radians rotation() const { return _rotation; }
  inline bool activeKicker() const { return _activeKicker; }

  inline static FutureAction stopRobot();
};

FutureAction chooseStrategy(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnnemyGoalPos> oEGP);

bool enterInGoal_D(FieldProperties fP, LidarDetailedInfos lDI);
bool enterInGoal_C(FieldProperties fP, MyGoalPos mGP, EnnemyGoalPos eGP);

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI);
bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI);

bool ballAhead(FieldProperties fP, BallPos bP);
bool ballAtLevel(FieldProperties fP, BallPos bP);
bool ballInCenter(FieldProperties fP, BallPos bP);
bool ballIsCaught(FieldProperties fP, BallPos bP);

FutureAction refrainFromLeavingStrategy(FieldProperties fP, CamInfos cS);
FutureAction goToBallStrategy(FieldProperties fP, BallPos bP);
FutureAction goToBallAvoidingBallStrategy_C(FieldProperties fP, BallPos bP);
FutureAction goToBallAvoidingBallStrategy_CD(FieldProperties fP, LidarDetailedInfos lDI, BallPos bP);
FutureAction accelerateToGoalStrategy_C(FieldProperties fP, EnnemyGoalPos eGP);
FutureAction accelerateToGoalStrategy_D(FieldProperties fP, LidarDetailedInfos lDI);
FutureAction shootStrategy_C(FieldProperties fP, EnnemyGoalPos eGP);
FutureAction shootStrategy_D(FieldProperties fP, LidarDetailedInfos lDI);
FutureAction slalowingBackwardsStrategy(FieldProperties fP, LidarDetailedInfos lDI);

#endif