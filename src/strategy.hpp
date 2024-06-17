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
  inline bool changeTarget() const { return _target.hasValue(); }
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
    Optional<EnemyGoalPos> oEGP);

bool enterInMyGoal_C(FieldProperties fP, MyGoalPos mGP);
bool enterInEnnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP);

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI);
bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI);

bool ballAhead(FieldProperties fP, BallPos bP);
bool ballAtLevel(FieldProperties fP, BallPos bP);
bool ballInCenter(FieldProperties fP, BallPos bP);
bool ballIsCaught(FieldProperties fP, BallPos bP);

FutureAction refrainLeavingField_D(FieldProperties fP, LidarDetailedInfos lDI);
FutureAction refrainLeavingField_B(FieldProperties fP, LidarBasicInfos lBI);

FutureAction refrainEnterInMyGoal_C(FieldProperties fP, MyGoalPos mGP);
FutureAction refrainEnterInEnnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP);

FutureAction goToBall_C(FieldProperties fP, BallPos bP);

FutureAction goToBallAvoidingBall_C(FieldProperties fP, BallPos bP);
FutureAction goToBallAvoidingBall_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI);

FutureAction accelerateToGoal_C(FieldProperties fP, EnemyGoalPos eGP);
FutureAction accelerateToGoal_D(FieldProperties fP, LidarDetailedInfos lDI);

FutureAction shoot_C(FieldProperties fP, EnemyGoalPos eGP);
FutureAction shoot_D(FieldProperties fP, LidarDetailedInfos lDI);

FutureAction slalowingBackwards_D(FieldProperties fP, LidarDetailedInfos lDI);

#endif