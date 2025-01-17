#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>

#include "cam_reader.hpp"
#include "lidar_analyzer_anc.hpp"
#include "logger.hpp"
#include "parameters.hpp"
#include "utilities.hpp"
#include "blue_reader.hpp"

class FutureAction {
 private:
  Optional<MutableVector2> _target;
  int _celerity;
  Radians _targetOrientation;
  bool _activeKicker;
  int _celerityDribbler;

 public:
  FutureAction(
      Vector2 target,
      int celerity,
      Radians targetOrientation,
      bool activeKicker,
      int celerityDribbler);

  FutureAction(
      int celerity,
      Radians targetOrientation,
      bool activeKicker,
      int celerityDribbler);

  inline Vector2 target() const { return _target.value().toVector2(); }
  inline bool changeTarget() const { return _target.hasValue(); }
  inline int celerity() const { return _celerity; }
  inline Radians targetOrientation() const { return _targetOrientation; }
  inline bool activeKicker() const { return _activeKicker; }
  inline int celerityDribbler() const { return _celerityDribbler; }

  inline static FutureAction stopRobot();
};

enum class Role {
  alone,
  attacker,
  defender
};

FutureAction chooseStrategyAttacker(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnemyGoalPos> oEGP,
    Optional<Vector2> oPP);

FutureAction chooseStrategyDefender(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnemyGoalPos> oEGP,
    Optional<Vector2> oPP);

EnemyGoalPos enemyGoalPosTheorical(FieldProperties fP);
MyGoalPos myGoalPosTheorical(FieldProperties fP);
Vector2 globalToLocalCoordinates(LidarDetailedInfos lDI, Vector2 target);

Role findMyRole(Optional<LidarDetailedInfos> oLDI,
                Optional<BallPos> oBP,
                MyGoalPos mGP,
                Optional<Vector2> otherPos,
                Optional<Vector2> otherBallPos);

bool enterInMyGoal_C(FieldProperties fP, MyGoalPos mGP);
bool enterInMyGoal_D(FieldProperties fP, LidarDetailedInfos lDI);
bool enterInEnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP);
bool enterInEnemyGoal_D(FieldProperties fP, LidarDetailedInfos lDI);

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI);
bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI);

bool ballInCorner_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI);
bool ballAhead(FieldProperties fP, BallPos bP);
bool ballAtLevel(FieldProperties fP, BallPos bP);
bool ballInCenter(FieldProperties fP, BallPos bP);
bool ballIsCaught(FieldProperties fP, BallPos bP);

bool closeEnoughToKick_D(FieldProperties fP, LidarDetailedInfos lDI);
bool closeEnoughToKick_C(FieldProperties fP, EnemyGoalPos eGP);
bool orientedTowardsEnemyGoal_D(FieldProperties fP, LidarBasicInfos lBI, LidarDetailedInfos lDI);
bool alignedWithBallAndGoal_D(FieldProperties fP, LidarBasicInfos lBI, LidarDetailedInfos lDI, BallPos bP);

bool enemyGoalInCenter(FieldProperties fP, EnemyGoalPos eGP);

bool robotOnSide(FieldProperties fP, LidarDetailedInfos lDI);
bool robotInCenter(FieldProperties fP, LidarDetailedInfos lDI);

FutureAction refrainLeavingField_D(FieldProperties fP, LidarDetailedInfos lDI);
FutureAction refrainLeavingField_B(FieldProperties fP, LidarBasicInfos lBI);

FutureAction refrainEnterInMyGoal_C(FieldProperties fP, MyGoalPos mGP);
FutureAction refrainEnterInEnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP);

FutureAction goToBall_C(FieldProperties fP, BallPos bP);

FutureAction goToBallChangingOrientation_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI);
FutureAction goToBallAvoidingBall_C(FieldProperties fP, BallPos bP);
FutureAction goToBallAvoidingBall_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI);

FutureAction accelerateToGoal_C(FieldProperties fP, EnemyGoalPos eGP);
FutureAction accelerateToGoal_D(FieldProperties fP, LidarDetailedInfos lDI, LidarBasicInfos lBI);

FutureAction spinToWin_D(FieldProperties fP, LidarDetailedInfos lDI);
FutureAction shoot_C(FieldProperties fP, EnemyGoalPos eGP);
FutureAction shoot_D(FieldProperties fP, LidarDetailedInfos lDI);

FutureAction slalomingBackwards_D(FieldProperties fP, LidarDetailedInfos lDI);

#endif