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

  inline Vector2 target() const { return _target.value().toVector2(); }
  inline int celerity() const { return _celerity; }
  inline Radians rotation() const { return _rotation; }
  inline bool activeKicker() const { return _activeKicker; }

  inline static FutureAction stopRobot();
};

FutureAction chooseStrategy(FieldProperties fP, Optional<CamInfos> optionalCI, Optional<LidarDetailedInfos> optionalLDI, Optional<LidarBasicInfos> optionalLBI);

bool camHasIssue(Optional<CamInfos> optionalCI);
bool lidarDetailedHasIssue(Optional<LidarDetailedInfos> optionalLDI);
bool lidarBasicHasIssue(Optional<LidarBasicInfos> optionalLBI);

bool enterInGoalWithLidarDetailed(FieldProperties fP, LidarDetailedInfos lDI);
bool enterInGoalWithLidarBasic(FieldProperties fP, LidarBasicInfos lBI);
bool enterInGoalWithCam(FieldProperties fP, CamInfos cI);

bool leavingFieldWithLidarDetailed(FieldProperties fP, LidarDetailedInfos lDI);
bool leavingFieldWithLidarBasic(FieldProperties fP, LidarBasicInfos lBI);
bool leavingFieldWithCam(FieldProperties fP, CamInfos cI);

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