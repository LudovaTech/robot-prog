#include "strategy.hpp"

//////// FutureAction

FutureAction::FutureAction(
    Vector2 target,
    int celerity,
    Radians rotation,
    bool activeKicker)
    : _target(Optional<MutableVector2>(target)),
      _celerity(celerity),
      _rotation(rotation),
      _activeKicker(activeKicker) {}

FutureAction::FutureAction(
    int celerity,
    Radians rotation,
    bool activeKicker)
    : _target(Optional<MutableVector2>()),
      _celerity(celerity),
      _rotation(rotation),
      _activeKicker(activeKicker) {}

FutureAction FutureAction::stopRobot() {
  return FutureAction(Vector2(0, 0), 0, 0, false);
}

//////// Functions

// TODO: remove parameters
const int criticalWallDistance = 25;
const int goalMinDistance = 90;  // 85 pour SN10 et 95 pour SN9
const int myGoalMinDistance = 82;
const int speedmotors = 120;
const int shootSpeed = 180;

FutureAction chooseStrategy(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnnemyGoalPos> oEGP) {
  if (camHasIssue(optionalCI)) {
    if (leavingField(fP, cI)) {
      return refrainFromLeavingStrategy(fP, cS);
    } else if (!ballIsDetected(fP, cS)) {
      SerialDebug.println("stopRobotStrategy");
      return FutureAction::stopRobot();

    } else if (ballIsCaught(fP, cS)) {
      if (!goalIsDetected(fP, cS)) {
        SerialDebug.println("stopRobotStrategy");
        return FutureAction::stopRobot();
      } else if (targetJustInFrontOfRobot(fP, cS, cS.enemyGoalPos())) {
        return shootStrategy(fP, cS);
      } else {
        return accelerateToGoalStrategyWithCam(fP, cS);
      }
    } else {
      if (targetInFrontOfRobotFromFront(fP, cS, cS.ballPos())) {
        return goToBallStrategy(fP, cS);
      } else {
        return goToBallAvoidingBallStrategyWithCam(fP, cS);
      }
    }

  } else {
    if (leavingField(fP, cS)) {
      return refrainFromLeavingStrategy(fP, cS);

    } else if (!ballIsDetected(fP, cS)) {
      return slalowingBackwardsStrategy(fP, cS, lA);

    } else if (ballIsCaught(fP, cS)) {
      if (targetJustInFrontOfRobot(fP, cS, cS.myPos().distanceRef(fP.enemyGoalPos()))) {
        return shootStrategy(fP, cS);
      } else {
        return accelerateToGoalStrategyWithLidar(fP, cS);
      }

    } else {
      if (targetInFrontOfRobotFromFront(fP, cS, cS.ballPos())) {
        return goToBallStrategy(fP, cS);
      } else {
        return goToBallAvoidingBallStrategyWithLidar(fP, cS);
      }
    }
  }
}

bool enterInGoal_D(FieldProperties fP, LidarDetailedInfos lDI) {
  // TODO with calculated positions
}

bool enterInGoal_C(FieldProperties fP, MyGoalPos mGP, EnnemyGoalPos eGP) {
  return (eGP.norm() < goalMinDistance && eGP.norm() > 1) ||
         (mGP.norm() < myGoalMinDistance && mGP.norm() > 1);
}

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI) {
  int distanceDevitementY;
  if (abs(lDI.coordinates().x()) < 40) {
    distanceDevitementY = 50;
  } else {
    distanceDevitementY = criticalWallDistance;
  }

  SerialDebug.println(distanceDevitementY);

  return (lDI.coordinates().x() < -fP.fieldWidth() / 2 + criticalWallDistance) ||
         (fP.fieldWidth() / 2 - criticalWallDistance < lDI.coordinates().x()) ||
         (lDI.coordinates().y() < -fP.fieldLength() / 2 + distanceDevitementY - 5) ||
         (fP.fieldLength() / 2 - distanceDevitementY < lDI.coordinates().y());
}

bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI) {
  return lBI.norm() < criticalWallDistance;
}

bool ballAhead(FieldProperties fP, BallPos bP) {
  float longRobot = (fP.robotRadius() * 1);
  return bP.y() > longRobot;
}

bool ballAtLevel(FieldProperties fP, BallPos bP) {
  return bP.y() > 0;
}

bool ballInCenter(FieldProperties fP, BallPos bP) {
  return abs(bP.x()) <= 25;  // TODO create parameter
}

bool ballIsCaught(FieldProperties fP, BallPos bP) {
  bool r = ballAtLevel(fP, bP) && ballInCenter(fP, bP) && bP.y() <= 40;  // TODO create parameter
  SerialDebug.println("ballIsCaught : " + String(r));
  return r;
}

float correctOrientation(LidarDetailedInfos lDI) {
  return sin(lDI.orientation()) * 25;
}

FutureAction refrainFromLeavingStrategy(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("refrainFromLeavingStrategy");

  float xDirection = 0;
  float yDirection = 0;

  if (!camHasIssue(fP, cS)) {
    int distanceDevitementY;
    if (abs(cS.myPos().x()) < 40) {
      distanceDevitementY = 50;
    } else {
      distanceDevitementY = criticalWallDistance;
    }

    if (cS.myPos().x() < -fP.fieldWidth() / 2 + criticalWallDistance) {
      xDirection = cos(cS.orientation());
      yDirection = sin(cS.orientation());
    } else if (fP.fieldWidth() / 2 - criticalWallDistance < cS.myPos().x()) {
      xDirection = -cos(cS.orientation());
      yDirection = -sin(cS.orientation());
    }

    if (cS.myPos().y() < -fP.fieldLength() / 2 + distanceDevitementY - 5) {
      xDirection = sin(cS.orientation());
      yDirection = cos(cS.orientation());
    } else if (fP.fieldLength() / 2 - distanceDevitementY < cS.myPos().y()) {
      xDirection = -sin(cS.orientation());
      yDirection = -cos(cS.orientation());
    }

    if (cS.nearestWall().norm() < criticalWallDistance) {
      xDirection = -cS.nearestWall().x();
      yDirection = -cS.nearestWall().y();
    }

  } else if (!lidarDetailedHasIssue(fP, cS)) {
    if (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) {
      xDirection = -cS.enemyGoalPos().x();
      yDirection = -cS.enemyGoalPos().y();
    } else if (cS.myGoalPos().norm() < myGoalMinDistance && cS.myGoalPos().norm() > 1) {
      xDirection = -cS.myGoalPos().x();
      yDirection = -cS.myGoalPos().y();
    }

    if (cS.nearestWall().norm() < criticalWallDistance) {
      xDirection = -cS.nearestWall().x();
      yDirection = -cS.nearestWall().y();
    }

  } else {
    if (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) {
      xDirection = -cS.enemyGoalPos().x();
      yDirection = -cS.enemyGoalPos().y();
    } else if (cS.myGoalPos().norm() < myGoalMinDistance && cS.myGoalPos().norm() > 1) {
      xDirection = -cS.myGoalPos().x();
      yDirection = -cS.myGoalPos().y();
    }
  }

  SerialDebug.println(String(xDirection) + " " + String(yDirection));
  return FutureAction(
      Vector2(
          xDirection,
          yDirection),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction goToBallStrategy(FieldProperties fP, BallPos bP) {
  SerialDebug.println("goToBallStrategy");

  return FutureAction(
      Vector2(
          bP.x(),
          bP.y() - fP.robotRadius() * 4),  // TODO make parameter
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction goToBallAvoidingBallStrategy_C(FieldProperties fP, BallPos bP) {
  SerialDebug.println("goToBallAvoidingBallStrategy_C");
  if (!ballAtLevel(fP, bP) && ballInCenter(fP, bP)) {
    return FutureAction(
        Vector2(10, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else if (bP.x() < 0) {
    return FutureAction(
        Vector2(2, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity
  } else if (bP.x() > 0) {
    return FutureAction(
        Vector2(-2, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity
  }
}

FutureAction goToBallAvoidingBallStrategy_CD(FieldProperties fP, LidarDetailedInfos lDI, BallPos bP) {
  SerialDebug.println("goToBallAvoidingBallStrategy_CD");
  if (!ballAtLevel(fP, bP) && ballInCenter(fP, bP)) {
    if (bP.x() < 0) {
      if (lDI.coordinates().x() > (fP.fieldWidth() / 2) - 6 * fP.robotRadius()) {
        return FutureAction(
            Vector2(-10, -10),
            speedmotors,
            0,
            false);  //@Gandalfph add orientation and celerity
      } else {
        return FutureAction(
            Vector2(10, -10),
            speedmotors,
            0,
            false);  //@Gandalfph add orientation and celerity
      }

    } else {
      if (-(fP.fieldWidth() / 2) + 6 * fP.robotRadius() > lDI.coordinates().x()) {
        return FutureAction(
            Vector2(10, -10),
            speedmotors,
            0,
            false);  //@Gandalfph add orientation and celerity

      } else {
        return FutureAction(
            Vector2(-10, -10),
            speedmotors,
            0,
            false);  //@Gandalfph add orientation and celerity
      }
    }

  } else if (bP.x() < 0 && bP.x() > -40) {
    return FutureAction(
        Vector2(5, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else if (bP.x() > 0 && bP.x() < 40) {
    return FutureAction(
        Vector2(-5, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else {
    return FutureAction(
        Vector2(0, -10),
        speedmotors,
        0,
        false);
  }
}

FutureAction accelerateToGoalStrategy_C(FieldProperties fP, EnnemyGoalPos eGP) {
  SerialDebug.println("accelerateToGoalStrategy_C");

  return FutureAction(
      eGP,
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction accelerateToGoalStrategy_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("accelerateToGoalStrategy_D");

  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x() - lDI.coordinates().x(),
          fP.enemyGoalPos().y() - lDI.coordinates().y()),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction shootStrategy_C(FieldProperties fP, EnnemyGoalPos eGP) {
  SerialDebug.println("shootStrategy_C");

  return FutureAction(
      accelerateToGoalStrategy_C(fP, eGP).target(),
      shootSpeed,
      0,
      true);  //@Gandalfph add orientation and celerity
}

FutureAction shootStrategy_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("shootStrategy_D");

  return FutureAction(
      accelerateToGoalStrategy_D(fP, lDI).target(),
      shootSpeed,
      0,
      true);  //@Gandalfph add orientation and celerity
}

FutureAction slalowingBackwardsStrategy(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("slalowingBackwardsStrategy");
  if (lDI.coordinates().y() < -50) {
    if (lDI.coordinates().x() < -5) {
      return FutureAction(
          Vector2(10, 0),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else if (5 < lDI.coordinates().x()) {
      return FutureAction(
          Vector2(-10, 0),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else {
      return FutureAction(
          Vector2(0, 10),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    }

  } else if (50 < lDI.coordinates().y()) {
    return FutureAction(
        Vector2(-20, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else {
    if (lDI.coordinates().x() < -fP.fieldWidth() / 6) {
      return FutureAction(
          Vector2(20, -10),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else if (fP.fieldWidth() / 6 < lDI.coordinates().x()) {
      return FutureAction(
          Vector2(-20, -10),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else {
      return FutureAction(
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    }
  }
}