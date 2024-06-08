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

FutureAction FutureAction::stopRobot() {
  return FutureAction(Vector2(0, 0), 0, 0, false);
}

//////// Functions

//TODO: remove parameters
const int criticalWallDistance = 25;
const int goalMinDistance = 90;  // 85 pour SN10 et 95 pour SN9
const int myGoalMinDistance = 82;
const int speedmotors = 120;
const int shootSpeed = 180;

FutureAction chooseStrategy(FieldProperties fP, CamInfos cS, FutureAction lA) {
  if (robotIsLost(fP, cS)) {
    if (leavingField(fP, cS)) {
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

bool lidarIssue(FieldProperties fP, CamInfos cS) {
  return cS.nearestWall() == Vector2(-9999, -9999);
}

bool robotIsLost(FieldProperties fP, CamInfos cS) {
  return cS.myPos() == Vector2(-999.9, -999.9);
}

bool leavingField(FieldProperties fP, CamInfos cS) {
  if (!robotIsLost(fP, cS)) {
    SerialDebug.println("Left wall : " + String(cS.myPos().x() < -fP.fieldWidth() / 2 + 3 * fP.robotRadius()) + " Right wall : " + String((fP.fieldWidth() / 2) - 3 * fP.robotRadius() < cS.myPos().x()) + " Back wall : " + String(cS.myPos().y() < -fP.fieldLength() / 2 + 3 * fP.robotRadius()) + " Front wall : " + String(fP.fieldLength() / 2 - 3 * fP.robotRadius() < cS.myPos().y()) + " Enemy goal : " + String(cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) + " My goal : " + String(cS.myGoalPos().norm() < myGoalMinDistance && cS.myGoalPos().norm() > 1) + " Nearest wall : " + String(cS.nearestWall().norm() < criticalWallDistance));

    int distanceDevitementY;
    if (abs(cS.myPos().x()) < 40) {
      distanceDevitementY = 50;
    } else {
      distanceDevitementY = criticalWallDistance;
    }

    SerialDebug.println(distanceDevitementY);

    return (cS.myPos().x() < -fP.fieldWidth() / 2 + criticalWallDistance) ||
           (fP.fieldWidth() / 2 - criticalWallDistance < cS.myPos().x()) ||
           (cS.myPos().y() < -fP.fieldLength() / 2 + distanceDevitementY - 5) ||
           (fP.fieldLength() / 2 - distanceDevitementY < cS.myPos().y()) ||
           (cS.nearestWall().norm() < criticalWallDistance);

  } else if (!lidarIssue(fP, cS)) {
    return (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) ||
           (cS.myGoalPos().norm() < myGoalMinDistance && cS.myGoalPos().norm() > 1) ||
           (cS.nearestWall().norm() < criticalWallDistance);

  } else {
    return (cS.enemyGoalPos().norm() < goalMinDistance && cS.enemyGoalPos().norm() > 1) ||
           (cS.myGoalPos().norm() < myGoalMinDistance && cS.myGoalPos().norm() > 1);
  }
}

bool ballIsDetected(FieldProperties fP, CamInfos cS) {
  return cS.ballPos() != Vector2(0, 0);
}

bool goalIsDetected(FieldProperties fP, CamInfos cS) {
  return cS.enemyGoalPos() != Vector2(0, 0);
}

bool targetInFrontOfRobotFromFront(FieldProperties fP, CamInfos cS, Vector2 tL) {
  float longRobot = (fP.robotRadius() * 1);  // Aussi modifié sur autre ordi
  return tL.y() > longRobot;
}

bool targetInFrontOfRobotFromMiddle(FieldProperties fP, CamInfos cS, Vector2 tL) {
  return tL.y() > 0;
}

bool targetCenterOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL) {
  return abs(tL.x()) <= 25;
}

bool targetJustInFrontOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL) {
  return targetInFrontOfRobotFromMiddle(fP, cS, tL) && targetCenterOfRobot(fP, cS, tL);
}

bool targetJustBehindOfRobot(FieldProperties fP, CamInfos cS, Vector2 tL) {
  return (!targetInFrontOfRobotFromMiddle(fP, cS, tL)) && targetCenterOfRobot(fP, cS, tL);
}

bool ballIsCaught(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("ballIsCaught : " + String(targetJustInFrontOfRobot(fP, cS, cS.ballPos()) && cS.ballPos().y() <= 40));
  return targetJustInFrontOfRobot(fP, cS, cS.ballPos()) && cS.ballPos().y() <= 40;
}

float correctOrientation(CamInfos cS) {
  return sin(cS.orientation()) * 25;
}

FutureAction refrainFromLeavingStrategy(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("refrainFromLeavingStrategy");

  float xDirection = 0;
  float yDirection = 0;

  if (!robotIsLost(fP, cS)) {
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

  } else if (!lidarIssue(fP, cS)) {
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

FutureAction goToBallStrategy(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("goToBallStrategy");

  return FutureAction(
      Vector2(
          cS.ballPos().x(),
          cS.ballPos().y() - fP.robotRadius() * 4),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction goToBallAvoidingBallStrategyWithCam(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("goToBallAvoidingBallStrategyWithCam");
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    return FutureAction(
        Vector2(10, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else if (cS.ballPos().x() < 0) {
    return FutureAction(
        Vector2(2, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity
  } else if (cS.ballPos().x() > 0) {
    return FutureAction(
        Vector2(-2, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity
  }
}

FutureAction goToBallAvoidingBallStrategyWithLidar(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("goToBallAvoidingBallStrategyWithLidar");
  if (targetJustBehindOfRobot(fP, cS, cS.ballPos())) {
    if (cS.ballPos().x() < 0) {
      if (cS.myPos().x() > (fP.fieldWidth() / 2) - 6 * fP.robotRadius()) {
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
      if (-(fP.fieldWidth() / 2) + 6 * fP.robotRadius() > cS.myPos().x()) {
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

  } else if (cS.ballPos().x() < 0 && cS.ballPos().x() > -40) {
    return FutureAction(
        Vector2(5, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else if (cS.ballPos().x() > 0 && cS.ballPos().x() < 40) {
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

FutureAction accelerateToGoalStrategyWithCam(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("accelerateToGoalStrategyWithCam");

  return FutureAction(
      Vector2(
          cS.enemyGoalPos().x(),
          cS.enemyGoalPos().y()),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction accelerateToGoalStrategyWithLidar(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("accelerateToGoalStrategyWithLidar");

  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x(),
          10),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction slalowingBackwardsStrategy(FieldProperties fP, CamInfos cS, FutureAction lA) {
  SerialDebug.println("slalowingBackwardsStrategy");
  if (cS.myPos().y() < -50) {
    if (cS.myPos().x() < -5) {
      return FutureAction(
          Vector2(10, 0),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else if (5 < cS.myPos().x()) {
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

  } else if (50 < cS.myPos().y()) {
    return FutureAction(
        Vector2(-20, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else {
    if (cS.myPos().x() < -fP.fieldWidth() / 6) {
      return FutureAction(
          Vector2(20, -10),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else if (fP.fieldWidth() / 6 < cS.myPos().x()) {
      return FutureAction(
          Vector2(-20, -10),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    } else {
      return FutureAction(
          lA.target(),
          speedmotors,
          0,
          false);  //@Gandalfph add orientation and celerity
    }
  }
}

FutureAction shootStrategy(FieldProperties fP, CamInfos cS) {
  SerialDebug.println("shootStrategy");

  return FutureAction(
      Vector2(0, 20),
      shootSpeed,
      0,
      true);  //@Gandalfph add orientation and celerity
}