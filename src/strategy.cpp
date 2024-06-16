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
    Optional<EnemyGoalPos> oEGP) {
  // First we look to see if there's a risk of leaving the field
  SerialDebug.println("in chooseStrategy");
  SerialDebug.flush();
  if (oLDI.hasValue()) {
    if (leavingField_D(fP, oLDI.value())) {
      return refrainLeavingField_D(fP, oLDI.value());
    }
  } else if (oLBI.hasValue()) {
    if (leavingField_B(fP, oLBI.value())) {
      return refrainLeavingField_B(fP, oLBI.value());
    }
  } else {
    return FutureAction::stopRobot();
  }
  if (oMGP.hasValue()) {
    if (enterInMyGoal_C(fP, oMGP.value())) {
      return refrainEnterInMyGoal_C(fP, oMGP.value());
    }
  }
  if (oEGP.hasValue()) {
    if (enterInEnnemyGoal_C(fP, oEGP.value())) {
      return refrainEnterInEnnemyGoal_C(fP, oEGP.value());
    }
  }

  SerialDebug.println("in chooseStrategy.chooseStrategy");
  SerialDebug.flush();
  // Then we choose the appropriate Strategy
  if (!oBP.hasValue()) {
    SerialDebug.println(String(oBP.hasValue()));
    // We don't know where is the ball
    if (oLDI.hasValue()) {
      return slalowingBackwards_D(fP, oLDI.value());
    } else {
      return FutureAction::stopRobot();
    }
  } else {
    SerialDebug.println("in else");
    SerialDebug.flush();
    BallPos bP = oBP.value();
    if (ballIsCaught(fP, bP)) {
      SerialDebug.println("in ball is caught");
      SerialDebug.flush();
      // The ball is caught
      if (oEGP.hasValue()) {
        if (ballInCenter(fP, bP)) {
          return shoot_C(fP, oEGP.value());
        } else {
          return accelerateToGoal_C(fP, oEGP.value());
        }
      } else if (oLDI.hasValue()) {
        if (ballInCenter(fP, bP)) {
          return shoot_D(fP, oLDI.value());
        } else {
          return accelerateToGoal_D(fP, oLDI.value());
        }
      } else {
        return FutureAction::stopRobot();
      }
    } else {
      SerialDebug.println("in ball is not caught");
      SerialDebug.flush();
      // The ball is not caught
      if (ballAhead(fP, bP)) {
        return goToBall_C(fP, bP);
      } else {
        if (oLDI.hasValue()) {
          return goToBallAvoidingBall_CD(fP, bP, oLDI.value());
        } else {
          return goToBallAvoidingBall_C(fP, bP);
        }
      }
    }
  }
}

bool enterInMyGoal_C(FieldProperties fP, MyGoalPos mGP) {
  return mGP.norm() < myGoalMinDistance && mGP.norm() > 1;
}

bool enterInEnnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  return (eGP.norm() < goalMinDistance && eGP.norm() > 1);
}

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI) {
  bool leftWall = lDI.coordinates().x() < -fP.fieldWidth() / 2 + criticalWallDistance;
  bool rightWall = fP.fieldWidth() / 2 - criticalWallDistance < lDI.coordinates().x();
  bool backWall = lDI.coordinates().y() < -fP.fieldLength() / 2 + criticalWallDistance - 5;
  bool frontWall = fP.fieldLength() / 2 - criticalWallDistance < lDI.coordinates().y();
  
  SerialDebug.println("Left Wall : " + String(leftWall) +
                      " Right Wall : " + String(rightWall) +
                      " Back Wall : " + String(backWall) +
                      " Front Wall : " + String(frontWall));
  return leftWall || rightWall || backWall || frontWall;
}

bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI) {
  bool approachingNearestWall = lBI.norm() < criticalWallDistance;
  SerialDebug.println("Nearest Wall too near: " + String(approachingNearestWall));
  return approachingNearestWall;
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
  return ballAtLevel(fP, bP) && ballInCenter(fP, bP) && bP.y() <= 40;  // TODO create parameter
}

FutureAction refrainLeavingField_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("Choosed strategy : refrainLeavingField_D");
  float xDirection = 0;
  float yDirection = 0;

  if (lDI.coordinates().x() < -fP.fieldWidth() / 2 + criticalWallDistance) {
    xDirection = cos(lDI.orientation());
    yDirection = sin(lDI.orientation());
  } else if (fP.fieldWidth() / 2 - criticalWallDistance < lDI.coordinates().x()) {
    xDirection = -cos(lDI.orientation());
    yDirection = -sin(lDI.orientation());
  }

  if (lDI.coordinates().y() < -fP.fieldLength() / 2 + criticalWallDistance - 5) {
    xDirection = sin(lDI.orientation());
    yDirection = cos(lDI.orientation());
  } else if (fP.fieldLength() / 2 - criticalWallDistance < lDI.coordinates().y()) {
    xDirection = -sin(lDI.orientation());
    yDirection = -cos(lDI.orientation());
  }
  return FutureAction(
      Vector2(
          xDirection,
          yDirection),
      speedmotors,
      0,
      false);
}

FutureAction refrainLeavingField_B(FieldProperties fP, LidarBasicInfos lBI) {
  SerialDebug.println("Choosed strategy : refrainLeavingField_B");
  return FutureAction(
      Vector2(
          -lBI.x(),
          -lBI.y()),
      speedmotors,
      0,
      false);
}

FutureAction refrainEnterInMyGoal_C(FieldProperties fP, MyGoalPos mGP) {
  SerialDebug.println("Choosed strategy : refrainEnterInMyGoal_C");
  return FutureAction(
      Vector2(
          -mGP.x(),
          -mGP.y()),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction refrainEnterInEnnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  SerialDebug.println("Choosed strategy : refrainEnterInEnnemyGoal_C");
  return FutureAction(
      Vector2(
          -eGP.x(),
          -eGP.y()),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction goToBall_C(FieldProperties fP, BallPos bP) {
  SerialDebug.println("Choosed strategy : goToBall_C");
  return FutureAction(
      Vector2(
          bP.x(),
          bP.y() - fP.robotRadius() * 4),  // TODO make parameter
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction goToBallAvoidingBall_C(FieldProperties fP, BallPos bP) {
  SerialDebug.println("Choosed strategy : goToBallAvoidingBall_C");
  if (!ballAtLevel(fP, bP) && ballInCenter(fP, bP)) {
    return FutureAction(
        Vector2(10, -10),
        speedmotors,
        0,
        false);  //@Gandalfph add orientation and celerity

  } else if (bP.x() <= 0) {
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
  } else {
    Serial.println("ERROR STRANGE");
  }
}

FutureAction goToBallAvoidingBall_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI) {
  SerialDebug.println("Choosed strategy : goToBallAvoidingBall_CD");
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

FutureAction accelerateToGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  SerialDebug.println("Choosed strategy : accelerateToGoal_C");
  return FutureAction(
      eGP,
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction accelerateToGoal_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("Choosed strategy : accelerateToGoal_D");
  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x() - lDI.coordinates().x(),
          fP.enemyGoalPos().y() - lDI.coordinates().y()),
      speedmotors,
      0,
      false);  //@Gandalfph add orientation and celerity
}

FutureAction shoot_C(FieldProperties fP, EnemyGoalPos eGP) {
  SerialDebug.println("Choosed strategy : shoot_C");
  return FutureAction(
      eGP,
      shootSpeed,
      0,
      true);  //@Gandalfph add orientation and celerity
}

FutureAction shoot_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("Choosed strategy : shoot_D");
  return FutureAction(
      Vector2(
          fP.enemyGoalPos().x() - lDI.coordinates().x(),
          fP.enemyGoalPos().y() - lDI.coordinates().y()),
      shootSpeed,
      0,
      true);  //@Gandalfph add orientation and celerity
}

FutureAction slalowingBackwards_D(FieldProperties fP, LidarDetailedInfos lDI) {
  SerialDebug.println("Choosed strategy : slalowingBackwards_D");
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