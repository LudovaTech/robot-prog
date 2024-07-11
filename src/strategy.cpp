#include "strategy.hpp"

//////// FutureAction

FutureAction::FutureAction(
    Vector2 target,
    int celerity,
    Radians targetOrientation,
    bool activeKicker,
    int celerityDribbler)
    : _target(Optional<MutableVector2>(target)),
      _celerity(celerity),
      _targetOrientation(targetOrientation),
      _activeKicker(activeKicker),
      _celerityDribbler(celerityDribbler) {}

FutureAction::FutureAction(
    int celerity,
    Radians targetOrientation,
    bool activeKicker,
    int celerityDribbler)
    : _target(Optional<MutableVector2>()),
      _celerity(celerity),
      _targetOrientation(targetOrientation),
      _activeKicker(activeKicker),
      _celerityDribbler(celerityDribbler) {}

FutureAction FutureAction::stopRobot() {
  return FutureAction(Vector2(0, 0), 0, 0, false, 0);
}

//////// Functions

EnemyGoalPos enemyGoalPosTheorical(FieldProperties fP) {
  return EnemyGoalPos(0, fP.distanceYGoalFromCenter());
}

MyGoalPos myGoalPosTheorical(FieldProperties fP) {
  return MyGoalPos(0, -fP.distanceYGoalFromCenter());
}

Vector2 globalToLocalCoordinates(LidarDetailedInfos lDI, Vector2 target) {
  return Vector2(
             target.x() - lDI.coordinates().x(),
             target.y() - lDI.coordinates().y())
      .rotate(-lDI.orientation());
}

// TODO: remove parameters
const int criticalWallDistance = 25;
const int criticalGoalDistance = 25;  // changer avec la bonne valeur
const int goalMinDistance = 90;       // 85 pour SN10 et 95 pour SN9
const int myGoalMinDistance = 82;
const int speedmotors = 140;
const int shootSpeed = 200;
const int distanceKickOK = 120;
bool wasSlalomingBackwards = false;
int dribblerSpeedIfLeavingField = 0;

Role memoryRole = Role::alone;

Role findMyRole(Optional<LidarDetailedInfos> oLDI,
                Optional<BallPos> oBP,
                MyGoalPos mGP,
                Optional<Vector2> otherPos,
                Optional<Vector2> otherBallPos) {
  if (oBP.hasValue() && otherBallPos.hasValue()) {
    // le plus proche de la balle est attaquant
    SerialDebug.println("balle");
    if (oBP.value().norm() < otherBallPos.value().norm()) {
      return Role::attacker;
    } else {
      return Role::defender;
    }
  } else if (otherPos.hasValue() && oLDI.hasValue()) {
    // le plus proche du goal ami est défenseur
    SerialDebug.println("goal");
    if (oLDI.value().coordinates().distance(mGP) < otherPos.value().distance(mGP)) {
      return Role::defender;
    } else {
      return Role::attacker;
    }
  } else {
    return Role::alone;
  }
}

FutureAction chooseStrategyAttacker(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnemyGoalPos> oEGP) {
  // First we look to see if there's a risk of leaving the field
  if (oLDI.hasValue()) {
    if (leavingField_D(fP, oLDI.value())) {
      return refrainLeavingField_D(fP, oLDI.value());
    } else if (enterInMyGoal_D(fP, oLDI.value())) {
      // TODO
      return refrainLeavingField_D(fP, oLDI.value());
    } else if (enterInEnemyGoal_D(fP, oLDI.value())) {
      // TODO
      return refrainLeavingField_D(fP, oLDI.value());
    }
  } else if (oLBI.hasValue()) {
    if (leavingField_B(fP, oLBI.value())) {
      return refrainLeavingField_B(fP, oLBI.value());
    }
  } else if (oMGP.hasValue()) {
    if (enterInMyGoal_C(fP, oMGP.value())) {
      return refrainEnterInMyGoal_C(fP, oMGP.value());
    }
  } else if (oEGP.hasValue()) {
    if (enterInEnemyGoal_C(fP, oEGP.value())) {
      return refrainEnterInEnemyGoal_C(fP, oEGP.value());
    }
  }
  // Then we choose the appropriate Strategy
  if (!oBP.hasValue()) {
    // We don't know where the ball is
    dribblerSpeedIfLeavingField = 0;
    if (oLDI.hasValue()) {
      return slalomingBackwards_D(fP, oLDI.value());
    } else {
      return FutureAction::stopRobot();
    }
  } else {
    BallPos bP = oBP.value();
    if (ballIsCaught(fP, bP)) {
      // The ball is caught
      dribblerSpeedIfLeavingField = fP.maxDribblerSpeed();
      if (oLDI.hasValue() && oLBI.hasValue()) {
        // if (robotOnSide(fP, oLDI.value())) {
        // return spinToWin_D(fP, oLDI.value());
        if (orientedTowardsEnemyGoal_D(fP, oLDI.value()) && closeEnoughToKick_D(fP, oLDI.value())) {
          return shoot_D(fP, oLDI.value());
        } else {
          return accelerateToGoal_D(fP, oLDI.value(), oLBI.value());
        }
      } else if (oEGP.hasValue()) {
        if (enemyGoalInCenter(fP, oEGP.value())) {
          return shoot_C(fP, oEGP.value());
        } else {
          return accelerateToGoal_C(fP, oEGP.value());
        }
      } else {
        return FutureAction::stopRobot();
      }
    } else {
      // The ball is not caught
      dribblerSpeedIfLeavingField = 0;
      if (oLDI.hasValue()) {
        // if (ballInCorner_CD(fP, bP, oLDI.value()) && false) {
        // return goToBallChangingOrientation_CD(fP, bP, oLDI.value());
        if (ballAhead(fP, bP)) {
          return goToBall_C(fP, bP);
        } else {
          return goToBallAvoidingBall_CD(fP, bP, oLDI.value());
        }
      } else {
        if (ballAhead(fP, bP)) {
          return goToBall_C(fP, bP);
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

bool enterInMyGoal_D(FieldProperties fP, LidarDetailedInfos lDI) {
  bool myGoal = (lDI.coordinates().y() < -fP.fieldLength() / 2 + criticalWallDistance + criticalGoalDistance) && (abs(lDI.coordinates().x()) < fP.goalWidth() / 2 + 10);
  log_a(StratLevel, "strategy.enterInMyGoal_D", "My Goal : " + String(myGoal));
  return myGoal;
}

bool enterInEnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  return (eGP.norm() < goalMinDistance && eGP.norm() > 1);
}

bool enterInEnemyGoal_D(FieldProperties fP, LidarDetailedInfos lDI) {
  bool enemyGoal = (lDI.coordinates().y() > fP.fieldLength() / 2 - criticalWallDistance - criticalGoalDistance) && (abs(lDI.coordinates().x()) < fP.goalWidth() / 2 + 10);
  log_a(StratLevel, "strategy.enterInEnemy_D", "Enemy Goal : " + String(enemyGoal));
  return enemyGoal;
}

bool leavingField_D(FieldProperties fP, LidarDetailedInfos lDI) {
  bool leftWall = lDI.coordinates().x() < -fP.fieldWidth() / 2 + criticalWallDistance;
  bool rightWall = fP.fieldWidth() / 2 - criticalWallDistance < lDI.coordinates().x();
  bool backWall = lDI.coordinates().y() < -fP.fieldLength() / 2 + criticalWallDistance;
  bool frontWall = fP.fieldLength() / 2 - criticalWallDistance < lDI.coordinates().y();

  log_a(StratLevel, "strategy.leavingField_D", "Left Wall : " + String(leftWall) + " Right Wall : " + String(rightWall) + " Back Wall : " + String(backWall) + " Front Wall : " + String(frontWall));
  return leftWall || rightWall || backWall || frontWall;
}

bool leavingField_B(FieldProperties fP, LidarBasicInfos lBI) {
  bool approachingNearestWall = lBI.norm() < criticalWallDistance;
  log_a(StratLevel, "strategy.leavingField_B", "Nearest Wall too near: " + String(approachingNearestWall));
  return approachingNearestWall;
}

bool ballInCorner_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI) {
  return (abs(lDI.coordinates().x() + bP.x()) > fP.goalWidth() / 2) && (abs(lDI.coordinates().y() + bP.y()) > fP.fieldLength() / 2 - criticalWallDistance - criticalGoalDistance);
}

bool ballAhead(FieldProperties fP, BallPos bP) {
  float longRobot = (fP.robotRadius() * 1);
  return bP.y() > longRobot;
}

bool ballAtLevel(FieldProperties fP, BallPos bP) {
  return bP.y() > 0;
}

bool ballInCenter(FieldProperties fP, BallPos bP) {
  return abs(bP.x()) <= 10;  // TODO create parameter
}

bool ballIsCaught(FieldProperties fP, BallPos bP) {
  return ballAtLevel(fP, bP) && ballInCenter(fP, bP) && bP.y() <= 34;  // TODO create parameter
}

bool closeEnoughToKick_D(FieldProperties fP, LidarDetailedInfos lDI) {
  return lDI.frontGoalCoordinates().norm() / 10 < distanceKickOK;
}

bool closeEnoughToKick_C(FieldProperties fP, EnemyGoalPos eGP) {
  return eGP.y() <= distanceKickOK;
}

bool orientedTowardsEnemyGoal_D(FieldProperties fP, LidarDetailedInfos lDI) {
  return lDI.orientation() < globalToLocalCoordinates(lDI, Vector2(enemyGoalPosTheorical(fP).x() - fP.goalWidth() + 5,
                                                                   enemyGoalPosTheorical(fP).y())).angle()
      && lDI.orientation() > globalToLocalCoordinates(lDI, Vector2(enemyGoalPosTheorical(fP).x() + fP.goalWidth() - 5,
                                                                   enemyGoalPosTheorical(fP).y())).angle();
}

bool enemyGoalInCenter(FieldProperties fP, EnemyGoalPos eGP) {
  return abs(eGP.x()) <= 7;  // TODO create parameter
}

bool robotOnSide(FieldProperties fP, LidarDetailedInfos lDI) {
  return (lDI.coordinates().y() > fP.fieldLength() / 2 - criticalWallDistance - criticalGoalDistance - fP.robotRadius() * 3) && (abs(lDI.coordinates().x()) > fP.goalWidth() / 2);
}

bool robotInCenter(FieldProperties fP, LidarDetailedInfos lDI) {
  return abs(lDI.coordinates().x()) <= 30;  // TODO create parameter
}

FutureAction refrainLeavingField_D(FieldProperties fP, LidarDetailedInfos lDI) {
  log_a(StratLevel, "strategy.refrainLeavingField_D", "Choosed strategy : refrainLeavingField_D");
  float xDirection = 0;
  float yDirection = 0;

  if (lDI.coordinates().x() < -fP.fieldWidth() / 2 + criticalWallDistance) {
    xDirection = cos(lDI.orientation());
    yDirection = sin(lDI.orientation());
  } else if (fP.fieldWidth() / 2 - criticalWallDistance < lDI.coordinates().x()) {
    xDirection = -cos(lDI.orientation());
    yDirection = -sin(lDI.orientation());
  }
  if (lDI.coordinates().y() < -fP.fieldLength() / 2 + criticalWallDistance + criticalGoalDistance) {
    xDirection = -sin(lDI.orientation());
    yDirection = cos(lDI.orientation());
  } else if (fP.fieldLength() / 2 - criticalWallDistance - criticalGoalDistance < lDI.coordinates().y()) {
    xDirection = sin(lDI.orientation());
    yDirection = -cos(lDI.orientation());
  }
  return FutureAction(
      Vector2(
          xDirection,
          yDirection),
      speedmotors,
      lDI.orientation(),
      false,
      dribblerSpeedIfLeavingField);
}

FutureAction refrainLeavingField_B(FieldProperties fP, LidarBasicInfos lBI) {
  log_a(StratLevel, "strategy.refrainLeavingField_B", "Choosed strategy : refrainLeavingField_B");
  return FutureAction(
      Vector2(
          -lBI.x(),
          -lBI.y()),
      speedmotors,
      0,
      false,
      dribblerSpeedIfLeavingField);
}

FutureAction refrainEnterInMyGoal_C(FieldProperties fP, MyGoalPos mGP) {
  log_a(StratLevel, "strategy.refrainEnterInMyGoal_C", "Choosed strategy : refrainEnterInMyGoal_C");
  return FutureAction(
      Vector2(
          -mGP.x(),
          -mGP.y()),
      speedmotors,
      0,
      false,
      dribblerSpeedIfLeavingField);
}

FutureAction refrainEnterInEnemyGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  log_a(StratLevel, "strategy.refrainEnterInEnemyGoal_C", "Choosed strategy : refrainEnterInEnemyGoal_C");
  return FutureAction(
      Vector2(  // TODO on ne retourne pas en arriere ?
          -eGP.x(),
          -eGP.y()),
      speedmotors,
      0,
      false,
      dribblerSpeedIfLeavingField);
}

FutureAction goToBallChangingOrientation_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI) {
  return FutureAction(
      bP,
      speedmotors,
      bP.angle() + lDI.orientation(),  // STRANGE
      false,
      fP.maxDribblerSpeed());
}

FutureAction goToBall_C(FieldProperties fP, BallPos bP) {
  log_a(StratLevel, "strategy.goToBall_C", "Choosed strategy : goToBall_C");
  int adjustedspeedmotors = speedmotors;
  float coefficient = 1.0;
  if (bP.norm() < 50) {
    adjustedspeedmotors -= 30;
    coefficient = 0.4;
  }

  return FutureAction(
      Vector2(
          bP.x() * coefficient,
          bP.y() - fP.robotRadius() * 3),  // TODO create parameter
      speedmotors,
      0,
      false,
      fP.maxDribblerSpeed());
}

FutureAction goToBallAvoidingBall_C(FieldProperties fP, BallPos bP) {
  log_a(StratLevel, "strategy.goToBallAvoidingBall_C", "Choosed strategy : goToBallAvoidingBall_C");
  if (!ballAtLevel(fP, bP) && ballInCenter(fP, bP)) {
    return FutureAction(
        Vector2(10, -10),
        speedmotors,
        0,
        false,
        0);

  } else if (bP.x() <= 0) {
    return FutureAction(
        Vector2(2, -10),
        speedmotors,
        0,
        false,
        0);

  } else if (bP.x() > 0) {
    return FutureAction(
        Vector2(-2, -10),
        speedmotors,
        0,
        false,
        0);

  } else {
    log_a(CriticalLevel, "goToBallAvoidingBall_C", "ERROR STRANGE");
    return goToBall_C(fP, bP);
  }
}

FutureAction goToBallAvoidingBall_CD(FieldProperties fP, BallPos bP, LidarDetailedInfos lDI) {
  log_a(StratLevel, "strategy.goToBallAvoidingBall_CD", "Choosed strategy : goToBallAvoidingBall_CD");
  if (!ballAtLevel(fP, bP) && ballInCenter(fP, bP)) {
    if (bP.x() < 0) {
      if (lDI.coordinates().x() > (fP.fieldWidth() / 2) - 6 * fP.robotRadius()) {
        return FutureAction(
            Vector2(-10, -10),
            speedmotors,
            0,
            false,
            0);

      } else {
        return FutureAction(
            Vector2(10, -10),
            speedmotors,
            0,
            false,
            0);
      }

    } else {
      if (-(fP.fieldWidth() / 2) + 6 * fP.robotRadius() > lDI.coordinates().x()) {
        return FutureAction(
            Vector2(10, -10),
            speedmotors,
            0,
            false,
            0);

      } else {
        return FutureAction(
            Vector2(-10, -10),
            speedmotors,
            0,
            false,
            0);
      }
    }

  } else if (bP.x() < 0 && bP.x() > -40) {
    return FutureAction(
        Vector2(5, -10),
        speedmotors,
        0,
        false,
        0);

  } else if (bP.x() > 0 && bP.x() < 40) {
    return FutureAction(
        Vector2(-5, -10),
        speedmotors,
        0,
        false,
        0);

  } else {
    return FutureAction(
        Vector2(0, -10),
        speedmotors,
        0,
        false,
        0);
  }
}

FutureAction accelerateToGoal_C(FieldProperties fP, EnemyGoalPos eGP) {
  log_a(StratLevel, "strategy.accelerateToGoal_C", "Choosed strategy : accelerateToGoal_C");
  return FutureAction(
      eGP,
      speedmotors,
      0,
      false,
      fP.maxDribblerSpeed());
}

FutureAction accelerateToGoal_D(FieldProperties fP, LidarDetailedInfos lDI, LidarBasicInfos lBI) {
  log_a(StratLevel, "strategy.accelerateToGoal_D", "Choosed strategy : accelerateToGoal_D");

  /* TEST EVITEMENT OBSTACLES */
  std::vector<Vector2> obstacles = lBI.obstacles();
  Vector2 directionGoal = lDI.frontGoalCoordinates();
  Vector2 direction = directionGoal;
  for (const auto& obstacle : obstacles) {
    Radians angle = directionGoal.angle() - obstacle.angle();
    float distance = obstacle.norm();

    if (abs(angle) < 0.5 && distance <= directionGoal.norm() - 10) {  // Paramètre (0.5) à modifier en fonction de la distance
      SerialDebug.println("obstacle dans le chemin : " + String(distance / 10) + ", angle : " + String(angle));
      if (angle >= 0) {
        direction.rotate(1);  // Paramètre à modifier en fonction de la distance
      } else {
        direction.rotate(-1);  // Paramètre à modifier en fonction de la distance
      }
    }
  }
  /******* *******/

  return FutureAction(
      direction,
      speedmotors,
      -lDI.frontGoalCoordinates().angle() + lDI.orientation(),
      false,
      fP.maxDribblerSpeed());
}

FutureAction spinToWin_D(FieldProperties fP, LidarDetailedInfos lDI) {
  Vector2 theoricalEGP = globalToLocalCoordinates(lDI, enemyGoalPosTheorical(fP));
  if (theoricalEGP.angle() - lDI.orientation() < PI / 6) {
    return FutureAction(
        Vector2(0, 10),
        shootSpeed,
        theoricalEGP.angle() + lDI.orientation(),
        true,
        fP.maxDribblerSpeed());

  } else {
    return FutureAction(
        Vector2(0, 0),
        speedmotors,
        theoricalEGP.angle() + lDI.orientation(),
        false,
        fP.maxDribblerSpeed());
  }
}

FutureAction shoot_C(FieldProperties fP, EnemyGoalPos eGP) {  // TODO refactor
  log_a(StratLevel, "strategy.shoot_C", "Choosed strategy : shoot_C");
  return FutureAction(
      eGP,
      shootSpeed,
      0,
      closeEnoughToKick_C(fP, eGP),
      fP.maxDribblerSpeed());
}

FutureAction shoot_D(FieldProperties fP, LidarDetailedInfos lDI) {
  log_a(StratLevel, "strategy.shoot_D", "Choosed strategy : shoot_D");
  return FutureAction(
      lDI.frontGoalCoordinates(),
      shootSpeed,
      -lDI.frontGoalCoordinates().angle() + lDI.orientation(),
      true,
      fP.maxDribblerSpeed());
}

FutureAction slalomingBackwards_D(FieldProperties fP, LidarDetailedInfos lDI) {
  log_a(StratLevel, "strategy.slalomingBackwards_D", "Choosed strategy : slalomingBackwards_D");
  if (lDI.coordinates().y() < -50) {
    wasSlalomingBackwards = true;
    if (lDI.coordinates().x() < -10) {
      return FutureAction(
          Vector2(10, 0),
          speedmotors,
          0,
          false,
          0);
    } else if (10 < lDI.coordinates().x()) {
      return FutureAction(
          Vector2(-10, 0),
          speedmotors,
          0,
          false,
          0);
    } else {
      return FutureAction(
          Vector2(0, 10),
          speedmotors,
          0,
          false,
          0);
    }

  } else if (50 < lDI.coordinates().y()) {
    wasSlalomingBackwards = true;
    return FutureAction(
        Vector2(-20, -10),
        speedmotors,
        0,
        false,
        0);

  } else {
    if (lDI.coordinates().x() < -fP.fieldWidth() / 6) {
      wasSlalomingBackwards = true;
      return FutureAction(
          Vector2(20, -10),
          speedmotors,
          0,
          false,
          0);
    } else if (fP.fieldWidth() / 6 < lDI.coordinates().x()) {
      wasSlalomingBackwards = true;
      return FutureAction(
          Vector2(-20, -10),
          speedmotors,
          0,
          false,
          0);
    } else if (wasSlalomingBackwards) {
      return FutureAction(
          speedmotors,
          0,
          false,
          0);
    } else {
      wasSlalomingBackwards = true;
      return FutureAction(
          Vector2(20, -10),
          speedmotors,
          0,
          false,
          0);
    }
  }
}

FutureAction chooseStrategyDefender(
    FieldProperties fP,
    Optional<LidarDetailedInfos> oLDI,
    Optional<LidarBasicInfos> oLBI,
    Optional<BallPos> oBP,
    Optional<MyGoalPos> oMGP,
    Optional<EnemyGoalPos> oEGP) {

  if (oLDI.hasValue() && abs(oLDI.value().coordinates().y() + fP.distanceYGoalFromCenter() - criticalWallDistance - criticalGoalDistance) < 10) {
    if (oBP.hasValue()) {
      return FutureAction(
          Vector2(oBP.value().rotate(oLDI.value().orientation()).x(), 
                  oLDI.value().coordinates().y() - fP.distanceYGoalFromCenter() + criticalWallDistance + criticalGoalDistance),
          speedmotors,
          0,
          false,
          0);
    } else {
      return FutureAction(
          Vector2(oLDI.value().rearGoalCoordinates().x()/10, // .rotate(oLDI.value().orientation())
                  -oLDI.value().coordinates().y() - fP.distanceYGoalFromCenter() + criticalWallDistance + criticalGoalDistance),
          speedmotors,
          0,
          false,
          0);
    }
  } else {
    return FutureAction::stopRobot();
  }
}