#include <gtest/gtest.h>

#include "../src/strategy.h"

TEST(FutureAction, accessToData) {
  FutureAction fA = FutureAction(Vector2(50, 13), true);
  ASSERT_EQ(fA.changeMove(), true);
  ASSERT_EQ(fA.target(), Vector2(50, 13));
  ASSERT_EQ(fA.activeKicker(), true);
}

TEST(FutureAction, accessToInfos) {
  FutureAction fA = FutureAction(true);
  ASSERT_EQ(fA.changeMove(), false);
  ASSERT_EQ(fA.target(), Vector2(0, 0));
  ASSERT_EQ(fA.activeKicker(), true);
}

TEST(FutureAction, leavingField) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, -115),  // myGoalPos
      Vector2(0, 115),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS0 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS0));

  RobotState cS1 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-83, 0),    // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS1));

  RobotState cS2 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-81, 0),    // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS2));

  RobotState cS3 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(83, 0),     // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS3));

  RobotState cS4 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 0),     // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS4));

  RobotState cS5 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 113),   // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS5));

  RobotState cS6 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, 111),   // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS6));

  RobotState cS7 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, -113),  // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS7));

  RobotState cS8 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(81, -111),  // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(500, 500)   // enemyGoalPos
  );

  ASSERT_FALSE(leavingField(fP, cS8));

  RobotState cS9 = RobotState(
      Vector2(0, 0),     // ballPos
      Vector2(0, 0),     // myPos
      Vector2(0, 0),     // partnerPos
      Vector2(0, 0),     // myGoalPos
      Vector2(500, 500)  // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS9));

  RobotState cS10 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(0, 0)       // enemyGoalPos
  );

  ASSERT_TRUE(leavingField(fP, cS10));
}

TEST(FutureAction, targetInFrontOfRobotFromFront) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetInFrontOfRobotFromFront(fP, cS, Vector2(0, 50)));
  ASSERT_FALSE(targetInFrontOfRobotFromFront(fP, cS, Vector2(0, 0)));
  ASSERT_FALSE(targetInFrontOfRobotFromFront(fP, cS, Vector2(0, -50)));
  ASSERT_TRUE(targetInFrontOfRobotFromFront(fP, cS, Vector2(0, 14)));
  ASSERT_FALSE(targetInFrontOfRobotFromFront(fP, cS, Vector2(0, 13)));
}

TEST(FutureAction, targetInFrontOfRobotFromMiddle) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetInFrontOfRobotFromMiddle(fP, cS, Vector2(0, 50)));
  ASSERT_FALSE(targetInFrontOfRobotFromMiddle(fP, cS, Vector2(0, 0)));
  ASSERT_FALSE(targetInFrontOfRobotFromMiddle(fP, cS, Vector2(0, -50)));
  ASSERT_TRUE(targetInFrontOfRobotFromMiddle(fP, cS, Vector2(0, 1)));
  ASSERT_FALSE(targetInFrontOfRobotFromMiddle(fP, cS, Vector2(0, -1)));
}

TEST(FutureAction, targetCenterOfRobot) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(0, 0)));
  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(-6, 0)));
  ASSERT_TRUE(targetCenterOfRobot(fP, cS, Vector2(6, 0)));
  ASSERT_FALSE(targetCenterOfRobot(fP, cS, Vector2(-7, 0)));
  ASSERT_FALSE(targetCenterOfRobot(fP, cS, Vector2(7, 0)));
}

TEST(FutureAction, targetJustInFrontOfRobot) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetJustInFrontOfRobot(fP, cS, Vector2(0, 1)));
  ASSERT_FALSE(targetJustInFrontOfRobot(fP, cS, Vector2(0, -1)));
  ASSERT_TRUE(targetJustInFrontOfRobot(fP, cS, Vector2(-6, 1)));
  ASSERT_FALSE(targetJustInFrontOfRobot(fP, cS, Vector2(-7, 1)));
  ASSERT_TRUE(targetJustInFrontOfRobot(fP, cS, Vector2(6, 1)));
  ASSERT_FALSE(targetJustInFrontOfRobot(fP, cS, Vector2(7, 1)));
  ASSERT_FALSE(targetJustInFrontOfRobot(fP, cS, Vector2(-6, -1)));
  ASSERT_FALSE(targetJustInFrontOfRobot(fP, cS, Vector2(6, -1)));
}

TEST(FutureAction, targetJustBehindOfRobot) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(0, 0),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(targetJustBehindOfRobot(fP, cS, Vector2(0, -1)));
  ASSERT_FALSE(targetJustBehindOfRobot(fP, cS, Vector2(0, 1)));
  ASSERT_TRUE(targetJustBehindOfRobot(fP, cS, Vector2(-6, -1)));
  ASSERT_FALSE(targetJustBehindOfRobot(fP, cS, Vector2(-7, -1)));
  ASSERT_TRUE(targetJustBehindOfRobot(fP, cS, Vector2(6, -1)));
  ASSERT_FALSE(targetJustBehindOfRobot(fP, cS, Vector2(7, -1)));
  ASSERT_FALSE(targetJustBehindOfRobot(fP, cS, Vector2(-6, 1)));
  ASSERT_FALSE(targetJustBehindOfRobot(fP, cS, Vector2(6, 1)));
}

TEST(FutureAction, ballIsCaught) {
  FieldProperties fP = FieldProperties(
      243,            // fieldLength
      182,            // fieldWidth
      12,             // spaceBeforeLineSide
      60,             // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),  // enemyGoalPos
      9,              // robotRadius
      2               // ballRadius
  );

  RobotState cS0 = RobotState(
      Vector2(0, 16),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_TRUE(ballIsCaught(fP, cS0));

  RobotState cS1 = RobotState(
      Vector2(0, 17),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_FALSE(ballIsCaught(fP, cS1));

  RobotState cS2 = RobotState(
      Vector2(-6, 16),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_TRUE(ballIsCaught(fP, cS2));

  RobotState cS3 = RobotState(
      Vector2(-7, 16),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_FALSE(ballIsCaught(fP, cS3));

  RobotState cS4 = RobotState(
      Vector2(6, 16),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_TRUE(ballIsCaught(fP, cS4));

  RobotState cS5 = RobotState(
      Vector2(7, 16),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_FALSE(ballIsCaught(fP, cS5));

  RobotState cS6 = RobotState(
      Vector2(-6, 17),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_FALSE(ballIsCaught(fP, cS6));

  RobotState cS7 = RobotState(
      Vector2(6, 17),  // ballPos
      Vector2(0, 0),  // myPos
      Vector2(0, 0),  // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );
  ASSERT_FALSE(ballIsCaught(fP, cS7));
}

TEST(FutureAction, refrainFromLeavingStrategy) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS0 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS0).target(), Vector2(0, 0));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS0).activeKicker());

  RobotState cS1 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-300, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS1).target(), Vector2(10, 0));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS1).activeKicker());

  RobotState cS2 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(300, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS2).target(), Vector2(-10, 0));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS2).activeKicker());

  RobotState cS3 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS3).target(), Vector2(0, -10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS3).activeKicker());

  RobotState cS4 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, -300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS4).target(), Vector2(0, 10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS4).activeKicker());

  RobotState cS5 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-300, 300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS5).target(), Vector2(10, -10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS5).activeKicker());

  RobotState cS6 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(300, 300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS6).target(), Vector2(-10, -10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS6).activeKicker());

  RobotState cS7 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(-300, -300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS7).target(), Vector2(10, 10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS7).activeKicker());

  RobotState cS8 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(300, -300),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS8).target(), Vector2(-10, 10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS8).activeKicker());

  RobotState cS9 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(0, -35),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS9).target(), Vector2(0, 0));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS9).activeKicker());

  RobotState cS10 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(0, -34),  // myGoalPos
      Vector2(400, 500)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS10).target(), Vector2(0, 10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS10).activeKicker());

  RobotState cS11 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(0, 35)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS11).target(), Vector2(0, 0));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS11).activeKicker());

  RobotState cS12 = RobotState(
      Vector2(0, 0),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(500, 500),  // myGoalPos
      Vector2(0, 34)   // enemyGoalPos
  );
  
  ASSERT_EQ(refrainFromLeavingStrategy(fP, cS12).target(), Vector2(0, -10));
  ASSERT_FALSE(refrainFromLeavingStrategy(fP, cS12).activeKicker());

  
}

TEST(FutureAction, goToBallStrategy) {
  FieldProperties fP = FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );

  RobotState cS = RobotState(
      Vector2(50, 50),      // ballPos
      Vector2(0, 0),      // myPos
      Vector2(0, 0),      // partnerPos
      Vector2(0, 0),  // myGoalPos
      Vector2(0, 0)   // enemyGoalPos
  );

  ASSERT_EQ(goToBallStrategy(fP, cS).target(), Vector2(50, 13.5));
  ASSERT_FALSE(goToBallStrategy(fP, cS).activeKicker());
}