#include <gtest/gtest.h>

#include "../src/states.h"

TEST(FieldProperties, accessData) {
  FieldProperties fP = FieldProperties(
    1,
    2,
    3,
    4,
    Vector2(5, 5),
    Vector2(6, 6),
    7,
    8
  );
  ASSERT_EQ(fP.fieldLength(), 1);
  ASSERT_EQ(fP.fieldWidth(), 2);
  ASSERT_EQ(fP.spaceBeforeLineSide(), 3);
  ASSERT_EQ(fP.goalWidth(), 4);
  ASSERT_EQ(fP.myGoalPos(), Vector2(5, 5));
  ASSERT_EQ(fP.enemyGoalPos(), Vector2(6, 6));
  ASSERT_EQ(fP.robotRadius(), 7);
  ASSERT_EQ(fP.ballRadius(), 8);
}

TEST(RobotState, accessData) {
  RobotState cS = RobotState(
    Vector2(1, 1),
    Vector2(2, 2),
    Vector2(3, 3),
    Vector2(4, 4),
    Vector2(5, 5)
  );
  ASSERT_EQ(cS.ballPos(), Vector2(1, 1));
  ASSERT_EQ(cS.myPos(), Vector2(2, 2));
  ASSERT_EQ(cS.partnerPos(), Vector2(3, 3));
  ASSERT_EQ(cS.myGoalPos(), Vector2(4, 4));
  ASSERT_EQ(cS.enemyGoalPos(), Vector2(5, 5));
}

/*
TEST(ReadingData, accessData) {
  ReadingData rD;
  ASSERT_EQ(rD.typeState(), 'x');
  ASSERT_EQ(rD.xReadingState(), "");
  ASSERT_EQ(rD.yReadingState(), "");
  ASSERT_EQ(rD.writingInXState(), true);
}

TEST(ReadingData, verifyAction) {
  ReadingData rD;
  char car = 'a';
  rD.reinitWith(car);
  ASSERT_EQ(rD.typeState(), car);
  ASSERT_EQ(rD.xReadingState(), "");
  ASSERT_EQ(rD.yReadingState(), "");
  ASSERT_EQ(rD.writingInXState(), true);
}

TEST(ReadingData, verifyAction) {
  ReadingData rD;

  std::string str = rD.toString();

  size_t typeState = str.find(rD.typeState());
  ASSERT_TRUE(typeState != std::string::npos);

  size_t xReadingState = str.find(rD.xReadingState());
  ASSERT_TRUE(xReadingState != std::string::npos);

  size_t yReadingState = str.find(rD.yReadingState());
  ASSERT_TRUE(yReadingState != std::string::npos);

  size_t writingInXState = str.find(rD.writingInXState());
  ASSERT_TRUE(writingInXState != std::string::npos);
}*/
