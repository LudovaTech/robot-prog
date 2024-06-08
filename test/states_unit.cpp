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
      8);
  ASSERT_EQ(fP.fieldLength(), 1);
  ASSERT_EQ(fP.fieldWidth(), 2);
  ASSERT_EQ(fP.spaceBeforeLineSide(), 3);
  ASSERT_EQ(fP.goalWidth(), 4);
  ASSERT_EQ(fP.myGoalPos(), Vector2(5, 5));
  ASSERT_EQ(fP.enemyGoalPos(), Vector2(6, 6));
  ASSERT_EQ(fP.robotRadius(), 7);
  ASSERT_EQ(fP.ballRadius(), 8);
}

TEST(ReadingData, accessData) {
  ReadingData rD;
  ASSERT_EQ(rD.typeState(), 'x');
  ASSERT_EQ(rD.xReadingState(), "");
  ASSERT_EQ(rD.yReadingState(), "");
  ASSERT_EQ(rD.writingInXState(), true);
}

TEST(ReadingData, reinitWith) {
  ReadingData rD;
  rD.addToActiveReadingState('a');
  rD.addToActiveReadingState('b');
  rD.reinitWith('e');
  ASSERT_EQ(rD.typeState(), 'e');
  ASSERT_EQ(rD.xReadingState(), "");
  ASSERT_EQ(rD.yReadingState(), "");
  ASSERT_EQ(rD.writingInXState(), true);
}

TEST(ReadingData, toString) {
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
}

TEST(ReadingData, writinginXY) {
  ReadingData rD;
  rD.addToActiveReadingState('a');
  rD.addToActiveReadingState('b');
  ASSERT_EQ(rD.xReadingState(), "ab");
  rD.nowWriteInYState();
  rD.addToActiveReadingState('c');
  rD.addToActiveReadingState('d');
  ASSERT_EQ(rD.xReadingState(), "ab");
  ASSERT_EQ(rD.yReadingState(), "cd");
}

TEST(CamInfos, accessData) {
  CamInfos cS = CamInfos(
      Vector2(1, 1),
      Vector2(2, 2),
      Vector2(3, 3),
      Vector2(4, 4),
      Vector2(5, 5));
  ASSERT_EQ(cS.ballPos(), Vector2(1, 1));
  ASSERT_EQ(cS.myPos(), Vector2(2, 2));
  ASSERT_EQ(cS.partnerPos(), Vector2(3, 3));
  ASSERT_EQ(cS.myGoalPos(), Vector2(4, 4));
  ASSERT_EQ(cS.enemyGoalPos(), Vector2(5, 5));
}

TEST(CamInfos, toString) {
  CamInfos cS = CamInfos(
      Vector2(1, 1.5),
      Vector2(2, 2.5),
      Vector2(3, 3.5),
      Vector2(4, 4.5),
      Vector2(5, 5.5));
  std::string str = cS.toString();
  size_t ballPosX = str.find(String(cS.ballPos().x()));
  ASSERT_TRUE(ballPosX != std::string::npos);
  size_t myPosX = str.find(String(cS.myPos().x()));
  ASSERT_TRUE(myPosX != std::string::npos);
  size_t partnerPosX = str.find(String(cS.partnerPos().x()));
  ASSERT_TRUE(partnerPosX != std::string::npos);
  size_t myGoalPosX = str.find(String(cS.myGoalPos().x()));
  ASSERT_TRUE(myGoalPosX != std::string::npos);
  size_t enemyGoalPosX = str.find(String(cS.enemyGoalPos().x()));
  ASSERT_TRUE(enemyGoalPosX != std::string::npos);
  size_t ballPosY = str.find(String(cS.ballPos().y()));
  ASSERT_TRUE(ballPosY != std::string::npos);
  size_t myPosY = str.find(String(cS.myPos().y()));
  ASSERT_TRUE(myPosY != std::string::npos);
  size_t partnerPosY = str.find(String(cS.partnerPos().y()));
  ASSERT_TRUE(partnerPosY != std::string::npos);
  size_t myGoalPosY = str.find(String(cS.myGoalPos().y()));
  ASSERT_TRUE(myGoalPosY != std::string::npos);
  size_t enemyGoalPosY = str.find(String(cS.enemyGoalPos().y()));
  ASSERT_TRUE(enemyGoalPosY != std::string::npos);
}