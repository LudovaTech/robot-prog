#include <gtest/gtest.h>

#include "../src/utilities.hpp"

TEST(Vector2, equalZeroXY) {
  ASSERT_EQ(Vector2(0, 0), Vector2(0, 0));
}

TEST(Vector2, equalOneX) {
  ASSERT_EQ(Vector2(1, 0), Vector2(1, 0));
}

TEST(Vector2, equalMinusOneX) {
  ASSERT_EQ(Vector2(-1, 0), Vector2(-1, 0));
}

TEST(Vector2, equalNumberX) {
  ASSERT_EQ(Vector2(36, 0), Vector2(36, 0));
}

TEST(Vector2, equalMinusNumberX) {
  ASSERT_EQ(Vector2(-36, 0), Vector2(-36, 0));
}

TEST(Vector2, equalOneY) {
  ASSERT_EQ(Vector2(0, 1), Vector2(0, 1));
}

TEST(Vector2, equalMinusOneY) {
  ASSERT_EQ(Vector2(0, -1), Vector2(0, -1));
}

TEST(Vector2, equalNumberY) {
  ASSERT_EQ(Vector2(0, 27), Vector2(0, 27));
}

TEST(Vector2, equalMinusNumberY) {
  ASSERT_EQ(Vector2(0, -27), Vector2(0, -27));
}

TEST(Vector2, equalNumberXY) {
  ASSERT_EQ(Vector2(42, -27), Vector2(42, -27));
}

TEST(Vector2, equalZNumber) {
  ASSERT_EQ(Vector2(3.6, 4.56), Vector2(3.6, 4.56));
}

TEST(Vector2, equalZMinusNumber) {
  ASSERT_EQ(Vector2(-3.6, 4.56), Vector2(-3.6, 4.56));
}

TEST(Vector2, nonEqualOrder) {
  ASSERT_NE(Vector2(5, 4), Vector2(4, 5));
}

TEST(Vector2, nonEqualMinus) {
  ASSERT_NE(Vector2(-5, 4), Vector2(5, 4));
}

TEST(Vector2, nonEqual) {
  ASSERT_NE(Vector2(-5.7, 4.94), Vector2(5.78, 20.46));
}

TEST(Vector2, toStringContainsInfo) {
  std::string str = Vector2(1.5, -20).toString();

  size_t firstPos = str.find("1.5");
  ASSERT_TRUE(firstPos != std::string::npos);

  // Find the second value after the first one
  size_t secondPos = str.find("-20", firstPos + 1);
  ASSERT_TRUE(secondPos != std::string::npos);
}

TEST(Vector2, distanceRef) {
  ASSERT_EQ(Vector2(34, -5.4).distanceRef(Vector2(10, 42)), Vector2(-24, 47.4));
}

TEST(Vector2, distanceRefZero) {
  ASSERT_EQ(Vector2(34, 5).distanceRef(Vector2(34, 5)), Vector2(0, 0));
}

TEST(Vector2, distance) {
  ASSERT_NEAR(Vector2(34, 5).distance(Vector2(10, 42)), 44.10215414239989252818, 1e-6);
}

TEST(Vector2, distanceZero) {
  ASSERT_EQ(Vector2(34, 5).distance(Vector2(34, 5)), 0);
}

TEST(Vector2, norm) {
  ASSERT_NEAR(Vector2(34, 5).norm(), 34.36568055487916566920, 1e-6);
}

TEST(Vector2, normZero) {
  ASSERT_EQ(Vector2(0, 0).norm(), 0);
}

TEST(Vector2, angle) {
  ASSERT_FLOAT_EQ(Vector2(10, 10).angle(), -PI/4);
}

TEST(Vector2, rotate) {
  ASSERT_NEAR(Vector2(1, 0).rotate(-PI/2).x(), 0, 1e-6);
  ASSERT_NEAR(Vector2(1, 0).rotate(-PI/2).y(), -1, 1e-6);
}

TEST(MutableVector2, toStringContainsInfo) {
  std::string str = MutableVector2(Vector2(1.5, -20)).toString();

  size_t firstPos = str.find("1.5");
  ASSERT_TRUE(firstPos != std::string::npos);

  // Find the second value after the first one
  size_t secondPos = str.find("-20", firstPos + 1);
  ASSERT_TRUE(secondPos != std::string::npos);
}

TEST(MutableVector2, toVector2Zero) {
  ASSERT_EQ(MutableVector2(Vector2(0, 0)).toVector2(), Vector2(0, 0));
}

TEST(MutableVector2, toVector2) {
  ASSERT_EQ(MutableVector2(Vector2(-24, 7.9)).toVector2(), Vector2(-24, 7.9));
}

TEST(MutableVector2, nonToVector2) {
  ASSERT_NE(MutableVector2(Vector2(-23, 9)).toVector2(), Vector2(-24, 7.9));
}

TEST(DegreeRadians, degreeFromFloat) {
  ASSERT_NEAR(float(Degree(3.97)), 3.97, 1e-7);
}

TEST(DegreeRadians, degreeFromFloatZero) {
  ASSERT_EQ(Degree(0), 0);
}

TEST(DegreeRadians, degreeFromFloatBig) {
  ASSERT_EQ(Degree(400), 400);
}

TEST(DegreeRadians, radiansFromFloat) {
  ASSERT_NEAR(float(Radians(0.87)), 0.87, 1e-7);
}

TEST(DegreeRadians, radiansFromFloatZero) {
  ASSERT_EQ(Radians(0), 0);
}

TEST(DegreeRadians, radiansFromFloatBig) {
  ASSERT_EQ(Radians(10), 10);
}

TEST(DegreeRadians, degreeFromRadians) {
  ASSERT_NEAR(Degree(Radians(0.87)), 49.847328176381616, 1e-6);
}

TEST(DegreeRadians, degreeFromRadiansZero) {
  ASSERT_EQ(Degree(Radians(0)), 0);
}

TEST(DegreeRadians, degreeFromRadiansBig) {
  ASSERT_NEAR(Degree(Radians(10)), 572.95779513082320877, 1e-4);
}

TEST(DegreeRadians, degreeFromRadiansRound) {
  ASSERT_EQ(Degree(Radians(3.14159265358979323)), 180);
}

TEST(DegreeRadians, radiansFromDegree) {
  ASSERT_NEAR(Radians(Degree(56.3)), 0.98262036887280755181, 1e-5);
}

TEST(DegreeRadians, radiansFromDegreeZero) {
  ASSERT_EQ(Radians(Degree(0)), 0);
}

TEST(DegreeRadians, radiansFromDegreeBig) {
  ASSERT_NEAR(Radians(Degree(400)), 6.9813170079773183077, 1e-5);
}

TEST(DegreeRadians, radiansFromDegreeRound) {
  ASSERT_NEAR(Radians(Degree(180)), 3.14159265358979323, 1e-5);
}

// TODO tests with functions

TEST(ResultOrErrorOptional, ResultOrErrorValue) {
  ResultOrError<int> r(5);
  ASSERT_FALSE(r.hasError());
  ASSERT_EQ(r.value(), 5);
}

TEST(ResultOrErrorOptional, ResultOrErrorError) {
  ResultOrError<int> r("error");
  ASSERT_TRUE(r.hasError());
  ASSERT_EQ(r.errorMessage(), "error");
}

TEST(ResultOrErrorOptional, OptionalValue) {
  Optional<int> r(5);
  ASSERT_TRUE(r.hasValue());
  ASSERT_EQ(r.value(), 5);
}

TEST(ResultOrErrorOptional, OptionalNoValue) {
  Optional<int> r;
  ASSERT_FALSE(r.hasValue());
}