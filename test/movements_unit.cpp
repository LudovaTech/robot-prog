#include <Arduino.h>
#include <gtest/gtest.h>

#include "../src/movements.hpp"


// class MotorMovTest : public ::testing::Test {
//  protected:
//   void SetUp() override {
//     // Réinitialiser l'état des pins avant chaque test
//     fakeArduinoPins = PinsClass(41);
//   }
// };

// TEST_F(MotorMovTest, ConstructorInitializesPinsCorrectly) {
//   MotorMov motor(5, 6, 7, 1.57);

//   EXPECT_EQ(fakeArduinoPins.getPinState(5), PinState::pOUTPUT);
//   EXPECT_EQ(fakeArduinoPins.getPinState(6), PinState::pOUTPUT);
//   EXPECT_EQ(fakeArduinoPins.getPinState(7), PinState::pINPUT);
// }

// TEST_F(MotorMovTest, StopSetsPwmToZero) {
//   MotorMov motor(5, 6, 7, 1.57);
//   motor.stop();
//   EXPECT_EQ(fakeArduinoPins.debugRead(5), 255);
// }

// TEST_F(MotorMovTest, MoveForwardSetsPwmAndDirectionCorrectly) {
//   MotorMov motor(5, 6, 7, 1.57);
//   motor.move(100);
//   EXPECT_EQ(fakeArduinoPins.debugRead(5), 155);
//   EXPECT_EQ(fakeArduinoPins.debugRead(6), HIGH);
// }

// TEST_F(MotorMovTest, MoveBackwardSetsPwmAndDirectionCorrectly) {
//   MotorMov motor(5, 6, 7, 1.57);
//   motor.move(-100);
//   EXPECT_EQ(fakeArduinoPins.debugRead(5), 155);
//   EXPECT_EQ(fakeArduinoPins.debugRead(6), LOW);
// }

// TEST_F(MotorMovTest, MoveToZeroStopsMotor) {
//   MotorMov motor(5, 6, 7, 1.57);
//   motor.move(100);
//   motor.move(0);
//   EXPECT_EQ(fakeArduinoPins.debugRead(5), 255);
// }

// // TODO
// /*
// TEST_F(MotorMovTest, AnglePowerAxisKickerCalculatesCorrectly) {
//     MotorMov motor(5, 6, 7, 1.57);
//     EXPECT_NEAR(motor.anglePowerAxisKicker(), 0.0, 1e-5);
// }*/

// class MotorsTest : public ::testing::Test {
//  protected:
//   MotorMov motor1 = MotorMov(1, 2, 3, 0);
//   MotorMov motor2 = MotorMov(4, 5, 6, PI / 2);
//   MotorMov motor3 = MotorMov(7, 8, 9, PI);
//   MotorMov motor4 = MotorMov(10, 11, 12, 3 * PI / 2);
//   Motors motors = Motors(motor1, motor2, motor3, motor4);
// };

// /*
// TEST_F(MotorsTest, ConstructorInitializesMotorsCorrectly) {
//     EXPECT_EQ(motors.frontRight().angleAxisKicker(), 0);
//     EXPECT_EQ(motors.frontLeft().angleAxisKicker(), PI / 2);
//     EXPECT_EQ(motors.backRight().angleAxisKicker(), PI);
//     EXPECT_EQ(motors.backLeft().angleAxisKicker(), 3 * PI / 2);
// }*/

// TEST_F(MotorsTest, FullStopStopsAllMotors) {
//   motors.fullStop();
//   EXPECT_EQ(fakeArduinoPins.debugRead(1), 255);
//   EXPECT_EQ(fakeArduinoPins.debugRead(4), 255);
//   EXPECT_EQ(fakeArduinoPins.debugRead(7), 255);
//   EXPECT_EQ(fakeArduinoPins.debugRead(10), 255);
// }

// /*
// TEST_F(MotorsTest, GoToSetsCorrectSpeeds) {
//   Vector2 distances(10, 10);
//   Radians orientation = 0;

//   motors.goTo(distances, 255, orientation);

//   // Vérifiez que les moteurs ont reçu les valeurs de vitesse attendues
//   // Exemple : ajustez les valeurs attendues en fonction de vos calculs
//   EXPECT_NEAR(fakeArduinoPins.debugRead(1), 255 - 255 * 0.6, 1);
//   EXPECT_NEAR(fakeArduinoPins.debugRead(4), 255 - 255 * 0.6, 1);
//   EXPECT_NEAR(fakeArduinoPins.debugRead(7), 255 - 255 * 0.6, 1);
//   EXPECT_NEAR(fakeArduinoPins.debugRead(10), 255 - 255 * 0.6, 1);
// }*/

// class DribblerKickerTest : public ::testing::Test {
//  protected:
//   MotorMov dribbler = MotorMov(13, 14, 15, 0);
//   DribblerKicker dribblerKicker = DribblerKicker(dribbler, 16, 17);
// };

// TEST_F(DribblerKickerTest, ConstructorInitializesPinsCorrectly) {
//   EXPECT_EQ(fakeArduinoPins.getPinState(16), PinState::pOUTPUT) << pinStateToString(fakeArduinoPins.getPinState(16));
//   EXPECT_EQ(fakeArduinoPins.getPinState(17), PinState::pOUTPUT) << pinStateToString(fakeArduinoPins.getPinState(17));
// }

// TEST_F(DribblerKickerTest, DribbleMovesMotorCorrectly) {
//   dribblerKicker.dribble(100);
//   EXPECT_EQ(fakeArduinoPins.debugRead(13), 155);
// }

// TEST_F(DribblerKickerTest, KickActivatesKickerPins) {
//   dribblerKicker.kick();
//   EXPECT_EQ(fakeArduinoPins.debugRead(16), HIGH);
//   EXPECT_EQ(fakeArduinoPins.debugRead(17), HIGH);
// }

// TEST_F(DribblerKickerTest, NoKickDeactivatesKickerPins) {
//   dribblerKicker.kick();
//   dribblerKicker.noKick();
//   EXPECT_EQ(fakeArduinoPins.debugRead(16), LOW);
//   EXPECT_EQ(fakeArduinoPins.debugRead(17), LOW);
// }*/