#include <gtest/gtest.h>

#include "./faking/Arduino.h"

TEST(fakeArduino, sq) {
  ASSERT_EQ(sq(0), 0);
  ASSERT_EQ(sq(2), 4);
  ASSERT_EQ(sq(-3), 9);
}

TEST(fakeArduino, StringFromstdString) {
  ASSERT_EQ(String("abc"), "abc");
}

TEST(fakeArduino, StringFromDouble) {
  ASSERT_EQ(String(32.5), "32.5");
}

TEST(fakeArduino, StringToFloatInt) {
  ASSERT_EQ(String("23").toFloat(), 23);
}

TEST(fakeArduino, StringToFloatFloat) {
  ASSERT_NEAR(String("23.57").toFloat(), 23.57, 1e-6);
}

TEST(fakeArduino, StringToFloatError) {
  ASSERT_THROW(String("abc").toFloat(), std::invalid_argument);
}

TEST(fakeArduino, StringToFloatErrorSim) {
  ASSERT_EQ(String("15abc").toFloat(), 15);
}

TEST(fakeArduino, removeZerosIntNoZero) {
  ASSERT_EQ(_removeZeros("14"), "14");
}

TEST(fakeArduino, removeZerosFloatNoZero) {
  ASSERT_EQ(_removeZeros("14.67"), "14.67");
}

TEST(fakeArduino, removeZerosIntEndingWithZero) {
  ASSERT_EQ(_removeZeros("140"), "140");
}

TEST(fakeArduino, removeZerosFloatEndingWithZero) {
  ASSERT_EQ(_removeZeros("14.6700"), "14.67");
}

TEST(fakeArduino, removeZerosFloatWhichIsInt) {
  ASSERT_EQ(_removeZeros("14.0"), "14.0");
}

TEST(fakeArduino, removeZerosFloatWhichIsIntMultipleZero) {
  ASSERT_EQ(_removeZeros("14.000"), "14.0");
}

TEST(fakeArduino, removeZerosIntZero) {
  ASSERT_EQ(_removeZeros("0"), "0");
}

TEST(fakeArduino, removeZerosFloatZero) {
  ASSERT_EQ(_removeZeros("0.000"), "0.0");
}

// Test suite for char search
TEST(fakeArduino, LastIndexOfChar) {
    String str("hello world");

    EXPECT_EQ(str.lastIndexOf('o'), 7);
    EXPECT_EQ(str.lastIndexOf('o', 6), 4);
    EXPECT_EQ(str.lastIndexOf('l'), 9);
    EXPECT_EQ(str.lastIndexOf('l', 7), 3);
    EXPECT_EQ(str.lastIndexOf('z'), -1);
    EXPECT_EQ(str.lastIndexOf('h'), 0);
}

// Test suite for string search
TEST(fakeArduino, LastIndexOfString) {
    String str("hello world");

    EXPECT_EQ(str.lastIndexOf("world"), 6);
    EXPECT_EQ(str.lastIndexOf("hello"), 0);
    EXPECT_EQ(str.lastIndexOf("l"), 9);
    EXPECT_EQ(str.lastIndexOf("l", 8), 3);
    EXPECT_EQ(str.lastIndexOf("z"), -1);
    EXPECT_EQ(str.lastIndexOf("h"), 0);
    EXPECT_EQ(str.lastIndexOf("world", 5), -1);
    EXPECT_EQ(str.lastIndexOf(""), -1);  // edge case for empty string
}

TEST(fakeArduino, LastIndexOfStringRealCase) {
    EXPECT_EQ(String("abcde").lastIndexOf("b"), 1);
    EXPECT_EQ(String("abcde").lastIndexOf("e"), 4);
    EXPECT_EQ(String("abcde fghbijklmne").lastIndexOf("e"), 16);
    EXPECT_EQ(String("abcde fghbijklmne").lastIndexOf("b"), 9);
    EXPECT_EQ(String("acde").lastIndexOf("b"), -1);
    EXPECT_EQ(String("acde").lastIndexOf("e"), 3);
    EXPECT_EQ(String("abcdeb").lastIndexOf("e"), 4);
    EXPECT_EQ(String("abcdeb").lastIndexOf("b"), 5);
    EXPECT_EQ(String("b...e").lastIndexOf("b"), 0);
    EXPECT_EQ(String("b...e").lastIndexOf("e"), 4);
}

TEST(fakeArduino, LastIndexOfStringRealCaseWithIndex) {
    EXPECT_EQ(String("abcde").lastIndexOf("b", 0), -1);
    EXPECT_EQ(String("abcde").lastIndexOf("b", 1), 1);
    EXPECT_EQ(String("abcde").lastIndexOf("b", 4), 1);
    EXPECT_EQ(String("abcde").lastIndexOf("e"), 4);
    EXPECT_EQ(String("abcde fghbijklmne").lastIndexOf("e", 8), 4);
    EXPECT_EQ(String("abcde fghbijklmne").lastIndexOf("b", 8), 1);
    EXPECT_EQ(String("abcdeb").lastIndexOf("e"), 4);
    EXPECT_EQ(String("abcdeb").lastIndexOf("b", 4), 1);
}

TEST(fakeArduino, serialClassReadOnly) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  ASSERT_EQ(serial.read(), 't');
}

TEST(fakeArduino, serialClassMultipleReadOnly) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  serial.debugWrite('e');
  serial.debugWrite('s');
  serial.debugWrite('t');
  ASSERT_EQ(serial.read(), 't');
  ASSERT_EQ(serial.read(), 'e');
  ASSERT_EQ(serial.read(), 's');
  ASSERT_EQ(serial.read(), 't');
}

TEST(fakeArduino, serialClassWriteOnly) {
  SerialClass serial = SerialClass();
  serial.write('t');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassMultipleWriteOnly) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.write('e');
  serial.write('s');
  serial.write('t');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassReadWrite) {
  SerialClass serial = SerialClass();
  serial.debugWrite('d');
  serial.write('r');
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 'r');
}

TEST(fakeArduino, serialClassMultipleReadWrite) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.debugWrite('d');
  serial.write('e');
  serial.write('s');
  serial.debugWrite('f');
  serial.write('t');
  serial.debugWrite('g');
  serial.debugWrite('h');
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.read(), 'f');
  ASSERT_EQ(serial.read(), 'g');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.read(), 'h');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassPrint) {
  SerialClass serial = SerialClass();
  serial.println("tes");
  serial.debugPrintln("df");
  serial.write('t');
  serial.debugPrint("gh");
  ASSERT_EQ(serial.read(), 'd');
  ASSERT_EQ(serial.debugRead(), 't');
  ASSERT_EQ(serial.debugRead(), 'e');
  ASSERT_EQ(serial.read(), 'f');
  ASSERT_EQ(serial.read(), '\n');
  ASSERT_EQ(serial.read(), 'g');
  ASSERT_EQ(serial.debugRead(), 's');
  ASSERT_EQ(serial.read(), 'h');
  ASSERT_NE(serial.read(), '\n');
  ASSERT_EQ(serial.debugRead(), '\n');
  ASSERT_EQ(serial.debugRead(), 't');
}

TEST(fakeArduino, serialClassAvailable) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  ASSERT_TRUE(serial.available());
}

TEST(fakeArduino, serialClassNotAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_FALSE(serial.available());
}

TEST(fakeArduino, serialClassAvailableAfterReading) {
  SerialClass serial = SerialClass();
  serial.debugWrite('t');
  serial.read();
  ASSERT_FALSE(serial.available());
}

TEST(fakeArduino, serialClassDebugAvailable) {
  SerialClass serial = SerialClass();
  serial.write('t');
  ASSERT_TRUE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassNotDebugAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_FALSE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassDebugAvailableAfterReading) {
  SerialClass serial = SerialClass();
  serial.write('t');
  serial.debugRead();
  ASSERT_FALSE(serial.debugAvailable());
}

TEST(fakeArduino, serialClassReadWhenNotAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_EQ(serial.read(), -1);
}

TEST(fakeArduino, serialClassReadWhenNotDebugAvailable) {
  SerialClass serial = SerialClass();
  ASSERT_EQ(serial.debugRead(), -1);
}

TEST(fakeArduino, PinsClassInRange) {
  PinsClass debugPins(10);
  ASSERT_TRUE(debugPins.inRange(0));
  ASSERT_TRUE(debugPins.inRange(3));
  ASSERT_TRUE(debugPins.inRange(9));
}

TEST(fakeArduino, PinsClassNotInRange) {
  PinsClass debugPins(10);
  ASSERT_FALSE(debugPins.inRange(-1));
  ASSERT_FALSE(debugPins.inRange(10));
  ASSERT_FALSE(debugPins.inRange(25));
}

TEST(fakeArduino, PinsClassAssertInRange) {
  PinsClass debugPins(10);
  EXPECT_NO_THROW(debugPins.assertInRange(7));
  EXPECT_THROW(debugPins.assertInRange(-1), std::invalid_argument);
  EXPECT_THROW(debugPins.assertInRange(10), std::invalid_argument);
}

TEST(fakeArduino, PinsClassGetPinState) {
  PinsClass debugPins(10);
  debugPins.pinMode(1, PinState::pINPUT);
  debugPins.pinMode(3, PinState::pOUTPUT);
  ASSERT_EQ(debugPins.getPinState(1), PinState::pINPUT);
  ASSERT_EQ(debugPins.getPinState(3), PinState::pOUTPUT);
  ASSERT_EQ(debugPins.getPinState(5), PinState::pUNDEF);
}

TEST(fakeArduino, PinsClassAssertIsOfType) {
  PinsClass debugPins(10);
  debugPins.pinMode(1, PinState::pINPUT);
  debugPins.pinMode(3, PinState::pOUTPUT);
  EXPECT_NO_THROW(debugPins.assertIsOfState(1, PinState::pINPUT));
  EXPECT_NO_THROW(debugPins.assertIsOfState(2, PinState::pUNDEF));
  EXPECT_NO_THROW(debugPins.assertIsOfState(3, PinState::pOUTPUT));
  EXPECT_THROW(debugPins.assertIsOfState(1, PinState::pOUTPUT), std::invalid_argument);
  EXPECT_THROW(debugPins.assertIsOfState(2, PinState::pINPUT), std::invalid_argument);
  EXPECT_THROW(debugPins.assertIsOfState(3, PinState::pUNDEF), std::invalid_argument);
  EXPECT_THROW(debugPins.assertIsOfState(3, PinState::pINPUT), std::invalid_argument);
  EXPECT_THROW(debugPins.assertIsOfState(-1, PinState::pINPUT), std::invalid_argument);
}

TEST(fakeArduino, PinsClassTnvalidDebugReadWrite) {
  PinsClass debugPins(10);
  ASSERT_THROW(debugPins.debugRead(-1), std::invalid_argument);
  ASSERT_THROW(debugPins.debugWrite(-1, 266), std::invalid_argument);
  ASSERT_THROW(debugPins.debugRead(10), std::invalid_argument);
  ASSERT_THROW(debugPins.debugWrite(10, 266), std::invalid_argument);
}

TEST(fakeArduino, PinsClassAnalogReadWrite) {
  PinsClass debugPins(10);
  debugPins.pinMode(1, PinState::pINPUT);
  debugPins.pinMode(2, PinState::pOUTPUT);
  debugPins.debugWrite(1, 255);
  debugPins.analogWrite(2, 155);
  ASSERT_EQ(debugPins.analogRead(1), 255);
  ASSERT_EQ(debugPins.debugRead(2), 155);
}

TEST(fakeArduino, PinsClassDigitalReadWrite) {
  PinsClass debugPins(10);
  debugPins.pinMode(1, PinState::pINPUT);
  debugPins.pinMode(2, PinState::pOUTPUT);
  debugPins.debugWrite(1, HIGH);
  ASSERT_EQ(debugPins.analogRead(1), HIGH);
  debugPins.analogWrite(2, HIGH);
  ASSERT_EQ(debugPins.debugRead(2), HIGH);
  debugPins.debugWrite(1, LOW);
  ASSERT_EQ(debugPins.analogRead(1), LOW);
}

TEST(fakeArduino, delay) {
  unsigned long start_millis = millis();
  delay(300);
  unsigned long elapsed = millis() - start_millis;
  ASSERT_TRUE((300 <= elapsed) && (elapsed <= 320));
}