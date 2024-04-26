#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H

#include <cmath>
#include <cctype>
#include <queue>
#include <string>
#include <cstdint>
#include <stdexcept>

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1

#define sqrt std::sqrt
#define isDigit std::isdigit
#define max std::max

double sq(double x);

std::string _removeZeros(std::string str);
class String : public std::string {
 public:
  String();
  String(const char* s);
  String(const std::string& s);
  String(double s);

  float toFloat();
};

class SerialClass {
 private:
  std::queue<char> _incomingData;
  std::queue<char> _outgoingData;

 public:
  SerialClass();
  void begin(int baud);
  char read();
  void write(char data);
  bool available();
  void println(const std::string& str);
  char debugRead();
  void debugWrite(char data);
  bool debugAvailable();
  void debugPrintln(const std::string& str);
};

extern SerialClass Serial;
extern SerialClass Serial1;
extern SerialClass Serial2;
extern SerialClass Serial3;
extern SerialClass Serial4;
extern SerialClass Serial5;
extern SerialClass Serial6;
extern SerialClass Serial7;
extern SerialClass Serial8;

enum class PinState { pINPUT, pOUPUT, pUNDEF};

class PinsClass {
  private:
    std::vector<PinState> pinsMode;
    std::vector<int> pinsValue;
  public:
    PinsClass(int numPins);
    void pinMode(int pin, PinState pinState);
    bool inRange(int pin);
    void assertInRange(int pin);
    PinState getPinState(int pin);
    void assertIsOfState(int pin, PinState wantedPinState);
    void analogWrite(int pin, int value);
    void digitalWrite(int pin, int value);
    int analogRead(int pin);
    int digitalRead(int pin);
    void debugWrite(int pin, int value);
    int debugRead(int pin);
};

extern PinsClass fakeArduinoPins;
void pinMode(int pin, int pinState);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int value);
int analogRead(int pin, int value);
int digitalRead(int pin);

#endif