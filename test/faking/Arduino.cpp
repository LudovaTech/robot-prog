#include "Arduino.h"

SerialClass Serial;
SerialClass Serial1;
SerialClass Serial2;
SerialClass Serial3;
SerialClass Serial4;
SerialClass Serial5;
SerialClass Serial6;
SerialClass Serial7;
SerialClass Serial8;

double sq(double x) {
  return x * x;
}

void delay(int time) {
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

std::string _removeZeros(std::string str) {
  size_t dotPos = str.find_last_of('.');
  if (dotPos != std::string::npos) {
    size_t nonZeroPos = str.find_last_not_of('0');
    if (nonZeroPos != std::string::npos && nonZeroPos > dotPos) {
      return str.substr(0, nonZeroPos + 1);
    } else if (nonZeroPos != std::string::npos && nonZeroPos == dotPos) {
      return str.substr(0, nonZeroPos + 2);
    }
  }
  return str;
}

String::String() : std::string() {}
String::String(const char* s) : std::string(s) {}
String::String(const std::string& s) : std::string(s) {}
String::String(double s) : std::string(_removeZeros(std::to_string(s))) {}
String::String(uint16_t s) : std::string(_removeZeros(std::to_string(s))) {}
String::String(int s) : std::string(_removeZeros(std::to_string(s))) {}
String::String(unsigned int s) : std::string(_removeZeros(std::to_string(s))) {}
String::String(unsigned long s) : std::string(_removeZeros(std::to_string(s))) {}

String pinStateToString(PinState pinState) {
  switch (pinState) {
    case PinState::pINPUT:
      return "pINPUT";
    case PinState::pOUTPUT:
      return "pOUTPUT";
    case PinState::pUNDEF:
      return "pUNDEF";
    default:
      return "strange type";
  }
}

float String::toFloat() {
  return std::stof(*this);
}

String String::substring(size_t from, size_t to) {
  if (from >= size()) {
    return "";
  }
  if (to == std::string::npos || to > size()) {
    to = size();
  }
  if (to <= from) {
    return "";
  }
  return substr(from, to - from);
}

SerialClass::SerialClass() {}

void SerialClass::begin(int baud) {}

char SerialClass::read() {
  if (!_incomingData.empty()) {
    char data = _incomingData.front();
    _incomingData.pop();
    return data;
  } else {
    return -1;
  }
}

void SerialClass::write(char data) {
  _outgoingData.push(data);
}

bool SerialClass::available() {
  return !_incomingData.empty();
}

bool SerialClass::debugAvailable() {
  return !_outgoingData.empty();
}

char SerialClass::debugRead() {
  if (!_outgoingData.empty()) {
    char data = _outgoingData.front();
    _outgoingData.pop();
    return data;
  } else {
    return -1;
  }
}

void SerialClass::debugWrite(char data) {
  _incomingData.push(data);
}

void SerialClass::print(const String& str) {
  for (char c : str) {
    write(c);
  }
}

void SerialClass::println(const String& str) {
  print(str);
  write('\n');
}

void SerialClass::debugPrint(const String& str) {
  for (char c : str) {
    debugWrite(c);
  }
}

void SerialClass::debugPrintln(const String& str) {
  debugPrint(str);
  debugWrite('\n');
}

bool SerialClass::find(String str) {
  // unsupported yet
  throw std::exception();
}

byte SerialClass::readBytes(byte* buffer, int length) {
  // unsupported yet
  throw std::exception();
}

void SerialClass::setTimeout(int time) {
  // unsupported yet, ignore
}

PinsClass::PinsClass(int numPins) : pinsMode(numPins, PinState::pUNDEF), pinsValue(numPins, 0) {}

bool PinsClass::inRange(int pin) {
  return pin >= 0 && pin < pinsMode.size();
}

void PinsClass::assertInRange(int pin) {
  if (!inRange(pin)) {
    throw std::invalid_argument("pin number out of range");
  }
}

PinState PinsClass::getPinState(int pin) {
  assertInRange(pin);
  return pinsMode[pin];
}

void PinsClass::assertIsOfState(int pin, PinState wantedPinState) {
  assertInRange(pin);
  if (getPinState(pin) != wantedPinState) {
    throw std::invalid_argument(
        "pin wrong state, is of type " +
        pinStateToString(getPinState(pin)) +
        " although wanted type is " +
        pinStateToString(wantedPinState));
  }
}

void PinsClass::pinMode(int pin, PinState pinState) {
  if (pinState == PinState::pUNDEF) {
    throw std::invalid_argument("cannot set a pin to UNDEF");
  }
  assertInRange(pin);
  assertIsOfState(pin, PinState::pUNDEF);
  pinsMode[pin] = pinState;
}

void PinsClass::analogWrite(int pin, int value) {
  assertInRange(pin);
  assertIsOfState(pin, PinState::pOUTPUT);
  pinsValue[pin] = value;
}

void PinsClass::digitalWrite(int pin, int value) {
  if (value == HIGH || value == LOW) {
    (*this).analogWrite(pin, value);
  } else {
    throw std::invalid_argument("wrong value for digitalWrite");
  }
}

int PinsClass::analogRead(int pin) {
  assertInRange(pin);
  assertIsOfState(pin, PinState::pINPUT);
  return pinsValue[pin];
}

int PinsClass::digitalRead(int pin) {
  int value = (*this).analogRead(pin);
  if (value == HIGH || value == LOW) {
    return value;
  } else {
    throw std::invalid_argument("non-digital value on this pin");
  }
}

void PinsClass::debugWrite(int pin, int value) {
  assertInRange(pin);
  assertIsOfState(pin, PinState::pINPUT);
  pinsValue[pin] = value;
}

int PinsClass::debugRead(int pin) {
  assertInRange(pin);
  assertIsOfState(pin, PinState::pOUTPUT);
  return pinsValue[pin];
}

PinsClass fakeArduinoPins(41);
void pinMode(int pin, int pinState) {
  if (pinState == INPUT) {
    fakeArduinoPins.pinMode(pin, PinState::pINPUT);
  } else if (pinState == OUTPUT) {
    fakeArduinoPins.pinMode(pin, PinState::pOUTPUT);
  } else {
    throw std::invalid_argument("wrong pinState");
  }
}
void analogWrite(int pin, int value) { fakeArduinoPins.analogWrite(pin, value); }
void digitalWrite(int pin, int value) { fakeArduinoPins.digitalWrite(pin, value); }
int analogRead(int pin, int value) { return fakeArduinoPins.analogRead(pin); }
int digitalRead(int pin) { return fakeArduinoPins.digitalRead(pin); }

unsigned long millis() {
  std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _start_time);
  return elapsed.count();
}