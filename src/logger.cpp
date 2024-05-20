#include "logger.h"

#ifndef UNIT_TEST_ACTIVATED
File _logFile;
unsigned int _logLevel;
bool _onSerial;

void setupLog(int logLevel, bool onSerial) {
  _onSerial = onSerial;
  _logLevel = logLevel;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("logger.setupLog : Erreur lors de l'initialisation de la carte microSD !, maybe there is no SD card, logger desactivated");
    return;
  }
  if (_logFile) {
    log_a(ErrorLevel, "logger.setupLog", "already setup");
  } else {
    _logFile = SD.open("log1.log", FILE_WRITE);
    if (!_logFile) {
      SerialDebug.println("logger.setupLog : cannot open a file, logger desactivated");
    }
    log_a(InfoLevel, "logger.setupLog", "------- NEW SESSION -------");
  }
}

void log_a(unsigned int level, String fromFun, String message) {
  if (level >= _logLevel) {
    String formattedMessage = getTimestamp() + " : " + logGetName(level) + " from " + fromFun + " : " + message;
    if (_onSerial) {
      SerialDebug.println(formattedMessage);
    }
    if (_logFile) {
      _logFile.println(formattedMessage);
      _logFile.flush();
    }
  }
}

#else
void setupLog(int logLevel, bool onSerial) {}
void log_a(unsigned int level, String fromFun, String message) {}

#endif

const unsigned int NoteLevel = 10;
const unsigned int DebugLevel = 20;
const unsigned int InfoLevel = 30;
const unsigned int ErrorLevel = 40;
const unsigned int CriticalLevel = 50;

String logGetName(unsigned int level) {
  switch (level) {
    case NoteLevel:
      return "NOTE";
    case DebugLevel:
      return "DEBUG";
    case InfoLevel:
      return "INFO";
    case ErrorLevel:
      return "!ERROR!";
    case CriticalLevel:
      return "!!!CRITICAL!!!";
    default:
      return "LEVEL" + String(level);
  }
}

String getTimestamp() {
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  String timestamp = "";
  timestamp += String(hours);
  timestamp += ':';
  timestamp += String(minutes % 60);
  timestamp += ':';
  timestamp += String(seconds % 60);
  timestamp += '.';
  timestamp += String(currentMillis % 1000);

  return timestamp;
}