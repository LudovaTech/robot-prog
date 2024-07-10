#include "logger.hpp"

#ifndef UNIT_TEST_ACTIVATED
File _logFile;
unsigned int _logLevel;
unsigned int _fixedMessageLength;

void setupLog(int logLevel, unsigned int fixedMessageLength) {
  _fixedMessageLength = fixedMessageLength;
  _logLevel = logLevel;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("logger.setupLog : Error initializing microSD !, maybe there is no SD card, logger desactivated");
  } else {
    if (_logFile) {
      log_a(ErrorLevel, "logger.setupLog", "already setup");
    } else {
      _logFile = SD.open("log1.log", FILE_WRITE);
      if (!_logFile) {
        Serial.println("logger.setupLog : cannot open a file, logger desactivated");
      }
    }
  }
  log_a(InfoLevel, "logger.setupLog", "------- NEW SESSION -------");
}

void log_a(unsigned int level, String fromFun, String message) {
  if (level >= _logLevel) {
    String formattedMessage = getTimestamp() + " : " + logGetName(level) + " from " + cutString(fromFun, _fixedMessageLength) + " : " + message;
    Serial.println(formattedMessage);
    Serial.flush();
    if (_logFile) {
      _logFile.println(formattedMessage);
      _logFile.flush();
    }
  }
}

#else
void setupLog(int logLevel) {}
void log_a(unsigned int level, String fromFun, String message) {}

#endif

String logGetName(unsigned int level) {
  switch (level) {
    case NoteLevel:
      return "NOTE ";
    case DebugLevel:
      return "DEBUG";
    case InfoLevel:
      return "INFO ";
    case StratLevel:
      return "STRAT";
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

  String secondString = seconds % 60;
  String minuteString = minutes % 60;
  String milliString = currentMillis % 1000;

  // Formater chaque composant avec des zéros en tête si nécessaire
  if (minuteString.length() < 2) {
    minuteString = "0" + minuteString;
  }

  if (secondString.length() < 2) {
    secondString = "0" + secondString;
  }

  while (milliString.length() < 3) {
    milliString = "0" + milliString;
  }

  return minuteString + ":" + secondString + "." + milliString;
}

String cutString(String input, int fixedLength) {
  int inputLength = input.length();

  // Si la chaîne d'origine est plus courte que la longueur fixe
  if (inputLength < fixedLength) {
    // Ajouter des espaces à la fin de la chaîne
    while (input.length() < fixedLength) {
      input += ' ';
    }
    return input;
  }

  // Si la chaîne d'origine est plus longue que la longueur fixe
  if (inputLength > fixedLength) {
    // Longueur des parties à garder de chaque côté
    int keepLength = (fixedLength - 3) / 2;
    // Gérer les cas où la longueur fixe est impaire
    int extraChar = (fixedLength - 3) % 2;

    // Partie de début et de fin
    String startPart = input.substring(0, keepLength + extraChar);
    String endPart = input.substring((size_t) inputLength - keepLength);

    // Retourner la chaîne formatée
    return startPart + "..." + endPart;
  }

  // Si la chaîne d'origine a exactement la longueur fixe
  return input;
}
