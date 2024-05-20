#ifndef LOGGER_H
#define LOOGER_H

#include <Arduino.h>

#ifndef UNIT_TEST_ACTIVATED
#include <SD.h>
#endif

#include "utilities.h"

extern const unsigned int NoteLevel;
extern const unsigned int DebugLevel;
extern const unsigned int InfoLevel;
extern const unsigned int ErrorLevel;
extern const unsigned int CriticalLevel;

String logGetName(unsigned int level);

void setupLog(int logLevel, bool onSerial);

void log_a(unsigned int level, String fromFun, String message);

String getTimestamp();

#endif