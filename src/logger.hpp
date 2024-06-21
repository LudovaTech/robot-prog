#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#ifndef UNIT_TEST_ACTIVATED
#include <SD.h>
#endif

#include "utilities.hpp"

const unsigned int NoteLevel = 10;
const unsigned int DebugLevel = 20;
const unsigned int StratLevel = 30;
const unsigned int InfoLevel = 40;
const unsigned int ErrorLevel = 50;
const unsigned int CriticalLevel = 60;

String logGetName(unsigned int level);

void setupLog(int logLevel);

void log_a(unsigned int level, String fromFun, String message);

String getTimestamp();

#endif