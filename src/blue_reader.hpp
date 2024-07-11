#ifndef BLUE_H
#define BLUE_H

#include "utilities.hpp"
#include "logger.hpp"

const int minBytesAvailableBlue = 17;
const int timeBlueCache = 50;

struct BlueInfosGlue {
  Optional<Vector2> partnerPos;
  Optional<Vector2> ballPos;
};

BlueInfosGlue getBlueInfos();
void sendBlueData(Vector2 partnerPos, Vector2 ballPos);

#endif