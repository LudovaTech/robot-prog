#ifndef BLUE_H
#define BLUE_H

#include "utilities.hpp"
#include "logger.hpp"

const int minBytesAvailable = 57;

struct BlueInfosGlue {
  Optional<Vector2> partnerPos;
  Optional<Vector2> ballPos;
};

BlueInfosGlue getBlueInfos();
void sendBlueData(Vector2 partnerPos, Vector2 ballPos);

#endif