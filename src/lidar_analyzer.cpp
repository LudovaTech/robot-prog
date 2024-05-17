#include "lidar_analyzer.h"

HoughLine::HoughLine(const double rho, const double theta, const double nb_accumulators, const double length)
    : rho(rho), theta(theta), nb_accumulators(nb_accumulators), length(length) {}

LidarInfos::LidarInfos(const Vector2 vcoordinates, Radians orientation, std::vector<MutableVector2> walls)
    : _coordinates(vcoordinates), orientation(orientation), walls(walls) {}

bool AnalyseLidarData::filterDistance(LidarPoint lidarPoint) {
  return lidarPoint.distance() > lidarDistanceMin && lidarPoint.distance() < lidarDistanceMax;
}

bool AnalyseLidarData::convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice) {
  convPoints[indice] = Vector2(
    lidarPoint.distance() * cos(lidarPoint.angle() / 18000.0 * PI),
    -lidarPoint.distance() * sin(lidarPoint.angle() / 18000.0 * PI)
  );
  return true;
}

bool AnalyseLidarData::convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer) {
  //TODO: must detect the last tour only
  for (unsigned int i = 0; i <= nbrLidarPoints; i++) {
    if (!lidarPointsBuffer.existValue(i)) {
      //log
      return false;
    }
    int distanceMax;
    LidarPoint lidarPoint = lidarPointsBuffer.getValue(i);
    if (filterDistance(lidarPoint)) {
      distanceMax = max(distanceMax, lidarPoint.distance());
      convCoordonneesCartesiennes(LidarPoint, i);
    }
  }
}