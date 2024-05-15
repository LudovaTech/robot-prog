#include "lidar_analyzer.h"

HoughLine::HoughLine(const double rho, const double theta, const double nb_accumulators, const double length)
    : rho(rho), theta(theta), nb_accumulators(nb_accumulators), length(length) {}

LidarInfos::LidarInfos(const Vector2 coordinates, Radians orientation, std::vector<MutableVector2> walls)
    : coordinates(coordinates), orientation(orientation), walls(walls) {}

LidarPoint[] ancfilterDistance(CircularLidarPointsBuffer lidarPointsBuffer, unsigned int LidarDistanceMin, unsigned int lidarDistanceMax) {
  points[];
  for (lidarPoint point in lidarPointsBuffer) {
    // on ne prend pas les points < 10cm et > 300cm
    if (lidarPoint.distance() > lidarDistanceMin && lidarPoint.distance() < lidarDistanceMax) {
      points2[nb_points++] = lidarPoint;
    }
  }
  return points
}

int findMaxDistance(LidarPoint[] lidarPoints) {
  int distanceMax;
  for (lidarPoint point in lidarPoints) {
    if (lidarPoint.distance() > distanceMax) {
      distanceMax = lidarPoint.distance();
    }
  }
  return distanceMax;
}

Vector2[] convCoordonneesCartesiennes(LidarPoint[] lidarPoints, size_t nbrVal) {
  Vector2[nbrVal] cartesiennes;
  for (size_t i = 0; i < nbrVal; i++) {
    float x = lidarPoint.distance() * cos(lidarPoint.angle() / 18000.0 * PI);
    float y = -lidarPoint.distance() * sin(lidarPoint.angle() / 18000.0 * PI);
    cartesiennes[i] = Vector2(x, y);
  }
  return cartesiennes;
}

houghTransform(std::vector<MutableVector2> points, int nbPoints, int distanceMax);

HoughLine[] sortLines(HoughLine[]);


bool filterDistance(LidarPoint lidarPoint) {
  return lidarPoint.distance() > lidarDistanceMin && lidarPoint.distance() < lidarDistanceMax;
}

