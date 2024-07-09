#ifndef LIDAR_ANALYZER_H
#define LIDAR_ANALYZER_H

#include <Arduino.h>

#include "parameters.hpp"
#include "utilities.hpp"

class LidarBasicInfos : public Vector2 {
 private:
  std::vector<Vector2> _obstacles;

 public:
  LidarBasicInfos(float x, float y, std::vector<Vector2> obstacles)
      : Vector2(x, y), _obstacles(obstacles) {}

  inline std::vector<Vector2> obstacles() { return _obstacles; }
};

class LidarDetailedInfos {
 private:
  Vector2 _coordinates;
  Radians _orientation;
  Vector2 _frontGoalCoordinates;
  Vector2 _rearGoalCoordinates;

 public:
  LidarDetailedInfos(Vector2 coordinates, Radians orientation, Vector2 frontGoalCoordinates, Vector2 rearGoalCoordinates)
      : _coordinates(coordinates), _orientation(orientation), _frontGoalCoordinates(frontGoalCoordinates), _rearGoalCoordinates(rearGoalCoordinates) {}

  inline Vector2 coordinates() { return _coordinates; }
  inline Radians orientation() { return _orientation; }
  inline Vector2 frontGoalCoordinates() { return _frontGoalCoordinates; }
  inline Vector2 rearGoalCoordinates() { return _rearGoalCoordinates; }
};

// TODO: temporaire
struct LidarInfosGlue {
  Optional<LidarDetailedInfos> oLDI;
  Optional<LidarBasicInfos> oLBI;
};

LidarInfosGlue getLidarInfos(FieldProperties fP, bool readFromLidar = true, bool show_log = false, const char* input = nullptr);
void testsLidar(FieldProperties fP);

Optional<LidarBasicInfos> getNearestWall(std::vector<Vector2> walls);

#endif