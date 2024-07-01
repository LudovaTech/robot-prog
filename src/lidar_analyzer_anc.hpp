#ifndef LIDAR_ANALYZER_H
#define LIDAR_ANALYZER_H

#include <Arduino.h>

#include "parameters.hpp"
#include "utilities.hpp"

class LidarBasicInfos : public Vector2 {
  using Vector2::Vector2;
};

class LidarDetailedInfos {
 private:
  Vector2 _coordinates;
  Radians _orientation;
  Vector2 _frontGoalCoordinates;
  Vector2 _rearGoalCoordinates;

 public:
  LidarDetailedInfos(Vector2 coordinates, Radians orientation, Vector2 frontGoalCoordinates, Vector2 rearGoalCoordinates)
    : _coordinates(coordinates), _orientation(orientation), 
    _frontGoalCoordinates(frontGoalCoordinates), _rearGoalCoordinates(rearGoalCoordinates) {}

  inline Vector2 coordinates() { return _coordinates; }
  inline Radians orientation() { return _orientation; }
  inline Vector2 frontGoalCoordinates() { return _frontGoalCoordinates; }
  inline Vector2 rearGoalCoordinates() { return _rearGoalCoordinates; }
};

class LidarAncInfos {
 public:
  LidarAncInfos(const Vector2& coordinates, double orientation, std::vector<Vector2> walls)
      : _coordinates(coordinates), _orientation(orientation), walls(walls) {}

  /* Retourne les coordonnées du robot dans le référentiel du terrain. Centre du terrain: x=0, y=0.
     Axe y positif dans la direction du regard du robot
  */
  Vector2 getCoordinates() {
    return _coordinates;
  }

  /* Retourne l'orientation du robot (en degrés) : 0° s'il regarde droit vers le goal, < 0 s'il regarde vers la gauche, > 0 s'il regarde vers la droite
     max 90° (ensuite tout le repère s'inverse)
  */
  double getOrientation() {
    return _orientation * (180.0 / M_PI);
  }

  double getOrientationRadians() {
    return _orientation;
  }

  /* retourne les murs (le point le plus proche de chaque mur) */
  std::vector<Vector2> getWalls() {
    return walls;
  }

  /* Retourne le point du mur le plus proche */
  Vector2 getNearestWall() {
    if (walls.empty()) {
      return Vector2({-9999, -9999});
    }

    float nearest = 100000;
    size_t indice;
    for (size_t i = 0; i < walls.size(); i++) {
      float distance = walls[i].distance({0, 0});
      if (distance < nearest) {
        nearest = distance;
        indice = i;
      }
    }
    return walls[indice];
  }

 private:
  Vector2 _coordinates;
  double _orientation;
  std::vector<Vector2> walls;
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