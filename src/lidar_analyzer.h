#ifndef LIDAR_ANALYZER2_H
#define LIDAR_ANALYZER2_H

#include "utilities.h"

const unsigned int nbrLidarPoints = 20;
Vector2[nbrLidarPoints] convPoints;

class HoughLine {
 private:
  const double rho;
  const double theta;
  const double nb_accumulators;
  const double length;

 public:
  HoughLine(const double rho, const double theta, const double nb_accumulators, const double length);
};

class LidarInfos {
 private:
  Vector2 coordinates;
  Radians orientation;
  MutableVector2[] walls;

 public:
  LidarInfos(const Vector2 coordinates, Radians orientation, std::vector<MutableVector2> walls);

  LidarInfos getFromLidarBuffer()

  /* Retourne les coordonnées du robot dans le référentiel du terrain. Centre du terrain: x=0, y=0.
   Axe y positif dans la direction du regard du robot
  */
  inline Vector2 coordinates() const { return coordinates; }

  /* retourne les murs (le point le plus proche de chaque mur) */
  inline std::vector<MutableVector2> getWalls() { return walls; }
};

bool filterDistance(LidarPoint lidarPoint);
bool convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice, Vector2* finalPoints);
bool convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer, Vector2* finalPoints);

#endif