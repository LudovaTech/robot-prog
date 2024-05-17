#ifndef LIDAR_ANALYZER2_H
#define LIDAR_ANALYZER2_H

#include "utilities.h"
#include "lidar.h"

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
  Vector2 _coordinates;
  Radians orientation;
  MutableVector2 walls[4];

 public:
  LidarInfos(const Vector2 vcoordinates, Radians orientation, std::vector<MutableVector2> walls);

  LidarInfos getFromLidarBuffer();

  /* Retourne les coordonnées du robot dans le référentiel du terrain. Centre du terrain: x=0, y=0.
   Axe y positif dans la direction du regard du robot
  */
  inline Vector2 coordinates() const { return _coordinates; }

  /* retourne les murs (le point le plus proche de chaque mur) */
  //TODO inline MutableVector2 getWalls() { return walls; }
};

class AnalyseLidarData {
 private:
  const static unsigned int nbrLidarPoints = 20;
  Vector2 convPoints[nbrLidarPoints];

 public:
  AnalyseLidarData() {}
  inline Vector2* getConvPoints() { return convPoints; }

  bool filterDistance(LidarPoint lidarPoint);
  bool convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice);
  bool convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer);
};

#endif