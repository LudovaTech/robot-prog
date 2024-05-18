#ifndef LIDAR_ANALYZER2_H
#define LIDAR_ANALYZER2_H

#include "lidar.h"
#include "utilities.h"
#include <algorithm>

class HoughLine {
 private:
  double _rho;
  double _theta;
  double _nb_accumulators;
  double _length;

 public:
  HoughLine(const double rho, const double theta, const double nb_accumulators)
      : _rho(rho), _theta(theta), _nb_accumulators(nb_accumulators), _length(0) {}
  HoughLine()
      : _rho(0), _theta(0), _nb_accumulators(0), _length(0) {}

  inline double rho() const { return _rho; }
  inline double theta() const { return _theta; }
  inline double nb_accumulators() const { return _nb_accumulators; }
  inline double length() const { return _length; }
};

class LidarInfos {
 private:
  Vector2 _coordinates;
  Radians orientation;
  MutableVector2 walls[4];

 public:
  LidarInfos(const Vector2 vcoordinates, Radians orientation);

  LidarInfos getFromLidarBuffer();

  /* Retourne les coordonnées du robot dans le référentiel du terrain. Centre du terrain: x=0, y=0.
   Axe y positif dans la direction du regard du robot
  */
  inline Vector2 coordinates() const { return _coordinates; }

  /* retourne les murs (le point le plus proche de chaque mur) */
  // TODO inline MutableVector2 getWalls() { return walls; }
};

class AnalyzeLidarData {
 private:
  const static unsigned int nbrLidarPoints = 20;
  const int lidarDistanceMax = 3000;
  const int lidarDistanceMin = 100;

  // Hough transform
  const int degreStep = 3;                             // degrés entre chaque droite calculée par Hough transform (3° -> 30 ms, 1° -> 90ms)
  const int HoughTransformAccumulatorsThreshold = 10;  // on exclut les lignes qui contiennent moins de 10 points
  const int HoughTransformMemorySize = 24000;          // taille de la matrice de Hough
  const double rhoTolerance = 500.0;                   // si une ligne est proche d'une autre de moins de 50cm, on l'exclut
  const double thetaMargin = 0.5;                      // si une ligne a un angle theta inférieur à 0,5 rad d'une autre, on l'exclut
  const double thetaToleranceParallel = 0.2;           // pour trouver le mur parallèle au premier, il faut une différence d'angle inférieur à 0,2 rad
  const double thetaTolerancePerpendiculaire = 0.2;    // pour trouver les murs perpendiculaires, il faut une différence d'angle inférieur à 0,2 rad (après - PI/2)

  uint16_t distanceMax;
  MutableVector2 convPoints[nbrLidarPoints];
  HoughLine lines[4095];  // The previous version already stopped at 4000

 public:
  AnalyzeLidarData() {}
  inline MutableVector2* _getConvPoints() { return convPoints; }

  bool analyze(CircularLidarPointsBuffer from);
  bool filterDistance(LidarPoint lidarPoint) const;
  bool convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice);
  bool convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer);
  bool houghTransform();
  bool sortLines();
};

#endif