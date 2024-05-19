#ifndef LIDAR_ANALYZER2_H
#define LIDAR_ANALYZER2_H

#include <algorithm>

#include "lidar.h"
#include "states.h"
#include "utilities.h"

class HoughLine {
 private:
  double _rho;
  double _theta;
  double _nb_accumulators;
  Optional<double> _length;

 public:
  HoughLine(const double rho, const double theta, const double nb_accumulators)
      : _rho(rho), _theta(theta), _nb_accumulators(nb_accumulators), _length(Optional<double>()) {}
  HoughLine()
      : _rho(0), _theta(0), _nb_accumulators(0), _length(0) {}

  inline double rho() const { return _rho; }
  inline double theta() const { return _theta; }
  inline double nb_accumulators() const { return _nb_accumulators; }
  inline Optional<double> length() const { return _length; }
  inline void setLength(double nLength) { _length = Optional<double>(nLength); }

  double calculateDistanceBetweenLines(HoughLine other);
  Optional<Vector2> intersectWith(HoughLine other);
};

class CarthesianLine {
 private:
  double _a, _b, _c;

 public:
  CarthesianLine(double a, double b, double c)
      : _a(a), _b(b), _c(c) {}

  CarthesianLine(HoughLine line);

  inline double a() const { return _a; }
  inline double b() const { return _b; }
  inline double c() const { return _c; }

  Radians calculateAngleBetweenLines(CarthesianLine other);
  double calculateDistanceToPoint(Vector2 point);
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
  const static unsigned int nbrLinesMax = 4095;
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

  // Longueur des segments sur les lignes de Hough
  const int pointToLineDistanceMax = 20;   // un point doit être à moins de 2cm d'une ligne pour en faire partie
  const int pointToPointDistanceMax = 70;  // un point doit être à moins de 7cm du prochain pour faire partie du même groupe
  const int lineLengthMin = 250;           // une ligne doit être longue d'au moins 25cm pour être prise en compte (permet de filtrer les robots)

  uint16_t distanceMax;
  MutableVector2 convPoints[nbrLidarPoints];
  HoughLine lines[nbrLinesMax];  // The previous version already stopped at 4000
  Optional<HoughLine> firstWall;
  Optional<HoughLine> paralleleWall;
  Optional<HoughLine> firstPerpendicularWall;
  Optional<HoughLine> secondPerpendicularWall;
  Optional<bool> firstWallIsLengh;
  MutableVector2 corners[4];
  MutableVector2 centroid;

 public:
  AnalyzeLidarData() {}
  inline MutableVector2* _getConvPoints() { return convPoints; }

  bool analyze(CircularLidarPointsBuffer from);
  bool filterDistance(LidarPoint lidarPoint) const;
  bool convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice);
  bool convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer);
  bool houghTransform();
  bool sortLines();
  bool findWalls(FieldProperties fP);
  bool detectFirstWall(HoughLine line, FieldProperties fP);
  bool detectParalleleWall(HoughLine line, FieldProperties fP);
  bool detectPerpendicularWall(HoughLine line, FieldProperties fP, bool isFirst);
  ResultOrError<float> distanceCalculatedWithGroups(CarthesianLine line);
  bool isLength(double distance, FieldProperties fP) const;
  bool isWidth(double distance, FieldProperties fP) const;
  bool has4Walls() const;
  bool calculateCorners();
  bool computeCentroid();
};

double calculateAngleBetweenLines(double a1, double b1, double c1, double a2, double b2, double c2);
double safe_acos(double value);

#endif