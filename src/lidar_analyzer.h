#ifndef LIDAR_ANALYZER2_H
#define LIDAR_ANALYZER2_H

#include <algorithm>

#include "lidar.h"
#include "logger.h"
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
  Optional<MutableVector2> intersectWith(HoughLine other);
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

  String toString() const;
  /* retourne les murs (le point le plus proche de chaque mur) */
  // TODO inline MutableVector2 getWalls() { return walls; }
};

class AnalyzeLidarData {
 public:
  const static unsigned int nbrLidarPoints = 20;
  const static unsigned int nbrLinesMax = 4095;
  const static int lidarDistanceMax = 3000;
  const static int lidarDistanceMin = 100;

  // Hough transform
  const static int degreStep = 3;                             // degrés entre chaque droite calculée par Hough transform (3° -> 30 ms, 1° -> 90ms)
  const static int HoughTransformAccumulatorsThreshold = 10;  // on exclut les lignes qui contiennent moins de 10 points
  const static int HoughTransformMemorySize = 24000;          // taille de la matrice de Hough
  const static double rhoTolerance = 500.0;                   // si une ligne est proche d'une autre de moins de 50cm, on l'exclut
  const static double thetaMargin = 0.5;                      // si une ligne a un angle theta inférieur à 0,5 rad d'une autre, on l'exclut
  const static double thetaToleranceParallel = 0.2;           // pour trouver le mur parallèle au premier, il faut une différence d'angle inférieur à 0,2 rad
  const static double thetaTolerancePerpendiculaire = 0.2;    // pour trouver les murs perpendiculaires, il faut une différence d'angle inférieur à 0,2 rad (après - PI/2)

  // Longueur des segments sur les lignes de Hough
  const static int pointToLineDistanceMax = 20;   // un point doit être à moins de 2cm d'une ligne pour en faire partie
  const static int pointToPointDistanceMax = 70;  // un point doit être à moins de 7cm du prochain pour faire partie du même groupe
  const static int lineLengthMin = 250;           // une ligne doit être longue d'au moins 25cm pour être prise en compte (permet de filtrer les robots)

 private:
  uint16_t distanceMax;
  MutableVector2 convPoints[nbrLidarPoints];
  HoughLine lines[nbrLinesMax];  // The previous version already stopped at 4000
  int accumulator[HoughTransformMemorySize] = {0};
  Optional<HoughLine> firstWall;
  Optional<HoughLine> paralleleWall;
  Optional<HoughLine> firstPerpendicularWall;
  Optional<HoughLine> secondPerpendicularWall;
  Optional<bool> firstWallIsLengh;
  MutableVector2 corners[4];
  MutableVector2 centroid;
  MutableVector2 longestWallFirstCorner;
  MutableVector2 longestWallSecondCorner;
  Radians orientation = Radians(0);
  MutableVector2 coordinates;

  bool doneDistanceMax = false;
  bool doneConvPoints = false;
  bool doneLines = false;
  bool doneAccumulator = false;
  // bool doneFirstWall = false;
  // bool doneParalleleWall = false;
  // bool doneFirstPerpendicularWall = false;
  // bool doneSecondPerpendicularWall = false;
  // bool doneFirstWallIsLengh = false;
  bool doneCorners = false;
  bool doneLongestWallFirstCorner = false;
  bool doneLongestWallSecondCorner = false;
  bool doneOrientation = false;
  bool doneCoordinates = false;

 public:
  AnalyzeLidarData() {}
  inline uint16_t* _getDistanceMax() { return &distanceMax; }
  inline MutableVector2* _getConvPoints() { return convPoints; }
  inline HoughLine* _getLines() { return lines; }
  inline Optional<HoughLine>* _getFirstWall() { return &firstWall; }
  inline Optional<HoughLine>* _getParalleleWall() { return &paralleleWall; }
  inline Optional<HoughLine>* _getFirstPerpendicularWall() { return &firstPerpendicularWall; }
  inline Optional<HoughLine>* _getSecondPerpendicularWall() { return &secondPerpendicularWall; }
  inline Optional<bool>* _getFirstWallIsLengh() { return &firstWallIsLengh; }
  inline MutableVector2* _getCorners() { return corners; }
  inline MutableVector2* _getCentroid() { return &centroid; }
  inline MutableVector2* _getLongestWallFirstCorner() { return &longestWallFirstCorner; }
  inline MutableVector2* _getLongestWallSecondCorner() { return &longestWallSecondCorner; }
  inline Radians* _getOrientation() { return &orientation; }
  inline MutableVector2* _getCoordinates() { return &coordinates; }

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
  bool sortCornersClockwise();
  bool findLongestRealWall();
  bool calculateAngle();
  bool calculateCoordinates();
  LidarInfos getLidarInfos() const;
};

double calculateAngleBetweenLines(double a1, double b1, double c1, double a2, double b2, double c2);
double safe_acos(double value);

#endif