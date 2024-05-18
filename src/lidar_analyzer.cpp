#include "lidar_analyzer.h"

//////////// HoughLine



//////////// LidarInfos

LidarInfos::LidarInfos(const Vector2 vcoordinates, Radians orientation)
    : _coordinates(vcoordinates), orientation(orientation) {}

//////////// AnalyzeLidarData

bool AnalyzeLidarData::filterDistance(LidarPoint lidarPoint) const {
  return lidarPoint.distance() > lidarDistanceMin && lidarPoint.distance() < lidarDistanceMax;
}

bool AnalyzeLidarData::convCoordonneesCartesiennes(LidarPoint lidarPoint, unsigned int indice) {
  convPoints[indice] = MutableVector2(
      lidarPoint.distance() * cos(lidarPoint.angle() / 18000.0 * PI),
      -lidarPoint.distance() * sin(lidarPoint.angle() / 18000.0 * PI));
  return true;
}

bool AnalyzeLidarData::convFromBuffer(CircularLidarPointsBuffer lidarPointsBuffer) {
  // TODO: must detect the last tour only
  for (unsigned int i = 0; i <= nbrLidarPoints; i++) {
    if (!lidarPointsBuffer.existValue(i)) {
      // log
      return false;
    }
    LidarPoint lidarPoint = lidarPointsBuffer.getValue(i);
    if (filterDistance(lidarPoint)) {
      distanceMax = max(distanceMax, lidarPoint.distance());
      if (!convCoordonneesCartesiennes(lidarPoint, i)) {
        return false;
      }
    }
  }
  return true;
}

/* Hough Transform
 *  algo : https://www.keymolen.com/2013/05/hough-transformation-c-implementation.html
 */
bool AnalyzeLidarData::houghTransform() {
  const int numTheta = 180;
  double thetaStep = degreStep * PI / 180.0;
  double rhoStep = distanceMax * 2.0 * numTheta / HoughTransformMemorySize;
  unsigned int indice_ajout = 0;
  int accumulator[HoughTransformMemorySize] = {0};

  // calcul de rho pour chaque valeur de theta
  for (unsigned int i = 0; i < nbrLidarPoints; i++) {
    MutableVector2 point = convPoints[i];
    for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex += degreStep) {
      double theta = thetaIndex * thetaStep / degreStep;
      int rhoIndex = round((distanceMax + (point.x() * cos(theta) + point.y() * sin(theta))) / rhoStep);
      int accuIndex = thetaIndex * distanceMax * 2 / rhoStep + rhoIndex;
      if (accuIndex >= 0 && accuIndex < HoughTransformMemorySize) {
        accumulator[accuIndex]++;
      }
    }
  }

  // on parcourt la matrice de Hough pour récupérer les lignes avec un nombre d'accumulateurs suffisamment élevé
  for (int thetaIndex = 0; thetaIndex < numTheta; thetaIndex += degreStep) {
    for (int rhoIndex = 0; rhoIndex < distanceMax * 2 / rhoStep; rhoIndex++) {
      int indice = thetaIndex * distanceMax * 2 / rhoStep + rhoIndex;
      if (indice >= 0 && indice < HoughTransformMemorySize) {
        if (accumulator[indice] >= HoughTransformAccumulatorsThreshold) {
          lines[indice_ajout] = HoughLine(
            rhoIndex * rhoStep - distanceMax,
            thetaIndex * thetaStep / degreStep,
            accumulator[indice]
          );
          indice_ajout++;
          if (indice_ajout >= 4095) {  // pour éviter le CRASH quand lines.size() == 4096 !!
            //log vérifier combien de fois cela arrive
            return true;
          }
        }
      }
    }
  }
  return true;
}

