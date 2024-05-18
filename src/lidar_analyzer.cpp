#include "lidar_analyzer.h"

//////////// HoughLine

void HoughLine::convertToGeneralForm(double& a, double& b, double& c) {
  a = cos(theta());
  b = sin(theta());
  c = -rho();
}
// Calcul l'angle entre deux droites d'équation ax + by + c = 0
double calculateAngleBetweenLines(double a1, double b1, double c1, double a2, double b2, double c2) {
  // Calculer les vecteurs directeurs des droites
  double v1x = b1;
  double v1y = -a1;
  double v2x = b2;
  double v2y = -a2;

  // Calculer le produit scalaire des vecteurs directeurs
  double dotProduct = v1x * v2x + v1y * v2y;

  // Calculer les normes des vecteurs directeurs
  double norm1 = sqrt(v1x * v1x + v1y * v1y);
  double norm2 = sqrt(v2x * v2x + v2y * v2y);

  if (norm1 * norm2 == 0) {  // pour éviter une division par 0
    return PI / 4.0;
  }

  // Calculer l'angle entre les droites en radians
  double angle = safe_acos(dotProduct / (norm1 * norm2));

  if (angle > PI / 2.0) {
    return PI - angle;  // retourne toujours l'angle aigu
  }
  return angle;
}

// acos qui retourne toujours une valeur valide
double safe_acos(double value) {
  if (value <= -1.0) {
    return PI;
  } else if (value >= 1.0) {
    return 0;
  } else {
    return acos(value);
  }
}

//////////// LidarInfos

LidarInfos::LidarInfos(const Vector2 vcoordinates, Radians orientation)
    : _coordinates(vcoordinates), orientation(orientation) {}

//////////// AnalyzeLidarData
bool AnalyzeLidarData::analyze(CircularLidarPointsBuffer from) {
  // TODO
  return true;
}

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
              accumulator[indice]);
          indice_ajout++;
          if (indice_ajout >= 4095) {
            // log vérifier combien de fois cela arrive
            return true;
          }
        }
      }
    }
  }
  return true;
}

bool AnalyzeLidarData::sortLines() {
  // Tri des lignes selon nb_accumulators du plus grand au plus petit
  std::sort(lines, lines + sizeof(lines) / sizeof(lines[0]), [](const HoughLine& a, const HoughLine& b) {
    return a.nb_accumulators() > b.nb_accumulators();
  });
  return true;
}

