#include "lidar_analyzer.h"

//////////// HoughLine

// calcul la distance entre deux lignes de Hough
double HoughLine::calculateDistanceBetweenLines(HoughLine other) {
  float x1 = rho() * cos(theta());
  float y1 = -rho() * sin(theta());

  float x2 = other.rho() * cos(other.theta());
  float y2 = -other.rho() * sin(other.theta());

  return Vector2(x1, y1).distance(Vector2(x2, y2));
}

Optional<Vector2> HoughLine::intersectWith(HoughLine other) {
  double sinTheta1 = sin(theta());
  double sinTheta2 = sin(other.theta());
  double cosTheta1 = cos(theta());
  double cosTheta2 = cos(other.theta());
  double determinant = cosTheta1 * sinTheta2 - sinTheta1 * cosTheta2;

  // Si le déterminant est 0, les lignes sont parallèles ou coïncidentes
  if (abs(determinant) < 1e-10) {
    return Optional<Vector2>();
  }

  return Optional<Vector2>(Vector2(
      (rho() * sinTheta2 - other.rho() * sinTheta1) / determinant,
      (other.rho() * cosTheta1 - rho() * cosTheta2) / determinant));
}

CarthesianLine::CarthesianLine(HoughLine line)
    : _a(cos(line.theta())), _b(sin(line.theta())), _c(-line.rho()) {}

// Calcul l'angle entre deux droites d'équation ax + by + c = 0
Radians CarthesianLine::calculateAngleBetweenLines(CarthesianLine other) {
  // Calculer les vecteurs directeurs des droites
  double v1x = b();
  double v1y = -a();
  double v2x = other.b();
  double v2y = -other.a();

  // Calculer le produit scalaire des vecteurs directeurs
  double dotProduct = v1x * v2x + v1y * v2y;

  // Calculer les normes des vecteurs directeurs
  double norm1 = sqrt(v1x * v1x + v1y * v1y);
  double norm2 = sqrt(v2x * v2x + v2y * v2y);

  if (norm1 * norm2 == 0) {  // pour éviter une division par 0
    return Radians(PI / 4.0);
  }

  // Calculer l'angle entre les droites en radians
  double angle = safe_acos(dotProduct / (norm1 * norm2));

  if (angle > PI / 2.0) {
    return Radians(PI - angle);  // retourne toujours l'angle aigu
  }
  return Radians(angle);
}

// distance d'un point à une droite d'équation ax + by + c = 0
double CarthesianLine::calculateDistanceToPoint(Vector2 point) {
  return abs(a() * point.x() + b() * point.y() + c()) / sqrt(sq(a()) + sq(b()));
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

bool AnalyzeLidarData::isLength(double distance, FieldProperties fP) const {
  return (distance > 0.9 * fP.fieldLength() && distance < 1.1 * fP.fieldLength());
}

bool AnalyzeLidarData::isWidth(double distance, FieldProperties fP) const {
  return (distance > 0.9 * fP.fieldWidth() && distance < 1.1 * fP.fieldWidth());
}

bool AnalyzeLidarData::has4Walls() const {
  return firstWall.hasValue() && paralleleWall.hasValue() && firstPerpendicularWall.hasValue() && secondPerpendicularWall.hasValue();
}

bool AnalyzeLidarData::findWalls(FieldProperties fP) {
  if (detectFirstWall(lines[0], fP)) {
    // si la première ligne est inféreure au minimum (= erreur) ou ne correspond
    // pas à une longueure connue, l'ensemble de la détection échoue
    // log
    return false;
  }
  for (unsigned int i = 1; i < nbrLinesMax; i++) {
    HoughLine line = lines[i];
    if (!paralleleWall.hasValue()) {
      detectParalleleWall(line, fP);
    }
    if (!firstPerpendicularWall.hasValue()) {
      detectPerpendicularWall(line, fP, true);
    } else if (!secondPerpendicularWall.hasValue()) {
      detectPerpendicularWall(line, fP, false);
    }
    if (has4Walls()) {
      break;
    }
  }
  if (!has4Walls()) {
    // TODO
    return false;
  }
  // now, we have 4 walls
  if (!calculateCorners()) {
    // log
    return false;
  }
  if (!computeCentroid()) {
    // log strange
    return false;
  }
  if (!sortCornersClockwise()) {
    // log strange
    return false;
  }
  if (!findLongestRealWall()) {
    // log strange
    return false;
  }
  if (!calculateAngle()) {
    // log strange
    return false;
  }
  if (!calculateCoordinates()) {
    // log strange
    return false;
  }

}

LidarInfos AnalyzeLidarData::getLidarInfos() const {
  return LidarInfos(
    coordinates.toVector2(),
    orientation
  );
}

bool AnalyzeLidarData::calculateCoordinates() {
  coordinates = MutableVector2(
    -centroid.y() * sin(orientation) - centroid.x() * cos(orientation),
    -centroid.y() * cos(orientation) + centroid.x() * sin(orientation)
  );
  return true;
}

bool AnalyzeLidarData::calculateAngle() {
  double deltaY = longestWallSecondCorner.y() - longestWallFirstCorner.y();
  double deltaX = longestWallSecondCorner.x() - longestWallFirstCorner.x();

  if (longestWallSecondCorner.y() < longestWallFirstCorner.y()) {
    deltaY = -deltaY;
    deltaX = -deltaX;
  }

  double angleRadians = atan2(deltaY, deltaX);  // renvoie un angle entre -PI et +PI
  double adjustedAngleRadians = angleRadians - PI / 2.0;
  orientation = Radians(adjustedAngleRadians);
  return true;
}

bool AnalyzeLidarData::findLongestRealWall() {
  double maxDistance = 0;
  Optional<MutableVector2> firstCornerIndex;
  Optional<MutableVector2> secondCornerIndex;
  for (unsigned int i = 0; i < 4; i++) {
    MutableVector2 actualFirstCornerIndex = corners[i];
    MutableVector2 actualSecondCornerIndex = corners[(i + 1) % 4];
    double distanceBetweenCorners = actualFirstCornerIndex.toVector2().distance(actualSecondCornerIndex.toVector2());
    if (distanceBetweenCorners > maxDistance) {
      maxDistance = distanceBetweenCorners;
      firstCornerIndex = Optional<MutableVector2>(actualFirstCornerIndex);
      secondCornerIndex = Optional<MutableVector2>(actualSecondCornerIndex);
    }
  }
  if (firstCornerIndex.hasValue() && secondCornerIndex.hasValue()) {
    longestWallFirstCorner = firstCornerIndex.value();
    longestWallSecondCorner = secondCornerIndex.value();
    return true;
  } else {
    return false;
  }
}

bool AnalyzeLidarData::sortCornersClockwise() {
  MutableVector2 center = centroid;
  std::sort(corners, corners + sizeof(corners) / sizeof(corners[0]), [center](MutableVector2 a, MutableVector2 b) {
    double angleA = atan2(a.y() - center.y(), a.x() - center.x());
    double angleB = atan2(b.y() - center.y(), b.x() - center.x());

    // Normalisation des angles pour s'assurer qu'ils sont dans le même intervalle
    angleA = fmod(angleA, 2 * PI);
    angleB = fmod(angleB, 2 * PI);

    // Comparaison des angles normalisés
    return angleA > angleB;
  });
  return true;
}

bool AnalyzeLidarData::computeCentroid() {
  centroid = MutableVector2(
      (corners[0].x() + corners[1].x() + corners[2].x() + corners[3].x()) / 4,
      (corners[0].y() + corners[1].y() + corners[2].y() + corners[3].y()) / 4);
  return true;
}

bool AnalyzeLidarData::calculateCorners() {
  // TODO protection
  Optional<Vector2> c1 = firstWall.value().intersectWith(firstPerpendicularWall.value());
  if (!c1.hasValue()) {
    return false;
  } else {
    corners[0] = c1.value();
  }
  Optional<Vector2> c2 = firstWall.value().intersectWith(firstPerpendicularWall.value());
  if (!c2.hasValue()) {
    return false;
  } else {
    corners[0] = c2.value();
  }
  Optional<Vector2> c3 = firstWall.value().intersectWith(firstPerpendicularWall.value());
  if (!c3.hasValue()) {
    return false;
  } else {
    corners[0] = c3.value();
  }
  Optional<Vector2> c4 = firstWall.value().intersectWith(firstPerpendicularWall.value());
  if (!c4.hasValue()) {
    return false;
  } else {
    corners[0] = c4.value();
  }
  return true;
}

bool AnalyzeLidarData::detectFirstWall(HoughLine line, FieldProperties fP) {
  // On décide que la première ligne, comme elle est la plus grande, appartient au terrain
  // Le seul cas où elle n'est pas la première ligne serait que sa longeur sa inférieur
  // au minimum ou ne correspond pas à la largeur ou la longueur
  // auquel cas l'ensemble de la détection échoue

  // bypass angle

  // bypass distance

  // testons la distance de la ligne pour voir si elle correspont à la largeur ou la longueur du terrain
  ResultOrError<float> distance = distanceCalculatedWithGroups(line);
  if (distance.hasError()) {
    // log
    return false;
  } else {
    if (distance.value() > lineLengthMin) {
      if (isLength(distance.value(), fP)) {
        firstWallIsLengh = Optional<bool>(true);
      } else if (isWidth(distance.value(), fP)) {
        firstWallIsLengh = Optional<bool>(false);
      } else {
        // log incohérent
        return false;
      }
      line.setLength(distance.value());
      firstWall = Optional<HoughLine>(line);
      return true;
    } else {
      // log
      return false;
    }
  }
}

bool AnalyzeLidarData::detectParalleleWall(HoughLine line, FieldProperties fP) {
  // TODO teste si firstWall vide
  // testons l'angle entre ce mur et le premier
  if (!(CarthesianLine(line).calculateAngleBetweenLines(firstWall.value()) < thetaToleranceParallel)) {
    return false;
  }

  // testons la distance entre ce mur et le premier mur
  double distanceBetweenParallelWalls = line.calculateDistanceBetweenLines(firstWall.value());
  if (isLength(distanceBetweenParallelWalls, fP)) {
    if (!firstWallIsLengh.value()) {
      // log incohérent
      return false;
    }
  } else if (isWidth(distanceBetweenParallelWalls, fP)) {
    if (firstWallIsLengh.value()) {
      // log incohérent
      return false;
    }
  } else {
    // log incohérent
    return false;
  }

  // testons la distance de la ligne pour voir si elle correspont à la largeur ou la longueur du terrain
  ResultOrError<float> distance = distanceCalculatedWithGroups(line);
  if (distance.hasError()) {
    // log strange
    return false;
  } else {
    if (distance.value() > lineLengthMin) {
      if (firstWallIsLengh.value() && isLength(distance.value(), fP)) {
      } else if ((!firstWallIsLengh.value()) && isWidth(distance.value(), fP)) {
      } else {
        // log incohérent
        return false;
      }
      line.setLength(distance.value());
      paralleleWall = Optional<HoughLine>(line);
      return true;
    } else {
      return false;
    }
  }
}

bool AnalyzeLidarData::detectPerpendicularWall(HoughLine line, FieldProperties fP, bool isFirst) {
  // TODO teste si firstWall vide et firstPerpendicularWall
  // testons l'angle entre ce mur et le premier
  if (!(abs(CarthesianLine(line).calculateAngleBetweenLines(firstWall.value()) - PI / 2.0) < thetaTolerancePerpendiculaire)) {
    return false;
  }

  // Si un mur perpendiculaire a déjà été trouvé, testons si cette ligne est à la bonne distance
  if (!isFirst) {
    double distanceBetweenPerpendicularWalls = line.calculateDistanceBetweenLines(firstPerpendicularWall.value());
    if (isWidth(distanceBetweenPerpendicularWalls, fP)) {
      if (firstWallIsLengh.value()) {
        // log incohérent
        return false;
      }
    } else if (isLength(distanceBetweenPerpendicularWalls, fP)) {
      if (!firstWallIsLengh.value()) {
        // log incohérent
        return false;
      }
    } else {
      // log incohérent
      return false;
    }
  }

  // testons la distance de la ligne pour voir si elle correspont à la largeur ou la longueur du terrain
  ResultOrError<float> distance = distanceCalculatedWithGroups(line);
  if (distance.hasError()) {
    // log strange
    return false;
  } else {
    if (distance.value() > lineLengthMin) {
      if (firstWallIsLengh.value() && isWidth(distance.value(), fP)) {
      } else if ((!firstWallIsLengh.value()) && isLength(distance.value(), fP)) {
      } else {
        // log incohérent
        return false;
      }
      line.setLength(distance.value());
      if (isFirst) {
        firstPerpendicularWall = Optional<HoughLine>(line);
      } else {
        secondPerpendicularWall = Optional<HoughLine>(line);
      }
      return true;
    } else {
      return false;
    }
  }
}

ResultOrError<float> AnalyzeLidarData::distanceCalculatedWithGroups(CarthesianLine line) {
  float maxDistance = 0.0;
  Optional<MutableVector2> groupFront;
  Optional<MutableVector2> groupEnd;
  unsigned int groupLen = 0;
  for (unsigned int i = 0; i < nbrLidarPoints; i++) {
    MutableVector2 point = convPoints[i];
    if (line.calculateDistanceToPoint(point.toVector2()) <= pointToLineDistanceMax) {
      if (!groupFront.hasValue()) {
        groupFront = Optional<MutableVector2>(point);
        groupEnd = Optional<MutableVector2>(point);
        groupLen = 1;
      } else {
        if (point.toVector2().distance(groupEnd.value().toVector2()) <= pointToPointDistanceMax) {
          groupEnd = Optional<MutableVector2>(point);
          groupLen++;
        } else {
          // alors c'est la fin de ce groupe, on le compare à la plus grande valeure
          if (groupLen >= 2) {
            maxDistance = max(maxDistance, groupFront.value().toVector2().distance(groupEnd.value().toVector2()));
          }
          groupFront = Optional<MutableVector2>(point);
          groupEnd = Optional<MutableVector2>(point);
          groupLen = 1;
        }
      }
    }
  }
  return ResultOrError<float>(maxDistance);
}