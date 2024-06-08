#include <string>

#include "lidar_reader.hpp"
#include "lidar_analyzer_anc.hpp"
#include "movements.hpp"
#include "cam_reader.hpp"
#include "strategy.hpp"
#include "utilities.hpp"

const FieldProperties fieldProperties = FieldProperties(
    243,               // fieldLength
    182,               // fieldWidth
    12,                // spaceBeforeLineSide
    60,                // goalWidth
    Vector2(0, -115),  // myGoalPos
    Vector2(0, 115),   // enemyGoalPos
    9,                 // robotRadius
    2                  // ballRadius
);

const Motors motors = Motors(

    // Arduino UNO
    // MotorMov(11, 12, 0, Degree(-40)),
    // MotorMov(5, 4, 0, Degree(40)),
    // MotorMov(6, 7, 0, Degree(-140)),
    // MotorMov(9, 8, 0, Degree(140))

    // Teensy
    MotorMov(15, 14, 0, Degree(-40)),
    MotorMov(36, 33, 0, Degree(40)),
    MotorMov(22, 19, 0, Degree(-140)),
    MotorMov(11, 12, 0, Degree(140)));

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);

  SerialCam.setTimeout(10);
  SerialLidar.setTimeout(10);

  pinMode(13, OUTPUT);
  pinMode(26, INPUT);
}

std::string extractLastCompleteSequence(const char* buffer) {
  std::string str(buffer);
  size_t lastE = str.find_last_of('e');
  if (lastE != std::string::npos) {
    size_t prevB = str.rfind('b', lastE);
    if (prevB != std::string::npos) {
      return str.substr(prevB, lastE - prevB);
    }
  }
  return "";
}

CamInfos getCamInfos() {
  size_t bytesAvailable = SerialCam.available();
  // SerialDebug.println("nb of bytes available: " + String(bytesAvailable));

  if (bytesAvailable >= 26) {
    byte buffer[65];
    size_t nbrBytesReceived = SerialCam.readBytes(buffer, min(bytesAvailable, sizeof(buffer) - 1));
    buffer[nbrBytesReceived] = '\0';
    // SerialDebug.println(" reçu: " + String((char*)buffer));

    std::string lastCompleteSequence = extractLastCompleteSequence((char*)buffer);
    if (!lastCompleteSequence.empty()) {
      // exemple : b-096-121+000+000+000+000e
      int ball_x, ball_y, my_goal_x, my_goal_y, enemy_goal_x, enemy_goal_y;
      if (sscanf(lastCompleteSequence.c_str(), "b%d%d%d%d%d%de", &ball_x, &ball_y, &my_goal_x, &my_goal_y,
                 &enemy_goal_x, &enemy_goal_y) == 6) {
        SerialDebug.println("Position balle: x=" + String(ball_x) + ", y=" + String(ball_y) + ", my goal x=" +
                            String(my_goal_x) + ", y=" + String(my_goal_y) + ", ennemy goal x=" + String(enemy_goal_x) + ", y=" + String(enemy_goal_y));

        return CamInfos(Vector2(ball_x, ball_y), Vector2(0, 0), Vector2(0, 0),
                          Vector2(my_goal_x, my_goal_y), Vector2(enemy_goal_x, enemy_goal_y),
                          Vector2(0, 0), 0);

      } else {
        SerialDebug.println("Erreur lors de l'extraction des données de la caméra: " + String(lastCompleteSequence.c_str()));
      }

    } else {
      SerialDebug.println("Aucune séquence complète trouvée, reçu: " + String((char*)buffer));
    }
  }
  return CamInfos(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), 0);
}

int compteur;

void aloop() {
  motors.goTo(Vector2(0, -100), 100, 0);
  delay(1000);
  motors.goTo(Vector2(0, 100), 100, 0);
  delay(1000);

  if (compteur % 2 == 0) {
    digitalWrite(13, HIGH);
    compteur = 1;
  } else {
    digitalWrite(13, LOW);
    compteur = 0;
  }
}

FutureAction lastAction = FutureAction(Vector2(0, 0), 0, 0, false);
FutureAction currentAction = FutureAction(Vector2(0, 0), 0, 0, false);

void loop() {
  unsigned long start_millis = millis();
  SerialDebug.println("***");

  // Faire clignoter la LED pour s'assurer que le code tourne correctement
  if (compteur % 2 == 0) {
    digitalWrite(13, HIGH);
    compteur = 1;
  } else {
    digitalWrite(13, LOW);
    compteur = 0;
  }

  // testsLidar(fieldProperties); // tests du LIDAR
  // return;

  // GETTING LIDAR DATA
  LidarInfos lidarInfos = getLidarInfos(fieldProperties, true, false);
  SerialDebug.println("Coordonnées robot: x=" + String(lidarInfos.getCoordinates().x() / 10.0) + " cm, y=" + String(lidarInfos.getCoordinates().y() / 10.0) + " cm, orientation: " + String(lidarInfos.getOrientation()) + "°, Nearest Wall distance=" + String(lidarInfos.getNearestWall().distance({0, 0}) / 10.0) + " cm");

  // GETTING CAM DATA
  CamInfos camInfos = getCamInfos();

  double orientation = lidarInfos.getOrientation();
  if (lidarInfos.getOrientationRadians() == -9999) {
    orientation = 0;
  }

  if (camInfos.enemyGoalPos().y() < 50 && camInfos.enemyGoalPos().y() != 0) {
    if (camInfos.enemyGoalPos().x() > 0) {
      orientation = -180;
    } else {
      orientation = 180;
    }
  } else if (camInfos.myGoalPos().y() > 50 && camInfos.myGoalPos().y() != 0) {
    if (camInfos.myGoalPos().x() > 0) {
      orientation = 180;
    } else {
      orientation = -180;
    }
  }

  CamInfos currentState = CamInfos(
      camInfos.ballPos(),
      Vector2(lidarInfos.getCoordinates().x() / 10, lidarInfos.getCoordinates().y() / 10),
      Vector2(0, 0),
      camInfos.myGoalPos(),
      camInfos.enemyGoalPos(),
      Vector2(lidarInfos.getNearestWall().x() / 10, lidarInfos.getNearestWall().y() / 10),
      Radians(Degree(orientation)));

  // DOING ACTION
  // TODO: must work without lidar data or without cam data
  FutureAction lastAction = currentAction;
  FutureAction currentAction = chooseStrategy(fieldProperties, currentState, lastAction);

  motors.goTo(currentAction.target(), currentAction.celerity(), currentAction.rotation());
  SerialDebug.println("Direction : " + currentAction.target().toString() + " Vitesse : " + String(currentAction.celerity()) + " Rotation : " + String(currentAction.rotation()));

  if (currentAction.activeKicker()) {
    // TODO active kicker
  }

  unsigned long elapsed = millis() - start_millis;
  SerialDebug.println("Temps loop : " + String(elapsed) + "ms");
}