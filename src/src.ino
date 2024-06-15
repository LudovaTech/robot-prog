#include <string>

#include "cam_reader.hpp"
#include "lidar_analyzer_anc.hpp"
#include "lidar_reader.hpp"
#include "movements.hpp"
#include "strategy.hpp"
#include "utilities.hpp"

const FieldProperties fieldProperties = FieldProperties(
    243,  // fieldLength
    182,  // fieldWidth
    12,   // spaceBeforeLineSide
    60,   // goalWidth
    115,  // distanceYGoalFromCenter
    9,    // robotRadius
    2     // ballRadius
);

const Motors motors = Motors(
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

// TODO: temporaire
struct CamInfosGlue {
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
};

CamInfosGlue getCamInfos() {
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
        return CamInfosGlue{
            BallPos(ball_x, ball_y),
            MyGoalPos(my_goal_x, my_goal_y),
            EnemyGoalPos(enemy_goal_x, enemy_goal_y)};
      } else {
        SerialDebug.println("Erreur lors de l'extraction des données de la caméra: " + String(lastCompleteSequence.c_str()));
      }

    } else {
      SerialDebug.println("Aucune séquence complète trouvée, reçu: " + String((char*)buffer));
    }
  }
  return CamInfosGlue();
}

bool ledCounter = true;

//TODO: temporary
MutableVector2 previousTarget;

void loop() {
  unsigned long start_millis = millis();
  SerialDebug.println("***");

  // Flash the LED to make sure the code is running correctly
  if (ledCounter) {
    digitalWrite(13, HIGH);
    ledCounter = false;
  } else {
    digitalWrite(13, LOW);
    ledCounter = true;
  }

  // GETTING LIDAR DATA
  LidarInfosGlue lidarInfos = getLidarInfos(fieldProperties, true, false);
  String full_log;
  if (lidarInfos.oLDI.hasValue()) {
    full_log += "Coordonnées robot: x=" + String(lidarInfos.oLDI.value().coordinates().x() / 10.0) + " cm, y=" + String(lidarInfos.oLDI.value().coordinates().y() / 10.0) + " cm, orientation: " + String(lidarInfos.oLDI.value().orientation()) + " rad, ";
  } else {
    full_log += "Coordonnées robot: x= not found, y= not found, orientation: not found, ";
  }
  if (lidarInfos.oLBI.hasValue()) {
    full_log += "Nearest Wall distance=" + String(lidarInfos.oLBI.value().distance(Vector2(0, 0)) / 10.0) + " cm";
  } else {
    full_log += "Nearest Wall distance= not found";
  }
  SerialDebug.println(full_log);

  // GETTING CAM DATA
  CamInfosGlue camInfos = getCamInfos();

  // calculating the orientation of the robot

  double orientation = 0;
  if (lidarInfos.oLDI.hasValue()) {
    orientation = lidarInfos.oLDI.value().orientation();
  }

  if (camInfos.enemyGoalPos.hasValue()) {
    if (camInfos.enemyGoalPos.value().y() < 50 && camInfos.enemyGoalPos.value().y() != 0) {
      if (camInfos.enemyGoalPos.value().x() > 0) {
        orientation = -180;
      } else {
        orientation = 180;
      }
    }
  }
  if (camInfos.myGoalPos.hasValue()) {
    if (camInfos.myGoalPos.value().y() > 50 && camInfos.myGoalPos.value().y() != 0) {
      if (camInfos.myGoalPos.value().x() > 0) {
        orientation = 180;
      } else {
        orientation = -180;
      }
    }
  }

  // DOING ACTION
  FutureAction currentAction = chooseStrategy(
      fieldProperties,
      lidarInfos.oLDI,
      lidarInfos.oLBI,
      camInfos.ballPos,
      camInfos.myGoalPos,
      camInfos.enemyGoalPos);

  if (currentAction.changeTarget()) {
    motors.goTo(currentAction.target(), currentAction.celerity(), orientation);
    previousTarget = currentAction.target();
  } else {
    motors.goTo(previousTarget.toVector2(), currentAction.celerity(), orientation);
  }
  SerialDebug.println("Direction : " + currentAction.target().toString() + " Vitesse : " + String(currentAction.celerity()) + " Rotation : " + String(currentAction.rotation()));

  if (currentAction.activeKicker()) {
    // TODO active kicker
  }

  unsigned long elapsed = millis() - start_millis;
  SerialDebug.println("Temps loop : " + String(elapsed) + "ms");
}