#include <string>

#include "utilities.h"
#include "logger.h"
#include "lidar.h"
#include "lidar_analyzer.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"

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

RobotState currentState = RobotState(
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0));

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);

  SerialCam.setTimeout(10);
  SerialLidar.setTimeout(10);

  setupLog(NoteLevel);
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

RobotState getCamInfos() {
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

        return RobotState(Vector2(ball_x, ball_y), Vector2(0, 0), Vector2(0, 0),
                          Vector2(my_goal_x, my_goal_y), Vector2(enemy_goal_x, enemy_goal_y));

      } else {
        SerialDebug.println("Erreur lors de l'extraction des données de la caméra: " + String(lastCompleteSequence.c_str()));
      }

    } else {
      SerialDebug.println("Aucune séquence complète trouvée, reçu: " + String((char*)buffer));
    }
  }
  return RobotState(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), Vector2(0, 0), Vector2(0, 0));
}

void loop() {
  unsigned long start_millis = millis();
  SerialDebug.println("***");

  // testsLidar(fieldProperties); // tests du LIDAR
  // LidarInfos lidarInfos = getLidarInfos(fieldProperties, true, false);
  // SerialDebug.println("Coordonnées robot: x=" + String(lidarInfos.getCoordinates().x() / 10.0) + " cm, y=" + String(lidarInfos.getCoordinates().y() / 10.0) + " cm, orientation: " + String(lidarInfos.getOrientation()) + "°, Nearest Wall distance=" + String(lidarInfos.getNearestWall().distance({0, 0}) / 10.0) + " cm");

  RobotState camInfos = getCamInfos();

  // GETTING CAM DATA
  /*while (true) {
    if (SerialCam.available()) {
      char newChar = SerialCam.read();
      if (currentState.updateFromString(readingData, newChar)) {
        break;
      }
    } else {
      break;
    }
  }*/

  int speedMotors = 80;
  if (camInfos.ballPos().x() == 0 && camInfos.ballPos().y() == 0) {
    motors.fullStop();
  } else {
    //motors.goTo(Vector2(camInfos.ballPos().x(), camInfos.ballPos().y()), speedMotors, lidarInfos.getOrientation());
  }

  // DOING ACTION
  // TODO: must work without lidar data or without cam data
  /*FutureAction action = chooseStrategy(fieldProperties, currentState);
  if (action.changeMove()) {
    // TODO: FutureAction must include rotation and speed !
    motors.goTo(action.target(), 255, currentState.enemyGoalPos().angle());
  }
  if (action.activeKicker()) {
    // TODO active kicker
  }*/

  unsigned long elapsed = millis() - start_millis;
  SerialDebug.println("Temps loop : " + String(elapsed) + "ms");
}
