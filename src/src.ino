#include <string>

#include "cam_reader.hpp"
#include "lidar_analyzer_anc.hpp"
#include "lidar_reader.hpp"
#include "logger.hpp"
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
    2,    // ballRadius
    255  // maxDribblerSpeed
);

const Motors motors(
    // Teensy
    MotorMov(15, 14, 18, Degree(-40)),
    MotorMov(36, 33, 37, Degree(40)),
    MotorMov(22, 19, 23, Degree(-140)),
    MotorMov(11, 12, 10, Degree(140)));

DribblerKicker dribblerKicker(
  MotorMov(6, 9, 5, 0),
  27,
  28
);

// TODO: temporary
struct CamInfosGlue {
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
};

uint8_t bigserialbuffer[4000];
uint8_t bigserialbufferlidar[10000];
const int pinLED = 13;
const int pinSwitch = 26;

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
  SerialBlue.begin(115200);
  setupLog(DebugLevel, 25);

  SerialCam.addMemoryForRead(&bigserialbuffer, sizeof(bigserialbuffer));
  SerialLidar.addMemoryForRead(&bigserialbufferlidar, sizeof(bigserialbufferlidar));

  SerialCam.setTimeout(10);
  SerialLidar.setTimeout(2); // !!! 2 !!!

  pinMode(pinLED, OUTPUT);
  pinMode(pinSwitch, INPUT);
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

int previousMyGoalX, previousMyGoalY, previousEnemyGoalX, previousEnemyGoalY, timePreviousMyGoal, timePreviousEnemyGoal; 

CamInfosGlue getCamInfos(Radians angleFrontGoalLidar, Radians angleRearGoalLidar) {
  size_t bytesAvailable = SerialCam.available();

  if (millis() - timePreviousMyGoal > 100) {
    previousMyGoalX = 0;
    previousMyGoalY = 0;
  } 
  if (millis() - timePreviousEnemyGoal > 100) {
    previousEnemyGoalX = 0; 
    previousEnemyGoalY = 0;
  }

  if (bytesAvailable >= 26) {
    size_t nbrBytesReceived = SerialCam.readBytes(bigserialbuffer, min(bytesAvailable, sizeof(bigserialbuffer) - 1));
    bigserialbuffer[nbrBytesReceived] = '\0';

    std::string lastCompleteSequence = extractLastCompleteSequence((char*)bigserialbuffer);
    if (!lastCompleteSequence.empty()) {
      // exemple : b+048+019+006+065-045+027+000+000+090+015+065+070+066-059e

      int ballX, ballY, myGoalsY[3], myGoalsX[3], enemyGoalsX[3], enemyGoalsY[3];

      if (sscanf(lastCompleteSequence.c_str(), "b%d%d%d%d%d%d%d%d%d%d%d%d%d%de",
        &ballX, &ballY, &myGoalsX[0], &myGoalsY[0], &myGoalsX[1], &myGoalsY[1], &myGoalsX[2], &myGoalsY[3],
        &enemyGoalsX[0], &enemyGoalsY[0], &enemyGoalsX[1], &enemyGoalsY[1], &enemyGoalsX[2], &enemyGoalsY[2]) == 14) {
        
        Radians angleMargin = 0.3;
        if(angleFrontGoalLidar == 999 || angleRearGoalLidar == 999) {
          angleMargin = 10;
        } 

        int myGoalX, myGoalY, enemyGoalX, enemyGoalY;

        Radians angleMyGoalsCam[3] = {Vector2(myGoalsX[0], myGoalsY[0]).angle(), 
                                      Vector2(myGoalsX[1], myGoalsY[1]).angle(),
                                      Vector2(myGoalsX[2], myGoalsY[2]).angle()};

        Radians angleEnemyGoalsCam[3] = {Vector2(enemyGoalsX[0], enemyGoalsY[0]).angle(), 
                                      Vector2(enemyGoalsX[1], enemyGoalsY[1]).angle(),
                                      Vector2(enemyGoalsX[2], enemyGoalsY[2]).angle()};

        for (int i=2; i>=0; i--) {
          if (abs(angleMyGoalsCam[i] - angleFrontGoalLidar) < angleMargin || 
              abs(angleMyGoalsCam[i] - angleRearGoalLidar) < angleMargin) {
            previousMyGoalX = myGoalsX[i];
            myGoalX = myGoalsX[i];
            previousMyGoalY = myGoalsY[i];
            myGoalY = myGoalsY[i];
            timePreviousMyGoal = millis();
          } else {
            myGoalX = previousMyGoalX;
            myGoalY = previousMyGoalY;
          }
        
          if (abs(angleEnemyGoalsCam[i] - angleFrontGoalLidar) < angleMargin || 
              abs(angleEnemyGoalsCam[i] - angleRearGoalLidar) < angleMargin) {
            previousMyGoalY = enemyGoalsX[i];
            enemyGoalX = enemyGoalsX[i];
            previousMyGoalY = enemyGoalsY[i];
            enemyGoalY = enemyGoalsY[i];
          } else {
            enemyGoalX = previousEnemyGoalX;
            enemyGoalY = previousEnemyGoalY;
            timePreviousEnemyGoal = millis();
          }
        }

        log_a(InfoLevel, "src.getCamInfos", "position-ball: x=" + String(ballX) + ", y=" + String(ballY) + ", my-goal x=" + String(myGoalX) + ", y=" + String(myGoalY) + ", enemy-goal x=" + String(enemyGoalX) + ", y=" + String(enemyGoalY));
        
        Optional<BallPos> bP;
        if (ballX != 0 && ballY != 0) {
          bP = BallPos(ballX, ballY);
        }
        Optional<MyGoalPos> mGP;
        if (myGoalX != 0 && myGoalY != 0) {
          mGP = MyGoalPos(myGoalX, myGoalY);
        }
        Optional<EnemyGoalPos> eGP;
        if (enemyGoalX != 0 && enemyGoalY != 0) {
          eGP = EnemyGoalPos(enemyGoalX, enemyGoalY);
        }
        CamInfosGlue cIG{
            bP,
            mGP,
            eGP};

        return cIG;
      } else {
        log_a(ErrorLevel, "src.getCamInfos", "Erreur lors de l'extraction des donnees de la camera: " + String(lastCompleteSequence.c_str()));
      }

    } else {
      log_a(ErrorLevel, "src.getCamInfos", "Aucune séquence complète trouvée, reçu: " + String((char*)bigserialbuffer));
    }
  }
  CamInfosGlue cIG{BallPos(0, 0), 
                  MyGoalPos(previousMyGoalX, previousMyGoalY), 
                  EnemyGoalPos(previousEnemyGoalX, previousEnemyGoalY)};
  return cIG;
}

bool ledCounter = true;

// TODO: temporary
MutableVector2 previousTarget;
Optional<LidarInfosGlue> previousLidarInfosGlue;

void loop() {
  if (SerialBlue.available()) {
    char incoming = SerialBlue.read();
    SerialDebug.print("OK");
  }
}

void bloop() {
  digitalWrite(27, HIGH);
  digitalWrite(28, HIGH);
  delay(40);
  digitalWrite(27, LOW);
  digitalWrite(28, LOW);
  delay(2000);
  if (ledCounter) {
    digitalWrite(pinLED, HIGH);
    ledCounter = false;
  } else {
    digitalWrite(pinLED, LOW);
    ledCounter = true;
  }
}

void aloop() {
  unsigned long start_millis = millis();
  log_a(InfoLevel, "src.loop", "***");

  // Flash the LED to make sure the code is running correctly
  if (ledCounter) {
    digitalWrite(pinLED, HIGH);
    ledCounter = false;
  } else {
    digitalWrite(pinLED, LOW);
    ledCounter = true;
  }

  // GETTING LIDAR DATA
  LidarInfosGlue lidarInfos;
  if (SerialLidar.available() < 2600 && previousLidarInfosGlue.hasValue()) {
    lidarInfos = previousLidarInfosGlue.value();
  } else {
    lidarInfos = getLidarInfos(fieldProperties, true, false);
    previousLidarInfosGlue = lidarInfos;
  }

  String full_log;
  if (lidarInfos.oLDI.hasValue()) {
    full_log += "robot-position: x=" + String(lidarInfos.oLDI.value().coordinates().x()) + " cm, y=" + String(lidarInfos.oLDI.value().coordinates().y()) + " cm, orientation: " + String(lidarInfos.oLDI.value().orientation()) + " rad, ";
  } else {
    full_log += "robot-position: x= not found, y= not found, orientation: not found, ";
  }
  if (lidarInfos.oLBI.hasValue()) {
    full_log += "nearest-wall-distance=" + String(lidarInfos.oLBI.value().distance(Vector2(0, 0))) + " cm";
  } else {
    full_log += "nearest-wall-distance= not found";
  }
  log_a(StratLevel, "src.loop", full_log);

  Radians angleFrontGoalLidar = 999; 
  Radians angleRearGoalLidar = 999;
  if (lidarInfos.oLDI.hasValue()) {
    angleFrontGoalLidar = Vector2(lidarInfos.oLDI.value().frontGoalCoordinates().x(), lidarInfos.oLDI.value().frontGoalCoordinates().y()).angle();
    angleRearGoalLidar = Vector2(lidarInfos.oLDI.value().rearGoalCoordinates().x(), lidarInfos.oLDI.value().rearGoalCoordinates().y()).angle();
    //SerialDebug.println(angleFrontGoalLidar);
  }

  // GETTING CAM DATA
  CamInfosGlue camInfos = getCamInfos(angleFrontGoalLidar, angleRearGoalLidar);

  // calculating the orientation of the robot
  Radians orientation = 0;
  if (lidarInfos.oLDI.hasValue()) {
    orientation = lidarInfos.oLDI.value().orientation();
  }

  if (camInfos.enemyGoalPos.hasValue()) {
    if (camInfos.enemyGoalPos.value().y() < 0) {
      orientation = -abs(camInfos.enemyGoalPos.value().x())/camInfos.enemyGoalPos.value().x() * PI/2;
    }
  }

  if (camInfos.myGoalPos.hasValue()) {
    if (camInfos.myGoalPos.value().y() > 0) {
      orientation = abs(camInfos.myGoalPos.value().x())/camInfos.myGoalPos.value().x() * PI/2;
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
    motors.goTo(currentAction.target(), currentAction.celerity(), orientation - currentAction.targetOrientation());
    previousTarget = currentAction.target();
  } else {
    motors.goTo(previousTarget.toVector2(), currentAction.celerity(), orientation - currentAction.targetOrientation());
  }

  String full_log2;
  if (currentAction.changeTarget()) {
    full_log2 += "Target : " + currentAction.target().toString() + " ";
  } else {
    full_log2 += "Target : Unchanged (" + previousTarget.toVector2().toString() + ") ";
  }
  full_log2 += "Vitesse : " + String(currentAction.celerity()) + " Rotation : " + String(orientation);
  log_a(InfoLevel, "src.loop", full_log2);

  if (currentAction.activeKicker()) {
    dribblerKicker.kick();
  }
  dribblerKicker.dribble(currentAction.celerityDribbler());

  unsigned long elapsed = millis() - start_millis;
  log_a(InfoLevel, "src.loop", "Temps loop : " + String(elapsed) + "ms");
}