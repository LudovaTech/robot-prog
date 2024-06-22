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
    2     // ballRadius
);

const Motors motors = Motors(
    // Teensy
    MotorMov(15, 14, 0, Degree(-40)),
    MotorMov(36, 33, 0, Degree(40)),
    MotorMov(22, 19, 0, Degree(-140)),
    MotorMov(11, 12, 0, Degree(140)));

// TODO: temporary
struct CamInfosGlue {
  Optional<BallPos> ballPos;
  Optional<MyGoalPos> myGoalPos;
  Optional<EnemyGoalPos> enemyGoalPos;
};

uint8_t bigserialbuffer[4000];
// uint8_t bigserialbufferlidar[4000];

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
  setupLog(DebugLevel, 25);

  SerialCam.addMemoryForRead(&bigserialbuffer, sizeof(bigserialbuffer));
  // SerialLidar.addMemoryForRead(&bigserialbufferlidar, sizeof(bigserialbufferlidar));

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

CamInfosGlue getCamInfos() {
  size_t bytesAvailable = SerialCam.available();
  // SerialDebug.println("nb of bytes available: " + String(bytesAvailable));

  if (bytesAvailable >= 26) {
    // byte buffer[301];
    size_t nbrBytesReceived = SerialCam.readBytes(bigserialbuffer, min(bytesAvailable, sizeof(bigserialbuffer) - 1));
    bigserialbuffer[nbrBytesReceived] = '\0';
    // SerialDebug.println(" reçu: " + String((char*)bigserialbuffer));

    std::string lastCompleteSequence = extractLastCompleteSequence((char*)bigserialbuffer);
    if (!lastCompleteSequence.empty()) {
      // exemple : b+048+019+006+065-045+027+000+000+090+015+065+070+066-059e

      int ballX, ballY,
          myGoalX1, myGoalY1, myGoalX2, myGoalY2, myGoalX3, myGoalY3,
          enemyGoalX1, enemyGoalY1, enemyGoalX2, enemyGoalY2, enemyGoalX3, enemyGoalY3;

      if (sscanf(lastCompleteSequence.c_str(), "b%d%d%d%d%d%d%d%d%d%d%d%d%d%de",
                 &ballX, &ballY, &myGoalX1, &myGoalY1, &myGoalX2, &myGoalY2, &myGoalX3, &myGoalY3,
                 &enemyGoalX1, &enemyGoalY1, &enemyGoalX2, &enemyGoalY2, &enemyGoalX3, &enemyGoalY3) == 14) {
        log_a(InfoLevel, "src.getCamInfos", "Position balle: x=" + String(ballX) + ", y=" + String(ballY) + ", my goal x=" + String(myGoalX1) + ", y=" + String(myGoalY1) + ", ennemy goal x=" + String(enemyGoalX1) + ", y=" + String(enemyGoalY1));
        Optional<BallPos> bP;
        if (ballX != 0 && ballY != 0) {
          bP = BallPos(ballX, ballY);
        }
        Optional<MyGoalPos> mGP;
        if (myGoalX1 != 0 && myGoalY1 != 0) {
          mGP = MyGoalPos(myGoalX1, myGoalY1);
        }
        Optional<EnemyGoalPos> eGP;
        if (enemyGoalX1 != 0 && enemyGoalY1 != 0) {
          eGP = EnemyGoalPos(enemyGoalX1, enemyGoalY1);
        }
        CamInfosGlue cIG{
            bP,
            mGP,
            eGP};

        return cIG;
      } else {
        log_a(ErrorLevel, "src.getCamInfos", "Erreur lors de l'extraction des données de la caméra: " + String(lastCompleteSequence.c_str()));
      }

    } else {
      log_a(ErrorLevel, "src.getCamInfos", "Aucune séquence complète trouvée, reçu: " + String((char*)bigserialbuffer));
    }
  }
  return CamInfosGlue();
}

bool ledCounter = true;

// TODO: temporary
MutableVector2 previousTarget;

void loop() {
  unsigned long start_millis = millis();
  log_a(InfoLevel, "src.loop", "***");

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
    full_log += "Coordonnees robot: x=" + String(lidarInfos.oLDI.value().coordinates().x()) + " cm, y=" + String(lidarInfos.oLDI.value().coordinates().y()) + " cm, orientation: " + String(lidarInfos.oLDI.value().orientation()) + " rad, ";
  } else {
    full_log += "Coordonnees robot: x= not found, y= not found, orientation: not found, ";
  }
  if (lidarInfos.oLBI.hasValue()) {
    full_log += "Nearest Wall distance=" + String(lidarInfos.oLBI.value().distance(Vector2(0, 0))) + " cm";
  } else {
    full_log += "Nearest Wall distance= not found";
  }
  log_a(StratLevel, "src.loop", full_log);

  // GETTING CAM DATA
  CamInfosGlue camInfos = getCamInfos();

  // calculating the orientation of the robot

  Radians orientation = 0;
  if (lidarInfos.oLDI.hasValue()) {
    orientation = lidarInfos.oLDI.value().orientation();
  }

  /*if (camInfos.enemyGoalPos.hasValue()) {
    if (camInfos.enemyGoalPos.value().y() < 50 && camInfos.enemyGoalPos.value().y() != 0) {
      if (camInfos.enemyGoalPos.value().x() > 0) {
        orientation = -PI/2;
      } else {
        orientation = PI/2;
      }
    }
  }
  if (camInfos.myGoalPos.hasValue()) {
    if (camInfos.myGoalPos.value().y() > 50 && camInfos.myGoalPos.value().y() != 0) {
      if (camInfos.myGoalPos.value().x() > 0) {
        orientation = PI/2;
      } else {
        orientation = -PI/2;
      }
    }
  }*/
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

  String full_log2;
  if (currentAction.changeTarget()) {
    full_log2 += "Target : " + currentAction.target().toString() + " ";
  } else {
    full_log2 += "Target : Unchanged (" + previousTarget.toVector2().toString() + ") ";
  }
  full_log2 += "Vitesse : " + String(currentAction.celerity()) + " Rotation : " + String(currentAction.rotation());
  log_a(InfoLevel, "src.loop", full_log2);

  if (currentAction.activeKicker()) {
    // TODO active kicker
  }

  unsigned long elapsed = millis() - start_millis;
  log_a(InfoLevel, "src.loop", "Temps loop : " + String(elapsed) + "ms");
}