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

uint8_t camSerialBuffer[4000];
uint8_t bigserialbufferlidar[10000];
const int pinLED = 13;
const int pinSwitch = 26;

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);
  SerialBlue.begin(115200);
  setupLog(ErrorLevel, 25);

  SerialCam.addMemoryForRead(&camSerialBuffer, sizeof(camSerialBuffer));
  SerialLidar.addMemoryForRead(&bigserialbufferlidar, sizeof(bigserialbufferlidar));

  SerialCam.setTimeout(10);
  SerialLidar.setTimeout(2); // !!! 2 !!!

  pinMode(pinLED, OUTPUT);
  pinMode(pinSwitch, INPUT);
}

bool ledCounter = true;

// TODO: temporary
MutableVector2 previousTarget;
Optional<LidarInfosGlue> previousLidarInfosGlue;
Optional<CamInfosGlue> previousCamInfosGlue;

void aloop() {
  if (SerialBlue.available()) {
    char incoming = SerialBlue.read();
    SerialDebug.print("OK");
  }
}

void loop() {
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
  }

  // GETTING CAM DATA
  CamInfosGlue camInfos;
  if (SerialCam.available() < 57 && previousCamInfosGlue.hasValue()) {
    camInfos = previousCamInfosGlue.value();
  } else {
    camInfos = getCamInfos(angleFrontGoalLidar, angleRearGoalLidar);
    previousCamInfosGlue = camInfos;
  }

  // calculating the orientation of the robot
  Radians orientation = 0;
  if (lidarInfos.oLDI.hasValue()) {
    orientation = lidarInfos.oLDI.value().orientation();
  }

  // Slowing down when approaching a wall
  float speedReductionRatio = 1;
  int minimumVelocityRatio = 0.5;
  int slowingDownWallDistance = 40;
  if (lidarInfos.oLBI.hasValue()) {
    speedReductionRatio = minimumVelocityRatio*(lidarInfos.oLBI.value().distance(Vector2(0, 0))/slowingDownWallDistance + 1);
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
    motors.goTo(currentAction.target(), currentAction.celerity()*speedReductionRatio, orientation - currentAction.targetOrientation());
    previousTarget = currentAction.target();
  } else {
    motors.goTo(previousTarget.toVector2(), currentAction.celerity()*speedReductionRatio, orientation - currentAction.targetOrientation());
  }

  String full_log2;
  if (currentAction.changeTarget()) {
    full_log2 += "Target : " + currentAction.target().toString() + " ";
  } else {
    full_log2 += "Target : Unchanged (" + previousTarget.toVector2().toString() + ") ";
  }
  full_log2 += "Vitesse : " + String(currentAction.celerity()*speedReductionRatio) + " Rotation : " + String(orientation);
  log_a(InfoLevel, "src.loop", full_log2);

  if (currentAction.activeKicker()) {
    dribblerKicker.kick();
  }
  dribblerKicker.dribble(currentAction.celerityDribbler());

  unsigned long elapsed = millis() - start_millis;
  log_a(InfoLevel, "src.loop", "Temps loop : " + String(elapsed) + "ms");
  delay(2);
}