#include "cam_reader.hpp"

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
  if (bytesAvailable >= 60*2) {//Comme ça on a au moins une sequence complete
    //size_t nbrBytesReceived = SerialCam.readBytes(camSerialBuffer, min(bytesAvailable, sizeof(camSerialBuffer) - 1));
    //camSerialBuffer[nbrBytesReceived] = '\0';

    //std::string lastCompleteSequence = extractLastCompleteSequence((char*)camSerialBuffer);
    String data;
    for (unsigned int i=0; i < bytesAvailable; i++) {
      int receive = SerialCam.read();
      if (receive == -1) {
        log_a(CriticalLevel, "getCamInfos", "strange, -1");
      }
      data += (char) receive;
    }
    std::string lastCompleteSequence = extractLastCompleteSequence(data.c_str());
    if (!lastCompleteSequence.empty()) {
      // exemple : b+048+019+006+065-045+027+000+000+090+015+065+070+066-059e

      int ballX, ballY, myGoalsY[3], myGoalsX[3], enemyGoalsX[3], enemyGoalsY[3];

      if (sscanf(lastCompleteSequence.c_str(), "b%d%d%d%d%d%d%d%d%d%d%d%d%d%de",
        &ballX, &ballY, &myGoalsX[0], &myGoalsY[0], &myGoalsX[1], &myGoalsY[1], &myGoalsX[2], &myGoalsY[2],
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
      log_a(ErrorLevel, "src.getCamInfos", "Aucune séquence complète trouvée, reçu: " + data);
    }
  }
  log_a(InfoLevel, "src.getCamInfos", "position-ball: x=0, y=0, my-goal x=0, y=0, enemy-goal x=0, y=0");
  CamInfosGlue cIG{BallPos(0, 0), 
                  MyGoalPos(previousMyGoalX, previousMyGoalY), 
                  EnemyGoalPos(previousEnemyGoalX, previousEnemyGoalY)};
  return cIG;
}

String extractLastCompleteSequence(String str) {
  int lastE = str.lastIndexOf('e');
  if (lastE != -1) {
    int prevB = str.lastIndexOf('b', lastE);
    if (prevB != -1) {
      return str.substring(prevB, lastE + 1);
    }
  }
  return "";
}