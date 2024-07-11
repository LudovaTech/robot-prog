#include "blue_reader.hpp"

String readFromBlue(int bytesAvailable) {
  String data;
  for (unsigned int i = 0; i < bytesAvailable; i++) {
    int receive = SerialBlue.read();
    if (receive == -1) {
      log_a(CriticalLevel, "readFromBlue", "strange, -1");
    }
    data += static_cast<char>(receive);
  }
  return data;
}

String extractLastCompleteSequenceBlue(String str) {
  int lastE = str.lastIndexOf('e');
  if (lastE != -1) {
    int prevB = str.lastIndexOf('b', lastE);
    if (prevB != -1) {
      return str.substring(prevB, lastE + 1);
    }
  }
  return "";
}

bool sequenceToValuesBlue(
    String lastCompleteSequence,
    int *partnerX,
    int *partnerY,
    int *ballX,
    int *ballY) {
  return sscanf(lastCompleteSequence.c_str(), "b%d%d%d%de",
                partnerX, partnerY, ballX, ballY) == 4;
}

BlueInfosGlue getBlueInfos() {
  size_t bytesAvailable = SerialBlue.available();
  Optional<Vector2> partnerPos;
  Optional<Vector2> ballPos;
  if (bytesAvailable >= minBytesAvailableBlue) {
    String data = readFromBlue(bytesAvailable);
    String lastCompleteSequence = extractLastCompleteSequenceBlue(data);
    if (lastCompleteSequence != "") {
      int partnerX, partnerY, ballX, ballY;
      if (sequenceToValuesBlue(lastCompleteSequence, &partnerX, &partnerY, &ballX, &ballY)) {
        if (partnerX != 0 && partnerY != 0) {
          partnerPos = Vector2(partnerX, partnerY);
        }
        if (ballX != 0 && ballY != 0) {
          ballPos = Vector2(ballX, ballY);
        }
      }
    } else {
      log_a(ErrorLevel, "getBlueInfos", "derniere sequence vide");
    }
  }
  return BlueInfosGlue{
    partnerPos,
    ballPos
  };
}

void sendBlueData(Vector2 myPos, Vector2 ballPos) {
  SerialBlue.printf("b%+04d%+04d%+04d%+04de", (int)myPos.x(), (int)myPos.y(), (int)ballPos.x(), (int)ballPos.y());
}
