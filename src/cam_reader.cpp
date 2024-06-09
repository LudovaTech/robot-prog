#include "cam_reader.hpp"

ReadingData::ReadingData() {}

void ReadingData::nowWriteInYState() {
  _writingInXState = false;
}

void ReadingData::addToActiveReadingState(char newChar) {
  if (writingInXState()) {
    _xReadingState += newChar;
  } else {
    _yReadingState += newChar;
  }
}

void ReadingData::reinitWith(char newChar) {
  _typeState = newChar;
  _xReadingState = "";
  _yReadingState = "";
  _writingInXState = true;
}

String ReadingData::toString() const {
  String s = "(ReadingData, ";
  s += typeState();
  s += ", ";
  s += xReadingState();
  s += ", ";
  s += yReadingState();
  s += ", ";
  s += writingInXState();
  s += ")";
  return s;
}

CamInfos::CamInfos(
    Optional<Vector2> ballPos,
    Optional<Vector2> myGoalPos,
    Optional<Vector2> enemyGoalPos)
    : _ballPos(ballPos),
      _myGoalPos(myGoalPos),
      _enemyGoalPos(enemyGoalPos) {}

// TODO change with new CamInfos
bool CamInfos::updateFromString(ReadingData readingData, char newChar) {
  if (newChar == 'b' || newChar == 'g' || newChar == 'G') {
    if (readingData.xReadingState() != "" && readingData.yReadingState() != "") {
      MutableVector2 newMutableVector2 = MutableVector2(Vector2(
          readingData.xReadingState().toFloat(),
          readingData.yReadingState().toFloat()));

      // SerialDebug.println("change to " + newMutableVector2.toString());
      switch (readingData.typeState()) {
        case 'b':
          _ballPos = newMutableVector2;
          break;
        case 'g':
          _myGoalPos = newMutableVector2;
          break;
        case 'G':
          _enemyGoalPos = newMutableVector2;
          break;
        default:
          SerialDebug.println("ERROR CATCHED CamInfos: unfinished data : '" + readingData.xReadingState() + " , " + readingData.yReadingState() + "'");
      }
      readingData.reinitWith(newChar);
      return true;
    } else if (!(readingData.typeState() == 'b' || readingData.typeState() == 'g' || readingData.typeState() == 'G')) {
      SerialDebug.println("ERROR CATCHED CamInfos: no typeState tracked");
    } else if (isDigit(newChar) || newChar == '.' || newChar == '-') {
      readingData.addToActiveReadingState(newChar);
    } else if (newChar == ',') {
      if (readingData.writingInXState()) {
        readingData.nowWriteInYState();
      } else {
        SerialDebug.println("ERROR CATCHED CamInfos : several characters ','");
        readingData.reinitWith('x');
      }
    } else {
      SerialDebug.println("ERROR CATCHED CamInfos : unknown char (skipped) '" + String(int(newChar)) + "'");
    }
  }
  return false;
}

String CamInfos::toString() const {
  String result = "CamInfos (ballPos: ";
  if (_ballPos.hasValue()) {
    result += _ballPos.value().toString();
  } else {
    result += "None";
  }
  result += " myGoalPos: ";
  if (_myGoalPos.hasValue()) {
    result += _myGoalPos.value().toString();
  } else {
    result += "None";
  }
  result += " enemyGoalPos: ";
  if (_enemyGoalPos.hasValue()) {
    result += _enemyGoalPos.value().toString();
  } else {
    result += "None";
  }
  result += ")";
  return result;
}