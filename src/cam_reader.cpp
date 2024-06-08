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
    Vector2 ballPos,
    Vector2 myPos,
    Vector2 partnerPos,
    Vector2 myGoalPos,
    Vector2 enemyGoalPos,
    Vector2 nearestWall,
    double orientation)
    : _ballPos(ballPos),
      _myPos(myPos),
      _partnerPos(partnerPos),
      _myGoalPos(myGoalPos),
      _enemyGoalPos(enemyGoalPos),
      _nearestWall(nearestWall),
      _orientation(orientation) {}

bool CamInfos::updateFromString(ReadingData readingData, char newChar) {
  if (newChar == 'b' || newChar == 'm' || newChar == 'p' || newChar == 'g' || newChar == 'G' || newChar == 'w') {
    if (readingData.xReadingState() != "" && readingData.yReadingState() != "") {
      MutableVector2 newMutableVector2 = MutableVector2(Vector2(
          readingData.xReadingState().toFloat(),
          readingData.yReadingState().toFloat()));

      // SerialDebug.println("change to " + newMutableVector2.toString());
      switch (readingData.typeState()) {
        case 'b':
          _ballPos = newMutableVector2;
          break;
        case 'm':
          _myPos = newMutableVector2;
          break;
        case 'p':
          _partnerPos = newMutableVector2;
          break;
        case 'g':
          _myGoalPos = newMutableVector2;
          break;
        case 'G':
          _enemyGoalPos = newMutableVector2;
          break;
        case 'w':
          _nearestWall = newMutableVector2;
          break;
        default:
          SerialDebug.println("ERROR CATCHED CamInfos: unfinished data : '" + readingData.xReadingState() + " , " + readingData.yReadingState() + "'");
      }
      readingData.reinitWith(newChar);
      return true;
    } else if (!(readingData.typeState() == 'b' || readingData.typeState() == 'm' || readingData.typeState() == 'p' || readingData.typeState() == 'g' || readingData.typeState() == 'G' || readingData.typeState() == 'w')) {
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
  result += _ballPos.toString();
  result += " myPos: ";
  result += _myPos.toString();
  result += " partnerPos: ";
  result += _partnerPos.toString();
  result += " myGoalPos: ";
  result += _myGoalPos.toString();
  result += " enemyGoalPos: ";
  result += _enemyGoalPos.toString();
  result += " nearestWall: ";
  result += _nearestWall.toString();
  result += " orientation: ";
  result += _orientation;
  result += ")";
  return result;
}