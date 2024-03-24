#include "lidar.h"

//////LIDARPOINT

LidarPoint::LidarPoint(uint16_t distance, uint8_t intensity, uint16_t angle)
    : _distance(distance), _intensity(intensity), _angle(angle) {}

//////MUTABLELIDARPOINT

MutableLidarPoint::MutableLidarPoint(LidarPoint lidarPoint)
    : _distance(lidarPoint.distance()),
      _intensity(lidarPoint.intensity()),
      _angle(lidarPoint.angle()) {}

MutableLidarPoint::MutableLidarPoint()
    : _distance(0), _intensity(0), _angle(0) {}

LidarPoint MutableLidarPoint::toLidarPoint() const {
  return LidarPoint(distance(), intensity(), angle());
}

//////CIRCULARLIDARPOINTSBUFFER

CircularLidarPointsBuffer::CircularLidarPointsBuffer(int bufferSize)
    : _size(bufferSize) {
  _buffer = new MutableLidarPoint[bufferSize];
}

CircularLidarPointsBuffer::~CircularLidarPointsBuffer() {
  delete[] _buffer;
}

void CircularLidarPointsBuffer::addValue(const LidarPoint newValue) {
  _buffer[_index] = MutableLidarPoint(newValue);
  if (_firstRound) {
    if (_index == _size - 1) {
      _firstRound = false;
    }
  }
  _index = (_index + 1) % _size;
}

bool CircularLidarPointsBuffer::existValue(size_t indice) const {
  if (_firstRound) {
    return indice < _index && indice >= 0;
  } else {
    return indice < _size && indice >= 0;
  }
}

LidarPoint CircularLidarPointsBuffer::getValue(size_t indice) const {
  // Be sure to check that the value exists with `existValue` before requesting it.
  return _buffer[(indice) % _size].toLidarPoint();
}

size_t CircularLidarPointsBuffer::sizeFilled() const {
  if (_firstRound) {
    return _index;
  } else {
    return _size;
  }
}

void CircularLidarPointsBuffer::flush() {
  _index = 0;
  _firstRound = true;
  delete[] _buffer;
  _buffer = new MutableLidarPoint[_size];
}

//////FUNCTIONS

uint8_t _calCRC8FromBuffer(uint8_t *p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;                                       // pre-calculated header and verlen values (crc = crcTable[(crc ^ 0x54) & 0xff];crc = crcTable[(crc ^ 0x2C) & 0xff];)
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {  // ignores the last value of the p array (which contains the crc check value)
    crc = crcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
  return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  if (startAngle <= endAngle) {
    return (endAngle - startAngle) / lenMinusOne;
  } else {
    return (36000 + endAngle - startAngle) / lenMinusOne;
  }
}

uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return (startAngle + (step * indice)) % 36000;
}

void savePointsLocal(uint16_t startAngle, uint16_t endAngle, LidarPoint *data) {
  // TODO modifier utilisant circularLidarPointsBuffer
  uint16_t step = angleStep(startAngle, endAngle);
  for (unsigned int i = 0; i < 12; i++) {
    uint16_t angle = angleFromStep(startAngle, step, i);
    SerialDebug.print(",(");
    SerialDebug.print(angle);
    SerialDebug.print(",");
    SerialDebug.print(data[i].distance());
    SerialDebug.print(")");
  }
}

void readPointsAndAddToBuffer(CircularLidarPointsBuffer& pointsBuffer) {
  if (!SerialLidar.find("T,")) {  // equivalent en char de 84 44 (decimal)
    SerialDebug.println("error, no header-verlen found in RX for the lidar LD19");
  } else {
    // The previous instruction (find) jumped to the beginning of the information
    // Now the stream is aligned
    byte buffer[45];
    size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
    if (nbrBytesReceived != 45) {
      SerialDebug.println("error, wrong number of bytes received (" + String(nbrBytesReceived) + ")");
    } else {
      uint16_t speed = _get2BytesLsbMsb(buffer, 0);
      uint16_t startAngle = _get2BytesLsbMsb(buffer, 2);

      LidarPoint data[] = {// no for loop possible due to 'const' in LidarPoint class
                           LidarPoint(_get2BytesLsbMsb(buffer, 4), buffer[6], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 7), buffer[9], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 10), buffer[12], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 13), buffer[15], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 16), buffer[18], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 19), buffer[21], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 22), buffer[24], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 25), buffer[27], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 28), buffer[30], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 31), buffer[33], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 34), buffer[36], 0),
                           LidarPoint(_get2BytesLsbMsb(buffer, 37), buffer[39], 0)};

      uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
      uint16_t timestamp = _get2BytesLsbMsb(buffer, 42);
      uint8_t crcCheck = buffer[44];

      if (_calCRC8FromBuffer(buffer, 44) == crcCheck) {
        uint16_t step = angleStep(startAngle, endAngle);
        for (unsigned int i = 0; i < 12; i++) {
          pointsBuffer.addValue(
              LidarPoint(
                  data[i].distance(),
                  data[i].intensity(),
                  angleFromStep(startAngle, step, i)));
        }
      }
    }
  }
}