

#include "DGMotor.h"

DGMotor::DGMotor(HardwareSerial& serial, uint8_t motorID)
  : _serial(serial), _motorID(motorID) {} 

void DGMotor::begin(unsigned long baud) {
  _serial.begin(baud); //start 
}

// send the specific ID-setting packet 6 times for ID change
void DGMotor::setMotorID(uint8_t newID) {
  uint8_t packet[10] = {
    0xAA,
    0x55,
    0x53,
    newID,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  for (int i = 0; i < 6; i++) {
    _serial.write(packet, 10);
  }

  _motorID = newID; // 
}

void DGMotor::setVelocityMode() {
  uint8_t packet[10] = {
    _motorID,
    0xA0,
    0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
}

void DGMotor::setPositionMode() {
  uint8_t packet[10] = {
    _motorID,
    0xA0,
    0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
}

void DGMotor::setMotorSpeed(int16_t rpm) {
  // speed val is RPM * 10
  int16_t speedValue = rpm * 10;
  uint8_t packet[10] = {
    _motorID,
    0x64,
    static_cast<uint8_t>((speedValue >> 8) & 0xFF),
    static_cast<uint8_t>(speedValue & 0xFF),
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
  
  
  
}

void DGMotor::setMotorDegrees(int16_t deg) {
  // map 0-360 degrees to 16-bit integer (0-32767) 
  float degVal_float = (float)32767 / (float)360 * (float)deg;
  uint16_t degVal_int = (uint16_t)ceil(degVal_float);

  uint8_t packet[10] = {
    _motorID,
    0x64,
    static_cast<uint8_t>((degVal_int >> 8) & 0xFF),
    static_cast<uint8_t>(degVal_int & 0xFF),
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
  Serial.println("Running Motor: "); 
}

void DGMotor::requestMotorStatus() {
  uint8_t packet[10] = {
    _motorID,
    0x74,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
}

void DGMotor::requestMotorMode() {
  uint8_t packet[10] = {
    _motorID,
    0x75,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  calculateAndSend(packet);
}

void DGMotor::brake() {
  uint8_t packet[10] = {
    _motorID,
    0x64,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50
  };
  _serial.write(packet, 10);
}

void DGMotor::calculateAndSend(uint8_t* packet) {
  uint8_t crc = 0x00;
  // CRC-8 (Maxim) polynomial: x^8 + x^5 + x^4 + 1 (0x8C representation for reflected)
  for (int i = 0; i < 9; i++) {
    crc ^= packet[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;
      } else {
        crc >>= 1;
      }
    }
  }
  packet[9] = crc;
  _serial.write(packet, 10);
}

