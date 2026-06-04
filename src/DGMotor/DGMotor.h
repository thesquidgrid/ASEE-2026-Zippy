#ifndef DGMotor_h
#define DGMotor_h

#include <Arduino.h>
#include <HardwareSerial.h>

class DGMotor {
public:
  // Constructor: takes a HardwareSerial reference and the motor ID (1 or 2)
  DGMotor(HardwareSerial& serial, uint8_t motorID);

  // Initialize the serial communication
  void begin(unsigned long baud = 115200);

  // Set motor ID (sends 0xAA 0x55 0x53 ID; repeated 6 times as in reference)
  void setMotorID(uint8_t newID);

  // Set operation modes
  void setVelocityMode();
  void setPositionMode();

  // Control commands
  void setMotorSpeed(int16_t rpm);
  void setMotorDegrees(int16_t deg);

  // Query/Utility
  void requestMotorStatus();
  void requestMotorMode();
  void brake(); // Force stop using 0x64 0x00 0x00 ... with CRC 0x50

  // Helper getters for internal state (optional, mainly for debugging)
  uint8_t getCurrentMotorID() const { return _motorID; }

private:
  HardwareSerial& _serial;
  uint8_t _motorID;

  // Internal methods
  void calculateAndSend(uint8_t* packet);
};

#endif // DGMotor_h
