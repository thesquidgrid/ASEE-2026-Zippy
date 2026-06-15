# ASEE-2026-Zippy

**Team:** //  
**Board:** Teensy 4.1  
**Competition:** ASEE 2026  

---

## Table of Contents
1. [Pinout](#1-pinout)
2. [Function Overview](#2-function-overview)
   - [drivebase.h](#drivebaseh)
   - [intake.h](#intakeh)
   - [collisionSense.h](#collisionsensorh)
3. [Known Issues](#3-known-issues)
4. [Dependencies](#4-dependencies)
5. [Setup & Upload](#5-setup--upload)

---

## 1. Pinout

### Collision Sensing (`collisionSense.h`)
| Pin | Signal | Type              | Connected To      |
| --- | ------ | ----------------- | ----------------- |
| 33  | sw1    | Digital IN PULLUP | Front bump switch |
| 34  | sw2    | Digital IN PULLUP | Rear bump switch  |

---

### Drive Motors (`drivebase.h` / `DGMotor.h`)

Each DGMotor controller communicates over a dedicated UART serial port (one per wheel).

| Pin | Serial Port | Signal  | Function             | Connected To       |
| --- | ----------- | ------- | -------------------- | ------------------ |
| 0   | Serial1 RX  | UART RX | Front-Left motor RX  | leftFront DGMotor  |
| 1   | Serial1 TX  | UART TX | Front-Left motor TX  | leftFront DGMotor  |
| 7   | Serial2 RX  | UART RX | Front-Right motor RX | rightFront DGMotor |
| 8   | Serial2 TX  | UART TX | Front-Right motor TX | rightFront DGMotor |
| 24  | Serial6 TX  | UART TX | Rear-Left motor TX   | leftRear DGMotor   |
| 25  | Serial6 RX  | UART RX | Rear-Left motor RX   | leftRear DGMotor   |
| 28  | Serial7 RX  | UART RX | Rear-Right motor RX  | rightRear DGMotor  |
| 29  | Serial7 TX  | UART TX | Rear-Right motor TX  | rightRear DGMotor  |

---

### Intake (`intake.h`)
| Pin | Signal | Type        | Connected To              |
| --- | ------ | ----------- | ------------------------- |
| 14  | input1 | Digital OUT | H-bridge IN1              |
| 15  | input2 | Digital OUT | H-bridge IN2              |
| 36  | enable | Digital OUT | H-bridge EN (always HIGH) |

---

### Deposit (`deposit.h`)

#### Color Sensor
| Pin | I2C Port  | Connected To |
| --- | --------- | ------------ |
| 16  | WIRE1 SDA | Color sensor |
| 17  | WIRE1 SCL | Color sensor |

#### Servo Motor
| Pin | Signal | Type | Connected To |
| --- | ------ | ---- | ------------ |
| 12  | servo  | PWM  | Servo signal |

---

### Gyroscope
| Pin | I2C Port | Connected To |
| --- | -------- | ------------ |
| 18  | WIRE SDA | Gyroscope    |
| 19  | WIRE SCL | Gyroscope    |

---

## 2. Function Overview

### `DGMotor.h`

**`.setMotorSpeed(int speed)`**  
Range: -255 to 255.

---

### `drivebase.h`

**`initMotors(unsigned long baud)`**  
Initialize drive base motors.

**`driveForward(int speed)`**  
Range: 0 to 255. Drive all motors forward.

**`driveBackward(int speed)`**  
Range: 0 to 255. Drive all motors backward.

**`stopMotors()`**  
Stop all motors.

---

### `intake.h`

**`intake_init()`**  
Initializes pins.

**`intake_on()`**  
Turns intake on.

**`intake_off()`**  
Turns intake off.

---

### `collisionSense.h`

**`collision_init()`**  
Initializes pins.

**`bool collision_detect()`**  
Checks to see if either switch has been pressed. If so, returns `true`.

---

### `ColorSensor.h`

**`BallColor detectBallColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c)`**  
Takes in rgb values from color sensor and updates enum `BallColor`

**`const char* ballColorToString(BallColor color)`**  
Returns current `BallColor` value. 

---

### `mpu9250.h`

**`int16_t readGyroZRaw()`**  
Accesses and returns current gyroscope value. 

**`void calibrateGyroZ()`**  
Calculates values of the gyroscope at rest. 

**`float updateYaw()`**  
Acesses gyro readings to get current yaw.

**`void init()`**  
Initializes and calibrates the gyroscope.

---
#### Tuning
| Parameter | Value | Notes             |
| --------- | ----- | ----------------- |
| KP        | xxx   | Proportional gain |
| KI        | xxx   | Integral gain     |
| KD        | xxx   | Derivative gain   |

---

## 3. Known Issues
- [ ] example
- [ ] example

---

## 4. Dependencies
| Library     | Version | Author |
| ----------- | ------- | ------ |
| Servo       | xxx     | xxx    |
| Teensyduino | xxx     | xxx    |
| Adafruit_TCS34725 | xxx     | xxx    |

---

## 5. Setup & Upload
1. Install dependencies above
2. Open `ASEE-2026-Zippy.ino` in Arduino IDE 2.x
3. Select **Tools → Board → Teensy 4.1**
4. Click **Upload**
