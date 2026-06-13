# ASEE-2026-Zippy

> One-line description of what the robot does.

**Team:** Names here  
**Board:** Teensy 4.1  
**Competition:** ASEE 2026

---

## Table of Contents

1. [Pinout](#1-pinout)
2. [Function Overview](#2-function-overview)

   * [Motors](#a-motors)
   * [Gate](#b-gate)
   * [Navigation](#c-navigation)

     * [Tuning](#tuning)
3. [Known Issues](#3-known-issues)
4. [Dependencies](#4-dependencies)
5. [Setup \& Upload](#5-setup--upload)

---

## 1\. Pinout

### Collision Sensing (`collisionSense.h`)

|Pin|Signal|Type|Connected To|
|-|-|-|-|
|33|sw1|Digital IN PULLUP|Front bump switch|
|34|sw2|Digital IN PULLUP|Rear bump switch|

---

### Drive Motors (`drivebase.h` / `DGMotor.h`)

Each DGMotor controller communicates over a dedicated UART serial port (one per wheel).

|Pin|Serial Port|Signal|Function|Connected To|
|-|-|-|-|-|
|0|Serial1 RX|UART RX|Front-Left motor RX|leftFront DGMotor|
|1|Serial1 TX|UART TX|Front-Left motor TX|leftFront DGMotor|
|7|Serial2 RX|UART RX|Front-Right motor RX|rightFront DGMotor|
|8|Serial2 TX|UART TX|Front-Right motor TX|rightFront DGMotor|
|24|Serial6 TX|UART TX|Rear-Left motor TX|leftRear DGMotor|
|25|Serial6 RX|UART RX|Rear-Left motor RX|leftRear DGMotor|
|28|Serial7 RX|UART RX|Rear-Right motor RX|rightRear DGMotor|
|29|Serial7 TX|UART TX|Rear-Right motor TX|rightRear DGMotor|

---

### Intake Pinout (`intake.h` / `intake.h`)

|Pin|Signal|Type|Connected To|
|-|-|-|-|
|14|input1|Digital OUT|H-bridge IN1|
|15|input2|Digital OUT|H-bridge IN2|
|36|enable|Digital OUT|H-bridge EN |



---

### Deposit Pinout (`deposit.h` / `deposit.h`)





### Color Sensor Pinout

|Pin|I2C PORT|
|-|-|
|16|WIRE1 SDA1|
|17|WIRE1 SCL1|



### Servo Motor Pinout



|Pin|Signal|Type|Connected To|
|-|-|-|-|
|33|servo|PWM|Servo PWM |



---

### Gyroscope Pinout

|Pin|I2C PORT|
|-|-|
|18|WIRE SDA|
|19|WIRE SCL|



---



## 2\. Function Overview

### `drivebase.h`

**`initMotors(unsigned long baud)`**  
Initialize drive base motors.

`.setMotorSpeed(int speed)`
-255 to 255

`driveForward(int speed)`
0 - 255
Drive all motors forward.

`driveBackward(int speed)`
0 -255
Drive all motors backward.

**`stopMotors()`**  
Stop all Motors
---

### b. `intake.h`

**`intake_init()`**  
Initializes pins.

`intake_on()`

`intake_off()`
---

### c. `collisionSense.h`

`collision_init()`
Initializes pins.

`bool collision_detect()`
Checks to see if either switch has been pressed. If so, returns true. 

#### Tuning

| Parameter | Value | Notes             |
| --------- | ----- | ----------------- |
| KP        | xxx   | Proportional gain |
| KI        | xxx   | Integral gain     |
| KD        | xxx   | Derivative gain   |

---

## 3\. Known Issues

* \[ ] example
* \[ ] example

---

## 4\. Dependencies

|Library|Version|Author|
|-|-|-|
|Servo|xxx|xxx|
|Teensyduino|xxx|xxx|



---

## 5\. Setup \& Upload

1. Install dependencies above
2. Open `ASEE-2026-Zippy.ino` in Arduino IDE 2.x
3. Select **Tools → Board → Teensy 4.1**
4. Click **Upload**

__
