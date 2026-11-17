// Motor control for the Freenove 4WD Mecanum car (Pico W board v1.5).
// Pins and wheel positions come from Freenove's own sketches
// (Mecanum_wheels/Sketches/01.1_Car_Move_and_Turn).
#pragma once
#include <Arduino.h>

#include "config.h"

// Each motor has two pins: PWM on pin A drives it forward, on pin B backward.
//   M1 front-left   GPIO 18 / 19
//   M2 rear-left    GPIO 21 / 20
//   M3 rear-right   GPIO  7 / 6
//   M4 front-right  GPIO  9 / 8
struct MotorPins {
  uint8_t a, b;
};
static const MotorPins MOTOR_PINS[4] = {{18, 19}, {21, 20}, {7, 6}, {9, 8}};
static const int8_t MOTOR_SIGN[4] = {MOTOR1_REVERSED ? -1 : 1, MOTOR2_REVERSED ? -1 : 1,
                                     MOTOR3_REVERSED ? -1 : 1, MOTOR4_REVERSED ? -1 : 1};

static int motorCmd[4] = {0, 0, 0, 0};  // last commanded speed per motor, -100..100

#define PIN_BATTERY 26
#define BATTERY_COEFFICIENT 3.95f  // Freenove's voltage-divider ratio

inline void motorsBegin() {
  analogWriteFreq(MOTOR_PWM_HZ);
  analogWriteRange(100);  // duty is 0-100 %
  for (const auto &p : MOTOR_PINS) {
    pinMode(p.a, OUTPUT);
    pinMode(p.b, OUTPUT);
    analogWrite(p.a, 0);
    analogWrite(p.b, 0);
  }
}

// speed -100..100 for motor index 0..3 (M1..M4)
inline void setMotor(int i, int speed) {
  speed = constrain(speed, -100, 100);
  motorCmd[i] = speed;
  int duty = abs(speed) * MAX_SPEED / 100;
  if (duty > 0) duty = MOTOR_MIN_DUTY + duty * (100 - MOTOR_MIN_DUTY) / 100;
  bool forward = (speed * MOTOR_SIGN[i]) >= 0;
  const MotorPins &p = MOTOR_PINS[i];
  analogWrite(forward ? p.b : p.a, 0);  // never drive both pins at once
  analogWrite(forward ? p.a : p.b, duty);
}

// Per-wheel control in Freenove's order: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right.
inline void setWheels(int m1, int m2, int m3, int m4) {
  setMotor(0, m1);
  setMotor(1, m2);
  setMotor(2, m3);
  setMotor(3, m4);
}

inline void stopMotors() { setWheels(0, 0, 0, 0); }

// Mecanum mixing. Each input -100..100:
//   vx   forward (+) / backward (-)
//   vy   slide right (+) / left (-)
//   turn spin clockwise (+) / counter-clockwise (-)
inline void drive(int vx, int vy, int turn) {
  int fl = vx + vy + turn;
  int rl = vx - vy + turn;
  int rr = vx + vy - turn;
  int fr = vx - vy - turn;
  int biggest = max(max(abs(fl), abs(rl)), max(abs(rr), abs(fr)));
  if (biggest > 100) {  // scale down together so the direction stays right
    fl = fl * 100 / biggest;
    rl = rl * 100 / biggest;
    rr = rr * 100 / biggest;
    fr = fr * 100 / biggest;
  }
  setWheels(fl, rl, rr, fr);
}

inline float readBatteryVolts() {
  analogReadResolution(10);
  int sum = 0;
  for (int i = 0; i < 5; i++) sum += analogRead(PIN_BATTERY);
  return (sum / 5) / 1023.0f * 3.3f * BATTERY_COEFFICIENT;
}
