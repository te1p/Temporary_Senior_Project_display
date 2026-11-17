// Per-robot settings.
#pragma once

// Robot number 1-8. laptop/upload_wifi.sh and laptop/upload_usb.sh set this for
// you (e.g. `upload_wifi.sh 3`); this default is only used by the Arduino IDE.
#ifndef ROBOT_ID
#define ROBOT_ID 1
#endif

// If the robot can't join the router in secrets.h, it makes its OWN Wi-Fi network
// "RobotDuel-<id>" (password: AP_PASSWORD in secrets.h).
#define AP_CHANNEL 6  // 1, 6 or 11; pick the quietest where you test

// Router name/password and both robot passwords live in secrets.h (git-ignored).
// Copy secrets.example.h to secrets.h and fill it in.
// If the router can't be reached within this time, the robot falls back to its own network.
#define JOIN_TIMEOUT_MS 15000

#define UDP_PORT 4210


// ---- Driving ----
// If no drive command arrives for this long, the motors stop. The laptop must
// keep resending the current command (the client and web page do, ~10-20 times a second).
#define FAILSAFE_MS 300

// Cap on motor power, % (100 = full). Lower it for gentle indoor testing.
#define MAX_SPEED 100

// Smallest duty that actually turns the wheels. Raise it (e.g. 30) if the motors
// hum but don't move at low speeds - common on weak/AA batteries.
#define MOTOR_MIN_DUTY 0

#define MOTOR_PWM_HZ 500  // same frequency as Freenove's code

// ---- Head (servo + LED eyes) ----
// Degrees to add so that "servo 90" looks straight ahead. Adjust if the head
// sits a bit crooked (e.g. -5 or 8).
#define SERVO_OFFSET 0

#define EYE_BRIGHTNESS 8  // 0 (dim) to 15 (brightest)

// Set to 1 if a wheel spins the wrong way in the wheel test.
#define MOTOR1_REVERSED 0  // front-left
#define MOTOR2_REVERSED 0  // rear-left
#define MOTOR3_REVERSED 0  // rear-right
#define MOTOR4_REVERSED 0  // front-right
