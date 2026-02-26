/*
Drive motor (ID 0) forward/backward by a user-entered distance (cm)
using ONLY the motor encoder (data.q).

Control approach:
1) Ask user for desired distance (cm)
2) Compute target motor position (q_target)
3) Decide direction ONCE from the user input sign (dist_cm)
4) WHILE not at target: command a constant slow speed (dq) toward target
5) When close to target: keep reducing speed by half and print "SLOWING DOWN"
6) When speed is very small AND error is within tolerance: stop motor

Assumptions / constants:
- USB port: /dev/ttyUSB0
- Wheel radius R = 114.3 mm = 0.1143 m
- Gear ratio G = 6.33 (motor rotations per wheel rotation)
*/

#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

// ---------- Hardware ----------
const char* USB_NO   = "/dev/ttyUSB0";
const int   DRIVE_ID = 0;

// ---------- Geometry ----------
const double WHEEL_RADIUS = 0.1143;  // meters
const double GEAR_RATIO   = 6.33;    // motor rotations per wheel rotation

// Convert cm -> motor radians:
// distance_m = cm * 0.01
// theta_motor = (distance_m / R) * G
const double RAD_PER_CM = (0.01 / WHEEL_RADIUS) * GEAR_RATIO;

// ---------- Control ----------
const double POS_TOL_RAD       = 0.1; // stop when |error| < this (motor rad)
const double POS_SLOW_BAND_RAD = 1.0;  // start halving speed when |error| < this (motor rad)

// Starting speed: choose a slow wheel speed, then convert to motor-side speed
const double WHEEL_SPEED_RAD_S = 6.0;                 // wheel rad/s (slow)
const double MOTOR_SPEED_START = WHEEL_SPEED_RAD_S * GEAR_RATIO; // motor rad/s
const double MIN_MOTOR_SPEED   = 1.0;                 // stop once command speed drops below this

/*
Move motor by a linear distance in cm:
- direction decided once from dist_cm sign
- speed control in loop until target reached
*/
void moveDistanceCm_speedControl(double dist_cm,
                                 MotorCmd &cmd,
                                 MotorData &data,
                                 SerialPort &serial)
{
  // Decide direction ONCE (no direction-from-error)
  // dist_cm > 0 => forward, dist_cm < 0 => backward
  double direction = (dist_cm > 0) ? 1.0 : -1.0;

  // Read starting motor position
  cmd.id   = DRIVE_ID;
  cmd.mode = 0;                 // stop/static read
  serial.sendRecv(&cmd, &data);
  double q_start = data.q;

  // Compute target motor position
  double q_target = q_start + RAD_PER_CM * dist_cm;

  std::cout << "\nRequested move: " << dist_cm << " cm\n";
  std::cout << "q_start  = " << q_start  << " rad\n";
  std::cout << "q_target = " << q_target << " rad\n";

  // Start at slow speed
  double speed_cmd = MOTOR_SPEED_START;

  // Loop until we reach target (within tolerance) AND speed has been reduced enough
  while (true) {
    // Read current position
    cmd.id   = DRIVE_ID;
    cmd.mode = 0;
    serial.sendRecv(&cmd, &data);
    double q_now = data.q;



    // SPEED CONTROL command (same style as your runTsec)
    cmd.id   = DRIVE_ID;
    cmd.mode = 1;       // FOC control
    cmd.kp   = 0.0;     // position off
    cmd.kd   = 0.01;    // speed damping
    cmd.q    = 0.0;     // unused (kp=0)
    cmd.dq   = direction * speed_cmd; // constant speed toward target
    cmd.tau  = 0.0;     // torque feedforward off

    serial.sendRecv(&cmd, &data);

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50 Hz
  }

  // Stop motor
  cmd.id   = DRIVE_ID;
  cmd.mode = 0;
  serial.sendRecv(&cmd, &data);

  std::cout << "Reached target, motor stopped.\n";
}

int main() {
  // Initialize serial + motor structs
  SerialPort serial(USB_NO);
  MotorCmd   cmd;
  MotorData  data;

  cmd.motorType  = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;

  std::cout << "RAD_PER_CM = " << RAD_PER_CM << " motor rad/cm\n";

  // User input loop
  while (true) {
    double cm;
    std::cout << "\nEnter desired distance (cm) (+ forward, - backward, 0 quit): ";
    std::cin >> cm;

    if (!std::cin) break;
    if (cm == 0) break;

    moveDistanceCm_speedControl(cm, cmd, data, serial);
  }

  // End: stop motor just in case
  cmd.id   = DRIVE_ID;
  cmd.mode = 0;
  serial.sendRecv(&cmd, &data);

  return 0;
}
