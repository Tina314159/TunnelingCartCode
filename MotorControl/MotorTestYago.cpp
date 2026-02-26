/*
Drive motor (ID 0) forward/backward by a user-entered distance (cm)
using ONLY the motor encoder (data.q) for feedback.

Control approach (what you asked for):
1) Ask user for desired distance (cm)
2) Compute target motor position (q_target)
3) WHILE not at target: command a constant slow speed (dq) in the correct direction
4) Stop when within tolerance

Assumptions / constants:
- USB port: /dev/ttyUSB0
- Wheel radius R = 114.3 mm = 0.1143 m
- Gear ratio G = 6.33 (motor rotations per wheel rotation)
- RAD_PER_CM computed from: theta_motor = (distance_m / R) * G
*/

#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

// ---------- Hardware ----------
const char* USB_NO = "/dev/ttyUSB0";  // drive motor USB port
const int   DRIVE_ID = 0;             // drive motor ID on this serial bus

// ---------- Geometry ----------
const double WHEEL_RADIUS = 0.1143;   // meters (114.3 mm)
const double GEAR_RATIO   = 6.33;     // motor rotations per wheel rotation

// Convert cm -> motor radians:
// distance_m = cm * 0.01
// theta_motor = (distance_m / R) * G
const double RAD_PER_CM = (0.01 / WHEEL_RADIUS) * GEAR_RATIO;

// ---------- Control ----------
const double POS_TOL_RAD = 0.02;      // stop when |q_target - q_now| < this
const double WHEEL_SPEED_RAD_S = 6.28; // slow output speed (wheel rad/s)
const double MOTOR_SPEED_CMD = WHEEL_SPEED_RAD_S * GEAR_RATIO; // motor rad/s

void print_MotorData(MotorCmd &cmd, MotorData &data) {
  std::cout << "\n";
  std::cout << "motor ID:   " << cmd.id    << "\n";
  std::cout << "pos (q):    " << data.q    << " rad\n";
  std::cout << "temp:       " << data.temp << " C\n";
  std::cout << "speed (dq): " << data.dq   << " rad/s\n";
  std::cout << "tau:        " << data.tau  << " Nm\n";
  std::cout << "\n";
}

/*
Move motor by a linear distance in cm using:
while(not at target) -> speed control at constant slow speed
*/
void moveDistanceCm_speedControl(double dist_cm,
                                 MotorCmd &cmd,
                                 MotorData &data,
                                 SerialPort &serial)
{
  // --- Read starting motor position ---
  cmd.id   = DRIVE_ID;
  cmd.mode = 0;                 // stop/static so we can read position
  serial.sendRecv(&cmd, &data);
  double q_start = data.q;

  // --- Compute target motor position (radians) ---
  double q_target = q_start + RAD_PER_CM * dist_cm;

  std::cout << "\nRequested move: " << dist_cm << " cm\n";
  std::cout << "q_start  = " << q_start  << " rad\n";
  std::cout << "q_target = " << q_target << " rad\n";

  // --- Control loop: while not at target, command slow speed toward it ---
  while (true) {
    // Read current motor position
    cmd.id   = DRIVE_ID;
    cmd.mode = 0;
    serial.sendRecv(&cmd, &data);
    double q_now = data.q;

    // Compute error (how far from target)
    double error = q_target - q_now;

    // Stop condition (reached target within tolerance)
    if (std::fabs(error) < POS_TOL_RAD) {
      break;
    }

    // Direction: if error positive, move forward; if negative, move backward
    double direction = (error > 0) ? 1.0 : -1.0;

    // SPEED CONTROL command (same style as your runTsec)
    cmd.id   = DRIVE_ID;
    cmd.mode = 1;       // FOC mode
    cmd.kp   = 0.0;     // position control off
    cmd.kd   = 0.01;    // speed damping (same as your original example)
    cmd.q    = 0.0;     // unused because kp=0
    cmd.dq   = direction * MOTOR_SPEED_CMD; // constant slow speed toward target
    cmd.tau  = 0.0;     // torque feedforward off

    serial.sendRecv(&cmd, &data);

    // Optional debug print (comment out if you want it quieter)
    if (data.correct) {
      print_MotorData(cmd, data);
    }

    // Loop rate (faster than 500ms so we stop closer to target)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Stop motor once reached target
  cmd.id   = DRIVE_ID;
  cmd.mode = 0;
  serial.sendRecv(&cmd, &data);

  std::cout << "Reached target, motor stopped.\n";
}

int main() {
  // Initialize serial port and motor structs
  SerialPort serial(USB_NO);
  MotorCmd   cmd;
  MotorData  data;

  cmd.motorType  = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;

  std::cout << "RAD_PER_CM = " << RAD_PER_CM << " motor rad/cm\n";

  // Main interactive loop
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
