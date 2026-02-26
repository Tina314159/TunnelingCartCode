/*
Based on your WORKING code mechanics:
- Uses cmd.mode=0 to read position
- Uses cmd.mode=1 with speed control (kp=0, kd=0.01, dq=...) to move
- Uses serial.sendRecv(&cmd,&data) exactly like your runTsec()

NEW behavior (for Cart motor only, USB0):
- Ask user for a distance in cm (+ forward, - backward)
- Convert cm -> desired motor position (q_target)
- WHILE not at q_target (within tolerance): run at constant speed using the SAME speed-control fields
- Stop motor at the end with mode 0

Keeps your second motor/USB1 code structure intact, but we focus on moving the cart motor by distance.
*/

#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

/*definitions/macros/variables*/
const char* USB_CartMotor = "/dev/ttyUSB0";
const char* USB_LA_Motor  = "/dev/ttyUSB1"; // still initialized, not used for distance move here

// Drive motor ID on Cart USB (if only 1 motor on that port, this is usually 0)
const int CART_ID = 0;

// Wheel radius (114.3 mm = 0.1143 m)
const double WHEEL_RADIUS = 0.1143; // meters

// Gear ratio used in your working code: dq = W * 6.33
const double GEAR_RATIO = 6.33;

// Convert cm -> motor radians:
// theta_motor = (distance_m / R) * G
const double RAD_PER_CM = (0.01 / WHEEL_RADIUS) * GEAR_RATIO;

// Stop tolerance in motor radians
const double POS_TOL_RAD = 0.1;

// Constant SPEED command (your request: make speed 6.33)
// This is motor-side rad/s, sent as cmd.dq
const double MOTOR_SPEED_CMD = 6.28 * 6.33;

/*function prototypes*/
void print_MotorData(MotorCmd &cmd, MotorData &data);
double readMotorPos(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial);
void moveCartDistanceCm(double dist_cm, MotorCmd &cmd, MotorData &data, SerialPort &serial);

/*main*/
int main() {
  SerialPort serial_Cart(USB_CartMotor);
  SerialPort serial_LA(USB_LA_Motor);

  MotorCmd  cmd;
  MotorData data;
  cmd.motorType  = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;

  // Print initial position for cart motor (same mechanic as your working code)
  double q0 = readMotorPos(CART_ID, cmd, data, serial_Cart);
  printf("Cart Motor (ID %d) Initial Position: %f rad\n", CART_ID, q0);

  // Interactive distance commands for cart motor
  while (true) {
    double cm;
    std::cout << "\nEnter cart distance (cm) (+ forward, - backward, 0 quit): ";
    std::cin >> cm;

    if (!std::cin) break;
    if (cm == 0) break;

    moveCartDistanceCm(cm, cmd, data, serial_Cart);
  }

  // Stop cart motor at end
  cmd.id   = CART_ID;
  cmd.mode = 0;
  serial_Cart.sendRecv(&cmd, &data);

  usleep(200);
  return 0;
}

/*
readMotorPos
- Uses your exact "mode 0 + sendRecv" method to read encoder position data.q
*/
double readMotorPos(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
  cmd.id   = motor_id;
  cmd.mode = 0;                 // static/stop so we can read position
  serial.sendRecv(&cmd, &data);
  return data.q;
}

/*
moveCartDistanceCm
- Uses your working speed-control mechanics (mode 1, kp=0, kd=0.01, dq set)
- Moves until the motor encoder reaches q_target within POS_TOL_RAD
*/
void moveCartDistanceCm(double dist_cm, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
  // Read starting position (mode 0)
  double q_start = readMotorPos(CART_ID, cmd, data, serial);

  // Compute desired motor position
  double q_target = q_start + RAD_PER_CM * dist_cm;

  std::cout << "\nRequested move: " << dist_cm << " cm\n";
  std::cout << "q_start  = " << q_start  << " rad\n";
  std::cout << "q_target = " << q_target << " rad\n";

  // Direction decided once from sign of cm
  double direction = (dist_cm > 0) ? 1.0 : -1.0;

  // Loop until we reach target
  while (true) {
    // Read current position (mode 0)
    double q_now = readMotorPos(CART_ID, cmd, data, serial);

    // Stop condition: "at desired position" within tolerance
    if ((q_target - q_now) < POS_TOL_RAD) {
      break;
    }

    // SPEED CONTROL COMMAND (same fields as your runTsec)
    cmd.id   = CART_ID;
    cmd.mode = 1;       // FOC
    cmd.kp   = 0.0;     // position off
    cmd.kd   = 0.01;    // speed control damping
    cmd.q    = 0.0;     // unused
    cmd.dq   = direction * MOTOR_SPEED_CMD; // constant speed
    cmd.tau  = 0.0;     // torque off

    serial.sendRecv(&cmd, &data);

    if (data.correct == true) {
      print_MotorData(cmd, data);
    }

    // Loop timing (like runTsec, but faster so it stops closer to target)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Stop motor (mode 0), same as your runTsec end
  cmd.id   = CART_ID;
  cmd.mode = 0;
  serial.sendRecv(&cmd, &data);

  std::cout << "Move complete (stopped)\n";
}

/* print_MotorData
same as your working code
*/
void print_MotorData(MotorCmd &cmd, MotorData &data) {
  std::cout <<  std::endl;
  std::cout <<  "motor ID is: "  << cmd.id    << std::endl;
  std::cout <<  "motor pos: "    << data.q    << " rad" << std::endl;
  std::cout <<  "motor Temp: "   << data.temp << " ℃"  << std::endl;
  std::cout <<  "motor speed: "  << data.dq   << " rad/s"<<std::endl;
  std::cout <<  "motor tau: "    << data.tau  << " Nm" << std::endl;
  std::cout <<  std::endl;
}
