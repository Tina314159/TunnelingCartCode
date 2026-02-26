/*
Two separate serial ports:
- Drive motor (forward/back) on /dev/ttyUSB0
- Arm rotation motor on /dev/ttyUSB1

This program:
1) Opens both USB serial ports
2) Reads and prints initial position for each motor
3) Repeatedly asks user for a distance in cm and moves DRIVE motor (motor on USB0) by that amount
   - positive = forward, negative = backward
   - uses position control to reach target and stop

Assumptions:
- Each USB port has ONE motor connected, so cmd.id is set to 0 for each port.
- Wheel radius = 114.3 mm
- Gear ratio = 6.33
- Therefore RAD_PER_CM ≈ 0.5538 motor-rad per cm of travel (for the DRIVE motor).
*/

#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

// --------- PORTS (EDIT IF NEEDED) ----------
const char* USB_DRIVE = "/dev/ttyUSB0";  // forward/back motor
const char* USB_ARM   = "/dev/ttyUSB1";  // arm rotation motor

// --------- DRIVE DISTANCE CONVERSION ----------
const double RAD_PER_CM  = 0.5538; // motor rad per cm (R=0.1143m, G=6.33)

// --------- POSITION CONTROL TUNING ----------
const double KP_POS      = 2.0;    // tune
const double KD_POS      = 0.05;   // tune
const double POS_TOL_RAD = 0.02;   // tune (motor rad)
const double TIMEOUT_SEC = 10.0;   // safety timeout

// If your SDK uses different mode numbers, keep consistent with your lecture/SDK:
// mode 0 = stop/static read
// mode 1 = FOC control
static const int MODE_STOP = 0;
static const int MODE_FOC  = 1;

// ---------- PRINT MOTOR DATA ----------
void print_MotorData(MotorCmd &cmd, MotorData &data) {
  std::cout << "\n";
  std::cout << "motor ID is: "  << cmd.id    << "\n";
  std::cout << "motor pos: "    << data.q    << " rad\n";
  std::cout << "motor Temp: "   << data.temp << " ℃\n";
  std::cout << "motor speed: "  << data.dq   << " rad/s\n";
  std::cout << "motor tau: "    << data.tau  << " Nm\n";
  std::cout << "\n";
}

// ---------- READ INITIAL POSITION ----------
double readInitialPosition(SerialPort &serial, MotorCmd &cmd, MotorData &data, int motor_id) {
  cmd.id   = motor_id;
  cmd.mode = MODE_STOP;
  serial.sendRecv(&cmd, &data);
  return data.q;
}

// ---------- MOVE DRIVE MOTOR BY CM (POSITION CONTROL) ----------
void moveDriveDistanceCm(double dist_cm,
                         SerialPort &serialDrive,
                         MotorCmd &cmdDrive,
                         MotorData &dataDrive,
                         int motor_id_drive = 0)
{
  // Convert cm -> motor radians
  double delta_q = RAD_PER_CM * dist_cm;

  // Read current position
  cmdDrive.id   = motor_id_drive;
  cmdDrive.mode = MODE_STOP;
  serialDrive.sendRecv(&cmdDrive, &dataDrive);

  double q_start  = dataDrive.q;
  double q_target = q_start + delta_q;

  std::cout << "\n[DRIVE] Requested move: " << dist_cm << " cm\n";
  std::cout << "[DRIVE] Start q:  " << q_start  << " rad\n";
  std::cout << "[DRIVE] Target q: " << q_target << " rad\n";

  using namespace std::chrono;
  auto t0 = steady_clock::now();

  while (true) {
    double elapsed = duration<double>(steady_clock::now() - t0).count();
    if (elapsed > TIMEOUT_SEC) {
      std::cout << "[DRIVE] TIMEOUT — stopping.\n";
      break;
    }

    // Read position
    cmdDrive.id   = motor_id_drive;
    cmdDrive.mode = MODE_STOP;
    serialDrive.sendRecv(&cmdDrive, &dataDrive);

    double q_now = dataDrive.q;
    double err   = q_target - q_now;

    if (std::fabs(err) < POS_TOL_RAD) {
      std::cout << "[DRIVE] Target reached.\n";
      break;
    }

    // Send position command
    cmdDrive.id   = motor_id_drive;
    cmdDrive.mode = MODE_FOC;

    cmdDrive.kp   = KP_POS;
    cmdDrive.kd   = KD_POS;

    cmdDrive.q    = q_target; // position target
    cmdDrive.dq   = 0.0;      // no speed target
    cmdDrive.tau  = 0.0;      // no torque feedforward

    serialDrive.sendRecv(&cmdDrive, &dataDrive);

    if (dataDrive.correct) {
      print_MotorData(cmdDrive, dataDrive);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50 Hz
  }

  // Stop
  cmdDrive.id   = motor_id_drive;
  cmdDrive.mode = MODE_STOP;
  serialDrive.sendRecv(&cmdDrive, &dataDrive);

  std::cout << "[DRIVE] Stopped.\n";
}

// -------------------- MAIN --------------------
int main() {
  // Create two serial ports
  SerialPort serialDrive(USB_DRIVE);
  SerialPort serialArm(USB_ARM);

  // Separate cmd/data structs for each port
  MotorCmd  cmdDrive;
  MotorData dataDrive;
  MotorCmd  cmdArm;
  MotorData dataArm;

  // Set motor type (adjust if your arm motor type differs)
  cmdDrive.motorType  = MotorType::GO_M8010_6;
  dataDrive.motorType = MotorType::GO_M8010_6;

  cmdArm.motorType    = MotorType::GO_M8010_6;
  dataArm.motorType   = MotorType::GO_M8010_6;

  // If each USB has exactly one motor connected, motor_id can be 0 on each port.
  const int DRIVE_ID = 0;
  const int ARM_ID   = 0;

  // Read initial positions
  double driveInit = readInitialPosition(serialDrive, cmdDrive, dataDrive, DRIVE_ID);
  std::cout << "DRIVE motor initial position: " << driveInit << " rad\n";

  double armInit   = readInitialPosition(serialArm, cmdArm, dataArm, ARM_ID);
  std::cout << "ARM motor initial position:   " << armInit   << " rad\n";

  // Interactive loop: move drive motor by user-entered cm
  while (true) {
    double cm;
    std::cout << "\nEnter DRIVE distance (cm) (+ forward, - backward, 0 quit): ";
    std::cin >> cm;

    if (!std::cin) {
      std::cout << "Input error. Exiting.\n";
      break;
    }
    if (cm == 0) {
      std::cout << "Exiting.\n";
      break;
    }

    moveDriveDistanceCm(cm, serialDrive, cmdDrive, dataDrive, DRIVE_ID);
  }

  // Final stop commands (just in case)
  cmdDrive.id = DRIVE_ID; cmdDrive.mode = MODE_STOP; serialDrive.sendRecv(&cmdDrive, &dataDrive);
  cmdArm.id   = ARM_ID;   cmdArm.mode   = MODE_STOP; serialArm.sendRecv(&cmdArm, &dataArm);

  usleep(200);
  return 0;
}
