/*
Drive motor forward/backward by user-entered distance (cm)
using ONLY the motor encoder (data.q).

Assumptions:
- Drive motor is on /dev/ttyUSB0
- Only one motor on this port -> motor ID = 0
- Wheel radius = 114.3 mm
- Gear ratio = 6.33
- Therefore: 1 cm travel ≈ 0.5538 motor radians
*/

#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

const char* USB_DRIVE = "/dev/ttyUSB0";

const double RAD_PER_CM = 0.5538;   // motor radians per cm

const double KP_POS = 2.0;          // position gain
const double KD_POS = 0.05;         // damping gain
const double POS_TOL_RAD = 0.02;    // stop tolerance (radians)

const int DRIVE_ID = 0;

static const int MODE_STOP = 0;
static const int MODE_FOC  = 1;

double readPosition(SerialPort &serial,
                    MotorCmd &cmd,
                    MotorData &data)
{
    cmd.id   = DRIVE_ID;
    cmd.mode = MODE_STOP;
    serial.sendRecv(&cmd, &data);
    return data.q;
}

void moveDistanceCm(double dist_cm,
                    SerialPort &serial,
                    MotorCmd &cmd,
                    MotorData &data)
{
    double delta_q  = RAD_PER_CM * dist_cm;

    double q_start  = readPosition(serial, cmd, data);
    double q_target = q_start + delta_q;

    std::cout << "\nMoving " << dist_cm << " cm\n";
    std::cout << "Start position:  " << q_start  << " rad\n";
    std::cout << "Target position: " << q_target << " rad\n";

    while (true)
    {
        double q_now = readPosition(serial, cmd, data);
        double error = q_target - q_now;

        if (std::fabs(error) < POS_TOL_RAD)
        {
            std::cout << "Target reached\n";
            break;
        }

        cmd.id   = DRIVE_ID;
        cmd.mode = MODE_FOC;

        cmd.kp   = KP_POS;
        cmd.kd   = KD_POS;

        cmd.q    = q_target;
        cmd.dq   = 0.0;
        cmd.tau  = 0.0;

        serial.sendRecv(&cmd, &data);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    cmd.id   = DRIVE_ID;
    cmd.mode = MODE_STOP;
    serial.sendRecv(&cmd, &data);

    std::cout << "Motor stopped\n";
}

int main()
{
    SerialPort serial(USB_DRIVE);

    MotorCmd  cmd;
    MotorData data;

    cmd.motorType  = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;

    double q0 = readPosition(serial, cmd, data);
    std::cout << "Initial motor position: " << q0 << " rad\n";

    while (true)
    {
        double cm;

        std::cout << "\nEnter distance in cm (+ forward, - backward, 0 quit): ";
        std::cin >> cm;

        if (cm == 0)
            break;

        moveDistanceCm(cm, serial, cmd, data);
    }

    cmd.id = DRIVE_ID;
    cmd.mode = MODE_STOP;
    serial.sendRecv(&cmd, &data);

    return 0;
}
