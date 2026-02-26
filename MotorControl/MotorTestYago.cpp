#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

const char* USB_DRIVE = "/dev/ttyUSB0";

/*
Wheel radius (meters)
114.3 mm = 0.1143 m
*/
const double WHEEL_RADIUS = 0.1143;

/*
Gear ratio G
6.33 motor rotations = 1 wheel rotation
*/
const double GEAR_RATIO = 6.33;

/*
Compute radians per cm automatically:

theta_motor = (distance / R) * G

For 1 cm:
distance = 0.01 m

So:
RAD_PER_CM = (0.01 / R) * G
*/
const double RAD_PER_CM = (0.01 / WHEEL_RADIUS) * GEAR_RATIO;

int main()
{
    SerialPort serial(USB_DRIVE);

    MotorCmd cmd;
    MotorData data;

    cmd.motorType  = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;

    const int ID = 0;

    while (true)
    {
        double cm;
        std::cout << "\nEnter distance (cm) (+ forward, - backward, 0 quit): ";
        std::cin >> cm;

        if (cm == 0)
            break;

        // Read starting position
        cmd.id   = ID;
        cmd.mode = 0;
        serial.sendRecv(&cmd, &data);
        double q_start = data.q;

        double q_target = q_start + RAD_PER_CM * cm;

        while (true)
        {
            cmd.id   = ID;
            cmd.mode = 0;
            serial.sendRecv(&cmd, &data);


            cmd.id   = ID;
            cmd.mode = 1;
            cmd.kp   = 2.0;
            cmd.kd   = 0.05;
            cmd.q    = q_target;
            cmd.dq   = 0.0;
            cmd.tau  = 0.0;

            serial.sendRecv(&cmd, &data);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        cmd.id   = ID;
        cmd.mode = 0;
        serial.sendRecv(&cmd, &data);

        std::cout << "Move complete\n";
    }

    return 0;
}
