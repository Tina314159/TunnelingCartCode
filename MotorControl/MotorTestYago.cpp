#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

const char* USB_DRIVE = "/dev/ttyUSB0";

// Conversion: 1 cm ≈ 0.5538 motor radians
const double RAD_PER_CM = 0.5538;

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

            double error = q_target - data.q;

            if (std::fabs(error) < 0.02)
                break;

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
