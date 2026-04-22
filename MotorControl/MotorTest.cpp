#include <unistd.h>
#include <iostream>
#include <cmath>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

const char* USB_LA_Motor = "/dev/ttyUSB1";

void print_MotorData(MotorCmd &cmd, MotorData &data);
void move90degStep(int motor_id, int direction, MotorCmd &cmd, MotorData &data, SerialPort &serial);

int main() {
    SerialPort serial_LA(USB_LA_Motor);

    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;

    int motor_id = 0;   // change if needed
    int input = -1;

    std::cout << "Motor Step Program\n";
    std::cout << "Enter 1 for +90 deg, 2 for -90 deg, 0 to exit\n";

    while (true) {
        std::cout << "> ";
        std::cin >> input;

        if (!std::cin) {
            std::cout << "Invalid input\n";
            break;
        }

        if (input == 0) {
            break;
        }
        else if (input == 1) {
            move90degStep(motor_id, +1, cmd, data, serial_LA);
        }
        else if (input == 2) {
            move90degStep(motor_id, -1, cmd, data, serial_LA);
        }
        else {
            std::cout << "Unknown command\n";
        }
    }

    cmd.id = motor_id;
    cmd.mode = 0;
    serial_LA.sendRecv(&cmd, &data);

    return 0;
}

void move90degStep(int motor_id, int direction, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    const double GEAR_RATIO = 6.33;
    const double STEP_OUTPUT_RAD = M_PI / 2.0;     // 90 degrees at output
    const double STEP_MOTOR_RAD = STEP_OUTPUT_RAD * GEAR_RATIO;
    const double POS_TOL_RAD = 0.05 * GEAR_RATIO;
    const int LOOP_DELAY_US = 10000;

    // Read current position
    cmd.id = motor_id;
    cmd.mode = 0;
    serial.sendRecv(&cmd, &data);

    if (!data.correct) {
        std::cout << "Failed to read current position\n";
        return;
    }

    double q_now = data.q;
    double q_target = q_now + direction * STEP_MOTOR_RAD;

    std::cout << "\nCurrent position: " << q_now << " rad\n";
    std::cout << "Target position:  " << q_target << " rad\n";

    if (direction == 1) {
        std::cout << "Rotating +90 degrees...\n";
    } else {
        std::cout << "Rotating -90 degrees...\n";
    }

    while (true) {
        cmd.id   = motor_id;
        cmd.mode = 1;
        cmd.kp   = 0.8;
        cmd.kd   = 0.05;
        cmd.q    = q_target;
        cmd.dq   = 0.0;
        cmd.tau  = 0.0;

        serial.sendRecv(&cmd, &data);

        if (data.correct) {
            print_MotorData(cmd, data);

            if (std::fabs(q_target - data.q) < POS_TOL_RAD) {
                break;
            }
        }

        usleep(LOOP_DELAY_US);
    }

    // Stop motor
    cmd.id = motor_id;
    cmd.mode = 0;
    serial.sendRecv(&cmd, &data);

    std::cout << "Reached target.\n\n";
}

void print_MotorData(MotorCmd &cmd, MotorData &data) {
    std::cout << "motor ID:    " << cmd.id << "\n";
    std::cout << "motor pos:   " << data.q << " rad\n";
    std::cout << "motor temp:  " << data.temp << " C\n";
    std::cout << "motor speed: " << data.dq << " rad/s\n";
    std::cout << "motor tau:   " << data.tau << " Nm\n";
    std::cout << std::endl;
}
