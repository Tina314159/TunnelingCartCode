#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

/* definitions */
const char* USB_CartMotor = "/dev/ttyUSB0";
const char* USB_LA_Motor  = "/dev/ttyUSB1";

const int CART_ID = 0;

// speeds
const double SLOW_SPEED = 6.28 * 3.0;
const double FAST_SPEED = 6.28 * 6.33;

/* function prototypes */
double readMotorPos(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial);
void stopMotor(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial);
void runMotor(int motor_id, double speed, MotorCmd &cmd, MotorData &data, SerialPort &serial);
void autoMode(MotorCmd &cmd, MotorData &data, SerialPort &serial);
void manualMode(MotorCmd &cmd, MotorData &data, SerialPort &serial);

/* main */
int main() {
    SerialPort serial_Cart(USB_CartMotor);
    SerialPort serial_LA(USB_LA_Motor); // initialized but not used here

    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;

    double q0 = readMotorPos(CART_ID, cmd, data, serial_Cart);
    std::cout << "Cart Motor Initial Position: " << q0 << " rad\n";

    while (true) {
        int mode;
        std::cout << "\nSelect mode:\n";
        std::cout << "1 = Auto mode\n";
        std::cout << "2 = Manual mode\n";
        std::cout << "0 = Emergency stop\n";
        std::cout << "Choice: ";
        std::cin >> mode;

        if (!std::cin) {
            break;
        }

        if (mode == 0) {
            stopMotor(CART_ID, cmd, data, serial_Cart);
            std::cout << "EMERGENCY STOP ACTIVATED\n";
        } 
        else if (mode == 1) {
            autoMode(cmd, data, serial_Cart);
        } 
        else if (mode == 2) {
            manualMode(cmd, data, serial_Cart);
        } 
        else {
            std::cout << "Invalid mode\n";
        }
    }

    stopMotor(CART_ID, cmd, data, serial_Cart);
    return 0;
}

/* read current motor position */
double readMotorPos(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    cmd.id = motor_id;
    cmd.mode = 0;
    serial.sendRecv(&cmd, &data);
    return data.q;
}

/* stop motor */
void stopMotor(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    cmd.id = motor_id;
    cmd.mode = 0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.tau = 0.0;
    serial.sendRecv(&cmd, &data);
}

/* run motor at given speed */
void runMotor(int motor_id, double speed, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    cmd.id = motor_id;
    cmd.mode = 1;     // FOC mode
    cmd.kp = 0.0;
    cmd.kd = 0.01;
    cmd.q = 0.0;
    cmd.dq = speed;
    cmd.tau = 0.0;
    serial.sendRecv(&cmd, &data);
}

/* auto mode */
void autoMode(MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    int directionChoice;
    std::cout << "\nAUTO MODE\n";
    std::cout << "Press 1 for forward, 2 for backward: ";
    std::cin >> directionChoice;

    if (!std::cin) return;

    double speed = 0.0;

    if (directionChoice == 1) {
        speed = FAST_SPEED;
        std::cout << "Auto mode: moving forward for 5 seconds...\n";
    } 
    else if (directionChoice == 2) {
        speed = -FAST_SPEED;
        std::cout << "Auto mode: moving backward for 5 seconds...\n";
    } 
    else {
        std::cout << "Invalid choice\n";
        return;
    }

    auto start = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();

        if (elapsed >= 5.0) {
            break;
        }

        runMotor(CART_ID, speed, cmd, data, serial);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    stopMotor(CART_ID, cmd, data, serial);
    std::cout << "Auto move complete\n";
}

/* manual mode */
void manualMode(MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    int speedChoice;
    std::cout << "\nMANUAL MODE\n";
    std::cout << "Press 1 for slow, 2 for fast: ";
    std::cin >> speedChoice;

    if (!std::cin) return;

    double baseSpeed = 0.0;

    if (speedChoice == 1) {
        baseSpeed = SLOW_SPEED;
        std::cout << "Manual mode set to SLOW\n";
    } 
    else if (speedChoice == 2) {
        baseSpeed = FAST_SPEED;
        std::cout << "Manual mode set to FAST\n";
    } 
    else {
        std::cout << "Invalid speed choice\n";
        return;
    }

    std::cout << "Use:\n";
    std::cout << "w = forward\n";
    std::cout << "s = backward\n";
    std::cout << "x = stop\n";
    std::cout << "q = quit manual mode\n";

    while (true) {
        char command;
        std::cout << "\nEnter command: ";
        std::cin >> command;

        if (!std::cin) return;

        if (command == 'w') {
            runMotor(CART_ID, baseSpeed, cmd, data, serial);
            std::cout << "Moving forward\n";
        } 
        else if (command == 's') {
            runMotor(CART_ID, -baseSpeed, cmd, data, serial);
            std::cout << "Moving backward\n";
        } 
        else if (command == 'x') {
            stopMotor(CART_ID, cmd, data, serial);
            std::cout << "Stopped\n";
        } 
        else if (command == 'q') {
            stopMotor(CART_ID, cmd, data, serial);
            std::cout << "Exiting manual mode\n";
            break;
        } 
        else {
            stopMotor(CART_ID, cmd, data, serial);
            std::cout << "No valid command, motor stopped\n";
        }
    }
}
