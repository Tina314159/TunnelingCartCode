#include <unistd.h>      
#include <iostream>       
#include <thread>         
#include <chrono>       
#include <cmath>           

#include "unitreeMotor/unitreeMotor.h"  
#include "serialPort/SerialPort.h"      

// Linux device file for the drive motor controller
const char* USB_DRIVE = "/dev/ttyUSB0";

/*
Wheel radius in meters.
Given: 114.3 mm = 0.1143 meters
This is the radius of the wheel attached to the gearbox output shaft.
*/
const double WHEEL_RADIUS = 0.1143;

/*
Gear ratio G.

Definition:
G = motor rotations / wheel rotations

If G = 6.33:
- Motor rotates 6.33 times
- Wheel rotates 1 time

So motor angle = wheel angle * 6.33
*/
const double GEAR_RATIO = 6.33;

/*
We want to convert centimeters of linear motion
into motor radians.

Physics:

distance = wheel_angle * R

So:
wheel_angle = distance / R

Because of gearbox:
motor_angle = wheel_angle * G

So:
motor_angle = (distance / R) * G

For 1 cm:
distance = 0.01 meters

So:
RAD_PER_CM = (0.01 / R) * G
*/
const double RAD_PER_CM = (0.01 / WHEEL_RADIUS) * GEAR_RATIO;

int main()
{
    // Create serial connection to motor controller
    SerialPort serial(USB_DRIVE);

    // Command structure (what we send to motor)
    MotorCmd cmd;

    // Data structure (what motor sends back)
    MotorData data;

    // Tell SDK what motor model we are using
    cmd.motorType  = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;

    // Motor ID on this serial bus (only one motor connected)
    const int ID = 0;

    while (true)
    {
        // Ask user how many centimeters to move
        double cm;
        std::cout << "\nEnter distance (cm) (+ forward, - backward, 0 quit): ";
        std::cin >> cm;

        // Exit if user enters 0
        if (cm == 0)
            break;

        // -----------------------------
        // STEP 1: Read current position
        // -----------------------------
        cmd.id   = ID;
        cmd.mode = 0;                 // Mode 0 = stop / passive read
        serial.sendRecv(&cmd, &data); // Send command and receive data

        double q_start = data.q;      // Current motor position (radians)

        // -----------------------------
        // STEP 2: Compute target angle
        // -----------------------------
        // Convert linear cm into motor radians
        double q_target = q_start + RAD_PER_CM * cm;

        // -----------------------------
        // STEP 3: Move toward target
        // -----------------------------
        while (true)
        {
            // Read updated motor position
            cmd.id   = ID;
            cmd.mode = 0;
            serial.sendRecv(&cmd, &data);

            // Set motor into active control mode
            cmd.id   = ID;
            cmd.mode = 1;        // Mode 1 = FOC control (active control)

            // Position controller gains
            cmd.kp   = 2.0;      // Proportional gain (pull strength toward target)
            cmd.kd   = 0.05;     // Damping (reduces oscillation)

            // Position target
            cmd.q    = q_target;

            // No velocity or torque feedforward used
            cmd.dq   = 0.0;
            cmd.tau  = 0.0;

            // Send control command
            serial.sendRecv(&cmd, &data);

            // Run loop at ~50 Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // -----------------------------
        // STEP 4: Stop motor
        // -----------------------------
        cmd.id   = ID;
        cmd.mode = 0;            // Stop motor
        serial.sendRecv(&cmd, &data);

        std::cout << "Move complete\n";
    }

    return 0;
}
