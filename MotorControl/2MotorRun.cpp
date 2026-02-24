/*
Test file 1: 
- will retrieve initial positions of all motors and print them to console
- have each of them turn for 10 seconds, one at a time
*/
#include "serialPort/SerialPort.h"
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include "unitreeMotor/unitreeMotor.h"

#define MOTOR_COUNT 4 // # of motors   

void run10s(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial) {
    using namespace std::chrono;
    auto startClock = steady_clock::now();
    std::cout <<  std::endl;
    std::cout <<  "NOW RUNNING MOTOR  "  << motor_id  << std::endl;
    usleep(1000000);
    while (steady_clock::now() - startClock < seconds(10)) {
        cmd.id    = motor_id;
        cmd.mode  = 1;
        cmd.kp    = 0.0;
        cmd.kd    = 0.01;
        cmd.q     = 0.0;
        cmd.dq     = 6.28*6.33;
        cmd.tau     = 0.0;
        serial.sendRecv(&cmd,&data);
        if(data.correct == true)
        {
          std::cout <<  std::endl;
          std::cout <<  "motor ID is: "  << cmd.id  << std::endl;
          std::cout <<  "motor.q: "    << data.q    << " rad" << std::endl;
          std::cout <<  "motor.Temp: "   << data.temp   << " ℃"  << std::endl;
          std::cout <<  "motor.W: "      << data.dq      << " rad/s"<<std::endl;
          std::cout <<  "motor.T: "      << data.tau      << " N.m" << std::endl;
          std::cout <<  std::endl;
        }
        std::this_thread::sleep_for(milliseconds(500)); // controls repetition speed
    }
    cmd.mode  = 0;
    serial.sendRecv(&cmd,&data);
}

int main() {

  SerialPort  serial("/dev/ttyUSB3");
  MotorCmd    cmd;
  MotorData   data;
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;
  double init_Pos[MOTOR_COUNT]; // initialize position storage
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id = i;
        cmd.mode = 0; // static, so we can read the position
        serial.sendRecv(&cmd, &data);
        init_Pos[i] = data.q; // initial position storage
        printf("Motor %d Inititial Position: %f rad\n", i, data.q);
        usleep(200);
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        run10s(i, cmd, data, serial); // run each motor for 10 seconds
    }

    uint8_t *p = (uint8_t *)cmd.get_motor_send_data();
    // for(int i =0; i<17; i++)
    printf("first without plus-ing: %02X", *p);
    printf("\n then with plus-ing:  %02X", *p++);
    printf("\n then without plus-ing:  %02X", *p);

    usleep(200);
  return 0;
}
