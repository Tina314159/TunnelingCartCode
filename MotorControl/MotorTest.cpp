/*
Draft 1: 
- will retrieve initial positions of 2 motors and print them to console
- have each of them turn for a given number of seconds, one at a time
*/

/*Library*/
#include <unistd.h>
#include <iostream> //input output stream
#include <thread>
#include <chrono>
#include "unitreeMotor/unitreeMotor.h"  //"" = custom libraries in SDK
#include "serialPort/SerialPort.h"

/*definitions/macros/variables*/
#define MOTOR_COUNT 2 // # of motors   

/*functions prototypes*/
void runTsec(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial, double T, double W);
void print_MotorData(MotorCmd &cmd, MotorData &data);

/*main
- initialize the USB port and communication structures

*/
int main() {
  /*initialization*/
  SerialPort  serial("/dev/ttyUSB3");
  MotorCmd    cmd;
  MotorData   data;
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;

  /*Motor initial position check*/
  double init_Pos[MOTOR_COUNT];              // initialize position storage
  for (int i = 0; i < MOTOR_COUNT; i++) {
      cmd.motorType = MotorType::GO_M8010_6;
      cmd.id = i;
      cmd.mode = 0; // motor = static, so we can read the position
      serial.sendRecv(&cmd, &data);
      init_Pos[i] = data.q; // initial position storage
      printf("Motor %d Inititial Position: %f rad\n", i, data.q);
      usleep(200);
  }

  /*Main program*/
  for (int i = 0; i < MOTOR_COUNT; i++) {
      runTsec(i, cmd, data, serial,5, 6.28);  
  }
  
  /*End Matter*/
  usleep(200);
  return 0;
}

/*runTsec
Purpose:
  - run motor with given id for T seconds
Input:
  - motor_id = id of motor (assign id to motor according to manual)
  - cmd = a structure representing command sent to motor
  - data = a structure used by motor to send motor data back
  - serial = port (usb)
  - T = # seconds the motor will run
  - W = speed in rad/s (example: 6.28 for 1 rev/s)
Output:
  - N/A
*/
void runTsec(int motor_id, MotorCmd &cmd, MotorData &data, SerialPort &serial, double T, double W) {
    using namespace std::chrono;           // set of tools for working with clock/time
    auto startClock = steady_clock::now(); //initialize clock
    std::cout <<  std::endl;               //change line
    std::cout <<  "NOW RUNNING MOTOR  "  << motor_id  << std::endl; // print to console
    usleep(1000000);                       //sleep for 10^6 microsec = 1s
    while (steady_clock::now() - startClock < seconds(T+1)) { 
        //while clock have not went over T sec, set up cmd info
        cmd.id    = motor_id;
        cmd.mode  = 1;    //mode 0 = stop, mode 1 = FOC = field oriented control
        cmd.kp    = 0.0;  //for position contorl. off.
        cmd.kd    = 0.01; //for speed control
        cmd.q     = 0.0;  //position control. off.
        cmd.dq     = W*6.33; //speed control, 6.28 is in rad/s, 6.33 = gear ratio
                                // so .dq = 6.28*6.33 means motor output is 6.28rad/s = 1 rev/s 
        cmd.tau     = 0.0;      // torque contorl. off.
        serial.sendRecv(&cmd,&data); //send cmd to & recieve data from motor
        if(data.correct == true) {
          print_MotorData(cmd,data); //print data to console
        }
        std::this_thread::sleep_for(milliseconds(500)); // controls repetition speed
    }
    cmd.mode  = 0;                  //stops the motor
    serial.sendRecv(&cmd,&data);    //sends command
}

/* print_MotorData
purpose:
  - print motor id, position, temp, speed, and torque to console
input:
  - cmd = command struct
  - data = data struct
output:
  - N/A
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