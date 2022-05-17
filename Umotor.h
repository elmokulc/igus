/*
  Umotor.h - UstepperS motor
*/

#ifndef Umotor_h
#define Umotor_h

#include <Arduino.h>
#include <uStepperS.h>

#define STEP_REVOLUTION 400   // Steps per revolution
#define MICRO_STEP      256   // Driver internal position counter is 256 microsteps per step

#define DIR_RIGHT  0  // Right direction
#define DIR_LEFT   1  // Left direction

#define MOTOR_STOPPED        0
#define MOTOR_CHECKED        1
#define MOTOR_RUNNING_RIGHT  2
#define MOTOR_RUNNING_LEFT   3


class Umotor : public uStepperS
{
  public:
    Umotor(int pin);
    int32_t getDriverSteps();
    int32_t getEncoderSteps();
    boolean getLimitSwitchState();
    uint8_t getMotorMode();
    void setMotorMode(uint8_t mode);
    void rotateSteps(int32_t nstps, uint8_t dir, float acceleration, float velocity);
   
  private:
    int inPin;
    uint8_t motorMode;
};

#endif
