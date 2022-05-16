/*
  Umotor.h - UstepperS motor
*/

#ifndef Umotor_h
#define Umotor_h

#include <Arduino.h>
#include <uStepperS.h>


class Umotor : public uStepperS
{
  public:
    Umotor(int pin);
    boolean getLimitSwitchStatus();
    int32_t encoderStep, driverStep;
    uint8_t motorStatus;
  private:
    int inPin;
};

#endif
