/********************************************************************************************
*        File: Umotor.h 														                                        *
*		  Version: 1.0 Initial release                                         				  		    *
*        Date: May 19th, 2022  	                                         			              *
*      Author: Olivier PERIA                                          					            *
* Description: Umotor class implementation used with the the uStepperS Arduino library      *
*    Features: - Gets the current position of the motor driver                              *
*              - Gets the encoder step counter value                                        *
*              - Gets the motor limit switch state                                          *
*              - Gets the motor state                                                       *
*              - Sets the motor state                                                       *
*              - Makes the motor rotate a predefined number of steps                        *
********************************************************************************************/

#ifndef Umotor_h
#define Umotor_h

#include <Arduino.h>
#include <uStepperS.h>

#define STEP_REVOLUTION 400   // Mumber of steps per revolution
#define MICRO_STEP      256   // Driver internal position counter is 256 microsteps per step

#define DIR_RIGHT  0  // Right direction
#define DIR_LEFT   1  // Left direction

#define MOTOR_STOPPED        0    // Motor mode Stopped
#define MOTOR_CHECKED        1    // Motor mode Schecked
#define MOTOR_RUNNING_RIGHT  2    // Motor mode Running Right
#define MOTOR_RUNNING_LEFT   3    // Motor mode Running Left


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
    int switchPin;
    uint8_t motorMode;
};

#endif
