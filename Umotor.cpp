/********************************************************************************************
*        File: Umotor.ccp														                                        *
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

#include "Umotor.h"



/********************************************************************************************
* Umotor class constructor                                                                  *
* Inherites of uStepperS class defined in uStepperS Arduino library                         *
* Initiates the motor limit switch pin and the motor mode                                   *
*                                                                                           *
* input : pin : Limit switch pin                                                            *
********************************************************************************************/
Umotor::Umotor(int pin) : uStepperS()
{
  switchPin = pin; 
  pinMode(switchPin, INPUT);  // Sets digital pin mode to INPUT

  motorMode = MOTOR_INIT;     // Sets initial motor mode to INIT
}



/********************************************************************************************
* Gets the current position of the driver from reference position. The reference position   *
* can be reseted at any time by using the driver.setHome() function                         *
*                                                                                           *
* return : Number of fullsteps                                                              *
********************************************************************************************/
int32_t Umotor::getDriverSteps()
{
  int32_t drvStps;

  // Position read in microsteps and converted to fullsteps (MICRO_STEP microsteps per fullsteps)
  drvStps = int32_t(0.5 + float(driver.getPosition()) / float(MICRO_STEP));   
  return(drvStps);
}



/********************************************************************************************
* Gets the encoder step counter value from reference position. The reference position can   *
* be reseted at any time by using the encoder.setHome() function                            *
*                                                                                           *
* return : Number of fullsteps                                                              *
********************************************************************************************/

// Returns encoder step counter value
int32_t Umotor::getEncoderSteps()
{
  int32_t encStps;

  // Position read in raw data and converted to fullsteps (2^16 positions per revolution)
  encStps = int32_t(0.5 + float(encoder.getAngleMovedRaw(false)) * float(STEP_REVOLUTION) / 65536.0);
  return(encStps);
}



/********************************************************************************************
* Gets the motor limit switch state                                                         *
*                                                                                           *
* return : true  : Switch activated                                                         *
*          false : Switch not activated                                                     *
********************************************************************************************/
boolean Umotor::getLimitSwitchState()
{
  if (digitalRead(switchPin)) return(false);  // Limit switch not pressed
    else return(true);                        // Limit switch pressed
}



/********************************************************************************************
* Gets the motor state                                                                      *
*                                                                                           *
* return : MOTOR_STOPPED : motor normaly stopped                                            *
*          MOTOR_ABORTED : motor stopped after emergency stop                               *
*          MOTOR_CHECKED : lost steps checked and corrected (position reached)              *
*          MOTOR_RUNNING_RIGHT : motor is moving right                                      *
*          MOTOR_RUNNING_LEFT  : motor is moving left                                       *
********************************************************************************************/
uint8_t Umotor::getMotorMode()
{
  return(motorMode);
}



/********************************************************************************************
* Sets the motor state                                                                      *
*                                                                                           *
* input : mode : Motor state - Possible values :                                            *
*                     MOTOR_STOPPED : motor normaly stopped                                 *
*                     MOTOR_ABORTED : motor stopped after emergency stop                    *
*                     MOTOR_CHECKED : lost steps checked and corrected (position reached)   *
*                     MOTOR_RUNNING_RIGHT : motor is moving right                           *
*                     MOTOR_RUNNING_LEFT  : motor is moving left                            *
********************************************************************************************/
void Umotor::setMotorMode(uint8_t mode)
{
  motorMode = mode;
}



/********************************************************************************************
* Makes the motor rotate a predefined number of steps, in right or left direction, using    *
* acceleration and velocity profile                                                         *
*                                                                                           *
* input : nstps :   number of fullsteps                                                     *
*         dir   :   right (CW) or left (CCW) direction                                      *
*         acclr :   maximum acceleration in steps/s^2                                       *
*         vlcty :   maximum velocity in steps/s                                             * 
********************************************************************************************/
void Umotor::rotateSteps(int32_t nstps, uint8_t dir, float acclr, float vlcty)
{
  if (nstps > 0) {

    int32_t nMstps;

    // Set motor acceleration and velocity
    setMaxAcceleration(acclr);
    setMaxVelocity(vlcty);
  
    switch (dir) {
      case DIR_RIGHT :  
        nMstps = int32_t(MICRO_STEP) * nstps; // Converts fullsteps to microsteps
        motorMode = MOTOR_RUNNING_RIGHT;      // Sets motor mode to Running Right
        moveSteps(nMstps);                    // Rotates the motor nMstps microsteps in right direction 
        break;
      case DIR_LEFT :  
        nMstps = int32_t(-1) * int32_t(MICRO_STEP) * nstps; // Converts fullsteps to microsteps
        motorMode = MOTOR_RUNNING_LEFT;                     // Sets motor mode to Running Left
        moveSteps(nMstps);                                  // Rotates the motor nMstps microsteps in left direction
        break;
      };
    }
}
