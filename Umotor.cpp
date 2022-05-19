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
*                                                                                           *
* Initiates the motor limit switch pin and the motor mode                                   *
********************************************************************************************/
Umotor::Umotor(int pin) : uStepperS()
{
  switchPin = pin; 
  pinMode(switchPin, INPUT);  // Digital INPUT MODE

  motorMode = MOTOR_STOPPED;  // Initial motor mode
}



/********************************************************************************************
* Gets the current position of the driver from reference position. The reference position   *
* can be reseted at any time by using the driver.setHome() function                         *
*                                                                                           *
* return : number of fullsteps                                                              *
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
* return : number of fullsteps                                                              *
********************************************************************************************/

// Returns encoder step counter value
int32_t Umotor::getEncoderSteps()
{
  int32_t encStps;

  encStps = int32_t(0.5 + float(encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65536.0);  // 2^16 positions per revolution
  return(encStps);
}



/********************************************************************************************
* Gets the motor limit switch state                                                         *
*                                                                                           *
* return : true if switch pressed                                                           *
*          false if not                                                                     *
********************************************************************************************/
boolean Umotor::getLimitSwitchState()
{
  if (digitalRead(switchPin)) return(false);  // Limit switch not pressed
    else return(true);                        // Limit switch pressed
}



/********************************************************************************************
* Gets the motor state                                                                      *
*                                                                                           *
* return : MOTOR_STOPPED : motor is stopped                                                 *
*          MOTOR_CHECKED : lost steps checked and corrected                                 *
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
* input : mode : motor state - Possible values :                                            *
*                     MOTOR_STOPPED : motor is stopped                                      *
*                     MOTOR_CHECKED : lost steps checked and corrected                      *
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
  if (nstps) {
    int32_t nMstps;

    // Set motor acceleration and velocity
    setMaxAcceleration(acclr);
    setMaxVelocity(vlcty);
  
    switch (dir) {
      case DIR_RIGHT :  
        nMstps = int32_t(MICRO_STEP) * nstps; // Convert fullsteps to microsteps
        moveSteps(nMstps);                    // Rotate the motor nMstps microsteps in right direction
        motorMode = MOTOR_RUNNING_RIGHT;      // Set motor mode to Running Right
        break;
      case DIR_LEFT :  
        nMstps = int32_t(-1) * int32_t(MICRO_STEP) * nstps; // Convert fullsteps to microsteps
        moveSteps(nMstps);                                  // Rotate the motor nMstps microsteps in left direction
        motorMode = MOTOR_RUNNING_LEFT;                     // Set motor mode to Running Left
        break;
      };

    // Activates Motor brake
    setBrakeMode(COOLBRAKE);
    }
}
