/*
  Umotor.cpp - UstepperS Motor
*/

#include "Umotor.h"



// Instantiate a new UstepperS Motor object
Umotor::Umotor(int pin) : uStepperS()
{
  inPin = pin; 
  pinMode(inPin, INPUT);

  motorMode = MOTOR_STOPPED;
}



// Returns driver step counter value
int32_t Umotor::getDriverSteps()
{
  int32_t drvStps;

  drvStps = int32_t(0.5 + float(driver.getPosition()) / float(MICRO_STEP));
  return(drvStps);
}



// Returns encoder step counter value
int32_t Umotor::getEncoderSteps()
{
  int32_t encStps;

  encStps = int32_t(0.5 + float(encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);
  return(encStps);
}



// Returns limit switch status (switch pressed / not pressed)
boolean Umotor::getLimitSwitchState()
{
  if (digitalRead(inPin)) return(false);  // Limit switch not pressed : returns FALSE
    else return(true);                    // Limit switch pressed : returns TRUE
}



// Returns motor mode (stopped / running right / runing left)
uint8_t Umotor::getMotorMode()
{
  return(motorMode);
}



// Sets motor mode (stopped / running right / runing left)
void Umotor::setMotorMode(uint8_t mode)
{
  motorMode = mode;
}



// Makes the motor rotate n steps right or left
void Umotor::rotateSteps(int32_t nstps, uint8_t dir, float acceleration, float velocity)
{
  int32_t nMstps;

  // Set motor acceleration and velocity
  setMaxAcceleration(acceleration);
  setMaxVelocity(velocity);
  
  switch (dir) {
    case DIR_RIGHT :  
      nMstps = int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      moveSteps(nMstps);
      motorMode = MOTOR_RUNNING_RIGHT;
      break;
    case DIR_LEFT :  
      nMstps = int32_t(-1) * int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      moveSteps(nMstps);
      motorMode = MOTOR_RUNNING_LEFT;
      break;
    };

  // Activates Motor brake
  setBrakeMode(COOLBRAKE);
}
