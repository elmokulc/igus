/***/

#include <uStepperS.h>
#include "Dinput.h"

#define STEP_REVOLUTION 400 // Steps per revolution
#define MICRO_STEP      256 // Driver internal position counter is 256 microsteps per step

#define LIMIT_STEP_MIN  0
#define LIMIT_STEP_MAX  45000

#define MOTOR_STOPPED        0
#define MOTOR_RUNNING_RIGHT  1
#define MOTOR_RUNNING_LEFT   2

#define ACCELERATION_SLOW 200  // 200 steps/s^2
#define ACCELERATION_FAST 1000 // 1000 steps/s^2

#define VELOCITY_SLOW     200  // 200 steps/2
#define VELOCITY_FAST     3000 // 3000 steps/s

#define DIR_RIGHT  0 // Right direction
#define DIR_LEFT   1 // Left direction

#define CMD_GOTORIGHT 0x31  // 1
#define CMD_GOTOLEFT  0x32  // 2
#define CMD_RESET     0x33  // 3
#define CMD_STOP      0x34  // 4
#define CMD_POSE      0x35  // 5

uStepperS stepperMotor;
Dinput limitSwitch(2, INPUT);    // Limit switch to digital input D2

int32_t encoderStep, motorStep, deltaStep, nStep;
uint8_t motorStatus, incomingByte;



/************************************************************************
SETUP
*************************************************************************/
void setup() {
  Serial.begin(9600);
  
  // Setup uStepperS
  stepperMotor.setup(NORMAL,STEP_REVOLUTION);         // Normal mode

  // Move carriage to initial podition
  motorStatus = MoveToInitialPosition();

  // reset motor step counter
  motorStep = 0;
}



/************************************************************************
MAIN LOOP
*************************************************************************/
void loop() {
  if(Serial.available())
  {
    incomingByte = Serial.read();
    // Serial.print("cmd = "); Serial.println(incomingByte);

    switch (incomingByte) {
      case CMD_GOTORIGHT :
        nStep = int32_t(Serial.parseInt());
        if ((motorStep + nStep) <= LIMIT_STEP_MAX) motorStatus = rotateSteps(nStep, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
        break;
      case CMD_GOTOLEFT :  
        nStep = int32_t(Serial.parseInt());
        if ((motorStep - nStep) >= LIMIT_STEP_MIN) motorStatus = rotateSteps(nStep, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
        break;
      case CMD_RESET :  
        motorStatus = MoveToInitialPosition();
        motorStep = 0;
        break; 
      case CMD_STOP :  
        stepperMotor.stop(HARD);
        motorStatus = MOTOR_STOPPED;
        break; 
      case CMD_POSE :  
        encoderStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);
        Serial.print("encoderStep = "); Serial.println(encoderStep);
        Serial.print("motorStep = "); Serial.println(motorStep);
        break;   
      }
    incomingByte = Serial.read(); // read CR
  }
  
  if (motorStatus != MOTOR_STOPPED) {
    if (stepperMotor.getMotorState(POSITION_REACHED) == 0) {
      Serial.println("POSITION REACHED");
      if (motorStatus == MOTOR_RUNNING_RIGHT) motorStep += nStep;
        else if (motorStatus == MOTOR_RUNNING_LEFT) motorStep -= nStep;
      motorStatus = checkSteps(motorStep); // Check and correct lost steps
      }
    }
}



/************************************************************************
Move carriage to initial position
*************************************************************************/
uint8_t MoveToInitialPosition()
{
  int32_t nMstps;
  
  // Set motor acceleration and velocity
  stepperMotor.setMaxAcceleration(ACCELERATION_FAST); // Fast acceleration
  stepperMotor.setMaxVelocity(VELOCITY_FAST);         // Fast velocity

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel());         // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
    }

   // Move carriage 600 steps right
  nMstps = int32_t(-1) * int32_t(MICRO_STEP) * 600; // Convert steps to microsteps
  stepperMotor.moveSteps(nMstps);

  // Wait for motor stopped
  while (stepperMotor.getMotorState(POSITION_REACHED));

  // Set motor acceleration and velocity
  stepperMotor.setMaxAcceleration(ACCELERATION_SLOW); // Slow acceleration
  stepperMotor.setMaxVelocity(VELOCITY_SLOW);         // Slow velocity

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel());         // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
  }

  // Move carriage 800 steps right
  nMstps = int32_t(-1) * int32_t(MICRO_STEP) * 800; // Convert steps to microsteps
  stepperMotor.moveSteps(nMstps);

  // Wait for motor stopped
  while (stepperMotor.getMotorState(POSITION_REACHED));

  // Reset motor encoder and driver
  stepperMotor.encoder.setHome();
  stepperMotor.driver.setHome();

  return (MOTOR_STOPPED);
}



/************************************************************************ 
Makes the motor rotate n steps
*************************************************************************/
uint8_t rotateSteps(int32_t nstps, uint8_t dir, float acceleration, float velocity)
{
  int32_t nMstps;
  uint8_t mStatus = MOTOR_STOPPED;

  // Set motor acceleration and velocity
  stepperMotor.setMaxAcceleration(acceleration);
  stepperMotor.setMaxVelocity(velocity);
  
  switch (dir) {
    case DIR_RIGHT :  
      nMstps = int32_t(-1) * int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      stepperMotor.moveSteps(nMstps);
      mStatus = MOTOR_RUNNING_RIGHT;
      break;
    case DIR_LEFT :  
      nMstps = int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      stepperMotor.moveSteps(nMstps);
      mStatus = MOTOR_RUNNING_LEFT;
      break;
    }
  return (mStatus);
}



/************************************************************************ 
Check and correct lost steps
*************************************************************************/
uint8_t checkSteps(int32_t mstep)
{
  int32_t encodStep, deltaStep;

  encodStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);
  deltaStep = mstep - encodStep;

  Serial.print("motorStep = "); Serial.println(mstep);
  Serial.print("encoderStep = "); Serial.println(encodStep);
  Serial.print("deltaStep = "); Serial.println(deltaStep);

  while (deltaStep != 0) {
    if (deltaStep > 0) rotateSteps(1, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      else rotateSteps(1, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);
    encodStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);  
    deltaStep = mstep - encodStep;
    Serial.print("motorStep = "); Serial.println(mstep);
    Serial.print("encoderStep = "); Serial.println(encodStep);
    Serial.print("deltaStep = "); Serial.println(deltaStep);
    }

  return(MOTOR_STOPPED);
}



/************************************************************************
Change distance (mm) to steps
*************************************************************************/
int32_t DistanceToSteps(float dist)
{
  int32_t nStps = int32_t (dist * float(STEP_REVOLUTION) / 1.5);  // 1.5 mm/round
      
  return(nStps);
}
