/***/

#include "Dinput.h"
#include "Umotor.h"

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

#define CMD_GOTORIGHT "R"
#define CMD_GOTOLEFT  "L"
#define CMD_GOHOME    "H"
#define CMD_STOP      "S"
#define CMD_INFO      "I"

#define LIMIT_SWITCH_PIN  2


Umotor stepperMotor(LIMIT_SWITCH_PIN);
int32_t nStep;



/************************************************************************
SETUP
*************************************************************************/
void setup() {
  Serial.begin(9600);
  
  // Setup uStepperS
  stepperMotor.setup(NORMAL,STEP_REVOLUTION);         // Normal mode

  // Move carriage to initial podition
  MoveToInitialPosition(&stepperMotor);
}



/************************************************************************
MAIN LOOP
*************************************************************************/
void loop() {
  if(Serial.available())
  {
    String incomingByte = Serial.readStringUntil('#');
    // Serial.print("cmd = "); Serial.println(incomingByte);

    if (incomingByte.equals(CMD_GOTORIGHT)) {
      nStep = int32_t(Serial.parseInt());
      if ((stepperMotor.driverStep + nStep) <= LIMIT_STEP_MAX) rotateSteps(&stepperMotor, nStep, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      }
      else if (incomingByte.equals(CMD_GOTOLEFT)) {
        nStep = int32_t(Serial.parseInt());
        if ((stepperMotor.driverStep - nStep) >= LIMIT_STEP_MIN) rotateSteps(&stepperMotor, nStep, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
        }
        else if (incomingByte.equals(CMD_GOHOME)) {
          MoveToInitialPosition(&stepperMotor);
          stepperMotor.driverStep = 0;
        }
        else if (incomingByte.equals(CMD_STOP)) {
          stepperMotor.stop(HARD);
          stepperMotor.motorStatus = MOTOR_STOPPED;
          } 
          else if (incomingByte.equals(CMD_INFO)) {
            stepperMotor.encoderStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);
            Serial.print("encoderStep = "); Serial.println(stepperMotor.encoderStep);
            Serial.print("driverStep = "); Serial.println(stepperMotor.driverStep);
            }  
      
    incomingByte = Serial.read(); // read CR
  }
  
  if (stepperMotor.motorStatus != MOTOR_STOPPED) {
    if (stepperMotor.getMotorState(POSITION_REACHED) == 0) {
      Serial.println("POSITION REACHED");
      if (stepperMotor.motorStatus == MOTOR_RUNNING_RIGHT) stepperMotor.driverStep += nStep;
        else if (stepperMotor.motorStatus == MOTOR_RUNNING_LEFT) stepperMotor.driverStep -= nStep;
      stepperMotor.encoderStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);
      checkSteps(&stepperMotor); // Check and correct lost steps
      }
    }
}



/************************************************************************
Move carriage to initial position
*************************************************************************/
void MoveToInitialPosition(class Umotor *motor)
{
  int32_t nMstps;
  
  // Set motor acceleration and velocity
  motor->setMaxAcceleration(ACCELERATION_FAST); // Fast acceleration
  motor->setMaxVelocity(VELOCITY_FAST);         // Fast velocity

  // Move carriage left to limit switch
  if (!motor->getLimitSwitchStatus()){                 // Limit switch not pressed (HIGH)
    motor->runContinous(CW);                 // Move carriage left
    while (!motor->getLimitSwitchStatus());  // Wait for limit switch pressed
    motor->stop(HARD);                       // Stop carriage
    }

   // Move carriage 600 steps right
  nMstps = int32_t(-1) * int32_t(MICRO_STEP) * 600; // Convert steps to microsteps
  motor->moveSteps(nMstps);

  // Wait for motor stopped
  while (stepperMotor.getMotorState(POSITION_REACHED));

  // Set motor acceleration and velocity
  motor->setMaxAcceleration(ACCELERATION_SLOW); // Slow acceleration
  motor->setMaxVelocity(VELOCITY_SLOW);         // Slow velocity

  // Move carriage left to limit switch
  if (!motor->getLimitSwitchStatus()){                 // Limit switch not pressed (HIGH)
    motor->runContinous(CW);                 // Move carriage left
    while (!motor->getLimitSwitchStatus());  // Wait for limit switch pressed
    motor->stop(HARD);                       // Stop carriage
  }

  // Move carriage 800 steps right
  nMstps = int32_t(-1) * int32_t(MICRO_STEP) * 800; // Convert steps to microsteps
  motor->moveSteps(nMstps);

  // Wait for position reached
  while (stepperMotor.getMotorState(POSITION_REACHED));

  // Reset motor driver step counter
  motor->driver.setHome();
  motor->driverStep = 0;

   // Reset motor encoder step counter
  motor->encoder.setHome();
  motor->encoderStep = 0;

  // Set motor status
  motor->motorStatus = MOTOR_STOPPED;
}



/************************************************************************ 
Makes the motor rotate n steps
*************************************************************************/
void rotateSteps(class Umotor *motor, int32_t nstps, uint8_t dir, float acceleration, float velocity)
{
  int32_t nMstps;
  uint8_t mStatus = MOTOR_STOPPED;

  // Set motor acceleration and velocity
  motor->setMaxAcceleration(acceleration);
  motor->setMaxVelocity(velocity);
  
  switch (dir) {
    case DIR_RIGHT :  
      nMstps = int32_t(-1) * int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      motor->moveSteps(nMstps);
      motor->motorStatus = MOTOR_RUNNING_RIGHT;
      break;
    case DIR_LEFT :  
      nMstps = int32_t(MICRO_STEP) * nstps; // Convert steps to microsteps
      motor->moveSteps(nMstps);
      motor->motorStatus = MOTOR_RUNNING_LEFT;
      break;
    };
}



/************************************************************************ 
Check and correct lost steps
*************************************************************************/
void checkSteps(class Umotor *motor)
{
  int32_t drvStep, encStep, deltaStep;


  deltaStep = motor->driverStep - motor->encoderStep;

  Serial.print("driverStep = "); Serial.println(motor->driverStep);
  Serial.print("encoderStep = "); Serial.println(motor->encoderStep);
  Serial.print("deltaStep = "); Serial.println(deltaStep);

  while (deltaStep != 0) {
    if (deltaStep > 0) rotateSteps(motor, 1, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      else rotateSteps(motor, 1, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);
    motor->encoderStep = int32_t(0.5 + float(stepperMotor.encoder.getAngleMovedRaw()) * float(STEP_REVOLUTION) / 65535.0);  
    deltaStep = motor->driverStep - motor->encoderStep;
    Serial.print("driverStep = "); Serial.println(motor->driverStep);
    Serial.print("encoderStep = "); Serial.println(motor->encoderStep);
    Serial.print("deltaStep = "); Serial.println(deltaStep);
    }

  motor->motorStatus = MOTOR_STOPPED;
}



/************************************************************************
Change distance (mm) to steps
*************************************************************************/
int32_t DistanceToSteps(float dist)
{
  int32_t nStps = int32_t (dist * float(STEP_REVOLUTION) / 1.5);  // 1.5 mm/round
      
  return(nStps);
}
