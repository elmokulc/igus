/***/

#include <uStepperS.h>
#include "Dinput.h"

#define STEP_REVOLUTION 400 // Steps per revolution
#define MICRO_STEP      256 // Driver internal position counter is 256 microsteps per step

#define FAST_ACCELERATION 500 // 500 fullsteps/s^2
#define SLOW_ACCELERATION  75 //  75 fullsteps/s^2

#define FAST_VELOCITY 750 // 750 fullsteps/s
#define SLOW_VELOCITY 100 // 100 fullsteps/2

#define RIGHT_DIR  0 // Right direction
#define LEFT_DIR   1 // Left direction



uStepperS stepperMotor;
Dinput limitSwitch(2, INPUT);    // Limit switch digital input to pin D2

int16_t pStep;
int32_t nStep;

/*
SETUP
*/
void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  stepperMotor.setup(NORMAL,STEP_REVOLUTION);        //Initialisation of the uStepper S

  // Move carriage to initial podition
  MoveToInitialPosition();
  
  // Set Acceleration and velocity
  stepperMotor.setMaxAcceleration(FAST_ACCELERATION); // Fast acceleration
  stepperMotor.setMaxVelocity(FAST_VELOCITY);         // Fast velocity
}


/*
MAIN LOOP
*/
void loop() {
  if(Serial.available())
  {
    pStep = Serial.parseInt();      //Read nb steps from serial

    nStep = int32_t(-1) * int32_t(pStep) * int32_t(MICRO_STEP);
    Serial.println(nStep);

    // stepperMotor.moveSteps(DistanceToSteps(3.0, RIGHT_DIR));
    stepperMotor.moveSteps(nStep);
  }
}


/*
Move carriage to initial position
*/
void MoveToInitialPosition()
{
  stepperMotor.setMaxAcceleration(FAST_ACCELERATION); // Fast acceleration
  stepperMotor.setMaxVelocity(FAST_VELOCITY);         // Fast velocity

  // Serial.println(stepperMotor.encoder.getAngleRaw());  //print out angle moved since last reset

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel());         // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
    }
   
  // Move carriage 3 mm right
  stepperMotor.moveSteps(DistanceToSteps(3.0, RIGHT_DIR));

  
  while (stepperMotor.getMotorState(STANDSTILL));

  stepperMotor.setMaxAcceleration(SLOW_ACCELERATION); // Slow acceleration
  stepperMotor.setMaxVelocity(SLOW_VELOCITY);         // Slow velocity

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel());         // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
  }
  
  // Move carriage 3 mm right
  stepperMotor.moveSteps(DistanceToSteps(3.0, RIGHT_DIR));

  while (stepperMotor.getMotorState(STANDSTILL));
  
  // Serial.print(stepperMotor.encoder.getAngleRaw());  //print out angle moved since last reset
}


/*
Change distance (mm) to microsteps
*/
int32_t DistanceToSteps(float dist, uint8_t dir)
{
  int32_t nStps;
  
  switch (dir) {

    case RIGHT_DIR :  
      nStps = int32_t (dist * float(STEP_REVOLUTION) * float(MICRO_STEP) / -1.5);
      break;

    case LEFT_DIR :  
      nStps = int32_t (dist * float(STEP_REVOLUTION) * float(MICRO_STEP) / 1.5);
      break;
    }
  
  return(nStps);
}
