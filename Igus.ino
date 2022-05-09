/***/

#include <uStepperS.h>
#include "Dinput.h"

#define STEP_REVOLUTION  400  // Steps per revolution
#define MICRO_STEPS  256      // Driver internal position counter is 256 microsteps per step


uStepperS stepperMotor;
Dinput limitSwitch(2, INPUT);    // Limit switch digital input to pin D2

int32_t angle = 360.0;        //amount of degrees to move
// int32_t nStep = int32_t((angle * float(STEP_REVOLUTION) * float(MICRO_STEPS))/ 360.0);
int32_t nStep = -3 * int32_t(STEP_REVOLUTION) * int32_t(MICRO_STEPS);


void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  stepperMotor.setup(NORMAL,STEP_REVOLUTION);        //Initialisation of the uStepper S

  // Move carriage to initial podition
  goToInitialPosition();
}

void loop() {
  // put your main code here, to run repeatedly:
  // if(!stepperMotor.getMotorState())          //If motor is at standstill
  // {
    // delay(1000);

    // Serial.print("Angle: "); Serial.println(angle); 
    // Serial.print("Steps: "); Serial.println(nStep); 
    
    // stepperMotor.moveAngle(angle);
    // angle = -angle;
    
    // stepperMotor.moveSteps(nStep);           //start new movement
    // nStep = -nStep;                     //invert angle variable, so the next move is in opposite direction
  // }
   // Serial.print("Angle: ");
   // Serial.print(stepperMotor.encoder.getAngleMoved());       //print out angle moved since last reset
   // Serial.println(" Degrees");
}


/*
Move carriage to initial position
*/
void goToInitialPosition()
{
  stepperMotor.setMaxAcceleration(500);     //use an acceleration of 2000 fullsteps/s^2
  stepperMotor.setMaxVelocity(500);         //Max velocity of 500 fullsteps/s

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel()) {}       // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
    }

  // Move carriage nSteps right
  stepperMotor.moveSteps(nStep);

  delay(3000);

  stepperMotor.setMaxAcceleration(100);
  stepperMotor.setMaxVelocity(100);

  // Move carriage left to limit switch
  if (limitSwitch.getLevel()){              // Limit switch not pressed (HIGH)
    stepperMotor.runContinous(CW);          // Move carriage left
    while (limitSwitch.getLevel()) {}       // Wait for limit switch pressed
    stepperMotor.stop(HARD);                // Stop carriage
  }
  
  // Move carriage 10000 steps right
  stepperMotor.moveSteps(nStep);
}
