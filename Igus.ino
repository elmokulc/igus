/********************************************************************************************
*        File: Igus.ino   													                                        *
*		  Version: 1.0 Initial release                                         				  		    *
*        Date: May 19th, 2022  	                                         			              *
*      Author: Olivier PERIA                                          					            *
* Description: Drives Linear Motion Module IGUS                                             *
*    Features:                                                                              *
********************************************************************************************/

#include "Umotor.h"

#define LIMIT_STEP_MIN  0      // Motor Home position
#define LIMIT_STEP_MAX  45000  // Motor maximum position

#define ACCELERATION_SLOW 200  // Slow acceleration (steps/s^2)
#define ACCELERATION_FAST 1000 // Fast acceleration (steps/s^2)

#define VELOCITY_SLOW   200    // Slow velocity (steps/s)
#define VELOCITY_FAST   3000   // Fast velocity (steps/s)

#define LIMIT_SWITCH    PD2    // Limit switch pin

#define CMD_CR          '\n'
#define CMD_DELIMITER   '#'
#define CMD_GOTORIGHT   "R"
#define CMD_GOTOLEFT    "L"
#define CMD_GOHOME      "H"
#define CMD_STOP        "S"
#define CMD_INFO        "I"

#define ACK_ERROR         "ERROR"
#define ACK_REACHED       "REACHED"
#define ACK_SYNTAX_ERROR  "SYNTAX ERROR"


Umotor stepperMotor(LIMIT_SWITCH);  // Instantiate stepperMotor object with limit switch connected to LIMIT_SWITCH pin

int32_t motorSteps = 0;   // Absolute step target
int32_t nSteps = 0;       // Relative step number



/********************************************************************************************
* Initialize the stepper motor and move to initial position                                     *
********************************************************************************************/
void setup() {
  Serial.begin(9600);
  
  // Setup stepperMotor
  stepperMotor.setup(NORMAL, STEP_REVOLUTION);    // Normal mode

  // Set stepperMotor driver direction
  stepperMotor.driver.setShaftDirection(1);   // Inverted direction

  // Move carriage to initial podition
  MoveToInitialPosition(&stepperMotor);
}



/********************************************************************************************
* Main loop                                                                                 *
********************************************************************************************/
void loop() {
  if(Serial.available())
  {
    String incomingByte = Serial.readStringUntil(CMD_DELIMITER);
    
    if (incomingByte.equals(CMD_GOTORIGHT)) {
      nSteps = int32_t(Serial.parseInt());
      if ((motorSteps + nSteps) <= LIMIT_STEP_MAX) stepperMotor.rotateSteps(nSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      }
      else if (incomingByte.equals(CMD_GOTOLEFT)) {
        nSteps = int32_t(Serial.parseInt());
        if ((motorSteps - nSteps) >= LIMIT_STEP_MIN) stepperMotor.rotateSteps(nSteps, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
        }
        else if (incomingByte.equals(CMD_GOHOME)) {
          MoveToInitialPosition(&stepperMotor);
          motorSteps = 0;
          nSteps = 0;
        }
        else if (incomingByte.equals(CMD_STOP)) {
          stepperMotor.stop(HARD);
          } 
          else if (incomingByte.equals(CMD_INFO)) {
            Serial.print("targetStep = "); Serial.println(motorSteps);
            Serial.print("encoderStep = "); Serial.println(stepperMotor.getEncoderSteps());
            }
            else Serial.println(ACK_SYNTAX_ERROR);
  }

  if (stepperMotor.getMotorMode() != MOTOR_CHECKED) {
    Serial.println("MOTOR NOT CHECKED");
    if (stepperMotor.getMotorState(POSITION_REACHED) == 0) {          // Position reached
      Serial.println("POSITION REACHED");
      if (stepperMotor.getMotorMode() == MOTOR_RUNNING_RIGHT) motorSteps += nSteps;
        else if (stepperMotor.getMotorMode() == MOTOR_RUNNING_LEFT) motorSteps -= nSteps;
      checkSteps(&stepperMotor, motorSteps); // Check and correct lost steps
      }
  }
}



/********************************************************************************************
* Move carriage to initial position                                                         *
*                                                                                           *
* input : motor : stepper motor                                                             *
********************************************************************************************/
void MoveToInitialPosition(class Umotor *motor)
{
  int32_t nMstps;
  
  // Set motor acceleration and velocity
  motor->setMaxAcceleration(ACCELERATION_FAST); // Fast acceleration
  motor->setMaxVelocity(VELOCITY_FAST);         // Fast velocity

  // Move carriage left to limit switch
  if (!motor->getLimitSwitchState()) {      // Limit switch not pressed
    motor->runContinous(CCW);               // Move carriage left
    while (!motor->getLimitSwitchState());  // Wait for limit switch pressed
    motor->stop(HARD);                      // Stop carriage
    }

  // Move carriage 600 steps right
  motor->rotateSteps(600, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);

  // Wait for motor stopped
  while (motor->getMotorState(POSITION_REACHED));

  // Set motor acceleration and velocity
  motor->setMaxAcceleration(ACCELERATION_SLOW); // Slow acceleration
  motor->setMaxVelocity(VELOCITY_SLOW);         // Slow velocity

  // Move carriage left to limit switch
  if (!motor->getLimitSwitchState()){       // Limit switch not pressed (HIGH)
    motor->runContinous(CCW);               // Move carriage left
    while (!motor->getLimitSwitchState());  // Wait for limit switch pressed
    motor->stop(HARD);                      // Stop carriage
  }

  // Move carriage 800 steps right
  motor->rotateSteps(800, DIR_RIGHT, ACCELERATION_SLOW, VELOCITY_SLOW);

  // Wait for position reached
  while (motor->getMotorState(POSITION_REACHED));

  // Reset motor driver and encoder step counter
  motor->driver.setHome();
  motor->encoder.setHome();

  // Activates Motor brake
  motor->setBrakeMode(COOLBRAKE);

  // Set motor mode to STOPPED
  motor->setMotorMode(MOTOR_STOPPED);
}



/********************************************************************************************
* Check and correct lost steps                                                              *
*                                                                                           *
* Compare the encoder step counter to the motor step target. If different, move the motor   *
* to make the encoder step counter equal to the motor step target                           *
*                                                                                           *
* input : motor : stepper motor                                                             *
*         mstep : motor step target                                                         *
********************************************************************************************/

void checkSteps(class Umotor *motor, int32_t msteps)
{
  int32_t encSteps, deltaSteps;

  encSteps = motor->getEncoderSteps();
  deltaSteps = msteps - encSteps;

  Serial.print("targetStep = "); Serial.println(msteps);
  Serial.print("encoderStep = "); Serial.println(encSteps);
  Serial.print("deltaStep = "); Serial.println(deltaSteps);

  while (deltaSteps != 0) {
    if (deltaSteps > 0) motor->rotateSteps(1, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      else motor->rotateSteps(1, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);
    encSteps = motor->getEncoderSteps();
    deltaSteps = msteps - encSteps;
    Serial.print("targetStep = "); Serial.println(msteps);
    Serial.print("encoderStep = "); Serial.println(encSteps);
    Serial.print("deltaStep = "); Serial.println(deltaSteps);
    }

  motor->setMotorMode(MOTOR_CHECKED);
  Serial.println("MOTOR CHECKED");
}



/********************************************************************************************
* Change distance to steps                                                                  *
*                                                                                           *
*  input : dist : distance in mm                                                            *
* return : fullsteps                                                                        *
********************************************************************************************/
int32_t DistanceToSteps(float dist)
{
  int32_t nStps = int32_t (dist * float(STEP_REVOLUTION) / 1.5);  // 1.5 mm per revolution
      
  return(nStps);
}
