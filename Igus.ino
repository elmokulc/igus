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

#define CMD_DELIMITER   '#'    // Deliliter character
#define CMD_GOTORIGHT   "R"    // command : Move Right
#define CMD_GOTOLEFT    "L"    // command : Move Left 
#define CMD_GOHOME      "H"    // command : Move Home 
#define CMD_STOP        "S"    // command : Emergency Stop 
#define CMD_POSE        "P"    // command : Get Absolute Step Position 

#define ACK_HOME        "HOME POSITION"
#define ACK_REACHED     "POSITION REACHED"
#define ACK_RUNNING     "MOTOR RUNNING"
#define ACK_ABORTED     "ABORTED"
#define ACK_ERROR       "SYNTAX ERROR"
#define ACK_INVALID     "INVALID ARGUMENT"



Umotor stepperMotor(LIMIT_SWITCH);  // Instantiates stepperMotor object with limit switch connected to LIMIT_SWITCH pin

int32_t motorSteps = 0;   // Absolute step target
int32_t nSteps = 0;       // Relative step number



/********************************************************************************************
* Initialize the stepper motor and move to initial position                                 *
********************************************************************************************/
void setup() {
  Serial.begin(9600);       // Sets the data rate for serial data transmission to 9600 baud
  Serial.setTimeout(100);   // sets the maximum milliseconds to wait for serial data to 500
  
  // Setup stepperMotor
  stepperMotor.setup(NORMAL, STEP_REVOLUTION);  // Normal mode (see Arduino UstepperS library)

  // Set stepperMotor driver direction
  stepperMotor.driver.setShaftDirection(1);     // Inverted direction

  // Move carriage to initial podition
  MoveToInitialPosition(&stepperMotor);
  Serial.println(ACK_HOME);
}



/********************************************************************************************
* Main loop                                                                                 *
********************************************************************************************/
void loop() {

  // Read command sent to serial line

  if(Serial.available())
  {
    // If motor aborted after emergency stop command sent
    if (stepperMotor.getMotorMode() == MOTOR_ABORTED) {
      Serial.println(ACK_ABORTED);
      String str = Serial.readString(); // Clear serial buffer
      }
      // Else motor normaly working
      else {
        // Gets the String read from the serial buffer, up to the delimiter character CMD_DELIMITER if found,
        // or the entire buffer if not found. The delimiter character itself is not returned in the string
        String cmdString = Serial.readStringUntil(CMD_DELIMITER);  
    
        // command Move Right
        if (cmdString.equals(CMD_GOTORIGHT)) {
          // Looks for the next valid integer in the serial buffer, returns 0 if not found
          nSteps = int32_t(Serial.parseInt());  
          if ((motorSteps + nSteps) <= LIMIT_STEP_MAX) stepperMotor.rotateSteps(nSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
            else Serial.println(ACK_INVALID);
          }
          // command Move Light
          else if (cmdString.equals(CMD_GOTOLEFT)) {
            // Looks for the next valid integer in the serial buffer, returns 0 if not found
            nSteps = int32_t(Serial.parseInt());  
            if ((motorSteps - nSteps) >= LIMIT_STEP_MIN) stepperMotor.rotateSteps(nSteps, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
              else Serial.println(ACK_INVALID);
            }
            // command Move Home
            else if (cmdString.equals(CMD_GOHOME)) {
              MoveToInitialPosition(&stepperMotor);
              motorSteps = 0;
              nSteps = 0;
              Serial.println(ACK_HOME);
            }
            // command Emergency Stop
            else if (cmdString.equals(CMD_STOP)) {
              stepperMotor.stop(HARD);
              stepperMotor.setMotorMode(MOTOR_ABORTED);
              Serial.println(ACK_ABORTED);
              } 
              // command Get Step Position
              else if (cmdString.equals(CMD_POSE)) {
                uint8_t mMode = stepperMotor.getMotorMode();
                if ((mMode == MOTOR_RUNNING_RIGHT) || (mMode == MOTOR_RUNNING_LEFT)) Serial.println(stepperMotor.getDriverSteps()); 
                  else Serial.println(motorSteps);
                }
                else Serial.println(ACK_ERROR);
        }
  }

  // Lost steps test and correct

  uint8_t mMode = stepperMotor.getMotorMode();
  if (mMode != MOTOR_ABORTED) {
    // Motor not Checked and Driver position reached
    if ((mMode != MOTOR_CHECKED) && (stepperMotor.getMotorState(POSITION_REACHED) == 0)) {              
      if (mMode == MOTOR_RUNNING_RIGHT) motorSteps += nSteps;
        else if (mMode == MOTOR_RUNNING_LEFT) motorSteps -= nSteps;
      checkSteps(&stepperMotor, motorSteps); // Check and correct lost steps
      Serial.println(ACK_REACHED);
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

  // Set motor mode to CHECKED (position reached)
  motor->setMotorMode(MOTOR_CHECKED);
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
  int32_t encSteps = motor->getEncoderSteps();
  int32_t deltaSteps = msteps - encSteps;

  // Serial.print("targetStep = "); Serial.println(msteps);
  // Serial.print("encoderStep = "); Serial.println(encSteps);
  // Serial.print("deltaStep = "); Serial.println(deltaSteps);

  while (deltaSteps != 0) {
    if (deltaSteps > 0) motor->rotateSteps(1, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      else motor->rotateSteps(1, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);
    encSteps = motor->getEncoderSteps();
    deltaSteps = msteps - encSteps;

    // Serial.print("targetStep = "); Serial.println(msteps);
    // Serial.print("encoderStep = "); Serial.println(encSteps);
    // Serial.print("deltaStep = "); Serial.println(deltaSteps);
    }

  motor->setMotorMode(MOTOR_CHECKED);
}



/********************************************************************************************
* Change distance to steps                                                                  *
*                                                                                           *
*  input : dist : distance in mm                                                            *
* return : fullsteps                                                                        *
********************************************************************************************/
int32_t DistanceToSteps(float dist)
{
  int32_t nStps = int32_t (0.5 + dist * float(STEP_REVOLUTION) / 1.5);  // 1.5 mm per revolution
      
  return(nStps);
}
