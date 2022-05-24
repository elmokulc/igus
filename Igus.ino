/********************************************************************************************
*        File: Igus.ino   													                                        *
*		  Version: 1.0 Initial release                                         				  		    *
*        Date: May 19th, 2022  	                                         			              *
*      Author: Olivier PERIA                                          					            *
* Description: Drives Linear Motion Module IGUS                                             *
*    Features: Possible commands sent on serial line :                                      *
*                 H or H# : Moving Home                                                     *
*                 P or P# : Getting absolute current step Position                          *
*                 S or S# : Emmergency Stop                                                 *
*                 R#[arg] : Moving [arg] steps Right - Example: R#10000                     *
*                 L#[arg] : Moving [arg] steps Left                                         *
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
#define CMD_POSE        "P"    // command : Get absolute current step Position 

#define ACK_START       "MOVING..."
#define ACK_HOME        "HOME POSITION"
#define ACK_INIT        "MOVE HOME FIRST"
#define ACK_REACHED     "POSITION REACHED"
#define ACK_WAIT        "INVALID COMMAND WHILE MOVING"
#define ACK_ERROR       "SYNTAX ERROR"
#define ACK_INVALID     "INVALID ARGUMENT"
#define ACK_OUTRANGE    "ARGUMENT OUT OF RANGE"
#define ACK_ABORTED     "ABORTED"
#define ACK_RESET       "MOTOR MUST BE RESETED"

typedef struct 
{
  int32_t motorSteps = 0;   // Absolute step target
  int32_t nSteps = 0;       // Relative step number to move

} step_rcd;

Umotor stepperMotor(LIMIT_SWITCH);  // Instantiates stepperMotor object with limit switch connected to LIMIT_SWITCH pin

step_rcd stepCntr;  // Step counter



/********************************************************************************************
* Initializes the stepper motor and move to initial position                                 *
********************************************************************************************/
void setup() {
  Serial.begin(9600);       // Sets the data rate for serial data transmission to 9600 baud
  Serial.setTimeout(100);   // sets the maximum milliseconds to wait for serial data to 500
  
  // Setup stepperMotor
  stepperMotor.setup(NORMAL, STEP_REVOLUTION);  // Normal mode (see Arduino UstepperS library)

  // Set stepperMotor driver direction
  stepperMotor.driver.setShaftDirection(1);     // Inverted direction

  // Set motor brake mode when motor is stopped
  stepperMotor.setBrakeMode(COOLBRAKE);
}



/********************************************************************************************
* Main loop                                                                                 *
********************************************************************************************/
void loop() {

  uint8_t mMode = stepperMotor.getMotorMode();

  // Reads and interprets the commands sent to serial line
  if (Serial.available()) commandInterpreter(&stepperMotor, mMode);
  
    
  // Lost steps test and correct
  if ((mMode != MOTOR_INIT) && (mMode != MOTOR_ABORTED)) {
    // Motor not Checked and Driver position reached
    if ((mMode != MOTOR_CHECKED) && (stepperMotor.getMotorState(POSITION_REACHED) == 0)) {    
      // Updates motorSteps          
      if (mMode == MOTOR_RUNNING_RIGHT) stepCntr.motorSteps += stepCntr.nSteps;
        else if (mMode == MOTOR_RUNNING_LEFT) stepCntr.motorSteps -= stepCntr.nSteps;
      checkSteps(&stepperMotor, &stepCntr); // Check and correct lost steps
      Serial.println(ACK_REACHED);
      }
    }
}



/********************************************************************************************
* Reads and interprets the commands sent to serial line                                     *
*                                                                                           *
* input : motor : Stepper motor                                                             *
*         steps : Absolute and relative step values                                         *
********************************************************************************************/
void commandInterpreter(class Umotor *motor, step_rcd *steps)
{
uint8_t mMode = motor->getMotorMode();

// If motor aborted after emergency stop command sent
  if (mMode == MOTOR_ABORTED) {
    Serial.println(ACK_RESET);
    String str = Serial.readString(); // Clear serial buffer
    }

    // Else motor normaly stopped and checked
    else {
      // Gets the String read from the serial buffer, up to the delimiter character CMD_DELIMITER if found, or the entire 
      // buffer if not found (after timeout defined in setup()). The delimiter character is not returned in the string
      String cmdString = Serial.readStringUntil(CMD_DELIMITER);  
  
      // Command Emergency Stop
      if (cmdString.equals(CMD_STOP)) {
        motor->stop(HARD);
        motor->setMotorMode(MOTOR_ABORTED);
        Serial.println(ACK_ABORTED);
        }       

        // Command Move Right
        else if (cmdString.equals(CMD_GOTORIGHT)) {
          // Motor normally stopped and checked
          if (mMode == MOTOR_CHECKED) {
            // Looks for the next valid integer in the serial buffer, returns 0 if not found (after timeout defined in setup())
            steps->nSteps = int32_t(Serial.parseInt());
            if (steps->nSteps <= 0) Serial.println(ACK_INVALID);
              else if ((steps->motorSteps + steps->nSteps) > LIMIT_STEP_MAX) Serial.println(ACK_OUTRANGE);
                    else {
                      Serial.println(ACK_START);
                      Serial.println(steps->nSteps);
                      motor->rotateSteps(steps->nSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
                      }
            }
            // Motor must moved Home first
            else if (mMode == MOTOR_INIT) {
              Serial.readString(); // Clear serial buffer
              Serial.println(ACK_INIT);
              }
              // Motor moving
              else {
                Serial.readString(); // Clear serial buffer
                Serial.println(ACK_WAIT);
                }
          }

          // Command Move Left
          else if (cmdString.equals(CMD_GOTOLEFT)) {
            // Motor normally stopped and checked
            if (mMode == MOTOR_CHECKED) {
              // Looks for the next valid integer in the serial buffer, returns 0 if not found (after timeout defined in setup())
              steps->nSteps = int32_t(Serial.parseInt());
              if (steps->nSteps <= 0) Serial.println(ACK_INVALID);
                else if ((steps->motorSteps - steps->nSteps) < LIMIT_STEP_MIN) Serial.println(ACK_OUTRANGE);
                    else {
                      Serial.println(ACK_START);
                      motor->rotateSteps(steps->nSteps, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
                      }
              }
              // Motor must move Home first
              else if (mMode == MOTOR_INIT) {
                Serial.readString(); // Clear serial buffer
                Serial.println(ACK_INIT);
                }
                // Motor moving
                else {
                  Serial.readString(); // Clear serial buffer
                  Serial.println(ACK_WAIT);
                  }
            }

            // Command Move Home
            else if (cmdString.equals(CMD_GOHOME)) {
              // Motor normally stopped and checked
              if ((mMode == MOTOR_INIT) || (mMode == MOTOR_CHECKED)) {
                Serial.println(ACK_START);
                MoveToInitialPosition(motor, steps);
                Serial.println(ACK_HOME);
                }
                // Motor moving
                else {
                  Serial.readString(); // Clear serial buffer
                  Serial.println(ACK_WAIT);
                  }
              }

              // Command Get Step Position
              else if (cmdString.equals(CMD_POSE)) {
                // Motor must move Home first
                if (mMode == MOTOR_INIT) {
                  Serial.readString(); // Clear serial buffer
                  Serial.println(ACK_INIT);
                  }
                  // Motor moving
                  else if ((mMode == MOTOR_RUNNING_RIGHT) || (mMode == MOTOR_RUNNING_LEFT)) Serial.println(stepperMotor.getDriverSteps()); 
                    else Serial.println(steps->motorSteps);
                }
                else Serial.println(ACK_ERROR);
      }
}



/********************************************************************************************
* Move carriage to initial position                                                         *
*                                                                                           *
* input : motor : Stepper motor                                                             *
*         steps : Absolute and relative step values                                         *
********************************************************************************************/
void MoveToInitialPosition(class Umotor *motor, step_rcd *steps)
{
  int32_t nMstps;
  
  // Set motor mode to INIT
  // motor->setMotorMode(MOTOR_INIT);
  
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

  // Reset steps recod
  steps->motorSteps = 0;
  steps->nSteps = 0;

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
*         steps : Absolute and relative step values                                         *
********************************************************************************************/

void checkSteps(class Umotor *motor, step_rcd *steps)
{
  int32_t encSteps = motor->getEncoderSteps();
  int32_t deltaSteps = steps->motorSteps - encSteps;

  // Serial.print("targetStep = "); Serial.println(msteps);
  // Serial.print("encoderStep = "); Serial.println(encSteps);
  // Serial.print("deltaStep = "); Serial.println(deltaSteps);

  while (deltaSteps != 0) {
    if (deltaSteps > 0) motor->rotateSteps(deltaSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
      else motor->rotateSteps(int32_t(-1) * deltaSteps, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);
    while (motor->getMotorState(POSITION_REACHED)); // Wait for position reached
    encSteps = motor->getEncoderSteps();
    deltaSteps = steps->motorSteps - encSteps;

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
