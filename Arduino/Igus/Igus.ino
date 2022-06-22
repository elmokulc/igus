/********************************************************************************************
*        File: Igus.ino                                                                     *
*     Version: 1.0 Initial release                                                          *
*        Date: May 19th, 2022                                                               *
*      Author: Olivier PERIA                                                                *
* Description: Drives Linear Motion Module IGUS                                             *
*    Features: Possible commands sent to serial line :                                      *
*                 H or H# : Moving Home                                                     *
*                 P or P# : Getting current absolute step Position                          *
*                 S or S# : Emmergency Stop                                                 *
*                 h of h# : Printing Help                                                   *
*                 R#[arg] : Moving [arg] steps Right - Example: R#10000                     *
*                 L#[arg] : Moving [arg] steps Left                                         *
********************************************************************************************/

#include "Umotor.h"

#define LIMIT_STEP_MIN  0      // Motor Home position
#define LIMIT_STEP_MAX  45000  // Motor maximum position

#define VELOCITY_SLOW   200 // Slow velocity (steps/s)

#define ACCELERATION_SLOW 200  // Slow acceleration (steps/s^2)
#define ACCELERATION_FAST 1000 // Fast acceleration (steps/s^2)

#define LIMIT_SWITCH    PD2    // Limit switch pin

#define CMD_DELIMITER   '#'    // Deliliter character
#define CMD_GOTORIGHT   "R"    // command : Move Right
#define CMD_GOTOLEFT    "L"    // command : Move Left 
#define CMD_GOHOME      "H"    // command : Move Home 
#define CMD_STOP        "S"    // command : Emergency Stop 
#define CMD_POSE        "P"    // command : Get current absolute step Position 
#define CMD_MODE        "M"      // command : Request to change mode
#define CMD_HELP        "h"    // command : Help

#define SET_VF          "VF" // Set velocity fast

#define ACK_READY       "READY"
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

#define MD_LINEAR 0
#define MD_ROTATE 1

typedef struct 
{
  int32_t absSteps = 0;   // Absolute step position (target position)
  int32_t nSteps = 0;     // Relative step position

} step_rcd;

Umotor stepperMotor(LIMIT_SWITCH);  // Instantiates stepperMotor object with limit switch connected to LIMIT_SWITCH pin

step_rcd stepCntr;  // Step counter (absolute and relative position)

uint8_t mMode;      // Motor mode


int32_t VELOCITY_FAST = 100; // Fast velocity (steps/s) VMAX = 3000
uint8_t WK_MODE = MD_LINEAR;   // Working mode can be : "MD_LINEAR", "MD_ROTATE" or "MD_SETUP"


/********************************************************************************************
* Initializes the stepper motor and moves to initial position                               *
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

  Serial.println(ACK_READY);
}



/********************************************************************************************
* Main loop                                                                                 *
********************************************************************************************/
void loop() 
{
  // Reads and interprets the commands sent to serial line
  if (Serial.available()) commandInterpreter(&stepperMotor, &stepCntr);

  if (WK_MODE == MD_LINEAR){
      // Lost steps detection and correction
      mMode = stepperMotor.getMotorMode();
      if ((mMode != MOTOR_INIT) && (mMode != MOTOR_ABORTED)) {
        // Motor not checked and Driver position reached
        if ((mMode != MOTOR_CHECKED) && (stepperMotor.getMotorState(POSITION_REACHED) == 0)) {    
          // Updates absolute step position          
          if (mMode == MOTOR_RUNNING_RIGHT) stepCntr.absSteps += stepCntr.nSteps;
            else if (mMode == MOTOR_RUNNING_LEFT) stepCntr.absSteps -= stepCntr.nSteps;
          // Checks and corrects lost steps  
          checkSteps(&stepperMotor, &stepCntr);
          Serial.println(ACK_REACHED);
          }
        }   
    }
   else if (WK_MODE == MD_ROTATE){
      // Lost steps detection and correction
      mMode = stepperMotor.getMotorMode();
      if (mMode != MOTOR_ABORTED) {
        // Motor not checked and Driver position reached
        if ((mMode != MOTOR_CHECKED) && (stepperMotor.getMotorState(POSITION_REACHED) == 0)) {    
          // Updates absolute step position          
          if (mMode == MOTOR_RUNNING_RIGHT) stepCntr.absSteps += stepCntr.nSteps;
            else if (mMode == MOTOR_RUNNING_LEFT) stepCntr.absSteps -= stepCntr.nSteps;
          // Checks and corrects lost steps  
          //checkSteps(&stepperMotor, &stepCntr);
          stepperMotor.setMotorMode(MOTOR_CHECKED);
          Serial.println(ACK_REACHED);
          }
        } 
    
    }
}


/********************************************************************************************
* Reads and interprets the commands sent to serial line                                     *
*                                                                                           *
* input : motor : stepper motor                                                             *
*         pose  : absolute and relative step positions                                      *
********************************************************************************************/
void commandInterpreter(Umotor *motor, step_rcd *pose)
{
  // Gets motor mode
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
            pose->nSteps = int32_t(Serial.parseInt());
            if (pose->nSteps <= 0) Serial.println(ACK_INVALID);
              else if ((pose->absSteps + pose->nSteps) > LIMIT_STEP_MAX) Serial.println(ACK_OUTRANGE);
                    else {
                      Serial.println(ACK_START);
                      motor->rotateSteps(pose->nSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);
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
              pose->nSteps = int32_t(Serial.parseInt());
              if (pose->nSteps <= 0) Serial.println(ACK_INVALID);
                else if ((pose->absSteps - pose->nSteps) < LIMIT_STEP_MIN) Serial.println(ACK_OUTRANGE);
                    else {
                      Serial.println(ACK_START);
                      motor->rotateSteps(pose->nSteps, DIR_LEFT, ACCELERATION_FAST, VELOCITY_FAST);
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
                MoveToInitialPosition(motor, pose);
                Serial.println(ACK_HOME);
                }
                // Motor moving
                else {
                  Serial.readString(); // Clear serial buffer
                  Serial.println(ACK_WAIT);
                  }
              }

              // Command Get Absolute Step Position
              else if (cmdString.equals(CMD_POSE)) {
                // Motor must move Home first
                if (mMode == MOTOR_INIT) {
                  Serial.readString(); // Clear serial buffer
                  Serial.println(ACK_INIT);
                  }
                  else Serial.println(stepperMotor.getEncoderSteps());
                }

                // Command Help
                else if (cmdString.equals(CMD_HELP)) {
                  Serial.println("H or H# : Moving Home");
                  Serial.println("P or P# : Getting current absolute step Position");
                  Serial.println("S or S# : Emmergency Stop");
                  Serial.println("h of h# : Printing Help");
                  Serial.println("R#[arg] : Moving [arg] steps Right - Example: R#10000");
                  Serial.println("L#[arg] : Moving [arg] steps Left");
                  Serial.println("M#[arg] : Switch mode 0=LINEAR 1=ROTATE - Example: M#0");
                  Serial.println("VF#[arg] : Set max velocity (max=3000) - Example: VF#1000");
                  }

                else if (cmdString.equals(CMD_MODE)){
                  
                  WK_MODE = int32_t(Serial.parseInt());
                  Serial.readString(); // Clear serial buffer
                  }
                  
                  else if (cmdString.equals(SET_VF)){
                    
                    VELOCITY_FAST = int32_t(Serial.parseInt());
                    Serial.readString(); // Clear serial buffer
                    }

                  // Command not found
                  else Serial.println(ACK_ERROR);
      }
}



/********************************************************************************************
* Moves the carriage to initial position                                                    *
*                                                                                           *
* input : motor : stepper motor                                                             *
*         pose  : rbsolute and relative step positions                                      *
********************************************************************************************/
void MoveToInitialPosition(Umotor *motor, step_rcd *pose)
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

  // Reset driver and encoder step counters
  motor->driver.setHome();
  motor->encoder.setHome();

  // Reset motor step positions
  pose->absSteps = 0;
  pose->nSteps = 0;

  // Activates Motor brake
  motor->setBrakeMode(COOLBRAKE);

  // Set motor mode to CHECKED (target position reached)
  motor->setMotorMode(MOTOR_CHECKED);
}



/********************************************************************************************
* Lost steps detection and correction                                                       *
*                                                                                           *
* Compares the encoder step counter (real position) with the motor step counter (target     *
* position). If different, move the motor to make the encoder step position equal to the    *
* motor step position                                                                       *
*                                                                                           *
* input : motor : stepper motor                                                             *
*         pose  : absolute and relative step positions                                      *
********************************************************************************************/

void checkSteps(class Umotor *motor, step_rcd *pose)
{
  // Error between Motor target step position and Encoder real step position
  int32_t deltaSteps = pose->absSteps - motor->getEncoderSteps();

  while (deltaSteps != 0) {

    if (deltaSteps > 0) motor->rotateSteps(deltaSteps, DIR_RIGHT, ACCELERATION_FAST, VELOCITY_FAST);  // Move right
      else motor->rotateSteps(int32_t(-1) * deltaSteps, DIR_LEFT, ACCELERATION_SLOW, VELOCITY_SLOW);  // Move left

    while (motor->getMotorState(POSITION_REACHED));   // Wait for position reached
  
    deltaSteps = pose->absSteps - motor->getEncoderSteps();
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
