/***/

#include "Umotor.h"

#define LIMIT_STEP_MIN  0
#define LIMIT_STEP_MAX  45000

#define ACCELERATION_SLOW 200  // 200 steps/s^2
#define ACCELERATION_FAST 1000 // 1000 steps/s^2

#define VELOCITY_SLOW     200  // 200 steps/2
#define VELOCITY_FAST     3000 // 3000 steps/s

#define CMD_DELIMITER '#'
#define CMD_GOTORIGHT "R"
#define CMD_GOTOLEFT  "L"
#define CMD_GOHOME    "H"
#define CMD_STOP      "S"
#define CMD_INFO      "I"

#define LIMIT_SWITCH_PIN  2


Umotor stepperMotor(LIMIT_SWITCH_PIN);

int32_t motorSteps = 0;   // Absolute step target
int32_t nSteps = 0;       // Relative step number



/************************************************************************
SETUP
*************************************************************************/
void setup() {
  Serial.begin(9600);
  
  // Setup uStepperS
  stepperMotor.setup(NORMAL,STEP_REVOLUTION);         // Normal mode

  // Set motor driver direction
  stepperMotor.driver.setShaftDirection(1);

  // Move carriage to initial podition
  MoveToInitialPosition(&stepperMotor);
}



/************************************************************************
MAIN LOOP
*************************************************************************/
void loop() {
  if(Serial.available())
  {
    String incomingByte = Serial.readStringUntil(CMD_DELIMITER);
    // Serial.print("cmd = "); Serial.println(incomingByte);

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
      
    incomingByte = Serial.read(); // read CR
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
  if (!motor->getLimitSwitchState()) {      // Limit switch not pressed (HIGH)
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

  // Set motor mode to STOPPED
  motor->setMotorMode(MOTOR_STOPPED);
}



/************************************************************************ 
Check and correct lost steps
*************************************************************************/
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



/************************************************************************
Change distance (mm) to steps
*************************************************************************/
int32_t DistanceToSteps(float dist)
{
  int32_t nStps = int32_t (dist * float(STEP_REVOLUTION) / 1.5);  // 1.5 mm/round
      
  return(nStps);
}
