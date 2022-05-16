/*
  Umotor.cpp - UstepperS Motor
*/

#include "Umotor.h"


// Instantiate a new UstepperS Motor object
Umotor::Umotor(int pin) : uStepperS()
{
  inPin = pin; 
  pinMode(inPin, INPUT);
}


// returns limit switch status (TRUE : switch pressed / FALSE : switch not pressed)
boolean Umotor::getLimitSwitchStatus()
{
  if (digitalRead(inPin)) return(false);  // Limit switch not pressed : returns FALSE
    else return(true);                    // Limit switch pressed : returns TRUE
}