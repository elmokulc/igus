/*
  Input.cpp - Digital input
*/

#include "Dinput.h"


// Instantiate a new DIGITAL INPUT class
Dinput::Dinput(int pin, byte mode) // pin = pin number, mode = INPUT, INPUT_PULLUP (and also INPUT_PULLDOWN for STM32)
{
  inPin = pin; 
  pinMode(inPin, mode);
}


// returns input level (HIGH or LOW)
boolean Dinput::getLevel()
{
  return(digitalRead(inPin));  // returns HIGH or LOW
}

