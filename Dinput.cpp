/*
  Dinput.cpp - Digital input
*/

#include "Dinput.h"


// Instantiate a new DIGITAL INPUT object
Dinput::Dinput(int pin, byte mode) // pin = pin number, mode = INPUT, INPUT_PULLUP
{
  inPin = pin; 
  pinMode(inPin, mode);
}


// returns input level (HIGH or LOW)
boolean Dinput::getLevel()
{
  return(digitalRead(inPin));  // returns HIGH or LOW
}
