/*
  Input.h -  - Digital input
*/

#ifndef Dinput_h
#define Dinput_h

#include <Arduino.h>


class Dinput
{
  public:
    Dinput(int pin, byte mode);  // (mode is WiringPinMode for STM32)
    boolean getLevel();
  private:
    int inPin;
};

#endif
