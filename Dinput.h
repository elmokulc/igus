/*
  Input.h -  - Digital input
*/

#ifndef Dinput_h
#define Dinput_h

#include <Arduino.h>


class Dinput
{
  public:
    Dinput(int pin, byte mode);
    boolean getLevel();
  private:
    int inPin;
};

#endif
