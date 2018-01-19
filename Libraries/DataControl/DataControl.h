#ifndef DataControl_h
#define DataControl_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

class DataControl {
  public:
    DataControl();
    float magnitude;

  private:
    float _xData;
    float _yData;
    float _zData;

    void takeData();
    float calcMagnitude();
};