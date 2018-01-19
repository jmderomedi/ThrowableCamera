#ifndef Sensor_Arrays_h
#define Sensor_Arrays_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

class Sensor_Arrays{
  public:
    Sensor_Arrays();
    
  private:
    int _count;                 //Variable used to keep track of where in array    
    
    void fillArray(float value);  //Adds magnitude value to array

};