#ifndef Sensor_Calculations_h
#define Sensor_Calculations_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

class Sensor_Calculations {
  public:
    unsigned long peakTime;

  private:
    float _magTotal;
    unsigned long _stopTime;
    unsigned long _startTime;

    void runningAverage(float magnitude);   //Used in integration
    void integration();                     //finds peakTime, runAvg/time diff
    void timeStart();                       //Saves when throw starts
    void timeStop();                        //Saves when throw ends
    
}
