#include "Sensor_Calculations.h"

Sensor_Calculations::Sensor_Calculations() {
  _magTotal = 0.0;
}

Sensor_Calculations::runningAverage(float magnitude) {
  _magTotal = _magTotal + magnitude;
}

Sensor_Calculations::timeStart(unsigned long timeValue){
  _startTime = timeValue;
}

Sensor_Calculations::timeStop(unsigned long timeValue){
  _stopTime = timeValue;
}

Sensor_Calculations::intergration() {
  unsigned long temp = (_stopTime - _startTime);    //Finds the time between values
  temp = float(temp);                           //Converts temp into a float
  peakTime = (_magTotal / temp);  
}
