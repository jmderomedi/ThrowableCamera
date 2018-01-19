#include "Sensor_Arrays.h"

Sensor_Arrays::Sensor_Arrays() {
  _count = 0;
  _windowSize = 10;
}

/**
 * @param value; The value of the magnitude of the readings from the sensor
 */
Sensor_Arrays::fillArray(float value) {
  if (_count != 60) {
    _rawDataArray[_count] = value;                  //Assigns value to _count position
    _count += 1;                                    //Increments count
  } else {
    for (int i = 0; i < (_count - 1); i += 1) {
      _rawDataArray[i] = _rawDataArray[i + 1];      //Moves all data down by one
    }
    _rawDataArray[_count] = value;                  //Assigns data to last position
  }
}