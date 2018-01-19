#include "DataControl.h"

DataControl::DataControl(){
  magnitude = 0.0;
  _xData = 0.0;
  _yData = 0.0;
  _zData = 0.0;
}

DataControl::takeData(){
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  _xData = a.acceleration.x;
  _yData = a.acceleration.y;
  _zData = a.acceleration.z;
}

DataControl::calcMagnitude(){
  magnitude = sqrt(pow(_xData, 2) + pow(_yData, 2) + pow(_zData, 2));
}
