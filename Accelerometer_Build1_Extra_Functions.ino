//-------------------------------------------------------------------------------------------------------
/**
   Saves the data for the beginning of the throw. Sets time to zero.
   @Param: None
   @Return: None
*/
/*void startThrow() {
  currentMillis = 0;
  startToss = currentMillis;                    //sets the start of the toss value to whenever button is pushed
  startAccel = magnitudeValue;                  //sets whatever the accelerometer is at, as starting acceleration
  }*/
//-------------------------------------------------------------------------------------------------------
/**
   Calibrates the device by taking an intial measurement and creating a magnitude value.
   This will subtracted from input data's magnitude to remove any variance and gravity.
   @Param: None
   @Return: None
   IN-PROGRESS
   See Note 4
*/
void calibration() {
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  calibrationMag = magnitude(a.acceleration.x, a.acceleration.y, (a.acceleration.z - GRAVITY)); //Finds the magnitude of when the device is not moving
  calibrationFlag = true;

  /*delay(100);
    digitalWrite(CALIBRATIONLED, LOW);   //Blinks the LED twice when calibrated
    delay(200);
    digitalWrite(CALIBRATIONLED, HIGH);
    delay(200);
    digitalWrite(CALIBRATIONLED, LOW);
    delay(200);
    digitalWrite(CALIBRATIONLED, HIGH);
    delay(150);
    digitalWrite(CALIBRATIONLED, LOW);*/
}
//-------------------------------------------------------------------------------------------------------
/**
   Finds the maximum magnitude of the acceleration. This point will be when the device is released.
   Saves the time, which will be used in intergration.
   @Param: None
   @Return: None
   IN-PROGRESS
   See Note 3
*/
void maxPoint () {
  if (previousMagnitude > magnitudeValue) {
    stopAccel = previousMagnitude;
    endToss = currentMillis;
    releasePointFlag = true;
  }
}


//-------------------------------------------------------------------------------------------------------
/**
  Since smoothVector keeps track of the magnitude array and keeps it fresh. I can than loop through that array looking
  for increase.
  Continously divides the larger magnitude by smaller current magnitude. If that ratio is larger than the last,
  the device is increasing acceleration. If that happens all 60 times, good chance of throw starting. Than sets flag
*/
/*
  void startThrow() {
  int count = 0;                                     //Used to count how many times the magnitude is increasing
  float ratio = 1.00001;                             //Inital value for the ratio between the first two values
  currentMillis = 0;                                 //Sets timer to zero each time this is called. Time value of the current magnitude
  for (int i = 0; i < QUEUELENGTH - 1; i += 1) {     //Loops through entire magArray expect last value
    if ((magArray[i + 1] / magArray[i]) > ratio) {   //If the the next value divided by the current is larger than the ratio                                                  //Means that is increasing in acceleration
      count += 1;                                    //Adds one to count
    }
  }
  if (count == 60) {                                 //If every data piece of the array is increasing
    //NEEDS to change with testing
    startThrowFlag = true;                           //Sets flag to be true that throw has started
    //No need to save magnitudeValue because runningSum() will take care of that
  }
  }*/
