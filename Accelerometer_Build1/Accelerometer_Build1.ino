/*
 * THIS CODE IS NOW OUTDATED, USE ONLY AS A REFERENCE
 * NEW CODE INVOLVED LIBARIES
 * NEW MAIN CODE AS WELL
 * I DONT KNOW WHAT IS HAPPEN WITH GIT
 * I THINK... I REALLY HOPE THAT THIS WILL WORK WHEN I SAVE, I CAN COMMIT IT IN GIT THAN PUSH IT EASLY
 */



/*

   ==========Accelerometer Setup=========
   Teensy SCL ----- LSM9 SCK
   Teensy SDA ----- LSM9 SDA
   Teensy 3.3v ---- LSM9 VIN
   Teensy GND ----- LSM9 GND

   ==========Button Setup================
   Teensy Pin2 ---- Button ---- +5V

   ==========Micro-SD Breakout===========
   Teensy SCK ----- MicroSD CLK
   Teensy DIN ----- MicroSD DO
   Teensy DOUT ---- MicroSD DI
   Teensy CS(10)--- MicroSD CS
   Teensy 3.3v ---- MicroSD 3v
   Teensy GND ----- MicroSD GND



   Author: James Deromedi
   Created: 11/15/2017
   Last Modified: 12/5/2017
*/

// 1) Move any global varaibles into functions if that is the only place they are called

// 2) Have a led that lights up when all the arrays have been filled. Therefore reducing the chance of currupted data

// 4) Change to calibration of each vector by themselves than to a single magnitude
//      4a) baseline the sensor before throwing
//      4b) Remove bias of sensors after button push
//      4c) Calibration will need to be done three times. Once per vector.
//            4c.i) Have leds that will turn on for which side the device needs to sit on to calibrate
//            4c.ii) Either have a time limit, sense low movement, or button push to calibrate the sides

// 5) Add Simpsons Rule function

// 6) Entire system is running in milliseconds
//      6a) CurrentMillis will be needed to change to micro

// 7) Turn off unused sensors
//      7a) Have accelerometer turn on and off when needed

// 8) Code in a way to turn on EL Wire

//Importing libaries needed
#include  <SD.h>
#include  <Wire.h>
#include  <SPI.h>
#include  <Adafruit_LSM9DS1.h>                  //LSM9 library with access to the sensors
#include  <Adafruit_Sensor.h>                   //Generic library containing methods to deal with sensors
#include  <elapsedMillis.h>                     //Inports timer libary
#include <TimerOne.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();      //Sets up an object for the LSM9
File myFile;                                    //File object, where data will be written

const int LSM9DS1_SCK = 19;
const int LSM9DS1_MOSI = 18;
const int WINDOWSIZE = 20;                      //Change to reduce/increase noise
const int QUEUELENGTH = 60;                     //Change to reduce/increase noise
const int NOTCALIBRATIONLED = 14;               //Calibration LED
const int CALIBRATIONLED = 15;
const int BUTTONINPUT = 2;                      //Pin that takes input from the button
const int CHIPSELECT = 10;                      //The pin for the slave's chip
const float GRAVITY = 9.80665;                  //Given by hardware
const int TESTINGLED = 16;

float zInputQueue[QUEUELENGTH],
      xInputQueue[QUEUELENGTH],
      yInputQueue[QUEUELENGTH];
float zOut, xOut, yOut;                         //Output values for smoothed vectors
float magnitudeValue;
float previousMagnitude = 0.0;        //Value of the devices magnitude and the previous magnitude
float initVelocity, peakThrow;      //Running sum of all magnitudes; inital velocity at launch
float calibrationMag;                           //The value of magntitude when first calibrated
float startAccel, stopAccel;                    //Value of the starting throw magnitude and release point magnitude

float xMin = 512.0;
float yMin = 512.0;
float zMin = 512.0;                 //Minimum values of the vectors during calibration
float xMax = 512.0;
float yMax = 512.0;
float zMax = 512.0;                 //Maximum values of the vectors during calibration

long startToss, endToss, deltaTime;             //Time at start throw; Time at release; Change between start and end

int buttonState = LOW;                          //Current state of the button input
int buttonPushes = 0;                           //Counts number of button pushes; Used on testing to stop loop()
int startButtonState = LOW;                     //Used to control startThrow();

bool releasePointFlag = false;                  //Flag to determine when the device is thrown
bool pictureFlag = false;                       //Flag to determine when pictures have completed
bool buttonPushFlag = false;                    //Flag to determine if the button was pushed
bool calibrationFlag = false;                   //Flag to determine if the device is calibrated
bool startThrowFlag = false;                    //Flag to determine if the device is being thrown

elapsedMillis currentMillis;                    //Variable to hold the time which throw begins4
elapsedMillis blinkInterval;

int counter = 0;                                //Counter for magArray, since it not filled in one swoop
float magArray[QUEUELENGTH];                    //Array for magnitude values. 60 data points long

//-------------------------------------------------------------------------------------------------------
void setup() {
  pinMode(TESTINGLED, OUTPUT);
  pinMode(NOTCALIBRATIONLED, OUTPUT);              //Blinks when the device is calibrated
  pinMode(CALIBRATIONLED, OUTPUT);              //Blinks when the device is calibrated
  pinMode(BUTTONINPUT, INPUT);                  //Sets up the input pin for the button

  Timer1.initialize(150000);
  Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds

  Serial.begin(115200);                         //The speed of bits per second from the Teensy to computer
  while (!Serial) {                             //Pauses the system until a serial window is open
    delay(1);
  }

  if (!lsm.begin()) {                           //Checks if the chip in 9DOF is currently plugged in properly
    Serial.println("Unable to initialize 9DOF");
    return;
  }
  Serial.println("Found LSM9DS1 9DOF");

  Serial.print("Initializing SD card...");      //Checks if the SD card is currently plugged in properly
  if (!SD.begin(CHIPSELECT)) {
    Serial.println("Initialization failed!");
    Serial.println("Is the card plugged in?");
    return;
  }
  Serial.println("SD card initialization done.");

  if (SD.exists("Data.txt")) {                  //Checks if the SD already has a file, if so, deletes it
    SD.remove("Data.txt");
  }
  myFile = SD.open("Data.txt", FILE_WRITE);     //Opens a file for Read and Write
  if (myFile) {                                 //If the file opened okay, print it out
    Serial.println("Opened \"Data.txt\" file properly");
  } else {
    Serial.println("error opening Data.txt");   //If the file didn't open, print an error
  }

  setupSensor();                                //Helper to just set the default scaling we want

  for (int i = 0; i < QUEUELENGTH; i = i + 1) { //To seed the array with inital data: might not be important
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    zInputQueue[i] = a.acceleration.z;
    xInputQueue[i] = a.acceleration.x;
    yInputQueue[i] = a.acceleration.y;
  }

  digitalWrite(NOTCALIBRATIONLED, LOW);           //Stays on until the device is calibrated; could be different color
  digitalWrite(CALIBRATIONLED, HIGH);
  digitalWrite(TESTINGLED, HIGH);
}//END SETUP

//-------------------------------------------------------------------------------------------------------

void loop() {
  vectorSmoothing();                           //Continously retrieves data and smoothes it; fluid input data

  buttonState = digitalRead(BUTTONINPUT);
  if (buttonState == HIGH && !(calibrationFlag == true)) {            //Checks for button push and if the device has been calibrated

    //Have a time loop here so it happens for a set amount of time
    float xReading = readXAverageAxisAcceleration();
    float yReading = readYAverageAxisAcceleration();
    float zReading = readZAverageAxisAcceleration();
    vectorAccelerationLimits(xReading, yReading, zReading);

    digitalWrite(NOTCALIBRATIONLED, HIGH);      //Turns off the calibration led, stating that it is now calibrated
    blinkLED(CALIBRATIONLED);                   //Blinks the calibration led, calibration complete

    //Once complete wil time loop, we will have the min and max values of the device while sitting still
    calibrationFlag = true;                     //Calibration will only happen once
    Serial.println("Calibration Complete");
  }

  buttonState = digitalRead(BUTTONINPUT);       //Checks for button push, calibrationFlag and if the device started already
  if (buttonState == HIGH && !(buttonPushFlag == true) && calibrationFlag == true) {
    buttonPush();                               //Should start ELwire
    Serial.println("Button pushed, starting program");
  }

  while (buttonPushFlag == true && calibrationFlag == true) {  //Checks if calibrated and button to start

    vectorSmoothing();
    startThrow();

    while ( startThrowFlag == true && pictureFlag != true ) { //After start of throw is found and picture not taken
      Serial.println("Start of throw has been found!");
      float magnitudeAverage = 0.0;                           //Varaible so the average can be printed
      vectorSmoothing();
      sumRunning();
      magnitudeAverage = CFAR(magnitudeValue);                //Has it assigned so it can be printed out, otherwise leave unassigned

      if (myFile) {   //Writing to SD card
        myFile.print(magnitudeValue);
        myFile.print("      ");
        myFile.print(magnitudeAverage);
        myFile.print("      ");
        myFile.print(yOut);
        myFile.print("      ");
        myFile.println(zOut);
      }

      buttonState = digitalRead(BUTTONINPUT);     //Following code used to 'end' program
      boolean endFlag = false;
      buttonPushes += 1;
      if (buttonState == HIGH && buttonPushes >= 30) {               //If button is pushed and a certian amount of time has passed
        Serial.print("FILE CLOSED");
        endFlag = true;
        if (endFlag == true) {
          myFile.close();
          while (endFlag == true) {               //Program does nothing, properly closes file
          }
        }
      }//END Data writing

      if (releasePointFlag == true) {             //Once the device is released from hand
        integration();
        maxHeight();
        releasePointFlag = false;                 //This should force the if loop to run once
      }

      if (currentMillis == peakThrow) {           //Once the device reaches time of max height, takes pictures
        /*Camera functions happen here; Possibility of different sketch
          //Cameras should be auto-focusing and quick shutter speed
          //Should take a burst of pictures, right before, during and after peak.
          //Send pictures to SD card
          //Be able to download pictures through USB
          //After picture are taken*/
        Serial.print("Peak of arc has been found");
        myFile.close();             //Closes just in case it gets here

        pictureFlag = true;                       //Picture has been taken, no more need to loop
        buttonPushFlag == false;                  //Resets button flag, looping until button push
        blinkLED(TESTINGLED);
      }

    }
  }
} //END LOOP


//-------------------------------------------------------------------------------------------------------
/**
   Returns the current magnitude value of the data
   @Param: None
   @Return: None
*/
float sumRunning() {
  return magnitudeValue
}



//-------------------------------------------------------------------------------------------------------
/**
   Sets a flag that the button has been pushed to start looping.
   Will turn on the EL wire for the device.
   @Param: None
   @Return: None
   IN-PROGRESS
   See Note 8
*/
void buttonPush() {
  buttonPushFlag = true;
}

//-------------------------------------------------------------------------------------------------------
/**
   Computes the intergral of acceleration by dividing the delta acceleration by delta time.
   This methods works because the acceleration of the throw should be constant.
   @Param: None
   @Return: None
*/
void integration () {
  deltaTime = endToss - startToss;
  initVelocity = runningSum / deltaTime;
}

//-------------------------------------------------------------------------------------------------------
/**
   Finds the magnitude at each data point. By using the equation M = sqrt(x^2+y^2+z^2).
   @Param: X Vector, Y Vector, Z Vector
   @Return: The magnitude calculated
*/
float magnitude(float x, float y, float z) {
  previousMagnitude = magnitudeValue;             //Saves the previous value magntitude
  magnitudeValue = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  float magnitudeGeneral = magnitudeValue;
  return magnitudeGeneral;
}

//-------------------------------------------------------------------------------------------------------
/*
   Used to calculate when in milliseconds the device will reach the peak point of the throw.
   Peak = Vo / g
   @Param: None
   @Return: None
*/
void maxHeight () {
  peakThrow = (initVelocity / GRAVITY);     //This might be in seconds
}



//-------------------------------------------------------------------------------------------------------
/**
   Gives the 9DOF specific variances on readings. This is subject to change.
   @Param: None
   @Return: None
*/
void setupSensor() {
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);   //Accelerometer Range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);    //Magnetomer Sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS); //Gryroscope setup
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

//-------------------------------------------------------------------------------------------------------
/**
   This function includes all the data reading and modification.
   Reduces the overall noise given by the accelerometer. It uses a sliding window to compute the average.
   The use of arrays that delete the first data, moves the rest down, and adds a new one
   Also adds the value computed for magnitude into an moving array
   @Param: None
   @Return: None
*/
void vectorSmoothing() {
  lsm.read();                                   //Asks for the data from the sensor
  sensors_event_t a, m, g, temp;                //Creates an event of which data is taken from the sensor
  lsm.getEvent(&a, &m, &g, &temp);              //Sends the data from sensor to Teensy

  for (int i = 0; i < WINDOWSIZE; i = i + 1) {  //Finds the average acceleration for each vector
    if (zInputQueue[i] > GRAVITY) {             //If the value is above 9.8, than removes the force of gravity
      zOut = (zOut - GRAVITY) + zInputQueue[i]; //Finds the Z vector without gravity
    } else {
      zOut = zOut + zInputQueue[i];
    }
    xOut = xOut + xInputQueue[i];
    yOut = yOut + yInputQueue[i];
  }
  zOut = float(zOut / WINDOWSIZE);
  xOut = float(xOut / WINDOWSIZE);
  yOut = float(yOut / WINDOWSIZE);

  magnitude(xOut, yOut, zOut);                  //Gives the magnitude of the three vectors after they are created

  if (counter < QUEUELENGTH) {                  //If the counter is smaller than the size of array
    magArray[counter] = magnitudeValue;         //Places newly calculated magnitude into array
    counter += 1;                               //Add one to the counter
  }
  if (counter == QUEUELENGTH) {                  //If the counter equals size of MagArray
    counter = QUEUELENGTH;                       //Keeps counter at max value
    for (int i = 0; i < QUEUELENGTH; i += 1) {   //Loops through entire array
      if (i == (QUEUELENGTH - 1)) {               //If i is one below max point in array
        magArray[counter] = magnitudeValue;        //Adds the new value of magnitude
      } else {
        magArray[i] = magArray[i + 1];          //Otherwise it just moves all the data over by one
      }
    }
  }

  for (int i = 0; i < QUEUELENGTH; i = i + 1) { //Moves all the data over and adds new data to continue smoothing
    if (i == (QUEUELENGTH - 1)) {
      zInputQueue[i] = a.acceleration.z;
      xInputQueue[i] = a.acceleration.x;
      yInputQueue[i] = a.acceleration.y;
    } else {
      zInputQueue[i] = zInputQueue[i + 1];
      xInputQueue[i] = xInputQueue[i + 1];
      yInputQueue[i] = yInputQueue[i + 1];
    }
  }
}

//-------------------------------------------------------------------------------------------------------
/**
   New calibration method!! used for finding all three vectors seperetly.
   First step is reading the sensor for average values
   If the param 'axis' does not work, will have to include a flag system that keeps track of which axis' have
   been completed.
   @Param: axis is the char of the axis being looked for
   @Return: The average reading of the axis during rest
*/
float readXAverageAxisAcceleration() {
  lsm.read();                                   //Asks for the data from the sensor
  sensors_event_t a, m, g, temp;                //Creates an event of which data is taken from the sensor
  lsm.getEvent(&a, &m, &g, &temp);              //Sends the data from sensor to Teensy

  float xAxisReading = 0.0;
  float averageReading = 0.0;

  for (int i = 0; i < WINDOWSIZE; i += 1) {
    xAxisReading += a.acceleration.x;
  }
  averageReading = xAxisReading / ((float)WINDOWSIZE);
  return averageReading;
}

//-------------------------------------------------------------------------------------------------------
float readYAverageAxisAcceleration() {
  lsm.read();                                   //Asks for the data from the sensor
  sensors_event_t a, m, g, temp;                //Creates an event of which data is taken from the sensor
  lsm.getEvent(&a, &m, &g, &temp);              //Sends the data from sensor to Teensy

  float yAxisReading = 0.0;
  float averageReading = 0.0;

  for (int i = 0; i < WINDOWSIZE; i += 1) {
    yAxisReading += a.acceleration.y;
  }
  averageReading = yAxisReading / ((float)WINDOWSIZE);
  return averageReading;
}

//-------------------------------------------------------------------------------------------------------
float readZAverageAxisAcceleration() {
  lsm.read();                                   //Asks for the data from the sensor
  sensors_event_t a, m, g, temp;                //Creates an event of which data is taken from the sensor
  lsm.getEvent(&a, &m, &g, &temp);              //Sends the data from sensor to Teensy

  float xAxisReading = 0.0;
  float yAxisReading = 0.0;
  float zAxisReading = 0.0;
  float averageReading = 0.0;

  for (int i = 0; i < WINDOWSIZE; i += 1) {
    zAxisReading += a.acceleration.z;
  }
  averageReading = zAxisReading / ((float)WINDOWSIZE);
  return averageReading;
}

/**
   New calibration method!! The actual comparing of the data to find the min and max values
   @Param: xAxis, yAxis, zAxis are the averaged values of the vectors during rest
   @Return: None
*/
void vectorAccelerationLimits(float xAxis, float yAxis, float zAxis) {
  if (xAxis < xMin) {                           //Finds min and max values of the X axis
    xMin = xAxis;
  }
  if (xAxis > xMax) {
    xMax = xAxis;
  }
  if (yAxis < yMin) {             // Finds the min and max values of the y Axis
    yMin = yAxis;
  }
  if (yAxis > yMax) {
    yMax = yAxis;
  }
  if (zAxis < zMin) {             // Finds the min and max values of the z Axis
    zMin = zAxis;
  }
  if (zAxis > zMax) {
    zMax = zAxis;
  }
}

//-------------------------------------------------------------------------------------------------------
/**
   Needs to implement interval with elapsedMillis
   @Param: None
   @Return: None
*/
void blinkLED(int ledPin) {
  int ledState = LOW;
  static int ledCount = 0;
  ledCount = ledCount % 2;
  while (ledCount != 0) {
    if (ledState == LOW) {
      ledState = HIGH;
      digitalWrite(ledPin, ledState);
    } else {
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
  }
}

/*
   Since smoothVectors keeps the magnitude array filled and moving, wont need to do it here.
   All it needs to do is average the data and add the threshold
   Since the raw data is averaged to reduce noise than created into a magnitude.
   This function than averages the magnitude. This will reduce even more noise.
   Than with the added thersold value, if the raw data pass the thershold, start looking for a peak
   Saves alot of work with dealing with noise
*/
//-------------------------------------------------------------------------------------------------------
float CFAR(float mag) {

  float magAverage = 0.0;
  float thersholdValue = 5.0;                       //This is up to change depending on testing

  for (int i = 0; i < WINDOWSIZE; i += 1) {         //Loops through the array, adding them together in a window size of 20
    magAverage += magArray[i];
  }
  magAverage = magAverage / float(WINDOWSIZE);      //Averaged value of the magnitudes

  if (mag > (magAverage + thersholdValue)) {        //If the current magnitude value is greater than the averaged magnitude plus the thershold
    if (previousMagnitude > magnitudeValue) {       //if the current magnitude value is greater than the previous, means a peak
      stopAccel = previousMagnitude;                //Saves the peak magnitude value
      endToss = currentMillis;                      //Saves the amount of time that has passed
      releasePointFlag = true;                      //Sets the flag to be true
      currentMillis = 0;                            //Resets the timer back to zero; Because peak has been found
    }
  }
  return magAverage;                                //Used to print out the averaged value in testing
}

//-------------------------------------------------------------------------------------------------------
/**
   Newest start throw. Uses a button that is pushed on start of throw and released on throw. Allowing distinct
   time of start and stop.
*/
void startThrow() {
  int throwButtonState = digitalRead(throwButton);
  static float totalMagnitude = 0.0;                    //Static varaible so doesn't lose value between calls
  static bool timeFlag = false;                         //flag to control when time is reset
  static bool throwFlag = false;                        //Flag to control when the device is being thrown
  if (throwButtonState = HIGH && timeFlag == false) {   //Saves the start value of the time of button push
    currentMillis = 0;
    flag = true;
  }
  if (throwButtonState == HIGH && timeFlag == true) {
    totalMagnitude = sumRunning();
    releaseFlag = true;
  }
  if (throwButtonState == LOW && timeFlag == true && throwFlag == true) {   //Check button state and other flag
    releasePointFlag = true;                              //Sets flag that device has been released
  }
}

