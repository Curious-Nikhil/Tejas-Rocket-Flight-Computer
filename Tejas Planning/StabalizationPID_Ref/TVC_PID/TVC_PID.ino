//https://github.com/saustin123/active-stabilization/blob/master/MASTER_PROGRAM


#include <Wire.h>               //Including libraries
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Adafruit_MMA8451.h>

#define RAD_DEG 57.295779 //Convert from radians to degrees
#define TAU 0.05          //Complementary Filter Time Constant
#define Kp 0.6            //PID constants
#define Ki 0
#define Kd 0.25

Adafruit_MMA8451 mma = Adafruit_MMA8451();

#define GYRO_CS 4 // labeled CS
#define GYRO_DO 5 // labeled SA0
#define GYRO_DI 6  // labeled SDA
#define GYRO_CLK 7 // labeled SCL
Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);

#define rollServo1_pin 9      //Sets Arduino pins for servos
#define pitchServo1_pin 11
#define calibrateLED_pin 2
#define launchReadyLED_pin 12
#define calibrateButton_pin 8
#define firstMotion_pin 13

Servo rollServo1;
Servo pitchServo1;

float rollAcc; //Accelerometer variables
float pitchAcc;
float zGravity;

float pitchGyro; //Gyro variables
float rollGyro;
float newTime;
float oldTime;
float pitchBias;
float rollBias;
boolean firstTime = 1;
float lastPitchRate;
float currentPitchRate;
float lastRollRate;
float currentRollRate;
float pitchRate;
float rollRate;
float dtSec;

float rollComplementary; //Complementary filter variables
float pitchComplementary;

float oldRollError; //PID variables
float rollErrorI;
float oldPitchError;
float pitchErrorI;
float oldTimePid;
float newTimePid;
boolean firstTimePid = 1;
float rollI;
float pitchI;
float rollPID;
float pitchPID;

boolean calibrate = 0;
boolean launchReady = 0;
boolean firstMotion = 0;

void setup() {
  Serial.begin(9600);      //Initializes IMU and microcontroller pins
  mma.begin();
  gyro.begin(gyro.L3DS20_RANGE_500DPS);
  mma.setRange(MMA8451_RANGE_8_G);

  pinMode(calibrateLED_pin, OUTPUT);
  pinMode(launchReadyLED_pin, OUTPUT);
  pinMode(calibrateButton_pin, INPUT);
  pinMode(firstMotion_pin, INPUT);

  digitalWrite(calibrateLED_pin, HIGH); //Calibrates the gyro and offsets the bias for each axis
  int N = 100;
  for (int i = 0; i < N; i++) {
    gyro.read();
    pitchBias += gyro.data.z;
    rollBias += gyro.data.y;
  }

  pitchBias /= (float) N;
  rollBias /= (float) N;
  
  rollServo1.attach(rollServo1_pin);
  pitchServo1.attach(pitchServo1_pin);

  rollServo1.write(90);       //Initializes and zeros the servos
  pitchServo1.write(90);
  digitalWrite(calibrateLED_pin, LOW);
}

void getGyroAngle() {
   if (firstTime) {     //Resets the time variables for the gyro if it is the first time through the loop
    oldTime = millis();
    newTime = millis();
    firstTime = 0;
    return;
  }
  gyro.read();          //Reads the gyro rate and the time since the function was last called
  newTime = millis();
  dtSec = ((newTime - oldTime) / 1000.0);

  currentPitchRate = (gyro.data.z - pitchBias); //Removes bias from each axis reading
  currentRollRate = (gyro.data.y  - rollBias);

  pitchRate = (lastPitchRate + currentPitchRate) / 2; //Takes an average of the readings to remove large spikes
  rollRate = (lastRollRate + currentRollRate) / 2;
 
  pitchGyro += (pitchRate * dtSec); //Calculates the angle by accumulating the rate of rotation over time
  rollGyro += (rollRate * dtSec);
  
  lastPitchRate = currentPitchRate; //Resets time variables
  lastRollRate = currentRollRate;
  oldTime = newTime;
}

void getAccAngle() {
  sensors_event_t event;  //Gets a new event from the accelerometer
  mma.getEvent(&event);
  
  float insideSqrt = 1-(event.acceleration.x * event.acceleration.x) - (event.acceleration.y * event.acceleration.y); //Removes the thrust vector from the accelerometer reading in order to isolate the gravity vector
  if(insideSqrt < 0) { //Removed negative values
    insideSqrt = 0;
  }
  
  zGravity = sqrt(insideSqrt); //Calculates the z component of gravity

  rollAcc = atan2(event.acceleration.y, zGravity) * RAD_DEG; //Calculates the angle using the arctangent function and the gravity vector values from the accelerometer
  pitchAcc =  atan2(event.acceleration.x, zGravity) * RAD_DEG;
}

void getComplementary() { //The Complementary Filter function obtains a more accurate angle reading by applying a low pass filter to the accelerometer and a high pass filter to the gyro
 getGyroAngle(); //Reads angles from gyro and accelerometer
 getAccAngle();
 
 float a = TAU / (TAU + dtSec); //calculates the filter coefficient a using the time constant and elapsed time

 rollComplementary = (a * rollGyro) + ((1-a) * pitchAcc); //Complementary Filter formulas
 pitchComplementary = (a * -pitchGyro) + ((1-a) * rollAcc);

 if(abs(rollComplementary) > 90) { //Resets filter values if the value exceed 90 - if the rocket arcs that far over, it is going to be almost impossible to correct
  rollComplementary = 0;
 }

 if(abs(pitchComplementary) > 90) {
  pitchComplementary = 0;
 }
}

void PID() {    //PID function
 float errorRoll; //Declares PID variables
 float errorPitch;
 float rollP;
 float rollD;
 float pitchP;
 float pitchD;
 float dPitchError;
 float dRollError;

 if (firstTimePid) {
    oldTimePid = millis(); //Resets time variables on the first loop
    newTimePid = millis();
    firstTimePid = 0;
    return;
  }
  getComplementary(); //Calls the complementary filter function and makes each reading the error for that axis (the setpoint is 0 degrees)
  newTimePid = millis();
  float dtSecPid = ((newTimePid - oldTimePid) / 1000.0);
  errorRoll = rollComplementary;
  errorPitch = pitchComplementary;

  rollP = Kp * errorRoll; //P value calculation
  pitchP = Kp * errorPitch;
  
  rollErrorI += (errorRoll * dtSecPid); //I value calculation
  pitchErrorI += (errorPitch * dtSecPid);
  rollI = Ki * rollErrorI;
  pitchI = Ki * pitchErrorI;
  
  dRollError = errorRoll - oldRollError; //D value calculation
  dPitchError = errorPitch - oldPitchError;
  rollD = Kd * dRollError;
  pitchD = Kd * dPitchError;

  rollPID = rollP + rollI + rollD; //Adds P, I, and D values for each axis
  pitchPID = pitchP + pitchI + pitchD;
  
  oldRollError = errorRoll; //Resets time variables
  oldPitchError = errorPitch;
  oldTimePid = newTimePid;

  if(rollPID > 90) { //Constrains PID values to [-90, 90]
  rollPID = 90;
 } else if(rollPID < -90) {
  rollPID = -90;
 }

if(pitchPID > 90) {
  pitchPID = 90;
 } else if(pitchPID < -90) {
  pitchPID = -90;
 }

  rollPID += 90;   //Maps output from [-90, 90] to [0, 180] required by servo
  pitchPID += 90;
}

void loop() {
  PID();    //Calls the PID function and adjusts the servos when the system is turned on
  rollServo1.write(rollPID);
  pitchServo1.write(pitchPID);

  if(digitalRead(calibrateButton_pin) == LOW) {     //Runs the calibration sequence like in the setup when the calibrate button is pressed
    digitalWrite(calibrateLED_pin, HIGH);           //Turns on the "calibrate" LED

    pitchGyro = 0; rollGyro = 0;                  //Resetting all variables
    newTime = 0; oldTime = 0;
    pitchBias = 0; rollBias = 0;
    firstTime = 1;
    lastPitchRate = 0; currentPitchRate = 0;
    lastRollRate = 0; currentRollRate = 0;

    oldRollError = 0; oldPitchError = 0;
    rollErrorI = 0; pitchErrorI = 0;
    oldTimePid = 0; newTimePid = 0;
    firstTimePid = 1;
    rollI = 0; pitchI = 0;
   
    int N = 100;
    for (int i = 0; i < N; i++) {
     gyro.read();
     pitchBias += gyro.data.z;
     rollBias += gyro.data.y;
    }

    pitchBias /= (float) N;
    rollBias /= (float) N;

    rollServo1.write(90);
    pitchServo1.write(90);
    
    launchReady = true;
    digitalWrite(calibrateLED_pin, LOW);    //Turns off the "calibrate" LED and turns on the "ready for launch" LED
    digitalWrite(launchReadyLED_pin, HIGH);
  }

  while(launchReady) {    //Enters "launch ready" state - keeps track of the angle without updating the servos.
    PID();
    if(digitalRead(firstMotion_pin) == HIGH) {    //When the "first motion" pin is pulled out at liftoff, it waits 0.4 seconds and then begins updating the servos.
      firstMotion = true;
      delay(400);
      while(firstMotion) {
       PID();
       rollServo1.write(rollPID);
       pitchServo1.write(pitchPID);
      }
    }
  }
}
