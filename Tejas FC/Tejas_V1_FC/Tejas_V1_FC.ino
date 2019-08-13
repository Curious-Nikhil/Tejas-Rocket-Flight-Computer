// =============================
// == PHASE ZERO & ONE Devs ====
//==============================

/*Tejas V1 - @Nikhil Mishra
   https://curious-nikhil.github.io/
   Tejas is a rocket computer
*/
// =========================================
// ==         CONFIGURATION              ==
//==========================================


//#define PID_MOD
//#define SERVO_MOD
//#define DMP_MOD
#define SERIAL_DEBUG

// =============================
// == Include and Define Vars ==
//==============================

//Header Files
#include <I2Cdev.h>
#include "MPU6050.h"
#include <Wire.h>
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"
#include <SD.h>


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RLED 6// Green LED
#define GLED 7// Green LED
#define buzzer 8

// =============================================
// ===          MISC Global Vars             ===
// =============================================

//Timers Vars
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long countsec;
const unsigned long period = 100;  //no of ms.

bool launch = false;
bool pyro = false;
bool landed = false;
bool ABORT = false;

// =============================================
// ===              MPU Vars                 ===
// =============================================

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool blinkState = false;


//INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// =============================================
// ===              BAROMETER                ===
// =============================================

static float meters;
float temperature;
float pascal;
float est_alt;
bool err = false;
String filename;
File myFile;

BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);


// ================================================================
// ===               THE SETUP FUNCTION                       ===
// ================================================================
void setup() {

  Serial.begin(38400);

  //Startup Sound
  tone(buzzer, 2500, 300);
  delay(1000);
  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);
  delay(1000);

  // configure LED for output
  //pinMode(LED_PIN, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);

  Serial.println(FreeRam()); //set a threshold for that.

  //PASS 1
  if (FreeRam() > 275) {
    RED();
    while(1);
  }

  // PASS 2: Initialize SD Module
  initializeSD();

  //PASS 3: Initialize Gyroscope and Servo
    //needs calibration method also.
  initializeMPU();

  //PASS 3: Initialize Baromter
  initializeBMP();

  //PASS 4: Pyro Check


}

// ================================================================
// ===                    MAIN LOOP                            ===
// ================================================================
void loop() {

  if(launch = false && pyro = false && landed = false) {
    GREEN();
  }
  
  /*************
   * Detect launch
   * Enable ABORT - for extreame tilt.
   * Detect Apogee
   * * Fire Pyros
   */

  if (1) {
    launch = true;

    if (angle > 30) {
      //eject pyros
      ABORT = true;
    }



    //data log
  } 
  

  //countTime();

  altimeter();

  //Data logging
  writeSD(countsec, meters, pascal, est_alt);
}



// ================================================================
// ===               SD CARD Begin                       ===
// ================================================================

void initializeSD() {

  Serial.print(F("InitSD"));

  if (!SD.begin(4)) {
    Serial.println(F("SDFAIL"));
    RED();
    while (1);
  }
  Serial.println(F("SDinit"));


  //      // create a new file
  //    char filename[] = "LOGGER00.txt";
  //    for (uint8_t i = 0; i < 100; i++) {
  //      filename[6] = i/10 + '0';
  //      filename[7] = i%10 + '0';
  //      if (! SD.exists(filename)) {
  //        // only open a new file if it doesn't exist
  //        myFile = SD.open(filename, FILE_WRITE);
  //        break;  // leave the loop!
  //      }
  //    }

  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println("Failed file");
    err = true;
    while (1);
    RED();
  }
  else {
    Serial.println("File created");
    RED();
  }

  Serial.println(filename);

  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    //Print Header Files  - - meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

    myFile.print("Time");
    myFile.print(",");
    myFile.print("Pascal");
    myFile.print(",");
    myFile.print("Kalman_Height");
    myFile.print(",");
    myFile.print("mpuPitch");
    myFile.print(",");
    myFile.print("mpuRoll");
    myFile.print(",");
    myFile.print("mpuYaw");
    myFile.print(",");
    myFile.print("OutputX");
    myFile.print(",");
    myFile.println("OutputY");

    myFile.close();
    Serial.println(F("F-1"));

  } else {
    Serial.print(F("F-0"));
    RED();
    while (1);
  }
}

// ================================================================
// ===                         BAROMETER                       ===
// ================================================================

void initializeBMP() {

  Serial.print(F("InintBMP"));
  if (bmp280.initialize()) Serial.println(F("BMP1")); //sensor found
  else
  {
    Serial.println(F("BMP0")); //Sensor not found
    err = true;
    RED();
    while (1) {}
  }

  //Calibration Settings - https://www.best-microcontroller-projects.com/bmp280.html#L1080
  bmp280.setPressureOversampleRatio(10); //Oversampling Ratio!
  bmp280.setTemperatureOversampleRatio(1);
  bmp280.setFilterRatio(4);
  bmp280.setStandby(0);


  // onetime-measure:
  bmp280.setEnabled(0);
  bmp280.triggerMeasurement();

  Serial.println(F("BMPInit1"));
}


// =========================================================
// ===                          DMP                      ===
// ==========================================================

void initializeMPU() {

  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //incomplete. add a RED FUNCTION
}

// ================================================================
// ===           ALTIMETER - BAROMETER & KALMAN FILTER          ===
// ================================================================

void altimeter() {

  bmp280.setEnabled(0);
  bmp280.triggerMeasurement();

  bmp280.getTemperature(temperature);  // throw away - needed for alt.
  bmp280.getPressure(pascal);     // throw away - needed for alt.
  bmp280.getAltitude(meters);

  float est_alt = pressureKalmanFilter.updateEstimate(meters);

  Serial.print(meters);
  Serial.print(",");
  Serial.print(pascal);
  Serial.print(",");
  Serial.println(est_alt);
}
// ================================================================
// ===           SD CARD WRITE AND STUFF                        ===
// ================================================================
//Create a new filename everytime.
boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = (String)i + "FL.txt";

    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }

  return file;
}
// meters, pascal, est_alt
void writeSD(unsigned long countsec, float meters, float pascal, float est_alt) {

  Serial.println(F("got to WriteSD fn"));
  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {
    //Serial Prints

#ifdef SERIAL_DEBUG
    Serial.print(countsec);
    Serial.print(F(","));
    Serial.print(meters);
    Serial.print(F(","));
    Serial.print(pascal);
    Serial.print(F(","));
    Serial.print(est_alt);

#endif

    //Writing in SD Card!
    myFile.print(countsec);
    myFile.print(",");
    myFile.print(meters);
    myFile.print(",");
    myFile.print(pascal);
    myFile.print(",");
    myFile.print(est_alt);
    //    myFile.print(mpuPitch);
    //    myFile.print(",");
    //    myFile.print(mpuRoll);
    //    myFile.print(",");
    //    myFile.print(mpuYaw);
    //    myFile.print(",");
    //    myFile.print(OutputX);
    //    myFile.print(",");
    //    myFile.println(OutputY);

    delay(100);

    myFile.close();

  } else {
#ifdef SERIAL_DEBUG
    Serial.println(F("error Opening the txt file"));
#endif
    err = true;
    while (1);
  }
}


// ================================================================
// ===                  MISC FUNCTIONS                          ===
// ================================================================

//Counts time passed since FC started.
void countTime() {
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    countsec = currentMillis;
    startMillis = currentMillis;
  }
}

void RED()
{
  digitalWrite(RLED, HIGH);
  tone(7, 2500, 100);
  delay(200);
  digitalWrite(RLED, LOW);

  tone(7, 2500, 100);
  delay(200);
  digitalWrite(RLED, HIGH);

  tone(7, 2000, 100);
  delay(500);
  tone(7, 2000, 100);
}

void GREEN() {
  //Everything is fine.. signal.
  unsigned long interval = 1000;
  unsigned long previousMillis;
  currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    digitalWrite(GLED, HIGH);

    tone(7, 2500, 100);
  }
  else {
    digitalWrite(GLED, LOW);
  }
}
