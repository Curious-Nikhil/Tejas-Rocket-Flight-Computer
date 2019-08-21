// =========================================
// ==         CONFIGURATION              ==
//==========================================


//#define PID_MOD
//#define SERVO_MOD
//#define DMP_MOD
#define SERIAL_DEBUG
//#define MPU_CALBIRATION
// =============================
// == Include and Define Vars ==
//==============================

//Header Files
#include <I2Cdev.h>
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"
#include <SD.h>


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RLED 6// Green LED
#define GLED 7// Green LED
#define buzzer 8
#define pyroPin 9
// =============================================
// ===          MISC Global Vars             ===
// =============================================

//Timers Vars
unsigned long previousMillis = 0;


bool launch = 0;
bool pyro = false;
bool pyroFired = false;
bool landed = false;
bool ABORT = false;

int lastAlt = 0;
unsigned long launchTime = 0;
unsigned long ApoTime = 0;
unsigned long landTime = 0;

// =============================================
// ===              BAROMETER                ===
// =============================================
BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

static float alt;
float est_alt;
float temperature;
float pascal;
float base_alt;
// =============================================
// ===              SD CARD                  ===
// =============================================
String filename;
File myFile;
int sd_count = 0; 
bool FL = false;




void setup() {

    Serial.begin(38400);

    initializeSD();
    initializeBMP();

}

void loop() {
    //get_alt();


    myFile = SD.open(filename, FILE_WRITE);
        
    Serial.println(myFile);

    if (myFile) {
    Serial.println("writing!");
    Serial.println(FreeRam());
    myFile.print(2);
    myFile.println("hello");
    delay(10);
    }
    else {
        while(1);
    }

    myFile.flush();


}
void get_alt() {
  bmp280.getAltitude(alt);
  bmp280.getPressure(pascal);
  est_alt = pressureKalmanFilter.updateEstimate(alt);

  return est_alt;
}

void initializeSD() {

  Serial.print(F("InitSD"));

  if (!SD.begin(4)) {
    Serial.println(F("SDFAIL"));
    RED();
    while (1);
  }
  Serial.println(F("SDinit"));

  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println(F("F-0"));
    while (1);
    RED();
  }
  else {
    Serial.println(F("F-1"));
    RED();
  }

  Serial.println(filename);

  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    //Print Header Files  - - alt, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

    myFile.print("Time");
    myFile.print(",");
    myFile.print("Pascal");
    myFile.print(",");
    myFile.println("KH");


    myFile.close();
    Serial.println(F("Fin-1"));

    dataFile.close();
  } else {
    Serial.print(F("Fin-0"));
    RED();
    while (1);
  }
}

void initializeBMP() {

  Serial.print(F("InintBMP"));
  if (bmp280.initialize()) Serial.println(F("BMP1")); //sensor found
  else
  {
    Serial.println(F("BMP0")); //Sensor not found
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

boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = (String)i + "FLT.txt";

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
